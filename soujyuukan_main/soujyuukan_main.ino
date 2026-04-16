#include <IcsBaseClass.h>
#include <IcsHardSerialClass.h>
#include <freertos/FreeRTOS.h>
#include <Preferences.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include "PidState.h"

// BLE設定
#define SERVICE_UUID "AAAA0001-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_UUID    "AAAA0002-36e1-4688-b7f5-ea07361b26a8"

#pragma pack(push, 1)
struct ControlData {
  float E_steer, R_steer;
  float E_trim, E_angle, R_angle;
  float e_servo_temp, r_servo_temp;
  char control_mode[12];
};
struct NavigationData {
  float pitch;
  float pitch_rate;  // ジャイロ Y軸 [°/s] — PID の D項に使用
};
#pragma pack(pop)

BLECharacteristic* pCharacteristic = NULL;
bool bleConnected = false;

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    bleConnected = true;
    Serial.printf("[%lu] BLE: connected\n", millis());
  }
  void onDisconnect(BLEServer* pServer) {
    bleConnected = false;
    Serial.printf("[%lu] BLE: disconnected, restart advertising\n", millis());
    pServer->getAdvertising()->start();
  }
};
volatile float currentPitch = 0.0;      // 姿勢角（ピッチ）[°]
volatile float currentPitchRate = 0.0;  // ピッチレート [°/s]
class MyCharCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pChar) {
    uint8_t* data = pChar->getData();
    size_t len = pChar->getLength();
    if (len == sizeof(NavigationData)) {
      NavigationData* incoming = (NavigationData*)data;
      currentPitch = incoming->pitch;
      currentPitchRate = incoming->pitch_rate;
    }
  }
};

Preferences preferences;
TaskHandle_t nvmTaskHandle = NULL;
bool settingsChanged = false;

const byte EN_PIN = 8;  //サーボ
const long BAUDRATE = 115200;
const int TIMEOUT = 25;
const int r_elevator = 2;  //可変抵抗エレベーター
const int r_rudder = 3;    //可変抵抗ラダー
const int trimE = 4;       //エレベータートリムスイッチ
const int LED = 5;
const int resetPin = 6;        // NVMリセット用ピンを変更 (GPIO 0番ピンをGNDとショートでリセット)
const int trimR1 = 9;          //トリムラダー
const int trimR2 = 10;         //トリムラダー

/*
ここがめっちゃ重要。詳しくは同じディレクトリにある。.docsを参照
*/

// 実舵角 x[°]（分度器）→ KRS（多項式本体・逆映射用はクランプなし）
static float ele2krs(float x) {
  return 0.012577102329*pow(x, 5) + 0.0194099102747*pow(x, 4) - 0.634353764183*pow(x, 3) - 0.815828278465*pow(x, 2) + 185.794437174*pow(x, 1) + 5701.22753031;
}

static float rud2krs(float x) {
  return -0.0105114837351*pow(x, 5) - 0.0659075647903*pow(x, 4) + 0.241297817826*pow(x, 3) + 2.97624922026*pow(x, 2) - 179.910899851*pow(x, 1) + 6563.03984539;
}

//KRS→ 舵角に変換する関数
float krs2ele(float x) {
  x = 2.9862304768e-18*pow(x, 5) - 7.42363558025e-14*pow(x, 4) + 3.95030812038e-10*pow(x, 3) + 2.10479251629e-06*pow(x, 2) - 0.0174092175976*pow(x, 1) + 18.1307278407;
  return constrain(x, -5, 5);
}
float krs2rud(float x) {
  x = -1.35085986658e-17*pow(x, 5) + 4.38562944215e-13*pow(x, 4) - 5.31829061512e-09*pow(x, 3) + 2.9458795701e-05*pow(x, 2) - 0.0758402111862*pow(x, 1) + 83.232581592;
  return constrain(x, -10, 10);
}

// 舵角上下限
float ElevatorDegMin = -5;
float ElevatorDegMed = 0;
float ElevatorDegMax = 5;
float RudderDegMin = -10.1;
float RudderDegMax = 9.3;

IcsHardSerialClass krs(&Serial0, EN_PIN, BAUDRATE, TIMEOUT);  //インスタンス＋ENピン(8番ピン)およびUARTの指定

// 変数の宣言 [°]
float Trimelevetor = 0.0;
float neutralTrimeEle = 0.0;
float Trimrudder = 0.0;


int is_pid = 0;  //今PID制御をONにするかどうか(0か1)

// PidState は PidState.h で定義
PidState pidElevator;
PidState pidRudder;         //一応ラダーも用意しているが、今回はラダーに関するオートパイロットは行わない

void pidInit(PidState *state, float kp, float ki, float kd, float integralMax) {
  state->kp = kp;
  state->ki = ki;
  state->kd = kd;
  state->integral = 0;
  state->integralMax = integralMax;
  state->lastError = 0;
  state->lastTime = millis();
}

void pidReset(PidState *state) {
  state->integral = 0;
  state->lastError = 0;
  state->lastTime = millis();
}

double pidCompute(PidState *state, double error, double gyroRate) {
  unsigned long now = millis();
  double dt = (now - state->lastTime) / 1000.0;
  if (dt <= 0) dt = 0.001;

  state->integral += error * dt;
  state->integral = constrain(state->integral, -state->integralMax, state->integralMax);

  // D項: 数値微分の代わりにジャイロ生値を使用
  // error = target - pitch なので d(error)/dt = -pitch_rate = -gyroRate
  double derivative = -gyroRate;

  double output = state->kp * error + state->ki * state->integral + state->kd * derivative;

  state->lastError = error;
  state->lastTime = now;

  return output;
}

// 設定を読み込む関数
void loadSettings() {
  preferences.begin("trim-data", true);
  Trimelevetor = preferences.getFloat("trimE", 0.0);
  neutralTrimeEle = preferences.getFloat("neutE", 0.0);
  Trimrudder = preferences.getFloat("trimR", 0.0);
  preferences.end();
  Serial.printf("Settings Loaded: E=%.2f, neutE=%.2f, R=%.2f\n", Trimelevetor, neutralTrimeEle, Trimrudder);
}

// 非同期でFlashに保存するタスク
void nvmTask(void *pvParameters) {
  while (1) {
    // 通知が来るまで待機
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    preferences.begin("trim-data", false);
    preferences.putFloat("trimE", Trimelevetor);
    preferences.putFloat("neutE", neutralTrimeEle);
    preferences.putFloat("trimR", Trimrudder);
    preferences.end();
    Serial.println(">> Settings saved to NVM");
  }
}

// リセットピンを常時監視するタスク
void resetMonitorTask(void *pvParameters) {

  while (1) {
    if (digitalRead(resetPin) == HIGH) {
      // チャタリング防止の少しの待機
      vTaskDelay(50 / portTICK_PERIOD_MS);

      if (digitalRead(resetPin) == HIGH) {
        Serial.println("!!! NVM Reset Triggered !!!");
        preferences.begin("trim-data", false);
        preferences.clear();
        preferences.end();

        // メモリ上の値も初期化
        Trimelevetor = 0.0;
        neutralTrimeEle = 0.0;
        Trimrudder = 0.0;

        // リセット成功の合図としてLEDを高速点滅(5回)
        for (int i = 0; i < 5; i++) {
          digitalWrite(LED, HIGH);
          vTaskDelay(100 / portTICK_PERIOD_MS);
          digitalWrite(LED, LOW);
          vTaskDelay(100 / portTICK_PERIOD_MS);
        }

        // ボタンが離されるまで待機（無限リセットループを防止）
        while (digitalRead(resetPin) == HIGH) {
          vTaskDelay(100 / portTICK_PERIOD_MS);
        }
      }
    }
    // 100msごとにボタンの状態をチェック
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void Ltika(void *pvParameters) {
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED, HIGH);
    vTaskDelay(300 / portTICK_PERIOD_MS);
    digitalWrite(LED, LOW);
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
  vTaskDelete(NULL);
}

// floatのmap関数
float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//中央値フィルタ
int getMedian(int* array, int size) {
  int temp[5];
  // tempのサイズ(5)を超える場合は安全のため5に制限する
  int copySize = size > 5 ? 5 : size;
  for (int i = 0; i < copySize; i++) temp[i] = array[i];

  // バブルソートで小さい順に並べ替え
  for (int i = 0; i < copySize - 1; i++) {
    for (int j = i + 1; j < copySize; j++) {
      if (temp[i] > temp[j]) {
        int t = temp[i];
        temp[i] = temp[j];
        temp[j] = t;
      }
    }
  }
  return temp[copySize / 2]; // 中央の値を返す
}
int detzoneMapping(int *ary,int x,float *y,float min,float med,float max){
  int r = 0;
  if(x <= ary[0]){
    *y = min;
  }else if(ary[0] < x && x <= ary[1]){
    *y = fmap(x,ary[0],ary[1],min,med);
  }else if(ary[1] < x && x <= ary[2]){
    *y = med;
    r = 1;
  }else if(ary[2] < x && x <= ary[3]){
    *y = fmap(x,ary[2],ary[3],med,max);
  }else{
    *y = max;
  }
  return r;
}

float elevetor = 0.0;  // [°]
float rudder = 0.0;    // [°]
int rawEle = 0;
int rawRud = 0;
int ELE;
int RUD;

int elergs[4] = { 1120, 1590, 1870, 2460 };  // エレベーター: 前限界, 前戻り, 後戻り, 後限界
int rudrgs[4] = { 1550, 2120, 2520, 3100 };

const int hisSize = 5;//getMedianの中のtemp配列の大きさを気にする
int eleHistory[hisSize]={0};
int rudHistory[hisSize]={0};

int prefELE = 0;
int prefRud = 0;
int is_center = 0;
// detzone へ入れる直前のフィルタ済み ADC（BLE の E_steer/R_steer 用）
int g_filteredEleAdc = 0;
int g_filteredRudAdc = 0;

void Potentiometer() {

  for (int i = 0;i < hisSize-1;i++){
    eleHistory[i] = eleHistory[i+1];
    rudHistory[i] = rudHistory[i+1];
  }
  rawEle = analogRead(r_elevator);
  rawRud = analogRead(r_rudder);
  eleHistory[hisSize-1] = rawEle;
  rudHistory[hisSize-1] = rawRud;
  ELE = getMedian(eleHistory,hisSize);
  RUD = getMedian(rudHistory,hisSize);

  float alfa = 0.3;
  int fELE = ELE*alfa + prefELE*(1-alfa);
  prefELE = fELE;

  int fRUD = RUD*alfa + prefRud*(1-alfa);
  prefRud = fRUD;

  g_filteredEleAdc = fELE;
  g_filteredRudAdc = fRUD;

  is_center = detzoneMapping(elergs, fELE, &elevetor, ElevatorDegMax, ElevatorDegMed, ElevatorDegMin);
  detzoneMapping(rudrgs, fRUD, &rudder, RudderDegMin, (RudderDegMin+RudderDegMax)/2, RudderDegMax);
}

unsigned long lastPushed = millis();  //ボタン4チャタリング防止用

// ニュートラル帯(2000..2300): 長押しはイン帯フレーム数、単押しは帯外が安定してから確定(@30Hz想定)
static const int NEUTRAL_LONG_PRESS_FRAMES = 20;       // >20 で発火（21 フレーム ≒ 0.7s）
static const int NEUTRAL_RELEASE_STABLE_FRAMES = 8;    // 帯外がこれ以上続けば離し確定（≒ 0.27s）
static bool neutralStrokeActive = false;
static bool neutralLongPressDone = false;
static int neutralBandHoldFrames = 0;
static int neutralOutsideStableFrames = 0;
static bool neutralWasInBand = false;

static void resetNeutralGesture() {
  neutralStrokeActive = false;
  neutralLongPressDone = false;
  neutralBandHoldFrames = 0;
  neutralOutsideStableFrames = 0;
  neutralWasInBand = false;
}

void trimElevetor() {
  int TrimE = analogRead(trimE);
  const bool inNeutralBand = (2000 <= TrimE && TrimE <= 2300);

  if (0 <= TrimE && TrimE <= 100) {  //優先度1
    resetNeutralGesture();
    Trimelevetor = Trimelevetor + 0.1;
    settingsChanged = true;

  } else if (1000 <= TrimE && TrimE <= 1200) {  // 優先度2
    resetNeutralGesture();
    Trimelevetor = Trimelevetor - 0.1;
    settingsChanged = true;

  } else if (inNeutralBand) {
    neutralOutsideStableFrames = 0;
    if (!neutralWasInBand) {
      neutralBandHoldFrames = 1;
    } else {
      neutralBandHoldFrames++;
    }
    neutralWasInBand = true;

    if (!neutralStrokeActive) {
      neutralStrokeActive = true;
      neutralLongPressDone = false;
    }
    //Serial.printf("neutralBandHoldFrames%d neutralLongPressDone:%d\n",neutralBandHoldFrames,neutralLongPressDone);
    if (neutralBandHoldFrames > NEUTRAL_LONG_PRESS_FRAMES && !neutralLongPressDone) {
      neutralLongPressDone = true;
      neutralBandHoldFrames = 0;
      neutralTrimeEle = Trimelevetor;
      settingsChanged = true;
      xTaskCreate(Ltika, "Ltika", 1024, NULL, 9, NULL);
    }

  } else if (3300 <= TrimE && TrimE <= 3500 && millis() - lastPushed > 250) {  //優先度4
    resetNeutralGesture();
    is_pid = !is_pid;
    lastPushed = millis();
  } else {
    neutralWasInBand = false;

    if (neutralStrokeActive) {
      neutralBandHoldFrames = 0;
      neutralOutsideStableFrames++;
      if (neutralOutsideStableFrames >= NEUTRAL_RELEASE_STABLE_FRAMES) {
        if (!neutralLongPressDone) {
          Trimelevetor = neutralTrimeEle;
          settingsChanged = true;
        }
        resetNeutralGesture();
      }
    } else {
      neutralOutsideStableFrames = 0;
    }

    if (settingsChanged && nvmTaskHandle != NULL) {
      xTaskNotifyGive(nvmTaskHandle);
      settingsChanged = false;
    }
  }
}

void trimRudder() {
  int nowTrimR1 = digitalRead(trimR1);
  int nowTrimR2 = digitalRead(trimR2);

  if (nowTrimR1 == HIGH) {
    Trimrudder = Trimrudder + 0.1;
    settingsChanged = true;
  }
  if (nowTrimR2 == HIGH) {
    Trimrudder = Trimrudder - 0.1;
    settingsChanged = true;
  }
}

float cachedTempE = 0.0;  // エレベータサーボ温度キャッシュ
float cachedTempR = 0.0;  // ラダーサーボ温度キャッシュ
int tempReadCounter = 0;

const long frequency = 30;
void mainloop(void *pvParameters) {
  const TickType_t xFrequency = (1000 / frequency) / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while (1) {

    Potentiometer();
    trimElevetor();
    trimRudder();

    float degE = elevetor + Trimelevetor;
    float degR = rudder + Trimrudder;


    float errorE = 0.0 - currentPitch;
    /*
    tempDegE は PID 出力（舵角オフセット相当）。is_center のときだけ積分・出力を更新。
    LED は PID モード ON の合図。中立外で is_pid を落とすとボタン押下と無関係に消灯するため、
    中立外は手動舵角のままモードだけ維持し、積分はリセットしてウィンドアップを防ぐ。
    */
    float tempDegE = 0.0f;
    int krsE = ele2krs(degE);
    int krsR = rud2krs(degR);

    if (is_pid) {
      digitalWrite(LED, HIGH);
      if (is_center) {
        tempDegE = (float)pidCompute(&pidElevator, errorE, currentPitchRate);
        krsE = ele2krs(tempDegE + Trimelevetor);
      } else {
        pidReset(&pidElevator);
      }
    } else {
      digitalWrite(LED, LOW);
      pidReset(&pidElevator);
    }


    int setpos0 = krs.setPos(0, krsR);  // ラダー
    int setpos1 = krs.setPos(1, krsE);  // エレベータ
    int getpos0 = krs.getPos(0);        // ラダー実位置
    int getpos1 = krs.getPos(1);        // エレベータ実位置

    // サーボ温度取得（1秒に1回）
    tempReadCounter++;
    if (tempReadCounter >= 30) {
      int rawTmpE = krs.getTmp(1);  // エレベータ
      int rawTmpR = krs.getTmp(0);  // ラダー
      if (rawTmpE != -1) cachedTempE = (float)rawTmpE;
      if (rawTmpR != -1) cachedTempR = (float)rawTmpR;
      tempReadCounter = 0;
    }

    // BLE送信
    // E_steer/R_steer: 中央値＋ローパス後の ADC をレバー歪みを [-30,30] に線形正規化
    // E_trim: エレベータのトリム角 [°]（KRS に変換する前の生の度数値を送信）
    // E_angle/R_angle: getPos の KRS から多項式の逆映射で実舵角[°]（分度器と同じ定義）
    // e_servo_temp / r_servo_temp: 近藤 ICS getTmp の戻り値。仕様上は摂氏[℃]（公式マニュアルで最終確認推奨）
    if (bleConnected && pCharacteristic != NULL) {
      ControlData txData;
      float adcE = constrain((float)g_filteredEleAdc, (float)elergs[0], (float)elergs[3]);
      float adcR = constrain((float)g_filteredRudAdc, (float)rudrgs[0], (float)rudrgs[3]);
      txData.E_steer = fmap(adcE, (float)elergs[0], (float)elergs[3], -30.f, 30.f);
      txData.R_steer = fmap(adcR, (float)rudrgs[0], (float)rudrgs[3], -30.f, 30.f);
      txData.E_trim = Trimelevetor;
      txData.E_angle = (getpos1 != -1) ? krs2ele((float)getpos1) : 0.f;
      txData.R_angle = (getpos0 != -1) ? krs2rud((float)getpos0) : 0.f;
      txData.e_servo_temp = cachedTempE;
      txData.r_servo_temp = cachedTempR;
      memset(txData.control_mode, 0, sizeof(txData.control_mode));
      strncpy(txData.control_mode, is_pid ? "assisted" : "manual", sizeof(txData.control_mode) - 1);
      pCharacteristic->setValue((uint8_t*)&txData, sizeof(ControlData));
      pCharacteristic->notify();
    }

    Serial.printf("E:%.1f R:%.1f krs:%d,%d raw:%d,%d getPos:%d,%d pitch:%.1f pid:%.1f\n", degE, degR, krsE, krsR, rawEle, rawRud, getpos0, getpos1, currentPitch, tempDegE);


    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
  vTaskDelete(NULL);
}

void setup() {

  Serial.begin(115200);
  Serial.printf("[%lu] setup: enter\n", millis());
  // 設定の読み込み
  loadSettings();

  krs.begin();
  krs.setSpd(0, 127);
  krs.setSpd(1, 127);

  pinMode(r_elevator, INPUT);
  pinMode(r_rudder, INPUT);
  pinMode(trimE, INPUT);
  pinMode(LED, OUTPUT);
  pinMode(trimR1, INPUT);
  pinMode(trimR2, INPUT);
  pinMode(resetPin, INPUT);

  pidInit(&pidElevator, -1.0, -0.1, -0.05, 30.0);  // Kd=-0.05 : ジャイロD項有効化, integralMax [°]
  pidInit(&pidRudder, 1.0, 0.1, 0.05, 30.0);

  // BLE初期化
  Serial.printf("[%lu] BLE: init begin\n", millis());
  BLEDevice::init("C3-CONTROL");
  Serial.printf("[%lu] BLE: init done\n", millis());
  BLEServer* pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService* pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
    CHAR_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pCharacteristic->addDescriptor(new BLE2902());
  pCharacteristic->setCallbacks(new MyCharCallbacks());
  pService->start();
  pServer->getAdvertising()->start();
  Serial.printf("[%lu] BLE: advertising as C3-CONTROL\n", millis());

  // 各タスクの生成
  xTaskCreate(nvmTask, "nvmTask", 4096, NULL, 2, &nvmTaskHandle);
  xTaskCreate(resetMonitorTask, "resetMonitor", 2048, NULL, 5, NULL);  // リセット監視タスクを追加
  xTaskCreate(mainloop, "mainloop", 10000, NULL, 10, NULL);
}

void loop() {}