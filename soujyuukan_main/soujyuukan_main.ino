#include <IcsBaseClass.h>
#include <IcsHardSerialClass.h>
#include <freertos/FreeRTOS.h>
#include <Preferences.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "DeadzoneResult.h"
#include "PidState.h"


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
const int trimR1 = 9;    //トリムラダー
const int trimR2 = 10;   //トリムラダー


//ADC検証モード
int is_finADC = 0;


volatile float currentPitch = 0.0f;
volatile float currentPitchRate = 0.0f;

// 通信断検知用: 最後に meinkiban3 から受信した時刻
volatile uint32_t g_lastPitchRecvMs = 0;
static const uint32_t PITCH_LINK_TIMEOUT_MS = 300;  // 300ms 無音で PID 停止


#pragma pack(push, 1)
struct ControlData {
  uint32_t magic;
  uint8_t role;
  float E_steer, R_steer;
  float E_trim, E_angle, R_angle;
  float e_servo_temp, r_servo_temp;
  bool is_assisted;
};
struct NavigationData {
  uint32_t magic;
  uint8_t role;
  float pitch;
  float pitch_rate;
};
#pragma pack(pop)
static const uint8_t WIFI_CHANNEL = 1;
static uint8_t BROADCAST_MAC[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
#define MAGIC 0x53333230u
#define ROLE_MEINKIBAN3 1
#define ROLE_SOUJYUUKAN 2
#define ROLE_LOGGER 3

void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  // meinkiban3 から来る NavigationData (pitch, pitch_rate) を受信する
  if (len != sizeof(NavigationData)) return;
  NavigationData pkt;
  memcpy(&pkt, data, sizeof(pkt));
  if (pkt.magic != MAGIC) return;
  if (pkt.role != ROLE_MEINKIBAN3) return;
  currentPitch = pkt.pitch;
  currentPitchRate = pkt.pitch_rate;
  g_lastPitchRecvMs = millis();
}


/*
ここがめっちゃ重要。詳しくは同じディレクトリにある。.docsを参照
*/

// 実舵角 x[°]（分度器）→ KRS（多項式本体・逆映射用はクランプなし）
static float ele2krs(float x) {
  x = 0.012577102329 * pow(x, 5) + 0.0194099102747 * pow(x, 4) - 0.634353764183 * pow(x, 3) - 0.815828278465 * pow(x, 2) + 185.794437174 * pow(x, 1) + 5701.22753031;
  return constrain(x,3500,8300);
}

static float rud2krs(float x) {
  x = -0.0105114837351 * pow(x, 5) - 0.0659075647903 * pow(x, 4) + 0.241297817826 * pow(x, 3) + 2.97624922026 * pow(x, 2) - 179.910899851 * pow(x, 1) + 6563.03984539;
  return constrain(x,4000,8900);
}

//KRS→ 舵角に変換する関数
float krs2ele(float x) {
  x = 2.9862304768e-18 * pow(x, 5) - 7.42363558025e-14 * pow(x, 4) + 3.95030812038e-10 * pow(x, 3) + 2.10479251629e-06 * pow(x, 2) - 0.0174092175976 * pow(x, 1) + 18.1307278407;
  return constrain(x, -5, 5);
}
float krs2rud(float x) {
  x = -1.35085986658e-17 * pow(x, 5) + 4.38562944215e-13 * pow(x, 4) - 5.31829061512e-09 * pow(x, 3) + 2.9458795701e-05 * pow(x, 2) - 0.0758402111862 * pow(x, 1) + 83.232581592;
  return constrain(x, -10, 10);
}

// 舵角上下限
float ElevatorDegMin = -5;
float ElevatorDegMed = 0;
float ElevatorDegMax = 5;
float RudderDegMin = -9.3;  //-10.1
float RudderDegMax = 9.3;

IcsHardSerialClass krs(&Serial0, EN_PIN, BAUDRATE, TIMEOUT);  //インスタンス＋ENピン(8番ピン)およびUARTの指定

// 変数の宣言 [°]
float Trimelevetor = 0.0;
float neutralTrimeEle = 0.0;
float Trimrudder = 0.0;


bool is_pid = 0;  //今PID制御をONにするかどうか(0か1)

// PidState は PidState.h で定義
PidState pidElevator;
PidState pidRudder;  //一応ラダーも用意しているが、今回はラダーに関するオートパイロットは行わない

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

double pidCompute(PidState *state, double error, double gyroRate, double dt) {

  state->integral += error * dt;
  state->integral = constrain(state->integral, -state->integralMax, state->integralMax);

  // D項: 数値微分の代わりにジャイロ生値を使用
  // error = target - pitch なので d(error)/dt = -pitch_rate = -gyroRate
  double derivative = -gyroRate;

  double output = state->kp * error + state->ki * state->integral + state->kd * derivative;

  state->lastError = error;

  return output;
}

int elergs[4] = { 1120, 1500, 2000, 2580 };  // エレベーター: 前限界, 前戻り, 後戻り, 後限界
int rudrgs[4] = { 450, 650, 730, 1430 }; //操縦桿main
//int rudrgs[4] = {1724,2310,2610,3220}; //操縦桿sub

// 設定を読み込む関数
void loadSettings() {
  preferences.begin("trim-data", true);
  Trimelevetor = preferences.getFloat("trimE", 0.0);
  neutralTrimeEle = preferences.getFloat("neutE", 0.0);
  Trimrudder = preferences.getFloat("trimR", 0.0);
  for (int i = 0; i < 4; i++) {
    char k[8];
    snprintf(k, sizeof(k), "ele%d", i);
    elergs[i] = preferences.getInt(k, elergs[i]);
    snprintf(k, sizeof(k), "rud%d", i);
    rudrgs[i] = preferences.getInt(k, rudrgs[i]);
  }
  preferences.end();
  Serial.printf("Settings Loaded: E=%.2f, neutE=%.2f, R=%.2f\n", Trimelevetor, neutralTrimeEle, Trimrudder);
  Serial.printf("elergs={%d,%d,%d,%d} rudrgs={%d,%d,%d,%d}\n",
                elergs[0], elergs[1], elergs[2], elergs[3],
                rudrgs[0], rudrgs[1], rudrgs[2], rudrgs[3]);
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
    for (int i = 0; i < 4; i++) {
      char k[8];
      snprintf(k, sizeof(k), "ele%d", i);
      preferences.putInt(k, elergs[i]);
      snprintf(k, sizeof(k), "rud%d", i);
      preferences.putInt(k, rudrgs[i]);
    }
    preferences.end();
    Serial.println(">> Settings saved to NVM");
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
void Ltika2(void *pvParameters) {
  while (1) {
    digitalWrite(LED, HIGH);
    vTaskDelay(300 / portTICK_PERIOD_MS);
    digitalWrite(LED, LOW);
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}


// floatのmap関数
float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

DeadzoneResult detzoneMapping(int *ary, int x, float min, float med, float max) {
  DeadzoneResult r;
  r.isCenter = false;

  if (x <= ary[0]) {
    r.mappedValue = min;
  } else if (ary[0] < x && x <= ary[1]) {
    r.mappedValue = fmap(x, ary[0], ary[1], min, med);
  } else if (ary[1] < x && x <= ary[2]) {
    r.mappedValue = med;
    r.isCenter = 1;
  } else if (ary[2] < x && x <= ary[3]) {
    r.mappedValue = fmap(x, ary[2], ary[3], med, max);
  } else {
    r.mappedValue = max;
  }
  return r;
}

float elevetor = 0.0;  // [°]
float rudder = 0.0;    // [°]
int rawEle = 0;
int rawRud = 0;
int ELE;
int RUD;

int is_center = 0;
void Potentiometer() {

  rawEle = analogRead(r_elevator);
  rawRud = analogRead(r_rudder);

  DeadzoneResult eler = detzoneMapping(elergs, rawEle, ElevatorDegMax, ElevatorDegMed, ElevatorDegMin);
  elevetor = eler.mappedValue;
  is_center = eler.isCenter;
  DeadzoneResult rudr = detzoneMapping(rudrgs, rawRud, RudderDegMax, 0, RudderDegMin);
  rudder = rudr.mappedValue;
}

unsigned long lastPushed = millis();  //ボタン4チャタリング防止用

// ニュートラル帯(2000..2300): 長押しはイン帯フレーム数、単押しは帯外が安定してから確定(@30Hz想定)
static const int NEUTRAL_LONG_PRESS_FRAMES = 30;     
static const int NEUTRAL_RELEASE_STABLE_FRAMES = 4;  
static bool neutralStrokeActive = false;
static bool neutralLongPressDone = false;
static int neutralBandHoldFrames = 0;
static int neutralOutsideStableFrames = 0;
static bool neutralWasInBand = false;

//pid制御
bool pidWasInBand = false;
static const int PID_LONG_PRESS_FRAMES = 30;
int pidBandHoldFrames = 0;


static void resetNeutralGesture() {
  neutralStrokeActive = false;
  neutralLongPressDone = false;
  neutralBandHoldFrames = 0;
  neutralOutsideStableFrames = 0;
  neutralWasInBand = false;
}

int TrimE_temp;
void trimElevetor() {
  int TrimE = analogRead(trimE);
  TrimE_temp = TrimE;

  if (0 <= TrimE && TrimE <= 100) {  //優先度1
    resetNeutralGesture();
    Trimelevetor = constrain(Trimelevetor - 0.1,ElevatorDegMin+2,ElevatorDegMax-2);
    settingsChanged = true;

  } else if (665 <= TrimE && TrimE <= 1675) {  // 優先度2
    resetNeutralGesture();
    Trimelevetor = constrain(Trimelevetor + 0.1,ElevatorDegMin+2,ElevatorDegMax-2);
    settingsChanged = true;

  } else if (1675 <= TrimE && TrimE <= 2820) {// 優先度3
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

  } else if (2820 <= TrimE && TrimE <= 3995) {  //優先度4

    if(is_pid && !pidWasInBand){
      is_pid = false;
    }

    if (!pidWasInBand) {
      pidBandHoldFrames = 1;
    } else {
      pidBandHoldFrames++;
    }
    pidWasInBand = true;

    Serial.printf("%d ",pidBandHoldFrames);

    if(pidBandHoldFrames >= 10){
      is_pid = true;
    }

  } else {
    pidWasInBand = false;
    pidWasInBand = false;
    pidBandHoldFrames = 0;



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

bool trimRudder() {
  int nowTrimR1 = digitalRead(trimR1);
  int nowTrimR2 = digitalRead(trimR2);
  
  if(nowTrimR1 == HIGH && nowTrimR2 == HIGH){
    return true;
  }

  if (nowTrimR1 == HIGH) {
    Trimrudder = Trimrudder - 0.1;
    settingsChanged = true;
  }
  if (nowTrimR2 == HIGH) {
    Trimrudder = Trimrudder + 0.1;
    settingsChanged = true;
  }

  Trimrudder = constrain(Trimrudder,RudderDegMin+2,RudderDegMax-2);
  if (settingsChanged && nvmTaskHandle != NULL) {
    xTaskNotifyGive(nvmTaskHandle);
    settingsChanged = false;
  }
  return false;
}

void AdcId(){
  // min/max は入室直後から連続スキャンで自動更新
  int maxEle = analogRead(r_elevator);
  int minEle = maxEle;
  int maxRud = analogRead(r_rudder);
  int minRud = maxRud;

  // 優先度1/2の押下スナップショット (各2個まで)
  int eleSnap[2] = { 0, 0 };
  int rudSnap[2] = { 0, 0 };
  int eleCount = 0;
  int rudCount = 0;
  bool prio1Prev = false;  // 立ち上がりエッジ検出用
  bool prio2Prev = false;

  TaskHandle_t ltikaHandle = NULL;
  xTaskCreate(Ltika2, "Ltika2", 1024, NULL, 9, &ltikaHandle);

  const TickType_t xFrequency = (1000 / 30) / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while(1){
    rawEle = analogRead(r_elevator);
    rawRud = analogRead(r_rudder);
    if(maxEle < rawEle) maxEle = rawEle;
    if(minEle > rawEle) minEle = rawEle;
    if(maxRud < rawRud) maxRud = rawRud;
    if(minRud > rawRud) minRud = rawRud;

    int trimEVal = analogRead(trimE);
    bool prio1Now = (0    <= trimEVal && trimEVal <= 100);   // ELE保存
    bool prio2Now = (665  <= trimEVal && trimEVal <= 1675);  // RUD保存
    bool prio4Now = (2820 <= trimEVal && trimEVal <= 3995);  // 脱出

    // 立ち上がりエッジでスナップショット保存。3回目以降は無視
    if(prio1Now && !prio1Prev && eleCount < 2){
      eleSnap[eleCount++] = rawEle;
    }
    if(prio2Now && !prio2Prev && rudCount < 2){
      rudSnap[rudCount++] = rawRud;
    }
    prio1Prev = prio1Now;
    prio2Prev = prio2Now;

    Serial.printf("ADCID ele[%d/2]=%d,%d rud[%d/2]=%d,%d minE%d maxE%d minR%d maxR%d rawE%d rawR%d\n",
                  eleCount, eleSnap[0], eleSnap[1],
                  rudCount, rudSnap[0], rudSnap[1],
                  minEle, maxEle, minRud, maxRud, rawEle, rawRud);

    // 脱出: 優先度4 押下 かつ ELE/RUD ともに2個保存済み
    if(prio4Now && eleCount == 2 && rudCount == 2){
      break;
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }

  // 大小判別して格納: 小=前戻り(elergs[1]) / 大=後戻り(elergs[2])
  elergs[0] = minEle;
  elergs[3] = maxEle;
  elergs[1] = (eleSnap[0] < eleSnap[1]) ? eleSnap[0] : eleSnap[1];
  elergs[2] = (eleSnap[0] < eleSnap[1]) ? eleSnap[1] : eleSnap[0];

  rudrgs[0] = minRud;
  rudrgs[3] = maxRud;
  rudrgs[1] = (rudSnap[0] < rudSnap[1]) ? rudSnap[0] : rudSnap[1];
  rudrgs[2] = (rudSnap[0] < rudSnap[1]) ? rudSnap[1] : rudSnap[0];

  // NVMへ非同期保存
  if(nvmTaskHandle != NULL){
    xTaskNotifyGive(nvmTaskHandle);
  }

  vTaskDelete(ltikaHandle);
  ltikaHandle = NULL;
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
    bool presd = trimRudder();

    if(presd && 1675 <= TrimE_temp && TrimE_temp <= 2820){
      AdcId();
    }

    float degE = elevetor + Trimelevetor;
    float degR = rudder + Trimrudder;


    float errorE = 0.0 - currentPitch;
    /*
    tempDegE は PID 出力（舵角オフセット相当）。is_center のときだけ積分・出力を更新。
    LED は PID モード ON の合図。中立外で is_pid を落とすとボタン押下と無関係に消灯するため、
    中立外は手動舵角のままモードだけ維持し、積分はリセットしてウィンドアップを防ぐ。
    */

    int krsE = ele2krs(degE);
    int krsR = rud2krs(degR);


    unsigned long now = millis();
    double dt = (now - pidElevator.lastTime) / 1000.0;
    if (dt <= 0) dt = 0.001;
    float tempDegE = pidCompute(&pidElevator, errorE, currentPitchRate,dt);
    pidElevator.lastTime = now;

    // 通信断フェイルセーフ: 300ms pitch を受信していなければ PID を使わない
    bool pitchLinkOk = (millis() - g_lastPitchRecvMs < PITCH_LINK_TIMEOUT_MS);

    if (is_pid && pitchLinkOk) {
      digitalWrite(LED, HIGH);
      if (is_center) {
        krsE = ele2krs(tempDegE + Trimelevetor);
      } else {
        is_pid = 0;
        pidReset(&pidElevator);
      }
    } else {
      // PID OFF か通信断時は手動入力のまま、PID はリセット
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

    ControlData nv;
    nv.magic = MAGIC;
    nv.role = ROLE_SOUJYUUKAN;
    nv.E_steer = (float)rawEle;
    nv.R_steer = (float)rawRud;
    nv.E_trim = Trimelevetor;
    nv.E_angle = (getpos1 != -1) ? krs2ele((float)getpos1) : -1;
    nv.R_angle = (getpos0 != -1) ? krs2rud((float)getpos0) : -1;
    nv.e_servo_temp = cachedTempE;
    nv.r_servo_temp = cachedTempR;
    nv.is_assisted = (is_pid && pitchLinkOk && is_center);

    esp_now_send(BROADCAST_MAC, (uint8_t *)&nv, sizeof(nv));

    Serial.printf("%d %d %d %d ::",rudrgs[0],rudrgs[1],rudrgs[2] ,rudrgs[3]);

    Serial.printf("E:%.1f R:%.1f krs:%d,%d raw:%d,%d getPos:%d,%d pitch:%.1f pid:%.1f temp:%d\n", degE, degR, krsE, krsR, rawEle, rawRud, getpos0, getpos1, currentPitch, tempDegE,TrimE_temp);

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

  pinMode(r_elevator, INPUT);
  pinMode(r_rudder, INPUT);
  pinMode(trimE, INPUT);
  pinMode(LED, OUTPUT);
  pinMode(trimR1, INPUT);
  pinMode(trimR2, INPUT);

  pidInit(&pidElevator, -1.0, -0.1, -0.05, 30.0);  // Kd=-0.05 : ジャイロD項有効化, integralMax [°]
  pidInit(&pidRudder, 1.0, 0.1, 0.05, 30.0);

  // ESP-NOW 初期化
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  Serial.print("自分の MAC: ");
  Serial.println(WiFi.macAddress());
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("[error] esp_now_init 失敗");
    while (1) delay(1000);
  }

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BROADCAST_MAC, 6);
  peer.channel = WIFI_CHANNEL;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("[error] add_peer 失敗");
    while (1) delay(1000);
  }

  esp_now_register_recv_cb(onRecv);
  Serial.println("ESP-NOW 初期化完了");

  // 各タスクの生成
  xTaskCreate(nvmTask, "nvmTask", 4096, NULL, 2, &nvmTaskHandle);
  xTaskCreate(mainloop, "mainloop", 10000, NULL, 10, NULL);
}

void loop() {}
