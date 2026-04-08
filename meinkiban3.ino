
#include <Wire.h>
//#include <BLEDevice.h>
//#include <BLEServer.h>
//#include <BLE2902.h>
#include <NimBLEDevice.h>
#include <Adafruit_MCP23X17.h>
#include <ICM20948_WE.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <FS.h>           // 追加：WebServerより先に読み込む
#include <LittleFS.h>     // 追加
#include <WiFi.h>
#include <time.h>
#include <sys/time.h>

#include "TFTand9axis_sensor.h"

using namespace fs;       // 名前空間を明示

Adafruit_MCP23X17 mcp;
TFTand9axis_sensor instrumentPanel;

#include <ArduinoJson.h>

// --- Include Custom Libraries ---
#include "TBT_GNSS.h"
#include "TBT_AndroidSerial.h"

/* ===== ピン・アドレス定義 ===== */
#define I2C0_SDA 21
#define I2C0_SCL 22
#define I2C1_SDA 25
#define I2C1_SCL 26
#define tgrsw_PIN 32
#define PHOTO1_PIN 34
#define PHOTO2_PIN 33
#define tktsw_PIN 35
#define ADDR_MB1242 0x70
#define SDP810_ADDR 0x25
#define LED1 2
#define LED2 0
#define LED3 1

// --- Web Controlled Data (Global Variables) ---  
double E_steer = 0;
double R_steer = 0;
double E_trim = 0;
double E_angle = 0;
double R_angle = 0;
double e_servo_temp = 0.0;
double r_servo_temp = 0.0;
String control_mode = "manual"; // "manual" or "assisted"
String electrical_errors = "[]"; 

// 1. 【UUID定義】（関数の外、一番上に書く）
#define serviceUUID  "AAAA0001-1fb5-459e-8fcc-c5c9c331914b"
#define charUUID "AAAA0002-36e1-4688-b7f5-ea07361b26a8"
// ★追加1：BLE通信用の構造体と変数
// ==========================================
#pragma pack(push, 1)
struct ControlData {
  float E_steer, R_steer;
  float E_trim, E_angle, R_angle;
  float e_servo_temp, r_servo_temp;
  char control_mode[12];
};
struct NavigationData {
  float pitch;
};
struct FullTelemetryPacket {
  ControlData ctrl;
  NavigationData nav;
  float front_rpm, rear_rpm;
  float air_speed, gnd_speed ,Altitude, heading;
  double lat, lon;
  float roll;
  uint32_t epoch_time;
  bool electrical_errors[12];
};
#pragma pack(pop)

// NimBLE仕様に変更
NimBLEAddress* addrLogger = nullptr;
NimBLEAddress* addrControl = nullptr;

NimBLEClient* pClientLogger = nullptr;
NimBLEClient* pClientControl = nullptr;

NimBLERemoteCharacteristic* pCharLogger = nullptr;
NimBLERemoteCharacteristic* pCharControl = nullptr;

bool connectedLogger = false;
bool connectedControl = false;

// --- 受信コールバック（操縦用C3からデータが届いた時） ---
// NimBLE仕様に変更
static void controlNotifyCallback(
  NimBLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
    if (length == sizeof(ControlData)) {
        ControlData* incoming = (ControlData*)pData;
        E_steer = incoming->E_steer;
        R_steer = incoming->R_steer;
        E_trim = incoming->E_trim;
        E_angle = incoming->E_angle;
        R_angle = incoming->R_angle;
        e_servo_temp = incoming->e_servo_temp;
        r_servo_temp = incoming->r_servo_temp;
        control_mode = String(incoming->control_mode);
    }
}

// ==========================================
// ★WROOM側の切断検知コールバック (NimBLE v2仕様に修正)
// ==========================================
class LoggerClientCallback : public NimBLEClientCallbacks {
  void onDisconnect(NimBLEClient* pclient, int reason) override { // reasonを追加
    connectedLogger = false;
    Serial.println("!!! Logger C3 connection lost (Auto Reset) !!!");
  }
};

class ControlClientCallback : public NimBLEClientCallbacks {
  void onDisconnect(NimBLEClient* pclient, int reason) override { // reasonを追加
    connectedControl = false;
    Serial.println("!!! Control C3 connection lost (Auto Reset) !!!");
  }
};

// ==========================================
// --- スキャンコールバック (NimBLE仕様に修正) ---
class MyAdvertisedDeviceCallbacks: public NimBLEScanCallbacks { // クラス名を変更
    // 引数に const を追加
    void onResult(const NimBLEAdvertisedDevice* advertisedDevice) override {
        String name = advertisedDevice->getName().c_str();
        if (name == "C3-LOGGER") {
            if (addrLogger != nullptr) delete addrLogger;
            addrLogger = new NimBLEAddress(advertisedDevice->getAddress());
            Serial.println("Found Logger C3");
        } 
        else if (name == "C3-CONTROL") {
            if (addrControl != nullptr) delete addrControl;
            addrControl = new NimBLEAddress(advertisedDevice->getAddress());
            Serial.println("Found Control C3");
        }
    }
};
/* ===== グローバル変数 ===== */
unsigned long previousMillis = 0; // 前回の更新時間を保存
const long interval = 120;      // 更新間隔（ミリ秒）
double pitch = 0.0;
double roll  = 0.0;
double heading = 0.0;
double Altitude = 0.0;
double air_speed = 0.0;
double gnd_speed = 0.0;
double front_rpm = 0.0;
double rear_rpm = 0.0;
int photo1;
int photo2;
int tktsw;
int tgrsw;
double pitch_rad;
double roll_rad;
double lat;
double lon;
double alt;
double GNSS_heading;
int fixType;
int16_t rawPressure;
double AIR_DENSITY;
bool waitingForUltra = false;    // 超音波センサーの計測待ちフラグ
unsigned long lastUltraTime = 0; // 最後に計測した時間を記録する変数
TaskHandle_t displayTaskHandle; // タスクを管理するためのハンドル
uint32_t epoch_time;
int year;
int month;
int day;
int hour;
int minute;
int second; 
double offset_roll = 0.0;
double offset_pitch = 0.0;
double slits = 16.0;
// 前部(Photo1)用の変数
volatile int pulseCount1 = 0;
volatile unsigned long lastTime1 = 0;
volatile unsigned long interval1 = 0;

// 後部(Photo2)用の変数
volatile int pulseCount2 = 0;
volatile unsigned long lastTime2 = 0;
volatile unsigned long interval2 = 0;

double p_rpm;
double avg_rpm;

// --- Custom Library Instances ---
TBT_GNSS mygnss;
TBT_AndroidSerial myAndroid;

// ================================================================
// 2. SETUP & LOOP
// ================================================================

/* ===== 関数プロトタイプ ===== */
void clearI2CBus(int sdaPin, int sclPin);
void readSiseikaku();
void startAltimeter();
void readAltimeter();
bool startMeasurement();
void setGPS();
void loopGPS();
void readkisoku();
void MCP23017_LED();
void sendAndoroid();
void confirmICM();
void IRAM_ATTR isrPhoto1();
void IRAM_ATTR isrPhoto2();
void calcRPM();
void bleScanTask(void *pvParameters);
void bleControlTask(void *pvParameters);
void bleLoggerTask(void *pvParameters);
bool mcp_active = false; // MCPが正しく認識されたか管理するフラグ
bool ultra_active = false;
bool gps_active = false;
bool sdp_active = false;
bool imu_active = false;

void setup() {
  delay(500);

  clearI2CBus(I2C0_SDA, I2C0_SCL);
  clearI2CBus(I2C1_SDA, I2C1_SCL);

  Serial.begin(115200);
  /* --- I2C0 : IMU 超音波センサ,9軸センサー --- */
  Wire.begin(I2C0_SDA, I2C0_SCL);
  Wire.setTimeOut(50);
  Wire.setClock(100000);

  /* --- I2C1 : 差圧センサ --- */
  Wire1.begin(I2C1_SDA, I2C1_SCL);
  Wire1.setTimeOut(50);
  Wire1.setClock(100000);

  delay(100);

  setGPS();

  pinMode(PHOTO1_PIN, INPUT);
  pinMode(PHOTO2_PIN,INPUT);
  pinMode(tktsw_PIN,INPUT);
  pinMode(tgrsw_PIN,INPUT);
  gpio_pullup_dis((gpio_num_t)I2C0_SDA);
  gpio_pullup_dis((gpio_num_t)I2C0_SCL);
  gpio_pullup_dis((gpio_num_t)I2C1_SDA);
  gpio_pullup_dis((gpio_num_t)I2C1_SCL);

  // ★追加：ピンの電圧が「HIGHからLOWに落ちた瞬間(FALLING)」にカウント関数を呼ぶよう設定
  attachInterrupt(digitalPinToInterrupt(PHOTO1_PIN), isrPhoto1, FALLING);
  attachInterrupt(digitalPinToInterrupt(PHOTO2_PIN), isrPhoto2, FALLING);

  instrumentPanel.begin();

  if (mcp.begin_I2C(0x20, &Wire1)) {
    Serial.println("MCP23017 Init OK");
    mcp.pinMode(LED1, OUTPUT);
    mcp.pinMode(LED2, OUTPUT);
    mcp.pinMode(LED3, OUTPUT);
    mcp_active = true;
  } else {
    Serial.println("MCP23017 Init Failed");
    mcp_active = false;
  }

  /* --- 超音波 初回測定 --- */
  startAltimeter();
  startMeasurement(); 
  delay(20);

  // NimBLEの初期設定
  NimBLEDevice::init("WROOM-MASTER");
  NimBLEScan* pScan = NimBLEDevice::getScan();
  
  // ★ここを setScanCallbacks に変更
  pScan->setScanCallbacks(new MyAdvertisedDeviceCallbacks());
  
  pScan->setActiveScan(true);
  
  xTaskCreatePinnedToCore(bleScanTask,    "BLE_Scan",    4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(bleControlTask, "BLE_Control", 8192, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(bleLoggerTask,  "BLE_Logger",  8192, NULL, 1, NULL, 1);
}

void loop() {
  photo1 = digitalRead(PHOTO1_PIN);
  photo2 = digitalRead(PHOTO2_PIN);
  tktsw = digitalRead(tktsw_PIN);

  //if(tktsw){
  //  instrumentPanel.offsets();
  //}

  if (millis() - lastUltraTime >= interval) {
    if (waitingForUltra) {
      readAltimeter(); // 前回スタートに成功していれば、値を読み取る
      if(Altitude < 765){
        Altitude = Altitude / 100.0;
      }else{
        Altitude = alt;
      }
    }
    
    startAltimeter(); // 失敗していても、必ず次の測定開始合図を送って再挑戦する
    lastUltraTime = millis(); // タイマーをリセット
  }

  static unsigned long lastPrint1 = 0;
  static unsigned long lastPrint2 = 50;
  instrumentPanel.getPitchAndRoll(&pitch_rad, &roll_rad);
  //heading = instrumentPanel.getHeading(pitch_rad, roll_rad);
  instrumentPanel.updata(E_trim, air_speed, front_rpm, Altitude);
  pitch = pitch_rad * (180.0 / PI);
  roll = roll_rad * (180.0 / PI);
  
  if (millis() - lastPrint1 >= interval) {
    tgrsw = digitalRead(tgrsw_PIN);
    readkisoku();
    confirmICM();
    calcRPM();

    MCP23017_LED();
    lastPrint1 = millis();
  }

  if (millis() - lastPrint2 >= interval) {
    loopGPS();
    sendAndoroid();
    //Serial.printf(
     // "Pitch:%6.1f deg | Roll:%6.1f deg | Alt:%4d cm | Photo1:%d | Photo2:%d | air_speed：%.3f | swich: %d\n",
     // pitch, roll, distance, photo1, photo2, air_speed, tktsw
    //);
    lastPrint2 = millis();
  }
}

/* ===== 関数定義 ===== */

void MCP23017_LED(){
  Wire1.beginTransmission(0x20);
  if (Wire1.endTransmission() == 0) {
      // 応答あり
    if (!mcp_active) {
      Serial.println("MCP23017 Recovered! Re-initializing pins...");
      // begin_I2Cを呼ばずに、直接ピン設定だけをやり直す
      mcp.pinMode(LED1, OUTPUT);
      mcp.pinMode(LED2, OUTPUT);
      mcp.pinMode(LED3, OUTPUT);
        
      // 確実に消灯状態からスタートさせる
      mcp.digitalWrite(LED1, LOW);
      mcp.digitalWrite(LED2, LOW);
      mcp.digitalWrite(LED3, LOW);
    }
    mcp_active = true;
      
  } else {
    // 応答なし（エラー状態）
    if (mcp_active) {
      Serial.println("MCP23017 Connection Lost!");
    }
    mcp_active = false; 
  }

  if (mcp_active) {
    // 正常に生きている時だけLEDを操作
    if (p_rpm <= 5){
      mcp.digitalWrite(LED1, LOW);
      mcp.digitalWrite(LED2, LOW);   
      mcp.digitalWrite(LED3, LOW);
    }
    else if (5< p_rpm && p_rpm <= 20){
      mcp.digitalWrite(LED1, HIGH);
      mcp.digitalWrite(LED2, LOW);   
      mcp.digitalWrite(LED3, LOW);
    }
    else if (20< p_rpm && p_rpm <= 60){
      mcp.digitalWrite(LED1, HIGH);
      mcp.digitalWrite(LED2, HIGH);   
      mcp.digitalWrite(LED3, LOW);
    }
    else if (60< p_rpm && p_rpm <= 80){
      mcp.digitalWrite(LED1, LOW);
      mcp.digitalWrite(LED2, HIGH);  
      mcp.digitalWrite(LED3, LOW);
    } 
    else if (80< p_rpm && p_rpm <= 100){
      mcp.digitalWrite(LED1, LOW);
      mcp.digitalWrite(LED2, HIGH);  
      mcp.digitalWrite(LED3, HIGH);
    }   
    else if(100< p_rpm && p_rpm <= 120){
      mcp.digitalWrite(LED1, LOW);
      mcp.digitalWrite(LED2, LOW);  
      mcp.digitalWrite(LED3, HIGH);
    }else if(120< p_rpm && p_rpm <= 140){
      mcp.digitalWrite(LED1, HIGH);
      mcp.digitalWrite(LED2, LOW);
      mcp.digitalWrite(LED3, HIGH);     
    }else{
      mcp.digitalWrite(LED1, HIGH);
      mcp.digitalWrite(LED2, HIGH);
      mcp.digitalWrite(LED3, HIGH);     
    }
  }
}

void startAltimeter() {
  Wire.beginTransmission(ADDR_MB1242);
  Wire.write(0x51);  // 測定開始コマンド

  if (Wire.endTransmission() == 0) {
    lastUltraTime = millis();
    waitingForUltra = true;
    ultra_active = true;
  } else {
    waitingForUltra = false;
    ultra_active = false;
  }
}

void readAltimeter() {
  Wire.requestFrom(ADDR_MB1242, 2);

  if (Wire.available() >= 2) {
    // 読み取り成功
    byte high = Wire.read();
    byte low  = Wire.read();
    Altitude = (double)((high << 8) | low);
    ultra_active = true;
  } else {
    // 読み取り失敗！ -> 即座にバスクリアと再初期化を行う
    Serial.println("I2C0 Error: Resetting Bus...");
    ultra_active = false;
    Wire.end(); // ペリフェラルを一旦停止
    
    // SCLピンを直接操作してバスのロックを解除
    clearI2CBus(I2C0_SDA, I2C0_SCL); 
    
    // I2Cを再起動
    Wire.begin(I2C0_SDA, I2C0_SCL);
    Wire.setTimeOut(50);
    Wire.setClock(100000);
    
    // IMUもI2C0にいるため、再初期化が必要になる場合があります
    // （IMUの内部レジスタ設定が飛んでいない限りは通信可能です）
    // imu.init(); // 必要に応じてコメントアウトを外す
    
    waitingForUltra = false; // フラグをリセットして次のループでstartAltimeterからやり直す
  }
}

bool startMeasurement() {
    Wire1.beginTransmission(SDP810_ADDR);
    Wire1.write(0x36);
    Wire1.write(0x15); // 平均化ありの連続測定
    byte error = Wire1.endTransmission();
    if (error == 0) {
        return true;
    } else {
        Serial.print("SDP810 Start Fail! Error: ");
        Serial.println(error);
        return false;
    }

}

void readkisoku() {
  int16_t dp_raw;
  int16_t temp_raw;
  float tempreature;
  float pressurePa;

  byte count = Wire1.requestFrom(SDP810_ADDR, 6);

  if (count == 6) {
    sdp_active = true;
    
    // (中略：既存の風速計算のコードはそのまま)

  } else {
    sdp_active = false;
    air_speed = 0.0;
    
    // ==========================================
    // ★賢いエラー判定（巻き添えリセット防止システム）
    // ==========================================
    if (mcp_active == true) {
      // パターンA: MCPが生きているなら、I2Cバス全体は正常！
      // 単にSDP810が接触不良なので、バスは破壊せずに「測定開始」だけもう一度試す
      startMeasurement();
      
    } else {
      // パターンB: MCPも死んでいるなら、I2Cバスが完全にフリーズしている！
      // ここで初めて、バスの強制リセットと全員の再起動を行う
      Serial.println("I2C1 Error: Resetting Bus...");
      Wire1.end();
      clearI2CBus(I2C1_SDA, I2C1_SCL);
      Wire1.begin(I2C1_SDA, I2C1_SCL);
      Wire1.setTimeOut(50);
      Wire1.setClock(100000);
          
      if (mcp.begin_I2C(0x20, &Wire1)) {
        // I2C通信が再開できたら、ピンの出力設定もやり直す
        mcp.pinMode(LED1, OUTPUT);
        mcp.pinMode(LED2, OUTPUT);
        mcp.pinMode(LED3, OUTPUT);
        mcp_active = true;
        Serial.println("MCP23017 Recovered!");
      } else {
        mcp_active = false;
        Serial.println("MCP23017 Recovery Failed.");
      }
      startMeasurement();
    }
  }
}

// I2Cバスのフリーズを強制解除する関数
void clearI2CBus(int sdaPin, int sclPin) {
  pinMode(sdaPin, INPUT);
  pinMode(sclPin, INPUT);
  gpio_pullup_dis((gpio_num_t)sdaPin);
  gpio_pullup_dis((gpio_num_t)sclPin);

  // もしSDAがLOWに張り付いていたら、スレーブがハングしている
  if (digitalRead(sdaPin) == LOW) {
    pinMode(sclPin, OUTPUT);
    // スレーブがSDAを離すまで、ダミークロックを最大9回送る
    for (int i = 0; i < 9; i++) {
      digitalWrite(sclPin, LOW);
      delayMicroseconds(5);
      digitalWrite(sclPin, HIGH);
      delayMicroseconds(5);
      // SDAがHIGH（解放）に戻ったらループを抜ける
      if (digitalRead(sdaPin) == HIGH) {
        break;
      }
    }
  }

  // 強制的にSTOPコンディションを作り、バスを初期状態に戻す
  pinMode(sdaPin, OUTPUT);
  digitalWrite(sdaPin, LOW);
  delayMicroseconds(5);
  pinMode(sclPin, INPUT); // SCLを先にHIGHへ
  delayMicroseconds(5);
  pinMode(sdaPin, INPUT); // その後SDAをHIGHへ
  gpio_pullup_dis((gpio_num_t)sdaPin);
  gpio_pullup_dis((gpio_num_t)sclPin);
}

void setGPS(){
  // Attach AndroidSerial to main Serial (USB)
  myAndroid.attach(115200);
  // Attach GNSS (Baud, RX, TX, UART#)
  mygnss.attach(921600, 16, 17, 1);

}

void loopGPS(){
  lat = mygnss.get(GNSS_LATITUDE);
  lon = mygnss.get(GNSS_LONGITUDE);
  alt = mygnss.get(GNSS_ALTITUDE);
  GNSS_heading = mygnss.get(GNSS_HEADING);
  gnd_speed = mygnss.get(GNSS_SPEED);
  year = mygnss.get(GNSS_YEAR);
  month = mygnss.get(GNSS_MONTH);
  day = mygnss.get(GNSS_DAY);
  hour= mygnss.get(GNSS_HOUR);
  minute = mygnss.get(GNSS_MINUTE);
  second = mygnss.get(GNSS_SECOND);
  struct tm timeinfo;
  if(0 < alt || alt <7.65){
    alt = 7.65;
  }else if(alt < 0){
    alt = 0;
  }
  // ③ 箱に数字を流し込む（※年と月に「C言語特有の罠」があるので注意！）
  timeinfo.tm_year = year - 1900; // 年は「1900」を引くルール
  timeinfo.tm_mon  = month - 1;   // 月は「1」を引くルール（1月=0、12月=11）
  timeinfo.tm_mday = day;
  timeinfo.tm_hour = hour + 9;
  timeinfo.tm_min  = minute;
  timeinfo.tm_sec  = second;
  epoch_time = mktime(&timeinfo);

  fixType = (int)mygnss.get(GNSS_FIX_TYPE);
}

void sendAndoroid(){
  myAndroid.resetData();

  // Web Data - Steering and Control
  myAndroid.add("E_steer", E_steer);
  myAndroid.add("R_steer", R_steer);
  myAndroid.add("E_trim", E_trim);
  myAndroid.add("E_angle", E_angle);
  myAndroid.add("R_angle", R_angle);
  myAndroid.add("e_servo_temp", e_servo_temp);
  myAndroid.add("r_servo_temp", r_servo_temp);

  
  // Web Data - Flight Parameters
  myAndroid.add("air_speed", air_speed);
  myAndroid.add("gnd_speed" , gnd_speed);
  myAndroid.add("pitch", pitch);
  myAndroid.add("roll", roll);
  myAndroid.add("front_rpm", front_rpm);
  myAndroid.add("rear_rpm", rear_rpm);
  myAndroid.add("p_rpm", p_rpm);
  myAndroid.add("control_mode", control_mode);

  // Send Location using dedicated method (creates nested object)
  myAndroid.addLocation(lat, lon);
  myAndroid.add("altitude",Altitude);
  myAndroid.add("heading", (int)heading);
  myAndroid.add("GNSS_heading", (int)GNSS_heading);

  // Accuracy logic
  if (fixType >= 3) { // 3D or better
    myAndroid.add("gps_accuracy", 0.5);
  } else if (fixType == 2) { // 2D
    myAndroid.add("gps_accuracy", 5.0);
  } else {
    myAndroid.add("gps_accuracy", 99.9);
  }

  // Electrical Errors Logic
  StaticJsonDocument<512> errorDoc;
  JsonArray errors = errorDoc.to<JsonArray>();

  // エラーがあれば番号を追加！
  if (fixType < 3)     errors.add(200); // GPSエラー
  if (!mcp_active)     errors.add(400); // MCP23017エラー
  if (!ultra_active)   errors.add(401); // 高度計エラー
  if (!sdp_active)     errors.add(402); //ピトー管エラー
  if (!imu_active)     errors.add(403); // 9軸センサーエラー
  if(!connectedControl) errors.add(500); //操縦桿との通信エラー
  if(!connectedLogger) errors.add(501); //ロガーとの通信エラー
  for (JsonVariant e : errors) {
    myAndroid.addError(e.as<int>());
  }
  
  // Send!
  myAndroid.sendData();

}

// ★前部の割り込み関数
void IRAM_ATTR isrPhoto1() {
  unsigned long now = micros();
  unsigned long diff = now - lastTime1;
  
  // 5000マイクロ秒(5ミリ秒)未満の超高速な反応は「光の反射ノイズ」として無視！
  if (diff > 5000) {
    if (interval1 == 0) {
      interval1 = diff; // 初回はそのまま記録
    } else {
      interval1 = (interval1 + diff) / 2; // 過去のデータと平均をとってブレを吸収！
    }
    lastTime1 = now;
  }
}

// ★後部の割り込み関数
void IRAM_ATTR isrPhoto2() {
  unsigned long now = micros();
  unsigned long diff = now - lastTime2;
  
  if (diff > 5000) {
    if (interval2 == 0) {
      interval2 = diff;
    } else {
      interval2 = (interval2 + diff) / 2;
    }
    lastTime2 = now;
  }
}

void calcRPM(){
  noInterrupts(); // データを安全に取り出すため一瞬だけ割り込みをストップ
  unsigned long current_interval1 = interval1;
  unsigned long current_lastTime1 = lastTime1;
  unsigned long current_interval2 = interval2;
  unsigned long current_lastTime2 = lastTime2;
  interrupts();   // すぐに再開

  // 安全対策：0.7秒以上パルスが来ていなければ「回転が止まった」とみなす
  unsigned long currentMicros = micros();
    
  // 【前部の計算】
  // 0.7秒経過、またはまだ1度も計測されていない場合は 0 にする
  if (currentMicros - current_lastTime1 > 700000 || current_lastTime1 == 0) {
    front_rpm = 0;
    interval1 = 0; // 完全に止まったら間隔もリセット
  } else if (current_interval1 > 0) {
    // (1スリットの間隔) × スリット総数 ＝ 1回転にかかる時間
    double time_per_rev1 = current_interval1 * slits;
    front_rpm = (int)(60000000.0 / time_per_rev1); // 1分(6000万us) ÷ 1回転の時間
  }

  // 【後部の計算】
  if (currentMicros - current_lastTime2 > 700000 || current_lastTime2 == 0) {
    rear_rpm = 0;
    interval2 = 0;
  } else if (current_interval2 > 0) {
    double time_per_rev2 = current_interval2 * slits;
    rear_rpm = (int)(60000000.0 / time_per_rev2);
  }

  p_rpm = rear_rpm + front_rpm;
  avg_rpm = p_rpm / 2.0;

}

// ==========================================
// 1. スキャン専用タスク
// ==========================================
void bleScanTask(void *pvParameters) {
  while (true) {
    if (addrControl == nullptr || addrLogger == nullptr) {
      NimBLEDevice::getScan()->start(2, false); 
      NimBLEDevice::getScan()->clearResults();
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS); 
  }
}

// ==========================================
// 2. 操縦器 (C3-CONTROL) 専属タスク
// ==========================================
void bleControlTask(void *pvParameters) {
  unsigned long lastBleSend = 0;

  if (pClientControl == nullptr) {
    pClientControl = NimBLEDevice::createClient();
    pClientControl->setClientCallbacks(new ControlClientCallback());
  }

  while (true) {
    if (!connectedControl && addrControl != nullptr) {
      Serial.println("Connecting to Control C3...");
      
      if (pClientControl->connect(*addrControl)) {
        // pClientControl->setMTU(200); // NimBLEは通常自動でMTUをネゴシエーションしますが必要なら残せます
        NimBLERemoteService* pSvc = pClientControl->getService(serviceUUID);
        if (pSvc) {
          pCharControl = pSvc->getCharacteristic(charUUID);
          if (pCharControl && pCharControl->canNotify()) {
            // NimBLEでは subscribe(true, コールバック関数) を使用します
            pCharControl->subscribe(true, controlNotifyCallback);
            connectedControl = true;
            Serial.println(">>> Connected to Control C3! <<<");
          }
        }
      }

      if (!connectedControl) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
      }
    }

    if (connectedControl && pCharControl != nullptr) {
      if (millis() - lastBleSend > 20) {
        NavigationData navData;
        navData.pitch = (float)pitch;
        pCharControl->writeValue((uint8_t*)&navData, sizeof(NavigationData));
        lastBleSend = millis();
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// ==========================================
// 3. ロガー (C3-LOGGER) 専属タスク
// ==========================================
void bleLoggerTask(void *pvParameters) {
  unsigned long lastBleSend = 0;

  if (pClientLogger == nullptr) {
    pClientLogger = NimBLEDevice::createClient();
    pClientLogger->setClientCallbacks(new LoggerClientCallback());
  }

  while (true) {
    if (tgrsw == HIGH) { 
      if (!connectedLogger && addrLogger != nullptr) {
        Serial.println("Connecting to Logger C3...");
        
        if (pClientLogger->connect(*addrLogger)) {
          NimBLERemoteService* pSvc = pClientLogger->getService(serviceUUID);
          if (pSvc) {
            pCharLogger = pSvc->getCharacteristic(charUUID);
            connectedLogger = true;
            Serial.println(">>> Connected to Logger C3! <<<");
          }
        }
        
        if (!connectedLogger) {
          vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
      }
      
      if (connectedLogger && pCharLogger != nullptr) {
        if (millis() - lastBleSend > 20) {
          FullTelemetryPacket packet;
          memset(&packet, 0, sizeof(packet));
          
          packet.nav.pitch = (float)pitch;
          
          packet.roll      = (float)roll;
          packet.lat       = lat;
          packet.lon       = lon;
          packet.Altitude  = (float)Altitude;
          packet.heading   = (float)heading;
          packet.air_speed = (float)air_speed;
          packet.gnd_speed = (float)gnd_speed;
          packet.front_rpm = (float)front_rpm;
          packet.rear_rpm  = (float)rear_rpm;
          packet.epoch_time = epoch_time;
          
          packet.ctrl.E_steer = (float)E_steer;
          packet.ctrl.R_steer = (float)R_steer;
          packet.ctrl.E_trim  = (float)E_trim;
          packet.ctrl.E_angle = (float)E_angle;
          packet.ctrl.R_angle = (float)R_angle;
          packet.ctrl.e_servo_temp = (float)e_servo_temp;
          packet.ctrl.r_servo_temp = (float)r_servo_temp;
          
          packet.electrical_errors[0] = gps_active;
          packet.electrical_errors[1] = mcp_active;
          packet.electrical_errors[2] = ultra_active;
          packet.electrical_errors[3] = sdp_active;
          packet.electrical_errors[4] = imu_active;
          
          strncpy(packet.ctrl.control_mode, control_mode.c_str(), sizeof(packet.ctrl.control_mode) - 1);
          
          pCharLogger->writeValue((uint8_t*)&packet, sizeof(FullTelemetryPacket));
          lastBleSend = millis();
        }
      }
    } else { 
      if (connectedLogger) {
        pClientLogger->disconnect();
        connectedLogger = false;
        Serial.println("Logger Disconnected (Switch OFF)");
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}
void confirmICM(){
  Wire.beginTransmission(0x68); // ※もし常にエラーになる場合は 0x69 に変えてみてください
  if (Wire.endTransmission() == 0) {
    imu_active = true;  // 返事あり！生きている
  } else {
    imu_active = false; // 返事なし！エラー発生
  }
}
