#include <Wire.h>
#include <Adafruit_MCP23X17.h>
#include <ICM20948_WE.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <time.h>
#include <sys/time.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "S320Protocol.h"
#include "TFTand9axis_sensor.h"
#include <ArduinoJson.h>
#include "TBT_GNSS.h"
#include "TBT_AndroidSerial.h"

// --- Library Instances ---
TBT_GNSS mygnss;
TBT_AndroidSerial myAndroid;
Adafruit_MCP23X17 mcp;
TFTand9axis_sensor instrumentPanel;

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

/* ======================= グローバル変数 ========================================================== */
//loop()内の時間を管理する変数
unsigned long previousMillis = 0;  // 前回の更新時間を保存
const long interval = 100;         // 更新間隔（ミリ秒）
int count1;     //LEDを光らせるときに使用するカウント変数
//姿勢角計
double pitch = 0.0;
double roll = 0.0;
double pitch_rate = 0.0;  // ジャイロ ピッチレート [°/s]
double roll_rate = 0.0;   // ジャイロ ロールレート [°/s]
double yaw_rate = 0;
double ax = 0;
double ay = 0;
double az = 0;

double heading = 0.0;
double pitch_rad;
double roll_rad;
double heading_rad;
//高度計
double raw_Altitude = 0.0;
double Altitude = 0.0;
double Alt_offset = 0.0;
double Alt_limmit = 7.65 - 0.05; //7.65が高度計MB1242の測定限界、たまに高度計に近づきすぎたりすると7.65になってしまいしっかりと測定できていない場合があるため-0.05
unsigned long lastUltraTime = 0;  // 最後に計測した時間を記録する変数
//ピトー管
double air_speed = 0.0;
double gnd_speed = 0.0;
int16_t rawPressure;
double AIR_DENSITY;
//スイッチが押されているかどうか判別する変数
int tktsw;
volatile int tgrsw;
/*===========GNSS関連の変数==========================================*/
//機体の位置を取得するための変数
int gnss_status;
double lat;
double lon;
double alt;          //gnnss_alt - ref_altを表わす変数で地面からどのくらいの距離かを表す変数
double ref_alt;      // 基準となる高度(GPS)
double gnss_alt;     //GNSSで取得した高度データ
double gnss_heading; //機体が進んでいる方向を表す変数
int fixType;
//時間を取得するための変数
uint32_t epoch_time;
int year;
int month;
int day;
int hour;
int minute;
int second;

/*=============回転数計系統の変数名====================*/
double slits = 18.0;
// 前部(Photo1)用の変数
volatile int pulseCount1 = 0;
volatile unsigned long lastTime1 = 0;
volatile unsigned long interval1 = 0;
// 後部(Photo2)用の変数
volatile int pulseCount2 = 0;
volatile unsigned long lastTime2 = 0;
volatile unsigned long interval2 = 0;
//回転数rpm
double p_rpm;
double avg_rpm;
double front_rpm = 0.0;
double rear_rpm = 0.0;

/*==========エラーを管理するbool型の変数=================================*/
bool mcp_active = false;  // MCPが正しく認識されたか管理するフラグ
bool ultra_active = false;
bool gps_active = false;
bool sdp_active = false;
bool imu_active = false;
bool CtrlStickCommunication_active = false;

/*========================操縦桿系統の変数====================================*/
volatile int16_t E_raw_adc = 0;
volatile int16_t R_raw_adc = 0;
volatile int16_t ele_param[4];
volatile int16_t rud_param[4];
volatile float E_stick_mapped, R_stick_mapped;
volatile int16_t E_krs,R_krs;
volatile float E_trim = 0;
volatile float E_angle = 0;
volatile float R_angle = 0;
volatile float e_servo_temp = 0.0;
volatile float r_servo_temp = 0.0;
volatile bool is_assisted = false;
volatile uint32_t ctrl_stk_t = 0;
volatile String electrical_errors = "[]";
/*-------------------------------------------------------------------*/
/* ===== 関数プロトタイプ ===== */
void clearI2CBus(int sdaPin, int sclPin);
void readSiseikaku();
void startAltimeter();
void getAltitude();
bool startMeasurementAir_speed();
void loopGPS();
void getAir_speed();
void MCP23017_LED();
void sendAndoroid();
void confirmICM();
void IRAM_ATTR isrPhoto1();
void IRAM_ATTR isrPhoto2();
void calcRPM();
void sendCtrlStick();
void sendLogger();
void commTask(void *pvParameters);


// ESP-NOW はコネクションレスなので、最終受信時刻で疎通を判定する
static volatile uint32_t g_lastRecvFromSoujyuukanMs = 0;
static const uint32_t LINK_TIMEOUT_MS = 500;

// I2C0 (Wire) は commTask(Core0) が独占する。キャリブ中は loop()(Core1) が
// IMU を長時間占有するので、commTask 側の I2C0 アクセスを一時停止する。
static volatile bool g_calibrating = false;

// --- 受信コールバック（操縦用C3からデータが届いた時） ---
void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(ControlData))
    return;
  ControlData pkt;
  memcpy(&pkt, data, sizeof(pkt));

  if (pkt.magic != MAGIC)
    return;

  if (pkt.role != ROLE_SOUJYUUKAN)
    return;

  E_raw_adc = pkt.E_raw_adc;
  R_raw_adc = pkt.R_raw_adc;
  for (int i = 0; i < 4; i++) {
    ele_param[i] = pkt.ele_param[i];
    rud_param[i] = pkt.rud_param[i];
  }
  E_krs = pkt.E_krs;
  R_krs = pkt.R_krs;
  E_stick_mapped = pkt.E_stick_mapped;
  R_stick_mapped = pkt.R_stick_mapped;
  E_trim = pkt.E_trim;
  E_angle = pkt.E_angle;
  R_angle = pkt.R_angle;
  e_servo_temp = pkt.e_servo_temp;
  r_servo_temp = pkt.r_servo_temp;
  is_assisted = pkt.is_assisted;
  ctrl_stk_t = pkt.ctrl_stk_t;
  g_lastRecvFromSoujyuukanMs = millis();
}

void onSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS) {
    Serial.println("[send] 送信失敗");
  }
  // 成功時は静かにする (毎回ログると邪魔)
}

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

  // Attach AndroidSerial to main Serial (USB)
  myAndroid.attach(115200);
  // Attach GNSS (Baud, RX, TX, UART#)
  mygnss.attach(921600, 16, 17, 1);

  pinMode(PHOTO1_PIN, INPUT);
  pinMode(PHOTO2_PIN, INPUT);
  pinMode(tktsw_PIN, INPUT);
  pinMode(tgrsw_PIN, INPUT);
  gpio_pullup_dis((gpio_num_t)I2C0_SDA);
  gpio_pullup_dis((gpio_num_t)I2C0_SCL);
  gpio_pullup_dis((gpio_num_t)I2C1_SDA);
  gpio_pullup_dis((gpio_num_t)I2C1_SCL);

  // ★追加：ピンの電圧が「HIGHからLOWに落ちた瞬間(FALLING)」にカウント関数を呼ぶよう設定
  attachInterrupt(digitalPinToInterrupt(PHOTO1_PIN), isrPhoto1, FALLING);
  attachInterrupt(digitalPinToInterrupt(PHOTO2_PIN), isrPhoto2, FALLING);

  instrumentPanel.begin();

  //不揮発性メモリからとりだす
  ref_alt = instrumentPanel.returnRef_alt(); 
  Alt_offset = instrumentPanel.returnAlt_offset();

  if (mcp.begin_I2C(0x20, &Wire1)) {
    mcp.pinMode(LED1, OUTPUT);
    mcp.pinMode(LED2, OUTPUT);
    mcp.pinMode(LED3, OUTPUT);
    mcp_active = true;
  } else {
    mcp_active = false;
  }

  /* --- 超音波 初回測定 --- */
  startAltimeter();
  startMeasurementAir_speed();
  delay(20);

  // ESP-NOWの初期設定
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  WiFi.setSleep(false);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("[error] esp_now_init 失敗");
    while (1)
      delay(500);
  }

  esp_now_peer_info_t peer = {};             // 構造体を全部ゼロで初期化
  memcpy(peer.peer_addr, BROADCAST_MAC, 6);  // 宛先 MAC をコピー
  peer.channel = WIFI_CHANNEL;               // 同じチャンネルを指定
  peer.encrypt = false;                      // ブロードキャストは暗号化不可
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("[error] add_peer 失敗");
    while (1)
      delay(1000);
  }

  esp_now_register_send_cb(onSent);
  esp_now_register_recv_cb(onRecv);

  // 通信処理を Core 0 に分離。loop() (Core 1) の画面更新と干渉させない
  xTaskCreatePinnedToCore(commTask, "commTask", 4096, NULL, 1, NULL, 0);
}

/*=====================================================================================================================================================
======================loop() メイン処理====================================================================================================
====================================================================================================================================================*/
void loop() {
  tktsw = digitalRead(tktsw_PIN);
  // タクトスイッチによるキャリブレーション（500ms長押し検知）
  // GPIO35はプルダウン構成：未押下=LOW、押下=HIGH
  {
  static unsigned long highStartTime = 0;
    static bool actionDone = false; // 「すでにどちらかの処理を実行したか」のフラグ

    if (tktsw == HIGH) {
      // ボタンを押した瞬間に時間を記録し、フラグをリセット
      if (highStartTime == 0) {
        highStartTime = millis();
        actionDone = false; 
      }

      // 押し続けている時間を計算
      unsigned long pressDuration = millis() - highStartTime;

      // 【5秒長押し】押し続けた時間が5000msに達した瞬間に実行
      if (!actionDone && pressDuration >= 5000) {
        g_calibrating = true;
        delay(25); // commTask が現在の Wire アクセスを完了するのを待つ (周期20ms)
        instrumentPanel.magCalibrate();
        g_calibrating = false;
        actionDone = true; // 処理済みマークをつける（指を離すまで何もしない）
      }
      
    } else { 
      // tktsw == LOW （ボタンを離した瞬間）
      if (highStartTime != 0) {
        unsigned long pressDuration = millis() - highStartTime;

        // 【0.5秒長押し】離した時点で、500ms以上かつ処理済みでなければ実行
        if (!actionDone && pressDuration >= 500) {
          g_calibrating = true;
          delay(25);
          instrumentPanel.calibrate();
          g_calibrating = false;
          ref_alt = gnss_alt;
          Alt_offset = raw_Altitude;
          instrumentPanel.saveAltOffsets(ref_alt, Alt_offset);
        }

        // 次回の計測のために時間をリセット
        highStartTime = 0; 
      }
    }
  }

  // 超音波 / IMU 読み / confirmICM は commTask(Core0) に移動済。
  // loop() は TFT 描画と Wire1 系 (SDP810, MCP23017) のみ担当する。

  static unsigned long lastPrint1 = 0;
  static unsigned long lastPrint2 = 50;

  instrumentPanel.updata(E_trim, air_speed, front_rpm, Altitude);

  if (millis() - lastPrint1 >= interval) {
    tgrsw = digitalRead(tgrsw_PIN);
    count1 += 1;
    if(count1 >= 40){
      count1 = 0;
    }
    getAir_speed();
    calcRPM();
    MCP23017_LED();
    lastPrint1 = millis();
  }

  if (millis() - lastPrint2 >= interval) {
    loopGPS();
    sendAndoroid();
    lastPrint2 = millis();
  }
}

/* =========== 関数定義 =============================================================================================================== */
// Core 0 で 50Hz の ESP-NOW 送信を担当する。
// 画面更新やセンサ I/O が走る loop() (Core 1) と分離することで、
// Wi-Fi 送信と SPI/I2C のリソース競合による表示の引っかかりを抑える。
void commTask(void *pvParameters) {
  const TickType_t xFrequency = 20 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  unsigned long lastConfirmIcmMs = 0;
  while (1) {
    // I2C0 (Wire) アクセスは commTask に集約。キャリブ中は loop() が IMU を
    // 占有するので、競合を避けるため Wire アクセスを丸ごとスキップする。
    if (!g_calibrating) {
      // IMU 50Hz サンプリング (PID D項用 pitch_rate を含む)
      instrumentPanel.getPitchAndRollAndHeading(&pitch_rad, &roll_rad, &heading_rad, &pitch_rate, &roll_rate, &yaw_rate, &ax, &ay, &az);
      pitch = pitch_rad * (180.0 / PI);
      roll = roll_rad * (180.0 / PI);
      heading = heading_rad * (180.0 / PI);

      // 超音波 MB1242 (interval = 100ms)
      if (millis() - lastUltraTime >= interval) {
        if (ultra_active) {
          getAltitude();
          if (Altitude < 0) {
            Altitude = 0;
          }
        } else {
          Altitude = alt;
        }
        startAltimeter();          // 失敗していても次の測定開始合図を送って再挑戦
        lastUltraTime = millis();
      }

      // IMU 生存 ping (100ms 間隔)
      if (millis() - lastConfirmIcmMs >= 100) {
        confirmICM();
        lastConfirmIcmMs = millis();
      }
    }

    sendCtrlStick();
    if (tgrsw == HIGH) {
      sendLogger();
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void sendCtrlStick() {
  NavigationData navData;
  navData.magic = MAGIC;
  navData.role = ROLE_MEINKIBAN3;
  navData.pitch = (float)pitch;
  navData.pitch_rate = (float)pitch_rate;
  esp_err_t r = esp_now_send(BROADCAST_MAC, (uint8_t *)&navData, sizeof(navData));
  if (r != ESP_OK) {
    Serial.printf("[send] error code=%d\n", r);
  }
}

void sendLogger() {
  FullTelemetryPacket packet;
  memset(&packet, 0, sizeof(packet));  // 未代入フィールドのゴミ値を防ぐ

  packet.magic = MAGIC;
  packet.role = ROLE_MEINKIBAN3;

  packet.pitch = (float)pitch;
  packet.roll = (float)roll;
  packet.pitch_rate = (float)pitch_rate;
  packet.roll_rate = (float)roll_rate;
  packet.yaw_rate = (float)yaw_rate;
  packet.ax = (float)ax;
  packet.ay = (float)ay;
  packet.az = (float)az;

  packet.lat = lat;
  packet.lon = lon;
  packet.Altitude = (float)Altitude;
  packet.heading = (float)heading;
  packet.air_speed = (float)air_speed;
  packet.gnd_speed = (float)gnd_speed;
  packet.front_rpm = (float)front_rpm;
  packet.rear_rpm = (float)rear_rpm;
  packet.epoch_time = epoch_time;

  packet.E_raw_adc = E_raw_adc;
  packet.R_raw_adc = R_raw_adc;
  packet.E_stick_mapped = E_stick_mapped;
  packet.R_stick_mapped = R_stick_mapped;
  for (int i = 0; i < 4; i++) {
    packet.ele_param[i] = ele_param[i];
    packet.rud_param[i] = rud_param[i];
  }
  packet.E_krs = E_krs;
  packet.R_krs = R_krs;
  packet.E_trim = (float)E_trim;
  packet.E_angle = (float)E_angle;
  packet.R_angle = (float)R_angle;
  packet.e_servo_temp = (float)e_servo_temp;
  packet.r_servo_temp = (float)r_servo_temp;
  packet.is_assisted = is_assisted;
  packet.ctrl_stk_t = ctrl_stk_t;
  packet.main_bord_t = millis();

  packet.electrical_errors[0] = gps_active;
  packet.electrical_errors[1] = mcp_active;
  packet.electrical_errors[2] = ultra_active;
  packet.electrical_errors[3] = sdp_active;
  packet.electrical_errors[4] = imu_active;

  esp_err_t r = esp_now_send(BROADCAST_MAC, (uint8_t *)&packet, sizeof(packet));
  if (r != ESP_OK) {
  }
}

void MCP23017_LED() {
  Wire1.beginTransmission(0x20);
  if (Wire1.endTransmission() == 0) {
    // 応答あり
    if (!mcp_active) {
      //  begin_I2Cを呼ばずに、直接ピン設定だけをやり直す
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
    }
    mcp_active = false;
  }
  if (mcp_active) {
    if (count1 <= 5) {
      mcp.digitalWrite(LED1, HIGH);
      mcp.digitalWrite(LED2, LOW);
      mcp.digitalWrite(LED3, LOW);
    } else if (5 < count1 && count1 <= 10) {
      mcp.digitalWrite(LED1, HIGH);
      mcp.digitalWrite(LED2, LOW);
      mcp.digitalWrite(LED3, LOW);
    } else if (10 < count1 && count1 <= 15) {
      mcp.digitalWrite(LED1, HIGH);
      mcp.digitalWrite(LED2, HIGH);
      mcp.digitalWrite(LED3, LOW);
    } else if (15 < count1 && count1 <= 20) {
      mcp.digitalWrite(LED1, LOW);
      mcp.digitalWrite(LED2, HIGH);
      mcp.digitalWrite(LED3, LOW);
    } else if (20 < count1 && count1 <= 25) {
      mcp.digitalWrite(LED1, LOW);
      mcp.digitalWrite(LED2, HIGH);
      mcp.digitalWrite(LED3, HIGH);
    } else if (25 < count1 && count1 <= 30) {
      mcp.digitalWrite(LED1, LOW);
      mcp.digitalWrite(LED2, LOW);
      mcp.digitalWrite(LED3, HIGH);
    } else if (30 < count1 && count1 <= 35) {
      mcp.digitalWrite(LED1, HIGH);
      mcp.digitalWrite(LED2, LOW);
      mcp.digitalWrite(LED3, HIGH);
    } else {
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
    ultra_active = true;
  } else {
    ultra_active = false;
  }
}

void getAltitude() {
  Wire.requestFrom(ADDR_MB1242, 2);

  if (Wire.available() >= 2) {
    // 読み取り成功
    byte high = Wire.read();
    byte low = Wire.read();
    raw_Altitude = ((double)((high << 8) | low) / 100);
    Altitude = raw_Altitude - Alt_offset;
    ultra_active = true;
    if (Altitude > Alt_limmit - Alt_offset) {
      Altitude = alt;
    }
  } else {
    // 読み取り失敗！ -> 即座にバスクリアと再初期化を行う
    ultra_active = false;
    Wire.end();  // ペリフェラルを一旦停止

    // SCLピンを直接操作してバスのロックを解除
    clearI2CBus(I2C0_SDA, I2C0_SCL);

    // I2Cを再起動
    Wire.begin(I2C0_SDA, I2C0_SCL);
    Wire.setTimeOut(50);
    Wire.setClock(100000);
  }
}

bool startMeasurementAir_speed() {
  Wire1.beginTransmission(SDP810_ADDR);
  Wire1.write(0x36);
  Wire1.write(0x15);  // 平均化ありの連続測定
  byte error = Wire1.endTransmission();
  if (error == 0) {
    return true;
  } else {
    return false;
  }
}

void getAir_speed() {
  int16_t dp_raw;            //生の差圧データ
  int16_t temp_raw;          //生の温度データ
  float tempreature;         //温度
  float pressurePa;          //差圧

  byte count = Wire1.requestFrom(SDP810_ADDR, 6);

  if (count == 6) {
    sdp_active = true;
    // 差圧
    dp_raw = (Wire1.read() << 8) | Wire1.read();
    Wire1.read();
    // 温度
    temp_raw = (Wire1.read() << 8) | Wire1.read();
    Wire1.read();
    pressurePa = (float)dp_raw / 60.0;
    tempreature = (float)temp_raw / 200.0;
    AIR_DENSITY = 101325 / (287 * (tempreature + 273.15));
    if (pressurePa < 0)
      pressurePa = 0;
    // 対気速度を計算
    air_speed = 1.23 * sqrt(2 * pressurePa / AIR_DENSITY);   //風洞実験から得られた係数1.23、ベルヌーイの定理を用いて計算
  } else {
    sdp_active = false;
    air_speed = 0.0;

    if (mcp_active == true) {
      startMeasurementAir_speed();
    } else {
      Wire1.end();
      clearI2CBus(I2C1_SDA, I2C1_SCL);
      Wire1.begin(I2C1_SDA, I2C1_SCL);
      Wire1.setTimeOut(50);
      Wire1.setClock(100000);

      if (mcp.begin_I2C(0x20, &Wire1)) {
        mcp.pinMode(LED1, OUTPUT);
        mcp.pinMode(LED2, OUTPUT);
        mcp.pinMode(LED3, OUTPUT);
        mcp_active = true;
      } else {
        mcp_active = false;
      }
      startMeasurementAir_speed();
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
  pinMode(sclPin, INPUT);  // SCLを先にHIGHへ
  delayMicroseconds(5);
  pinMode(sdaPin, INPUT);  // その後SDAをHIGHへ
  gpio_pullup_dis((gpio_num_t)sdaPin);
  gpio_pullup_dis((gpio_num_t)sclPin);
}

void loopGPS() {
  gnss_status = (int)mygnss.get(GNSS_STATUS);
  lat = mygnss.get(GNSS_LATITUDE);
  lon = mygnss.get(GNSS_LONGITUDE);
  gnss_alt = mygnss.get(GNSS_ALTITUDE);
  gnss_heading = mygnss.get(GNSS_HEADING);
  gnd_speed = mygnss.get(GNSS_SPEED);
  year = mygnss.get(GNSS_YEAR);
  month = mygnss.get(GNSS_MONTH);
  day = mygnss.get(GNSS_DAY);
  hour = mygnss.get(GNSS_HOUR);
  minute = mygnss.get(GNSS_MINUTE);
  second = mygnss.get(GNSS_SECOND);
  struct tm timeinfo;
  alt = gnss_alt - ref_alt;
  if (alt < 0) {
    alt = -alt;
  }
  // ③ 箱に数字を流し込む（※年と月に「C言語特有の罠」があるので注意！）
  timeinfo.tm_year = year - 1900;  // 年は「1900」を引くルール
  timeinfo.tm_mon = month - 1;     // 月は「1」を引くルール（1月=0、12月=11）
  timeinfo.tm_mday = day;
  timeinfo.tm_hour = hour + 9;
  timeinfo.tm_min = minute;
  timeinfo.tm_sec = second;
  epoch_time = mktime(&timeinfo);

  fixType = (int)mygnss.get(GNSS_FIX_TYPE);
}

void sendAndoroid() {
  myAndroid.resetData();
  // Steering and Control
  myAndroid.add("E_raw_adc",E_raw_adc);
  myAndroid.add("R_raw_adc",R_raw_adc);
  myAndroid.add("ele_param0",ele_param[0]);
  myAndroid.add("ele_param1",ele_param[1]);
  myAndroid.add("ele_param2",ele_param[2]);
  myAndroid.add("ele_param3",ele_param[3]);
  myAndroid.add("rud_param0",rud_param[0]);
  myAndroid.add("rud_param1",rud_param[1]);
  myAndroid.add("rud_param2",rud_param[2]);
  myAndroid.add("rud_param3",rud_param[3]);
  myAndroid.add("E_krs",E_krs);
  myAndroid.add("R_krs",R_krs);
  myAndroid.add("E_steer", E_stick_mapped);
  myAndroid.add("R_steer", R_stick_mapped);
  myAndroid.add("E_trim", E_trim);
  myAndroid.add("E_angle", E_angle);
  myAndroid.add("R_angle", R_angle);
  myAndroid.add("e_servo_temp", e_servo_temp);
  myAndroid.add("r_servo_temp", r_servo_temp);

  // Flight Parameters
  myAndroid.add("air_speed", air_speed);
  myAndroid.add("gnd_speed", gnd_speed);
  myAndroid.add("pitch", pitch);
  myAndroid.add("roll", roll);
  myAndroid.add("heading",heading);
  myAndroid.add("pitch_rate", pitch_rate);
  myAndroid.add("roll_rate", roll_rate);
  myAndroid.add("front_rpm", front_rpm);
  myAndroid.add("rear_rpm", rear_rpm);
  myAndroid.add("p_rpm", p_rpm);
  myAndroid.add("is_assisted", is_assisted);

  // Send Location using dedicated method (creates nested object)
  myAndroid.addLocation(lat, lon);
  myAndroid.add("altitude", Altitude);
  myAndroid.add("heading", (int)heading);
  myAndroid.add("gnss_heading", (int)gnss_heading);

  // Electrical Errors Logic
  StaticJsonDocument<512> errorDoc;
  JsonArray errors = errorDoc.to<JsonArray>();

  // エラーがあれば番号を追加！
  if (fixType < 3 || fixType == 5 || gnss_status != 1)
    errors.add(200);  // GPS受信不良
  if(gnss_status != 1)
    errors.add(404);   //GPS接続エラー
  if (!mcp_active)
    errors.add(400);  // MCP23017エラー
  if (!ultra_active)
    errors.add(401);  // 高度計エラー
  if (!sdp_active)
    errors.add(402);  // ピトー管エラー
  if (!imu_active) {
    errors.add(201);  // 9軸センサーエラー
    errors.add(403);
  }
  if (millis() - g_lastRecvFromSoujyuukanMs > LINK_TIMEOUT_MS){
    errors.add(500);  // 操縦桿との通信エラー (ESP-NOW: 500ms 無音で判定)
    CtrlStickCommunication_active = false;
  }else{
    CtrlStickCommunication_active = true;
  }
  if(CtrlStickCommunication_active){
    if (e_servo_temp < 5)
      errors.add(501);  // エレベーターサーボ温度異常
    if (r_servo_temp < 5)
      errors.add(502);  // ラダーサーボ温度異常
  }
  // 600 (ロガー通信エラー) はロガーからのハートビート実装後に復活予定
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
      interval1 = diff;  // 初回はそのまま記録
    } else {
      interval1 = (interval1 + diff) / 2;  // 過去のデータと平均をとってブレを吸収！
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

void calcRPM() {
  noInterrupts();  // データを安全に取り出すため一瞬だけ割り込みをストップ
  unsigned long current_interval1 = interval1;
  unsigned long current_lastTime1 = lastTime1;
  unsigned long current_interval2 = interval2;
  unsigned long current_lastTime2 = lastTime2;
  interrupts();  // すぐに再開

  // 安全対策：0.7秒以上パルスが来ていなければ「回転が止まった」とみなす
  unsigned long currentMicros = micros();

  // 【前部の計算】
  // 0.7秒経過、またはまだ1度も計測されていない場合は 0 にする
  if (currentMicros - current_lastTime1 > 700000 || current_lastTime1 == 0) {
    front_rpm = 0;
    interval1 = 0;  // 完全に止まったら間隔もリセット
  } else if (current_interval1 > 0) {
    // (1スリットの間隔) × スリット総数 ＝ 1回転にかかる時間
    double time_per_rev1 = current_interval1 * slits;
    front_rpm = (int)(60000000.0 / time_per_rev1);  // 1分(6000万us) ÷ 1回転の時間
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

void confirmICM() {
  Wire.beginTransmission(0x68);  // 常にエラーの場合 0x68 → 0x69
  if (Wire.endTransmission() == 0) {
    imu_active = true;
  } else {
    imu_active = false; 
  }
}
