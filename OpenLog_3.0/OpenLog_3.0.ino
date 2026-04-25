#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// ===============================
// ピン設定
// ===============================
static const int RX_PIN  = 20;   // XIAO RX  <- OpenLog TXO
static const int TX_PIN  = 21;   // XIAO TX  -> OpenLog RXI
static const int LED_PIN = 2;

// ===============================
// UART
// ===============================
HardwareSerial OpenLog(1);



static const uint8_t WIFI_CHANNEL = 1;
static uint8_t BROADCAST_MAC[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

#pragma pack(push, 1)
struct FullTelemetryPacket {
  uint32_t magic;
  uint8_t role;
  float E_steer, R_steer;
  float E_trim, E_angle, R_angle;
  float e_servo_temp, r_servo_temp;
  bool is_assisted;
  float pitch;
  float roll;
  float pitch_rate;
  float roll_rate;
  float front_rpm, rear_rpm;
  float air_speed, gnd_speed, Altitude, heading;
  double lat, lon;
  uint32_t epoch_time;
  bool electrical_errors[12];
};
#pragma pack(pop)
#define MAGIC 0x53333230u
#define ROLE_MEINKIBAN3 1
#define ROLE_LOGGER 3


bool diagOk = false; //OpenLog診断が成功したかどうか。
bool openlogReady = false;
bool bleConnected = true;//ESPNOWはUDPに近いのでこの概念がない。
String currentLogFile = "";

unsigned long lastBlinkMs = 0;
bool ledState = false;

// ===============================
// LED
// 診断成功後: 2秒おき点滅
// BLE接続中: 点灯
// ===============================
void updateLed() {
  if (!diagOk) {
    digitalWrite(LED_PIN, LOW);
    return;
  }

  if (bleConnected) {
    digitalWrite(LED_PIN, HIGH);
    return;
  }

  unsigned long now = millis();
  if (now - lastBlinkMs >= 2000) {
    lastBlinkMs = now;
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState ? HIGH : LOW);
  }
}

// ===============================
// OpenLog 補助
// ===============================
void drainOpenLog(unsigned long ms = 20) {
  unsigned long t0 = millis();
  while (millis() - t0 < ms) {
    while (OpenLog.available()) {
      OpenLog.read();
    }
    delay(1);
  }
}

bool waitChar(char ch, unsigned long timeout_ms) {
  unsigned long t0 = millis();
  while (millis() - t0 < timeout_ms) {
    while (OpenLog.available()) {
      char c = OpenLog.read();
      Serial.write(c);
      if (c == ch) return true;
    }
    delay(1);
  }
  return false;
}

bool waitForLogMode(unsigned long timeout_ms = 2000) {
  unsigned long t0 = millis();
  while (millis() - t0 < timeout_ms) {
    while (OpenLog.available()) {
      char c = OpenLog.read();
      Serial.write(c);
      if (c == '<') return true;
    }
    delay(1);
  }
  return false;
}

bool readReply(String &out, unsigned long timeout_ms = 3000) {
  out = "";
  unsigned long t0 = millis();
  unsigned long last = millis();

  while (millis() - t0 < timeout_ms) {
    bool got = false;

    while (OpenLog.available()) {
      char c = OpenLog.read();
      Serial.write(c);
      out += c;
      last = millis();
      got = true;

      if (c == '>') return true;
    }

    if (!got && millis() - last > 150 && out.length() > 0) {
      return true;
    }

    delay(1);
  }

  return out.length() > 0;
}

bool enterCommandMode() {
  drainOpenLog();

  Serial.println("[enterCommandMode]");

  OpenLog.write(26);
  delay(30);
  OpenLog.write(26);
  delay(30);
  OpenLog.write(26);

  return waitChar('>', 2500);
}

bool exitAppendMode() {
  for (int retry = 0; retry < 3; retry++) {
    Serial.print("[exitAppendMode retry ");
    Serial.print(retry);
    Serial.println("]");

    OpenLog.write(26);
    delay(30);
    OpenLog.write(26);
    delay(30);
    OpenLog.write(26);

    if (waitChar('>', 2500)) {
      return true;
    }

    delay(150);
  }
  return false;
}

bool sendCmdGet(const String &cmd, String &resp) {
  drainOpenLog();

  Serial.print("[SEND] ");
  Serial.println(cmd);

  OpenLog.print(cmd);
  OpenLog.write('\r');

  bool ok = readReply(resp, 3000);

  Serial.println();
  Serial.println("[RECV]");
  Serial.println(resp);

  return ok;
}

bool readSmallFileClean(const String &name, String &out, unsigned long overall_ms = 3000) {
  out = "";
  drainOpenLog();

  Serial.print("[READ] ");
  Serial.println(name);

  OpenLog.print("read ");
  OpenLog.print(name);
  OpenLog.write('\r');

  unsigned long start = millis();
  unsigned long last = millis();

  while (millis() - start < overall_ms) {
    bool got = false;

    while (OpenLog.available()) {
      char c = OpenLog.read();
      Serial.write(c);
      got = true;
      last = millis();

      if (c == '>') {
        out.replace("\r", "");
        out.replace("\n>", "");
        out.replace("...", "");
        return true;
      }

      out += c;
    }

    if (!got && millis() - last > 150 && out.length() > 0) {
      out.replace("\r", "");
      out.replace("...", "");
      return true;
    }

    delay(1);
  }

  out.replace("\r", "");
  out.replace("...", "");
  return out.length() > 0;
}

// ===============================
// CSV
// ===============================
String csvHeader() {
  return
    "epoch_time,"
    "lat,lon,"
    "altitude,heading,"
    "air_speed,gnd_speed,"
    "pitch,roll,"
    "pitch_rate,roll_rate,"
    "front_rpm,rear_rpm,"
    "E_steer,R_steer,E_trim,E_angle,R_angle,"
    "e_servo_temp,r_servo_temp,is_assisted,"
    "err0,err1,err2,err3,err4,err5,err6,err7,err8,err9,err10,err11";
}

String packetToCsv(const FullTelemetryPacket& p) {
  String s;
  s.reserve(360);

  s += String((uint32_t)p.epoch_time); s += ",";
  s += String(p.lat, 7); s += ",";
  s += String(p.lon, 7); s += ",";
  s += String(p.Altitude, 3); s += ",";
  s += String(p.heading, 3); s += ",";
  s += String(p.air_speed, 3); s += ",";
  s += String(p.gnd_speed, 3); s += ",";
  s += String(p.pitch, 3); s += ",";
  s += String(p.roll, 3); s += ",";
  s += String(p.pitch_rate, 7); s += ",";
  s += String(p.roll_rate, 7); s += ",";
  s += String(p.front_rpm, 1); s += ",";
  s += String(p.rear_rpm, 1); s += ",";
  s += String(p.E_steer, 3); s += ",";
  s += String(p.R_steer, 3); s += ",";
  s += String(p.E_trim, 3); s += ",";
  s += String(p.E_angle, 3); s += ",";
  s += String(p.R_angle, 3); s += ",";
  s += String(p.e_servo_temp, 3); s += ",";
  s += String(p.r_servo_temp, 3); s += ",";
  s += String(p.is_assisted ? "1" : "0");

  for (int i = 0; i < 12; i++) {
    s += ",";
    s += p.electrical_errors[i] ? "1" : "0";
  }

  return s;
}

// ===============================
// 診断
// ===============================
bool runOpenLogDiagnostic() {
  Serial.println("=== OpenLog diagnostic start ===");

  String resp;
  String body;

  if (!enterCommandMode()) {
    Serial.println("DIAG FAIL: command mode");
    return false;
  }

  sendCmdGet("rm test_diag.txt", resp);

  if (!sendCmdGet("new test_diag.txt", resp)) {
    Serial.println("DIAG FAIL: create test_diag.txt");
    return false;
  }

  drainOpenLog();
  Serial.println("[SEND] append test_diag.txt");
  OpenLog.print("append test_diag.txt");
  OpenLog.write('\r');

  if (!waitForLogMode(2000)) {
    Serial.println("DIAG FAIL: append enter log mode");
    return false;
  }

  delay(100);

  Serial.println("[WRITE] DIAG_OK_1");
  OpenLog.print("DIAG_OK_1\r\n");
  delay(80);

  Serial.println("[WRITE] DIAG_OK_2");
  OpenLog.print("DIAG_OK_2\r\n");
  delay(300);

  if (!exitAppendMode()) {
    Serial.println("DIAG FAIL: exit append");
    return false;
  }

  if (!readSmallFileClean("test_diag.txt", body, 3000)) {
    Serial.println("DIAG FAIL: read test_diag.txt");
    return false;
  }

  if (body.indexOf("DIAG_OK_1") < 0 || body.indexOf("DIAG_OK_2") < 0) {
    Serial.println("DIAG FAIL: verify contents");
    Serial.println("[BODY]");
    Serial.println(body);
    return false;
  }

  if (!sendCmdGet("rm test_diag.txt", resp)) {
    Serial.println("DIAG FAIL: delete test_diag.txt");
    return false;
  }

  Serial.println("=== OpenLog diagnostic success ===");
  return true;
}

// ===============================
// LOGSディレクトリと本番ファイル準備
// ===============================
bool isCommandErrorResponse(const String &resp) {
  if (resp.indexOf("!>") >= 0) return true;
  if (resp.indexOf("Error creating file") >= 0) return true;
  if (resp.indexOf("Failed") >= 0) return true;
  if (resp.indexOf("failed") >= 0) return true;
  if (resp.indexOf("error") >= 0) return true;
  if (resp.indexOf("Error") >= 0) return true;
  if (resp.indexOf("Can't") >= 0) return true;
  if (resp.indexOf("cannot") >= 0) return true;
  if (resp.indexOf("No such file") >= 0) return true;
  return false;
}

bool ensureLogsDirectory() {
  String resp;

  Serial.println("=== ensureLogsDirectory ===");

  if (!sendCmdGet("cd LOGS", resp)) return false;
  if (!isCommandErrorResponse(resp)) return true;

  if (!sendCmdGet("md LOGS", resp)) return false;

  // 既に存在していても次のcdが通ればよい
  if (!sendCmdGet("cd LOGS", resp)) return false;
  if (isCommandErrorResponse(resp)) return false;

  return true;
}

bool createNewFileIfAvailable(const String &filename) {
  String resp;

  if (!sendCmdGet("new " + filename, resp)) {
    return false;
  }

  if (isCommandErrorResponse(resp)) {
    return false;
  }

  return true;
}

String findAvailableLogFile() {
  for (int i = 1; i <= 999; i++) {
    char buf[16];
    snprintf(buf, sizeof(buf), "LOG%03d.csv", i);
    String name(buf);

    if (createNewFileIfAvailable(name)) {
      return name;
    }
  }
  return "";
}

bool openLogFileForAppend(const String &filename) {
  drainOpenLog();

  Serial.print("[SEND] append ");
  Serial.println(filename);

  OpenLog.print("append ");
  OpenLog.print(filename);
  OpenLog.write('\r');

  if (!waitForLogMode(2000)) {
    Serial.println("LOG SETUP FAIL: no '<' after append");
    return false;
  }

  delay(100);
  return true;
}

bool setupOpenLogLogging() {
  Serial.println("=== OpenLog logging setup start ===");

  if (!ensureLogsDirectory()) {
    Serial.println("LOG SETUP FAIL: LOGS directory");
    return false;
  }

  currentLogFile = findAvailableLogFile();
  if (currentLogFile.length() == 0) {
    Serial.println("LOG SETUP FAIL: no filename");
    return false;
  }

  if (!openLogFileForAppend(currentLogFile)) {
    Serial.println("LOG SETUP FAIL: append open");
    return false;
  }

  delay(100);
  OpenLog.print(csvHeader());
  OpenLog.print("\r\n");
  delay(100);

  Serial.print("Logging file: ");
  Serial.println(currentLogFile);
  Serial.println("=== OpenLog logging setup success ===");
  return true;
}

// ===============================
// onRecv
// ===============================
void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(FullTelemetryPacket)) return;
  FullTelemetryPacket p;
  memcpy(&p,data,sizeof(p));
  if(p.magic != MAGIC) return;
  if(p.role != ROLE_MEINKIBAN3) return;
  String line = packetToCsv(p);
  OpenLog.print(line);
  OpenLog.print("\r\n");
}


void setupESPNOW() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
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
  esp_now_register_recv_cb(onRecv);
  Serial.println("ESPNOW logger ready");
}

// ===============================
// setup / loop
// ===============================
void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("=== OpenLog BLE Logger Boot ===");
  Serial.print("sizeof(FullTelemetryPacket) = ");
  Serial.println(sizeof(FullTelemetryPacket));

  OpenLog.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);

  Serial.println("Waiting for OpenLog boot...");
  delay(3000);
  drainOpenLog(200);

  diagOk = runOpenLogDiagnostic();
  if (!diagOk) {
    Serial.println("SYSTEM HALT: diagnostic failed");
    return;
  }

  openlogReady = setupOpenLogLogging();
  if (!openlogReady) {
    Serial.println("SYSTEM HALT: logging setup failed");
    diagOk = false;
    return;
  }

  setupESPNOW();
}

void loop() {
  updateLed();
  delay(10);
}