#pragma once

#include <Arduino.h>
#include <TFT_eSPI.h>
#include <Wire.h>
#include <ICM20948_WE.h>
#include <Preferences.h>
#include <cmath>
#include <cstdio>

class TFTand9axis_sensor {
private:
  static constexpr int ICM20948_ADDR = 0x68;
  static constexpr int SIZE = 4;

  static constexpr float z_max = 1000.f;
  static constexpr int h_num = 10;
  static constexpr float hrs = z_max / (float)h_num;
  static constexpr float vrs = hrs / 10.f;
  static constexpr int v_num = 11;
  static constexpr float ground_width = 200.f;

  TFT_eSPI tft;
  TFT_eSprite fixedSprite;
  TFT_eSprite dynamicSprite;
  ICM20948_WE myIMU;
  Preferences _prefs;

  bool spritesOk = false;
  bool _calibrated = false;
  TFT_eSPI *canvas = nullptr;

  double pitch;
  double roll;
  double heading;
  xyzFloat magOfs = {0.0f, 0.0f, 0.0f};

  // --- Madgwick AHRS (姿勢推定) ---
  // 旧来の myIMU.getPitch()/getRoll()(加速度のみ・フュージョンなし)を廃止し、
  // 加速度+ジャイロの相補フュージョン(Madgwick)に置き換えた。ICM20948_minimal から移植。
  // ピッチ/ロールの符号は従来(getPitch/getRoll)と同じ(機首上げ=負)。
  // 下流のPID(Kp=-1.0で反転を補償)がこの符号前提のため、意図的に維持している。
  double mq_w = 1.0, mq_x = 0.0, mq_y = 0.0, mq_z = 0.0;  // 姿勢クオータニオン(単位元で初期化)
  unsigned long _lastAttUs = 0;  // 前回更新時刻 [us] (dt 計測用)
  bool _attInit = false;         // クオータニオンを加速度でシード済みか

  typedef struct
  {
    float x, y, z, w;
  } Vector;
  typedef struct
  {
    Vector a, b;
  } Line;
  typedef struct
  {
    float x, y;
  } Point;

  float T[SIZE][SIZE];
  float Rx[SIZE][SIZE];
  float Rz[SIZE][SIZE];
  float Rz_tri[2][2];
  float M_temp[SIZE][SIZE];
  float M[SIZE][SIZE];

  Line hline[h_num];
  Line vline[v_num];
  int hxy[h_num][4];
  int vxy[v_num][4];

  Point topTri[3];
  const float gy_dis = 200.f;
  const float scaleIntervalAngle = (float)(5.0 * (PI / 180.0));
  int tickCount = 8;

  void initMatrices();
  void applyMatrix(double a, double b, double c, double d, double *x, double *y);
  TFT_eSPI *activeCanvas();
  void DrawPixel(int x, int y, uint16_t color);
  void DrawLine(int x0, int y0, int x1, int y1, uint16_t color);
  void DrawTriangle(int x0, int y0, int x1, int y1, int x2, int y2, uint16_t color);
  void topTriinitializer();
  void DrawRollArcPolyline(uint16_t rollScaleColor);
  int DrawFixedGUI(int lineSpacing = 5);
  void Drawchar(double torim, double IAS, double rpm, double ALT);
  void multiply_matrix(const float A[SIZE][SIZE], const float B[SIZE][SIZE], float C[SIZE][SIZE]);
  void applyMatrix4(const float M4[SIZE][SIZE], Vector *V);
  void hlineinitializer();
  void vlineinitializer();
  void updataLegacy(float torim, double IAS, double rpm, double ALT);
  void updataSprites(float torim, double IAS, double rpm, double ALT);
  bool shouldRedrawChar(float torim, double IAS, double rpm, double ALT);
  bool loadOffsets();
  void saveOffsets();

  // Madgwick フィルタ本体: a=加速度[g](内部で正規化), g*=角速度[rad/s], dt=[s]。
  void madgwickUpdate(double ax, double ay, double az,
                      double gx, double gy, double gz, double dt);
  // 現在の重力方向からクオータニオンを初期化(ヨー=0)。収束待ちを省くためのシード。
  void seedAttitudeFromAccel(double ax, double ay, double az);

public:
  TFTand9axis_sensor();

  float ref_alt;
  float Alt_offset;

  void applyIMUSettings();
  void begin();

  // p, r は姿勢角 [rad]、pr, rr はジャイロ角速度 [deg/s] を返す。
  // 角度と角速度で単位系が混在しているので、呼び出し側で取り違えないこと。
  void getPitchAndRollAndHeading(double *p, double *r, double *h, double *pr, double *rr,double *yr,double *ax,double *ay,double *az);

  void drawCalStatus(const char *phase, int current = -1, int total = -1);
  void calibrate();
  void magCalibrate();
  void saveAltOffsets(double a1t, double Altitude);
  bool isCalibrated();
  float returnRef_alt();    // NVSに保存されたref_altをmeinkiban3.inoのsetup()で返す
  float returnAlt_offset(); // NVSに保存されたAlt_offsetをmeinkiban3.inoのsetup()で返す
  void updata(float torim, double IAS, double rpm, double ALT);
};
