#include "TFTand9axis_sensor.h"

void TFTand9axis_sensor::initMatrices() {
  memset(T, 0, sizeof(T));
  memset(Rx, 0, sizeof(Rx));
  memset(Rz, 0, sizeof(Rz));
  T[0][0] = T[1][1] = T[2][2] = T[3][3] = 1.f;
  Rx[0][0] = Rx[1][1] = Rx[2][2] = Rx[3][3] = 1.f;
  Rz[0][0] = Rz[1][1] = Rz[2][2] = Rz[3][3] = 1.f;
}

void TFTand9axis_sensor::applyMatrix(double a, double b, double c, double d, double *x, double *y) {
  double newX = a * *x + b * *y;
  double newY = c * *x + d * *y;
  *x = newX;
  *y = newY;
}

TFT_eSPI *TFTand9axis_sensor::activeCanvas() {
  return canvas ? canvas : &tft;
}

void TFTand9axis_sensor::DrawPixel(int x, int y, uint16_t color) {
  TFT_eSPI *c = activeCanvas();
  int hw = c->width() / 2;
  int hh = c->height() / 2;
  c->drawPixel(x + hw, -y + hh, color);
}

void TFTand9axis_sensor::DrawLine(int x0, int y0, int x1, int y1, uint16_t color) {
  TFT_eSPI *c = activeCanvas();
  int hw = c->width() / 2;
  int hh = c->height() / 2;
  c->drawLine(x0 + hw, -y0 + hh, x1 + hw, -y1 + hh, color);
}

void TFTand9axis_sensor::DrawTriangle(int x0, int y0, int x1, int y1, int x2, int y2, uint16_t color) {
  TFT_eSPI *c = activeCanvas();
  int hw = c->width() / 2;
  int hh = c->height() / 2;
  x0 += hw;
  y0 = -y0 + hh;
  x1 += hw;
  y1 = -y1 + hh;
  x2 += hw;
  y2 = -y2 + hh;
  c->drawTriangle(x0, y0, x1, y1, x2, y2, color);
}

void TFTand9axis_sensor::topTriinitializer() {
  const int size = 20;
  topTri[0].x = 0;
  topTri[0].y = gy_dis;
  topTri[1].x = (float)(size / 2);
  topTri[1].y = gy_dis - (float)size;
  topTri[2].x = (float)(-size / 2);
  topTri[2].y = gy_dis - (float)size;
}

void TFTand9axis_sensor::DrawRollArcPolyline(uint16_t rollScaleColor) {
  const float R = gy_dis;
  int dar = (int)(gy_dis * sinf(scaleIntervalAngle * (float)tickCount));
  if (dar < 1)
    dar = 1;
  const int N = 48;
  for (int k = 0; k < N; k++) {
    float t0 = -dar + (2.f * (float)dar * (float)k) / (float)N;
    float t1 = -dar + (2.f * (float)dar * (float)(k + 1)) / (float)N;
    float rr0 = R * R - t0 * t0;
    float rr1 = R * R - t1 * t1;
    if (rr0 < 0.f)
      rr0 = 0.f;
    if (rr1 < 0.f)
      rr1 = 0.f;
    int y0 = (int)sqrtf(rr0);
    int y1 = (int)sqrtf(rr1);
    DrawLine((int)t0, y0, (int)t1, y1, rollScaleColor);
  }
}

int TFTand9axis_sensor::DrawFixedGUI(int lineSpacing) {
  uint16_t scaleColor = TFT_WHITE;
  DrawLine(-5, 0, 5, 0, scaleColor);

  for (int i = 1; i <= 10; i++) {
    if (i % 5 == 0)
      DrawLine(-15, lineSpacing * i, 15, lineSpacing * i, scaleColor);
    else
      DrawLine(-10, lineSpacing * i, 10, lineSpacing * i, scaleColor);
  }
  for (int i = 1; i <= 10; i++) {
    if (i % 5 == 0)
      DrawLine(-15, -lineSpacing * i, 15, -lineSpacing * i, scaleColor);
    else
      DrawLine(-10, -lineSpacing * i, 10, -lineSpacing * i, scaleColor);
  }

  uint16_t dir_tri_outline_clr = TFT_WHITE;
  int ax = 13;
  int bx = ax + 48;
  int cx = bx;
  int dx = cx - 19;
  int ay = 0;
  int by = ay - 15;
  int cy = by - 7;
  int dy = cy;
  DrawTriangle(ax, ay, bx, by, dx, dy, dir_tri_outline_clr);
  DrawTriangle(bx, by, cx, cy, dx, dy, dir_tri_outline_clr);
  DrawTriangle(-ax, ay, -bx, by, -dx, dy, dir_tri_outline_clr);
  DrawTriangle(-bx, by, -cx, cy, -dx, dy, dir_tri_outline_clr);

  uint16_t baseTriangleColor = TFT_WHITE;
  int ex = 100;
  int fx = ex + 25;
  int ey = 0;
  int fy = 10;
  DrawTriangle(ex, ey, fx, fy, fx, -fy, baseTriangleColor);
  DrawTriangle(-ex, ey, -fx, fy, -fx, -fy, baseTriangleColor);

  uint16_t rollScaleColor = TFT_WHITE;
  int scaleLength = 10;

  double gx = 0, gy = (double)gy_dis, hx = 0, hy = gy + scaleLength;
  double startAngle = -scaleIntervalAngle * (tickCount + 1);
  applyMatrix(cos(startAngle), -sin(startAngle), sin(startAngle), cos(startAngle), &gx, &gy);
  applyMatrix(cos(startAngle), -sin(startAngle), sin(startAngle), cos(startAngle), &hx, &hy);

  for (int i = 0; i < tickCount * 2 + 1; i++) {
    applyMatrix(cos(scaleIntervalAngle), -sin(scaleIntervalAngle), sin(scaleIntervalAngle), cos(scaleIntervalAngle), &gx, &gy);
    applyMatrix(cos(scaleIntervalAngle), -sin(scaleIntervalAngle), sin(scaleIntervalAngle), cos(scaleIntervalAngle), &hx, &hy);
    DrawLine((int)gx, (int)gy, (int)hx, (int)hy, rollScaleColor);
  }

  DrawRollArcPolyline(rollScaleColor);

  TFT_eSPI *c = activeCanvas();
  c->setTextDatum(BR_DATUM);
  c->setTextSize(2);
  double anglem8 = 47 * PI / 180;
  int x_calc = (int)(((double)gy_dis + 23.0) * cos(anglem8));
  int y_calc = (int)(((double)gy_dis + 23.0) * sin(anglem8));
  int tft_x = x_calc + c->width() / 2;
  int tft_y = -y_calc + c->height() / 2;
  c->drawString("-8", tft_x, tft_y);

  anglem8 = 129 * PI / 180;
  x_calc = (int)(((double)gy_dis + 15.0) * cos(anglem8));
  y_calc = (int)(((double)gy_dis + 15.0) * sin(anglem8));
  tft_x = x_calc + c->width() / 2;
  tft_y = -y_calc + c->height() / 2;
  c->drawString("8", tft_x, tft_y);

  anglem8 = 88 * PI / 180;
  x_calc = (int)(((double)gy_dis + 15.0) * cos(anglem8));
  y_calc = (int)(((double)gy_dis + 15.0) * sin(anglem8));
  tft_x = x_calc + c->width() / 2;
  tft_y = -y_calc + c->height() / 2;
  c->drawString("0", tft_x, tft_y);

  return 0;
}

void TFTand9axis_sensor::Drawchar(double torim, double IAS, double rpm, double ALT) {
  tft.setTextDatum(MR_DATUM);
  tft.setTextSize(3);
  char str[50];
  int x = -25;
  int y = -150;
  int tft_x = x + tft.width() / 2;
  int tft_y = -y + tft.height() / 2;
  snprintf(str, sizeof(str), "TRM:%.1f", torim);
  tft.drawString(str, tft_x, tft_y);

  x = -25;
  y = -150 - 50;
  tft_x = x + tft.width() / 2;
  tft_y = -y + tft.height() / 2;
  snprintf(str, sizeof(str), "IAS:%.1f", IAS);
  tft.drawString(str, tft_x, tft_y);

  x = 145;
  y = -150;
  tft_x = x + tft.width() / 2;
  tft_y = -y + tft.height() / 2;
  snprintf(str, sizeof(str), "RPM:%4.0f", rpm);
  tft.drawString(str, tft_x, tft_y);

  x = 145;
  y = -150 - 50;
  tft_x = x + tft.width() / 2;
  tft_y = -y + tft.height() / 2;
  snprintf(str, sizeof(str), "ALT:%4.1f", ALT);
  tft.drawString(str, tft_x, tft_y);
}

void TFTand9axis_sensor::multiply_matrix(const float A[SIZE][SIZE], const float B[SIZE][SIZE], float C[SIZE][SIZE]) {
  for (int i = 0; i < SIZE; i++) {
    for (int j = 0; j < SIZE; j++) {
      C[i][j] = 0.f;
      for (int k = 0; k < SIZE; k++) {
        C[i][j] += A[i][k] * B[k][j];
      }
    }
  }
}

void TFTand9axis_sensor::applyMatrix4(const float M4[SIZE][SIZE], Vector *V) {
  Vector temp;
  temp.x = M4[0][0] * V->x + M4[0][1] * V->y + M4[0][2] * V->z + M4[0][3] * V->w;
  temp.y = M4[1][0] * V->x + M4[1][1] * V->y + M4[1][2] * V->z + M4[1][3] * V->w;
  temp.z = M4[2][0] * V->x + M4[2][1] * V->y + M4[2][2] * V->z + M4[2][3] * V->w;
  temp.w = M4[3][0] * V->x + M4[3][1] * V->y + M4[3][2] * V->z + M4[3][3] * V->w;
  *V = temp;
}

void TFTand9axis_sensor::hlineinitializer() {
  for (int i = 0; i < h_num; i++) {
    hline[i].a.x = -ground_width / 2.f;
    hline[i].b.x = ground_width / 2.f;
    hline[i].a.y = hline[i].b.y = 0.f;
    hline[i].a.z = hline[i].b.z = (float)(i + 1) * hrs;
    hline[i].a.w = hline[i].b.w = 1.f;
  }
}

void TFTand9axis_sensor::vlineinitializer() {
  for (int i = 0; i < v_num; i++) {
    vline[i].a.x = vline[i].b.x = ((float)i - (float)(v_num / 2)) * vrs;
    vline[i].a.y = vline[i].b.y = 0.f;
    vline[i].a.z = z_max;
    vline[i].b.z = 10.f;
    vline[i].a.w = vline[i].b.w = 1.f;
  }
}

void TFTand9axis_sensor::updataLegacy(float torim, double IAS, double rpm, double ALT) {
  double h = (ALT + 1.0) * 2.0;
  float Spacing = 10.f;
  float f = 1000.f;
  float theta = (float)(roll * (PI / 180.0));

  for (int i = 0; i < h_num; i++) {
    DrawLine(hxy[i][0], hxy[i][1], hxy[i][2], hxy[i][3], TFT_BLACK);
  }
  for (int i = 0; i < v_num; i++) {
    DrawLine(vxy[i][0], vxy[i][1], vxy[i][2], vxy[i][3], TFT_BLACK);
  }

  hlineinitializer();
  vlineinitializer();

  float triAngle = -theta * scaleIntervalAngle * (float)(180.0 / PI);
  Rz_tri[0][0] = cosf(triAngle);
  Rz_tri[0][1] = -sinf(triAngle);
  Rz_tri[1][0] = sinf(triAngle);
  Rz_tri[1][1] = cosf(triAngle);
  DrawTriangle(topTri[0].x, topTri[0].y, topTri[1].x, topTri[1].y, topTri[2].x, topTri[2].y, TFT_BLACK);
  topTriinitializer();
  for (int i = 0; i < 3; i++) {
    float px = topTri[i].x, py = topTri[i].y;
    float nx = Rz_tri[0][0] * px + Rz_tri[0][1] * py;
    float ny = Rz_tri[1][0] * px + Rz_tri[1][1] * py;
    topTri[i].x = nx;
    topTri[i].y = ny;
  }
  DrawTriangle(topTri[0].x, topTri[0].y, topTri[1].x, topTri[1].y, topTri[2].x, topTri[2].y, TFT_GREEN);

  float Spfai = -Spacing * (float)pitch;
  float numerator = -(Spfai * z_max + (float)h * f * cosf(theta));
  float denominator = z_max * f * cosf(theta) - Spfai * (float)h;
  float phi = atan2f(numerator, denominator);

  memset(T, 0, sizeof(T));
  T[0][0] = T[1][1] = T[2][2] = T[3][3] = 1.f;
  T[1][3] = -(float)h;

  memset(Rx, 0, sizeof(Rx));
  Rx[0][0] = Rx[3][3] = 1.f;
  Rx[1][1] = cosf(-phi);
  Rx[1][2] = sinf(-phi);
  Rx[2][1] = -sinf(-phi);
  Rx[2][2] = cosf(-phi);

  memset(Rz, 0, sizeof(Rz));
  Rz[2][2] = Rz[3][3] = 1.f;
  Rz[0][0] = cosf(-theta);
  Rz[1][0] = sinf(-theta);
  Rz[0][1] = -sinf(-theta);
  Rz[1][1] = cosf(-theta);

  multiply_matrix(Rz, Rx, M_temp);
  multiply_matrix(M_temp, T, M);

  for (int i = 0; i < h_num; i++) {
    applyMatrix4(M, &(hline[i].a));
    applyMatrix4(M, &(hline[i].b));
    if (hline[i].a.z > 1.f && hline[i].b.z > 1.f) {
      hxy[i][0] = (int)(f * hline[i].a.x / hline[i].a.z);
      hxy[i][1] = (int)(f * hline[i].a.y / hline[i].a.z);
      hxy[i][2] = (int)(f * hline[i].b.x / hline[i].b.z);
      hxy[i][3] = (int)(f * hline[i].b.y / hline[i].b.z);
    } else {
      hxy[i][0] = hxy[i][1] = hxy[i][2] = hxy[i][3] = 0;
    }
  }

  for (int i = 0; i < v_num; i++) {
    applyMatrix4(M, &(vline[i].a));
    applyMatrix4(M, &(vline[i].b));
    if (vline[i].a.z > 0.f && vline[i].b.z > 0.f) {
      vxy[i][0] = (int)(f * vline[i].a.x / vline[i].a.z);
      vxy[i][1] = (int)(f * vline[i].a.y / vline[i].a.z);
      vxy[i][2] = (int)(f * vline[i].b.x / vline[i].b.z);
      vxy[i][3] = (int)(f * vline[i].b.y / vline[i].b.z);
    }
  }

  for (int i = 0; i < h_num; i++) {
    DrawLine(hxy[i][0], hxy[i][1], hxy[i][2], hxy[i][3], TFT_GREEN);
  }
  for (int i = 0; i < v_num; i++) {
    DrawLine(vxy[i][0], vxy[i][1], vxy[i][2], vxy[i][3], TFT_GREEN);
  }

  DrawFixedGUI((int)Spacing);
}

void TFTand9axis_sensor::updataSprites(float torim, double IAS, double rpm, double ALT) {
  double h = (ALT + 1.0) * 2.0;
  float Spacing = 10.f;
  float f = 1000.f;
  float theta = (float)(roll * (PI / 180.0));

  canvas = &dynamicSprite;
  dynamicSprite.fillSprite(TFT_BLACK);

  hlineinitializer();
  vlineinitializer();

  float triAngle = -theta * scaleIntervalAngle * (float)(180.0 / PI);
  Rz_tri[0][0] = cosf(triAngle);
  Rz_tri[0][1] = -sinf(triAngle);
  Rz_tri[1][0] = sinf(triAngle);
  Rz_tri[1][1] = cosf(triAngle);
  topTriinitializer();
  for (int i = 0; i < 3; i++) {
    float px = topTri[i].x, py = topTri[i].y;
    float nx = Rz_tri[0][0] * px + Rz_tri[0][1] * py;
    float ny = Rz_tri[1][0] * px + Rz_tri[1][1] * py;
    topTri[i].x = nx;
    topTri[i].y = ny;
  }
  DrawTriangle(topTri[0].x, topTri[0].y, topTri[1].x, topTri[1].y, topTri[2].x, topTri[2].y, TFT_GREEN);

  float Spfai = -Spacing * (float)pitch;
  float numerator = -(Spfai * z_max + (float)h * f * cosf(theta));
  float denominator = z_max * f * cosf(theta) - Spfai * (float)h;
  float phi = atan2f(numerator, denominator);

  memset(T, 0, sizeof(T));
  T[0][0] = T[1][1] = T[2][2] = T[3][3] = 1.f;
  T[1][3] = -(float)h;

  memset(Rx, 0, sizeof(Rx));
  Rx[0][0] = Rx[3][3] = 1.f;
  Rx[1][1] = cosf(-phi);
  Rx[1][2] = sinf(-phi);
  Rx[2][1] = -sinf(-phi);
  Rx[2][2] = cosf(-phi);

  memset(Rz, 0, sizeof(Rz));
  Rz[2][2] = Rz[3][3] = 1.f;
  Rz[0][0] = cosf(-theta);
  Rz[1][0] = sinf(-theta);
  Rz[0][1] = -sinf(-theta);
  Rz[1][1] = cosf(-theta);

  multiply_matrix(Rz, Rx, M_temp);
  multiply_matrix(M_temp, T, M);

  for (int i = 0; i < h_num; i++) {
    applyMatrix4(M, &(hline[i].a));
    applyMatrix4(M, &(hline[i].b));
    if (hline[i].a.z > 1.f && hline[i].b.z > 1.f) {
      hxy[i][0] = (int)(f * hline[i].a.x / hline[i].a.z);
      hxy[i][1] = (int)(f * hline[i].a.y / hline[i].a.z);
      hxy[i][2] = (int)(f * hline[i].b.x / hline[i].b.z);
      hxy[i][3] = (int)(f * hline[i].b.y / hline[i].b.z);
    } else {
      hxy[i][0] = hxy[i][1] = hxy[i][2] = hxy[i][3] = 0;
    }
  }

  for (int i = 0; i < v_num; i++) {
    applyMatrix4(M, &(vline[i].a));
    applyMatrix4(M, &(vline[i].b));
    if (vline[i].a.z > 0.f && vline[i].b.z > 0.f) {
      vxy[i][0] = (int)(f * vline[i].a.x / vline[i].a.z);
      vxy[i][1] = (int)(f * vline[i].a.y / vline[i].a.z);
      vxy[i][2] = (int)(f * vline[i].b.x / vline[i].b.z);
      vxy[i][3] = (int)(f * vline[i].b.y / vline[i].b.z);
    }
  }

  for (int i = 0; i < h_num; i++) {
    DrawLine(hxy[i][0], hxy[i][1], hxy[i][2], hxy[i][3], TFT_GREEN);
  }
  for (int i = 0; i < v_num; i++) {
    DrawLine(vxy[i][0], vxy[i][1], vxy[i][2], vxy[i][3], TFT_GREEN);
  }

  canvas = nullptr;
  dynamicSprite.pushSprite(0, 0);
  fixedSprite.pushSprite(0, 0, (uint16_t)TFT_BLACK);
}

bool TFTand9axis_sensor::shouldRedrawChar(float torim, double IAS, double rpm, double ALT) {
  static bool first = true;
  static unsigned long lastMs = 0;
  static float prevTorim = 0.f;
  static double prevIAS = 0.0, prevRpm = 0.0, prevAlt = 0.0;
  const unsigned long kPeriod = 100;
  unsigned long now = millis();
  if (first) {
    first = false;
    lastMs = now;
    prevTorim = torim;
    prevIAS = IAS;
    prevRpm = rpm;
    prevAlt = ALT;
    return true;
  }
  if (now - lastMs >= kPeriod) {
    lastMs = now;
    prevTorim = torim;
    prevIAS = IAS;
    prevRpm = rpm;
    prevAlt = ALT;
    return true;
  }
  if (fabsf(torim - prevTorim) > 0.05f || fabs(IAS - prevIAS) > 0.1 || fabs(rpm - prevRpm) > 1.0 || fabs(ALT - prevAlt) > 0.5) {
    lastMs = now;
    prevTorim = torim;
    prevIAS = IAS;
    prevRpm = rpm;
    prevAlt = ALT;
    return true;
  }
  return false;
}

bool TFTand9axis_sensor::loadOffsets() {
  _prefs.begin("imu_cal", true);  // 読み取り専用
  bool valid = _prefs.getBool("valid", false);
  if (valid) {
    xyzFloat accOfs, gyrOfs;
    accOfs.x = _prefs.getFloat("ax", 0);
    accOfs.y = _prefs.getFloat("ay", 0);
    accOfs.z = _prefs.getFloat("az", 0);
    gyrOfs.x = _prefs.getFloat("gx", 0);
    gyrOfs.y = _prefs.getFloat("gy", 0);
    gyrOfs.z = _prefs.getFloat("gz", 0);
    magOfs.x = _prefs.getFloat("mx", 0);
    magOfs.y = _prefs.getFloat("my", 0);
    magOfs.z = _prefs.getFloat("mz", 0);
    ref_alt = _prefs.getFloat("ref_alt", 0);
    Alt_offset = _prefs.getFloat("Alt_offset",0);
    _prefs.end();
    myIMU.setAccOffsets(accOfs);
    myIMU.setGyrOffsets(gyrOfs);
    return true;
  }
  _prefs.end();
  return false;
}

void TFTand9axis_sensor::saveOffsets() {
  xyzFloat accOfs = myIMU.getAccOffsets();
  xyzFloat gyrOfs = myIMU.getGyrOffsets();
  _prefs.begin("imu_cal", false);  // 書き込みモード
  _prefs.putFloat("ax", accOfs.x);
  _prefs.putFloat("ay", accOfs.y);
  _prefs.putFloat("az", accOfs.z);
  _prefs.putFloat("gx", gyrOfs.x);
  _prefs.putFloat("gy", gyrOfs.y);
  _prefs.putFloat("gz", gyrOfs.z);
  _prefs.putFloat("mx", magOfs.x);
  _prefs.putFloat("my", magOfs.y);
  _prefs.putFloat("mz", magOfs.z);
  _prefs.putFloat("ref_alt", ref_alt);
  _prefs.putFloat("Alt_offset",Alt_offset);
  _prefs.putBool("valid", true);
  _prefs.end();
}

TFTand9axis_sensor::TFTand9axis_sensor()
  : tft(), fixedSprite(&tft), dynamicSprite(&tft), myIMU(ICM20948_ADDR) {}

void TFTand9axis_sensor::applyIMUSettings() {
  myIMU.setAccRange(ICM20948_ACC_RANGE_2G);
  myIMU.setAccDLPF(ICM20948_DLPF_6);
  myIMU.setAccSampleRateDivider(10);
  // Madgwick はジャイロを rad/s で使う。getGyrValues のスケール係数(gyrRangeFactor)は
  // setGyrRange で確定するため、NVS読込のみで起動する経路でも必ず設定しておく。
  myIMU.setGyrRange(ICM20948_GYRO_RANGE_250);
  myIMU.setGyrDLPF(ICM20948_DLPF_6);
  myIMU.setGyrSampleRateDivider(10);
}

void TFTand9axis_sensor::begin() {
  if (!myIMU.init()) {
    Serial.println("ICM20948 does not respond");
  } else {
    Serial.println("ICM20948 is connected");
  }

  myIMU.initMagnetometer();
  myIMU.setMagOpMode(AK09916_CONT_MODE_100HZ); // 地磁気を1秒間に20回連続計測するモード

  if (loadOffsets()) {
    _calibrated = true;
    Serial.println("IMU offsets loaded from NVS");
  } else {
    _calibrated = false;
    Serial.println("No calibration data. Press button to calibrate.");
  }

  applyIMUSettings();

  tft.begin();
  tft.setRotation(2);

  tft.fillScreen(TFT_BLACK);

  int w = tft.width();
  int h = tft.height();
  fixedSprite.setColorDepth(16);
  dynamicSprite.setColorDepth(16);
  fixedSprite.createSprite(w, h);
  dynamicSprite.createSprite(w, h);
  spritesOk = fixedSprite.created() && dynamicSprite.created();
  if (!spritesOk) {
    if (fixedSprite.created())
      fixedSprite.deleteSprite();
    if (dynamicSprite.created())
      dynamicSprite.deleteSprite();
  }

  if (spritesOk) {
    fixedSprite.setRotation(2);
    dynamicSprite.setRotation(2);
    fixedSprite.fillSprite(TFT_BLACK);
    canvas = &fixedSprite;
    DrawFixedGUI(10);
    canvas = nullptr;
  }

  initMatrices();
  topTriinitializer();
}

// p, r は姿勢角 [rad]、pr, rr はジャイロ角速度 [deg/s] を返す。
// 角度と角速度で単位系が混在しているので、呼び出し側で取り違えないこと。
//
// 姿勢角(pitch/roll)は Madgwick フィルタ(加速度+ジャイロのフュージョン)で算出する。
// 旧来の myIMU.getPitch()/getRoll()(加速度のみ)は廃止した。
// ※ pitch/roll の符号は従来(getPitch/getRoll)と同じ(機首上げ=負)。下流のPID(Kp=-1.0で
//   反転を補償)がこの符号前提のため意図的に維持している。AGENTS.md「符号規約」参照。
// ※ heading は従来どおり地磁気の傾き補正で算出(Madgwick のヨーは使わない)。
void TFTand9axis_sensor::getPitchAndRollAndHeading(double *p, double *r, double *h, double *pr, double *rr,double *yr,double *ax,double *ay,double *az) {
  if (!_calibrated) {
    *p = 0.0;
    *r = 0.0;
    *h = 0.0;
    *pr = 0.0;
    *rr = 0.0;
    *yr = 0;
    return;
  }

  myIMU.readSensor();

  xyzFloat gVal;
  myIMU.getGValues(&gVal);      // 加速度 [g] (キャリブ済みオフセット適用後)
  *ax=gVal.x;*ay=gVal.y;*az=gVal.z;
  
  xyzFloat gyrVal;
  myIMU.getGyrValues(&gyrVal);  // ジャイロ [deg/s]

  // --- dt 計測 [s] (micros はアンダーフロー安全な符号なし差分) ---
  unsigned long nowUs = micros();
  double dt = (_lastAttUs == 0) ? 0.02 : (double)(nowUs - _lastAttUs) / 1000000.0;
  _lastAttUs = nowUs;
  if (dt <= 0.0 || dt > 0.1) dt = 0.02;  // 初回・タスク停止後などの異常値を保護

  // --- Madgwick 更新 ---
  if (!_attInit) {
    // 起動直後/キャリブ直後は現在の重力方向でクオータニオンをシードして即収束させる
    seedAttitudeFromAccel(gVal.x, gVal.y, gVal.z);
    _attInit = true;
  } else {
    // ジャイロは rad/s に変換して渡す
    madgwickUpdate(gVal.x, gVal.y, gVal.z,
                   gyrVal.x * DEG_TO_RAD, gyrVal.y * DEG_TO_RAD, gyrVal.z * DEG_TO_RAD,
                   dt);
  }

  // --- クオータニオン → ピッチ/ロール ---
  // pitch : Y軸まわり = asin(2(wy-zx)) = asin(-gx) … 従来 getPitch と同符号
  // roll  : X軸まわり = atan2(2(wx+yz), 1-2(x^2+y^2)) … 従来 getRoll と同符号
  double sinp = 2.0 * (mq_w * mq_y - mq_z * mq_x);
  if (sinp > 1.0) sinp = 1.0;
  else if (sinp < -1.0) sinp = -1.0;
  double pitchRad = asin(sinp);
  double rollRad = atan2(2.0 * (mq_w * mq_x + mq_y * mq_z),
                         1.0 - 2.0 * (mq_x * mq_x + mq_y * mq_y));

  pitch = pitchRad * (180.0 / PI);  // 表示用メンバ [deg]
  roll = rollRad * (180.0 / PI);

  *p = pitchRad;
  *r = rollRad;

  // ジャイロ角速度 [deg/s] は生値のまま返す (PID の D項が直接使用するため加工しない)
  *pr = gyrVal.y;  // ピッチ軸角速度 [deg/s]
  *rr = gyrVal.x;  // ロール軸角速度 [deg/s]
  *yr = gyrVal.z;

  // --- 地磁気による方位(傾き補正付き) ---
  xyzFloat magVal;
  myIMU.getMagValues(&magVal);

  // 地磁気のハードアイアン補正(magCalibrate で求めたオフセットを引く)
  magVal.x -= magOfs.x;
  magVal.y -= magOfs.y;
  magVal.z -= magOfs.z;

  double Xh = magVal.x * cos(pitchRad) + magVal.y * sin(rollRad) * sin(pitchRad) + magVal.z * cos(rollRad) * sin(pitchRad);
  double Yh = magVal.y * cos(rollRad) - magVal.z * sin(rollRad);

  // アークタンジェントで方位角(弧度法)を計算
  heading = atan2(Yh, Xh) * (180.0 / PI) + 80;

  if (heading < 0) {
    heading += 360.0;  //出力を0°から360°に制限
  } else if (heading >= 360.0) {
    heading -= 360.0;
  }

  *h = heading * (PI / 180.0);
}

// ICM20948_minimal の Madgwick 実装を移植。メンバ mq_w/x/y/z(姿勢クオータニオン)を更新する。
// 勾配項の符号は minimal で検証済みのもの(誤りを修正済み)。
void TFTand9axis_sensor::madgwickUpdate(double ax, double ay, double az,
                                        double gx, double gy, double gz, double dt) {
  double w = mq_w, x = mq_x, y = mq_y, z = mq_z;
  const double beta = 0.1;  // フィルタゲイン(大=加速度信頼/速い, 小=ジャイロ信頼/滑らか。振動時は0.04前後)

  // ジャイロによるクオータニオン速度 q_dot = 0.5 * q ⊗ ω
  double qdW = 0.5 * (-x * gx - y * gy - z * gz);
  double qdX = 0.5 * (w * gx + y * gz - z * gy);
  double qdY = 0.5 * (w * gy - x * gz + z * gx);
  double qdZ = 0.5 * (w * gz + x * gy - y * gx);

  // 加速度が有効なときのみ重力方向への補正(勾配降下)を加える
  double anorm = sqrt(ax * ax + ay * ay + az * az);
  if (anorm > 1e-9) {
    ax /= anorm;
    ay /= anorm;
    az /= anorm;

    // 目的関数 f = (推定重力方向 in body) - (実測重力方向)
    double fx = 2.0 * (x * z - w * y) - ax;
    double fy = 2.0 * (y * z + w * x) - ay;
    double fz = 1.0 - 2.0 * (x * x + y * y) - az;

    // 勾配 (J^T f)
    double gradW = -2.0 * y * fx + 2.0 * x * fy;
    double gradX = 2.0 * z * fx + 2.0 * w * fy - 4.0 * x * fz;
    double gradY = -2.0 * w * fx + 2.0 * z * fy - 4.0 * y * fz;
    double gradZ = 2.0 * x * fx + 2.0 * y * fy;

    // 勾配の正規化
    double gnorm = sqrt(gradW * gradW + gradX * gradX + gradY * gradY + gradZ * gradZ);
    if (gnorm > 1e-9) {
      qdW -= beta * (gradW / gnorm);
      qdX -= beta * (gradX / gnorm);
      qdY -= beta * (gradY / gnorm);
      qdZ -= beta * (gradZ / gnorm);
    }
  }

  // 積分
  w += qdW * dt;
  x += qdX * dt;
  y += qdY * dt;
  z += qdZ * dt;

  // クオータニオン正規化
  double qnorm = sqrt(w * w + x * x + y * y + z * z);
  if (qnorm < 1e-9) qnorm = 1e-9;
  mq_w = w / qnorm;
  mq_x = x / qnorm;
  mq_y = y / qnorm;
  mq_z = z / qnorm;
}

// 現在の重力方向からクオータニオンを初期化する(ヨー=0)。
// 起動直後・キャリブ直後に呼び、Madgwick の収束待ち(数秒)を省いて即正しい姿勢から始める。
void TFTand9axis_sensor::seedAttitudeFromAccel(double ax, double ay, double az) {
  double anorm = sqrt(ax * ax + ay * ay + az * az);
  if (anorm < 1e-9) {
    mq_w = 1.0;
    mq_x = 0.0;
    mq_y = 0.0;
    mq_z = 0.0;
    return;
  }
  ax /= anorm;
  ay /= anorm;
  az /= anorm;

  // 重力方向 → ロール/ピッチ (上の抽出式と同じ定義)
  double roll0 = atan2(ay, az);
  double sp = -ax;
  if (sp > 1.0) sp = 1.0;
  else if (sp < -1.0) sp = -1.0;
  double pitch0 = asin(sp);

  // オイラー(roll0, pitch0, yaw=0) → クオータニオン (ZYX)
  double cr = cos(roll0 * 0.5), sr = sin(roll0 * 0.5);
  double cp = cos(pitch0 * 0.5), sptmp = sin(pitch0 * 0.5);
  mq_w = cr * cp;
  mq_x = sr * cp;
  mq_y = cr * sptmp;
  mq_z = -sr * sptmp;

  double qnorm = sqrt(mq_w * mq_w + mq_x * mq_x + mq_y * mq_y + mq_z * mq_z);
  if (qnorm < 1e-9) qnorm = 1e-9;
  mq_w /= qnorm;
  mq_x /= qnorm;
  mq_y /= qnorm;
  mq_z /= qnorm;
}

void TFTand9axis_sensor::drawCalStatus(const char *phase, int current, int total) {
  tft.fillRect(0, tft.height() / 2 - 30, tft.width(), 60, TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.setTextSize(2);
  if (current >= 0 && total > 0) {
    char buf[40];
    snprintf(buf, sizeof(buf), "%s %d/%d", phase, current, total);
    tft.drawString(buf, tft.width() / 2, tft.height() / 2);
  } else {
    tft.drawString(phase, tft.width() / 2, tft.height() / 2);
  }
}

void TFTand9axis_sensor::calibrate() {
  Serial.println("Calibration started...");
  tft.fillScreen(TFT_BLACK);
  loadOffsets();
  // autoOffsets()と同等の設定
  myIMU.setGyrDLPF(ICM20948_DLPF_6);
  myIMU.setGyrRange(ICM20948_GYRO_RANGE_250);
  myIMU.setAccRange(ICM20948_ACC_RANGE_2G);
  myIMU.setAccDLPF(ICM20948_DLPF_6);
  delay(100);

  // Phase 1: 安定化（300回空読み）
  drawCalStatus("[CAL] Stabilizing", 0, 300);
  for (int i = 0; i < 300; i++) {
    myIMU.readSensor();
    delay(1);
    if (i % 50 == 0)
      drawCalStatus("[CAL] Stabilizing", i, 300);
  }

  // Phase 2: 加速度サンプリング（1000回）
  xyzFloat accOfs = { 0, 0, 0 };
  xyzFloat accRaw;
  drawCalStatus("[CAL] Acc sampling", 0, 1000);
  for (int i = 0; i < 1000; i++) {
    myIMU.readSensor();
    myIMU.getAccRawValues(&accRaw);
    accOfs.x += accRaw.x;
    accOfs.y += accRaw.y;
    accOfs.z += accRaw.z;
    delay(1);
    if (i % 50 == 0)
      drawCalStatus("[CAL] Acc sampling", i, 1000);
  }
  accOfs.x /= 1000.0f;
  accOfs.y /= 1000.0f;
  accOfs.z /= 1000.0f;
  accOfs.z -= 16384.0f;  // 重力1G補正

  // Phase 3: ジャイロサンプリング（1000回）
  xyzFloat gyrOfs = { 0, 0, 0 };
  xyzFloat gyrRaw;
  drawCalStatus("[CAL] Gyr sampling", 0, 1000);
  for (int i = 0; i < 1000; i++) {
    myIMU.readSensor();
    myIMU.getGyrRawValues(&gyrRaw);
    gyrOfs.x += gyrRaw.x;
    gyrOfs.y += gyrRaw.y;
    gyrOfs.z += gyrRaw.z;
    delay(1);
    if (i % 50 == 0)
      drawCalStatus("[CAL] Gyr sampling", i, 1000);
  }
  gyrOfs.x /= 1000.0f;
  gyrOfs.y /= 1000.0f;
  gyrOfs.z /= 1000.0f;

  // Phase 4: オフセット適用
  myIMU.setAccOffsets(accOfs);
  myIMU.setGyrOffsets(gyrOfs);

  // Phase 5: NVS保存
  drawCalStatus("[CAL] Saving NVS...");
  saveOffsets();

  // 完了
  applyIMUSettings();
  _calibrated = true;
  _attInit = false;  // 次回の姿勢取得で加速度からクオータニオンを再シードする
  _lastAttUs = 0;    // dt をリセット(キャリブ中の長い経過時間を持ち越さない)
  drawCalStatus("[CAL] Done!");
  Serial.println("Calibration saved to NVS");
  delay(500);                 // Done!を視認できるよう少し待つ
  tft.fillScreen(TFT_BLACK);  // 通常描画に戻る前に画面クリア
}

void TFTand9axis_sensor::magCalibrate(){
  Serial.println("Calibration started...");
  tft.fillScreen(TFT_BLACK);

  loadOffsets();
  // autoOffsets()と同等の設定
  myIMU.setGyrDLPF(ICM20948_DLPF_6);
  myIMU.setGyrRange(ICM20948_GYRO_RANGE_250);
  myIMU.setAccRange(ICM20948_ACC_RANGE_2G);
  myIMU.setAccDLPF(ICM20948_DLPF_6);
  delay(100);
  // ▼▼▼ ここから Phase 3.5 として地磁気キャリブレーションを追加 ▼▼▼
  drawCalStatus("[CAL] Mag: Wave in 8-shape", 0, 15);
  float m_min_x = 30000.0f, m_max_x = -30000.0f;
  float m_min_y = 30000.0f, m_max_y = -30000.0f;
  float m_min_z = 30000.0f, m_max_z = -30000.0f;

  unsigned long magStart = millis();
  int lastSec = 0;

  // 15秒間ループして最大値と最小値を探す
  while(millis() - magStart < 15000) {
    myIMU.readSensor();
    xyzFloat mRaw;
    myIMU.getMagValues(&mRaw);

    // データが取得できている場合のみ更新
    if(mRaw.x != 0.0f || mRaw.y != 0.0f) {
      if(mRaw.x < m_min_x) m_min_x = mRaw.x;
      if(mRaw.x > m_max_x) m_max_x = mRaw.x;
      if(mRaw.y < m_min_y) m_min_y = mRaw.y;
      if(mRaw.y > m_max_y) m_max_y = mRaw.y;
      if(mRaw.z < m_min_z) m_min_z = mRaw.z;
      if(mRaw.z > m_max_z) m_max_z = mRaw.z;
    }

    // 画面のカウントダウン更新
    int sec = (millis() - magStart) / 1000;
    if (sec != lastSec) {
      lastSec = sec;
      drawCalStatus("[CAL] Mag: Wave in 8-shape", sec, 15);
    }
    delay(10);
  }

  // 最大値と最小値の中間をオフセットとする
  magOfs.x = (m_max_x + m_min_x) / 2.0f;
  magOfs.y = (m_max_y + m_min_y) / 2.0f;
  magOfs.z = (m_max_z + m_min_z) / 2.0f;

  // ▲▲▲ ここまで追加 ▲▲▲
  drawCalStatus("[CAL] Saving NVS...");
  saveOffsets();

  applyIMUSettings();
  _calibrated = true;
  _attInit = false;  // 次回の姿勢取得で加速度からクオータニオンを再シードする
  _lastAttUs = 0;
  drawCalStatus("[CAL] Done!");
  Serial.println("Calibration saved to NVS");
  delay(500);                 // Done!を視認できるよう少し待つ
  tft.fillScreen(TFT_BLACK);  // 通常描画に戻る前に画面クリア
}

void TFTand9axis_sensor::saveAltOffsets(double a1t, double Altitude) {
  ref_alt = (float)a1t;
  Alt_offset = (float)Altitude;

  // 他の設定値（RAMに保持されているIMUのオフセット等）と一緒にNVSに書き込む
  saveOffsets();
}

bool TFTand9axis_sensor::isCalibrated() {
  return _calibrated;
}

float TFTand9axis_sensor::returnRef_alt() {   //NVSに保存されたref_altをmeinkiban3.inoのsetup()で返す
  return ref_alt;
}

float TFTand9axis_sensor::returnAlt_offset() { //NVSに保存されたAlt_offsetをmeinkiban3.inoのsetup()で返す
  return Alt_offset;
}

void TFTand9axis_sensor::updata(float torim, double IAS, double rpm, double ALT) {
  if (!_calibrated) {
    static bool msgDrawn = false;
    if (!msgDrawn) {
      tft.fillScreen(TFT_BLACK);
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
      tft.setTextDatum(MC_DATUM);
      tft.setTextSize(2);
      tft.drawString("No Calibration", tft.width() / 2, tft.height() / 2 - 15);
      tft.drawString("Push Button", tft.width() / 2, tft.height() / 2 + 15);
      msgDrawn = true;
    }
    return;
  }

  if (spritesOk) {
    updataSprites(torim, IAS, rpm, ALT);
  } else {
    canvas = nullptr;
    updataLegacy(torim, IAS, rpm, ALT);
  }

  if (shouldRedrawChar(torim, IAS, rpm, ALT)) {
    Drawchar(torim, IAS, rpm, ALT);
  }
}
