#include <TFT_eSPI.h>
#include <Wire.h>
#include <ICM20948_WE.h>
#include <Preferences.h>
#include <cmath>
#include <cstdio>

class TFTand9axis_sensor
{
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
    Preferences _prefs;

    bool spritesOk = false;
    bool _calibrated = false;
    bool _calibrated = false;
    TFT_eSPI *canvas = nullptr;

    double pitch;
    double roll;
    float ref_alt;

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

    void initMatrices()
    {
        memset(T, 0, sizeof(T));
        memset(Rx, 0, sizeof(Rx));
        memset(Rz, 0, sizeof(Rz));
        T[0][0] = T[1][1] = T[2][2] = T[3][3] = 1.f;
        Rx[0][0] = Rx[1][1] = Rx[2][2] = Rx[3][3] = 1.f;
        Rz[0][0] = Rz[1][1] = Rz[2][2] = Rz[3][3] = 1.f;
    }

    void applyMatrix(double a, double b, double c, double d, double *x, double *y)
    {
        double newX = a * *x + b * *y;
        double newY = c * *x + d * *y;
        *x = newX;
        *y = newY;
    }

    inline TFT_eSPI *activeCanvas() { return canvas ? canvas : &tft; }

    void DrawPixel(int x, int y, uint16_t color)
    {
        TFT_eSPI *c = activeCanvas();
        int hw = c->width() / 2;
        int hh = c->height() / 2;
        c->drawPixel(x + hw, -y + hh, color);
    }

    void DrawLine(int x0, int y0, int x1, int y1, uint16_t color)
    {
        TFT_eSPI *c = activeCanvas();
        int hw = c->width() / 2;
        int hh = c->height() / 2;
        c->drawLine(x0 + hw, -y0 + hh, x1 + hw, -y1 + hh, color);
    }

    void DrawTriangle(int x0, int y0, int x1, int y1, int x2, int y2, uint16_t color)
    {
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

    void topTriinitializer()
    {
        const int size = 20;
        topTri[0].x = 0;
        topTri[0].y = gy_dis;
        topTri[1].x = (float)(size / 2);
        topTri[1].y = gy_dis - (float)size;
        topTri[2].x = (float)(-size / 2);
        topTri[2].y = gy_dis - (float)size;
    }

    void DrawRollArcPolyline(uint16_t rollScaleColor)
    {
        const float R = gy_dis;
        int dar = (int)(gy_dis * sinf(scaleIntervalAngle * (float)tickCount));
        if (dar < 1)
            dar = 1;
        const int N = 48;
        for (int k = 0; k < N; k++)
        {
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

    int DrawFixedGUI(int lineSpacing = 5)
    {
        uint16_t scaleColor = TFT_WHITE;
        DrawLine(-5, 0, 5, 0, scaleColor);

        for (int i = 1; i <= 10; i++)
        {
            if (i % 5 == 0)
                DrawLine(-15, lineSpacing * i, 15, lineSpacing * i, scaleColor);
            else
                DrawLine(-10, lineSpacing * i, 10, lineSpacing * i, scaleColor);
        }
        for (int i = 1; i <= 10; i++)
        {
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

        for (int i = 0; i < tickCount * 2 + 1; i++)
        {
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

    void Drawchar(double torim, double IAS, double rpm, double ALT)
    {
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

    void multiply_matrix(const float A[SIZE][SIZE], const float B[SIZE][SIZE], float C[SIZE][SIZE])
    {
        for (int i = 0; i < SIZE; i++)
        {
            for (int j = 0; j < SIZE; j++)
            {
                C[i][j] = 0.f;
                for (int k = 0; k < SIZE; k++)
                {
                    C[i][j] += A[i][k] * B[k][j];
                }
            }
        }
    }

    void applyMatrix4(const float M4[SIZE][SIZE], Vector *V)
    {
        Vector temp;
        temp.x = M4[0][0] * V->x + M4[0][1] * V->y + M4[0][2] * V->z + M4[0][3] * V->w;
        temp.y = M4[1][0] * V->x + M4[1][1] * V->y + M4[1][2] * V->z + M4[1][3] * V->w;
        temp.z = M4[2][0] * V->x + M4[2][1] * V->y + M4[2][2] * V->z + M4[2][3] * V->w;
        temp.w = M4[3][0] * V->x + M4[3][1] * V->y + M4[3][2] * V->z + M4[3][3] * V->w;
        *V = temp;
    }

    void hlineinitializer()
    {
        for (int i = 0; i < h_num; i++)
        {
            hline[i].a.x = -ground_width / 2.f;
            hline[i].b.x = ground_width / 2.f;
            hline[i].a.y = hline[i].b.y = 0.f;
            hline[i].a.z = hline[i].b.z = (float)(i + 1) * hrs;
            hline[i].a.w = hline[i].b.w = 1.f;
        }
    }

    void vlineinitializer()
    {
        for (int i = 0; i < v_num; i++)
        {
            vline[i].a.x = vline[i].b.x = ((float)i - (float)(v_num / 2)) * vrs;
            vline[i].a.y = vline[i].b.y = 0.f;
            vline[i].a.z = z_max;
            vline[i].b.z = 10.f;
            vline[i].a.w = vline[i].b.w = 1.f;
        }
    }

    void updataLegacy(float torim, double IAS, double rpm, double ALT)
    {
        double h = (ALT + 1.0) * 2.0;
        float Spacing = 10.f;
        float f = 1000.f;
        float theta = (float)(roll * (PI / 180.0));

        for (int i = 0; i < h_num; i++)
        {
            DrawLine(hxy[i][0], hxy[i][1], hxy[i][2], hxy[i][3], TFT_BLACK);
        }
        for (int i = 0; i < v_num; i++)
        {
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
        for (int i = 0; i < 3; i++)
        {
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

        for (int i = 0; i < h_num; i++)
        {
            applyMatrix4(M, &(hline[i].a));
            applyMatrix4(M, &(hline[i].b));
            if (hline[i].a.z > 1.f && hline[i].b.z > 1.f)
            {
                hxy[i][0] = (int)(f * hline[i].a.x / hline[i].a.z);
                hxy[i][1] = (int)(f * hline[i].a.y / hline[i].a.z);
                hxy[i][2] = (int)(f * hline[i].b.x / hline[i].b.z);
                hxy[i][3] = (int)(f * hline[i].b.y / hline[i].b.z);
            }
            else
            {
                hxy[i][0] = hxy[i][1] = hxy[i][2] = hxy[i][3] = 0;
            }
        }

        for (int i = 0; i < v_num; i++)
        {
            applyMatrix4(M, &(vline[i].a));
            applyMatrix4(M, &(vline[i].b));
            if (vline[i].a.z > 0.f && vline[i].b.z > 0.f)
            {
                vxy[i][0] = (int)(f * vline[i].a.x / vline[i].a.z);
                vxy[i][1] = (int)(f * vline[i].a.y / vline[i].a.z);
                vxy[i][2] = (int)(f * vline[i].b.x / vline[i].b.z);
                vxy[i][3] = (int)(f * vline[i].b.y / vline[i].b.z);
            }
        }

        for (int i = 0; i < h_num; i++)
        {
            DrawLine(hxy[i][0], hxy[i][1], hxy[i][2], hxy[i][3], TFT_GREEN);
        }
        for (int i = 0; i < v_num; i++)
        {
            DrawLine(vxy[i][0], vxy[i][1], vxy[i][2], vxy[i][3], TFT_GREEN);
        }

        DrawFixedGUI((int)Spacing);
    }

    void updataSprites(float torim, double IAS, double rpm, double ALT)
    {
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
        for (int i = 0; i < 3; i++)
        {
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

        for (int i = 0; i < h_num; i++)
        {
            applyMatrix4(M, &(hline[i].a));
            applyMatrix4(M, &(hline[i].b));
            if (hline[i].a.z > 1.f && hline[i].b.z > 1.f)
            {
                hxy[i][0] = (int)(f * hline[i].a.x / hline[i].a.z);
                hxy[i][1] = (int)(f * hline[i].a.y / hline[i].a.z);
                hxy[i][2] = (int)(f * hline[i].b.x / hline[i].b.z);
                hxy[i][3] = (int)(f * hline[i].b.y / hline[i].b.z);
            }
            else
            {
                hxy[i][0] = hxy[i][1] = hxy[i][2] = hxy[i][3] = 0;
            }
        }

        for (int i = 0; i < v_num; i++)
        {
            applyMatrix4(M, &(vline[i].a));
            applyMatrix4(M, &(vline[i].b));
            if (vline[i].a.z > 0.f && vline[i].b.z > 0.f)
            {
                vxy[i][0] = (int)(f * vline[i].a.x / vline[i].a.z);
                vxy[i][1] = (int)(f * vline[i].a.y / vline[i].a.z);
                vxy[i][2] = (int)(f * vline[i].b.x / vline[i].b.z);
                vxy[i][3] = (int)(f * vline[i].b.y / vline[i].b.z);
            }
        }

        for (int i = 0; i < h_num; i++)
        {
            DrawLine(hxy[i][0], hxy[i][1], hxy[i][2], hxy[i][3], TFT_GREEN);
        }
        for (int i = 0; i < v_num; i++)
        {
            DrawLine(vxy[i][0], vxy[i][1], vxy[i][2], vxy[i][3], TFT_GREEN);
        }

        canvas = nullptr;
        dynamicSprite.pushSprite(0, 0);
        fixedSprite.pushSprite(0, 0, (uint16_t)TFT_BLACK);
    }

    bool shouldRedrawChar(float torim, double IAS, double rpm, double ALT)
    {
        static bool first = true;
        static unsigned long lastMs = 0;
        static float prevTorim = 0.f;
        static double prevIAS = 0.0, prevRpm = 0.0, prevAlt = 0.0;
        const unsigned long kPeriod = 100;
        unsigned long now = millis();
        if (first)
        {
            first = false;
            lastMs = now;
            prevTorim = torim;
            prevIAS = IAS;
            prevRpm = rpm;
            prevAlt = ALT;
            return true;
        }
        if (now - lastMs >= kPeriod)
        {
            lastMs = now;
            prevTorim = torim;
            prevIAS = IAS;
            prevRpm = rpm;
            prevAlt = ALT;
            return true;
        }
        if (fabsf(torim - prevTorim) > 0.05f || fabs(IAS - prevIAS) > 0.1 ||
            fabs(rpm - prevRpm) > 1.0 || fabs(ALT - prevAlt) > 0.5)
        {
            lastMs = now;
            prevTorim = torim;
            prevIAS = IAS;
            prevRpm = rpm;
            prevAlt = ALT;
            return true;
        }
        return false;
    }

    bool loadOffsets()
    {
        _prefs.begin("imu_cal", true); // 読み取り専用
        bool valid = _prefs.getBool("valid", false);
        if (valid)
        {
            xyzFloat accOfs, gyrOfs;
            accOfs.x = _prefs.getFloat("ax", 0);
            accOfs.y = _prefs.getFloat("ay", 0);
            accOfs.z = _prefs.getFloat("az", 0);
            gyrOfs.x = _prefs.getFloat("gx", 0);
            gyrOfs.y = _prefs.getFloat("gy", 0);
            gyrOfs.z = _prefs.getFloat("gz", 0);
            ref_alt = _prefs.getFloat("ref_alt", 0);
            _prefs.end();
            myIMU.setAccOffsets(accOfs);
            myIMU.setGyrOffsets(gyrOfs);
            return true;
        }
        _prefs.end();
        return false;
    }

    void saveOffsets()
    {
        xyzFloat accOfs = myIMU.getAccOffsets();
        xyzFloat gyrOfs = myIMU.getGyrOffsets();
        _prefs.begin("imu_cal", false); // 書き込みモード
        _prefs.putFloat("ax", accOfs.x);
        _prefs.putFloat("ay", accOfs.y);
        _prefs.putFloat("az", accOfs.z);
        _prefs.putFloat("gx", gyrOfs.x);
        _prefs.putFloat("gy", gyrOfs.y);
        _prefs.putFloat("gz", gyrOfs.z);
        _prefs.putFloat("ref_alt", ref_alt);
        _prefs.putBool("valid", true);
        _prefs.end();
    }

    bool loadOffsets()
    {
        _prefs.begin("imu_cal", true); // 読み取り専用
        bool valid = _prefs.getBool("valid", false);
        if (valid)
        {
            xyzFloat accOfs, gyrOfs;
            accOfs.x = _prefs.getFloat("ax", 0);
            accOfs.y = _prefs.getFloat("ay", 0);
            accOfs.z = _prefs.getFloat("az", 0);
            gyrOfs.x = _prefs.getFloat("gx", 0);
            gyrOfs.y = _prefs.getFloat("gy", 0);
            gyrOfs.z = _prefs.getFloat("gz", 0);
            _prefs.end();
            myIMU.setAccOffsets(accOfs);
            myIMU.setGyrOffsets(gyrOfs);
            return true;
        }
        _prefs.end();
        return false;
    }

    void saveOffsets()
    {
        xyzFloat accOfs = myIMU.getAccOffsets();
        xyzFloat gyrOfs = myIMU.getGyrOffsets();
        _prefs.begin("imu_cal", false); // 書き込みモード
        _prefs.putFloat("ax", accOfs.x);
        _prefs.putFloat("ay", accOfs.y);
        _prefs.putFloat("az", accOfs.z);
        _prefs.putFloat("gx", gyrOfs.x);
        _prefs.putFloat("gy", gyrOfs.y);
        _prefs.putFloat("gz", gyrOfs.z);
        _prefs.putBool("valid", true);
        _prefs.end();
    }

public:
    TFTand9axis_sensor() : tft(), fixedSprite(&tft), dynamicSprite(&tft), myIMU(ICM20948_ADDR) {}

    void applyIMUSettings()
    {
        myIMU.setAccRange(ICM20948_ACC_RANGE_2G);
        myIMU.setAccDLPF(ICM20948_DLPF_6);
        myIMU.setAccSampleRateDivider(10);
    }

    void begin()
    {
        if (!myIMU.init())
        {
            Serial.println("ICM20948 does not respond");
        }
        else
        {
            Serial.println("ICM20948 is connected");
        }

        if (loadOffsets())
        {
            _calibrated = true;
            Serial.println("IMU offsets loaded from NVS");
        }
        else
        {
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
        if (!spritesOk)
        {
            if (fixedSprite.created())
                fixedSprite.deleteSprite();
            if (dynamicSprite.created())
                dynamicSprite.deleteSprite();
        }

        if (spritesOk)
        {
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

    void getPitchAndRoll(double *p, double *r)
    {
        if (!_calibrated)
        {
            *p = 0.0;
            *r = 0.0;
            return;
        }
        myIMU.readSensor();
        pitch = myIMU.getPitch();
        roll = myIMU.getRoll();
        *p = pitch * (PI / 180.0);
        *r = roll * (PI / 180.0);
    }

    void drawCalStatus(const char *phase, int current = -1, int total = -1)
    {
        tft.fillRect(0, tft.height() / 2 - 30, tft.width(), 60, TFT_BLACK);
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.setTextDatum(MC_DATUM);
        tft.setTextSize(2);
        if (current >= 0 && total > 0)
        {
            char buf[40];
            snprintf(buf, sizeof(buf), "%s %d/%d", phase, current, total);
            tft.drawString(buf, tft.width() / 2, tft.height() / 2);
        }
        else
        {
            tft.drawString(phase, tft.width() / 2, tft.height() / 2);
        }
    }

    void calibrate()
    {
        Serial.println("Calibration started...");
        tft.fillScreen(TFT_BLACK);

        // autoOffsets()と同等の設定
        myIMU.setGyrDLPF(ICM20948_DLPF_6);
        myIMU.setGyrRange(ICM20948_GYRO_RANGE_250);
        myIMU.setAccRange(ICM20948_ACC_RANGE_2G);
        myIMU.setAccDLPF(ICM20948_DLPF_6);
        delay(100);

        // Phase 1: 安定化（300回空読み）
        drawCalStatus("[CAL] Stabilizing", 0, 300);
        for (int i = 0; i < 300; i++)
        {
            myIMU.readSensor();
            delay(1);
            if (i % 50 == 0)
                drawCalStatus("[CAL] Stabilizing", i, 300);
        }

        // Phase 2: 加速度サンプリング（1000回）
        xyzFloat accOfs = {0, 0, 0};
        xyzFloat accRaw;
        drawCalStatus("[CAL] Acc sampling", 0, 1000);
        for (int i = 0; i < 1000; i++)
        {
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
        accOfs.z -= 16384.0f; // 重力1G補正

        // Phase 3: ジャイロサンプリング（1000回）
        xyzFloat gyrOfs = {0, 0, 0};
        xyzFloat gyrRaw;
        drawCalStatus("[CAL] Gyr sampling", 0, 1000);
        for (int i = 0; i < 1000; i++)
        {
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
        drawCalStatus("[CAL] Done!");
        Serial.println("Calibration saved to NVS");
        delay(500);                // Done!を視認できるよう少し待つ
        tft.fillScreen(TFT_BLACK); // 通常描画に戻る前に画面クリア
    }

    bool isCalibrated()
    {
        return _calibrated;
    }

    void getRef_alt(double a1t)
    {
        ref_alt = (float)a1t;
    }

    void drawCalStatus(const char *phase, int current = -1, int total = -1)
    {
        tft.fillRect(0, tft.height() / 2 - 30, tft.width(), 60, TFT_BLACK);
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.setTextDatum(MC_DATUM);
        tft.setTextSize(2);
        if (current >= 0 && total > 0)
        {
            char buf[40];
            snprintf(buf, sizeof(buf), "%s %d/%d", phase, current, total);
            tft.drawString(buf, tft.width() / 2, tft.height() / 2);
        }
        else
        {
            tft.drawString(phase, tft.width() / 2, tft.height() / 2);
        }
    }

    void calibrate()
    {
        Serial.println("Calibration started...");
        tft.fillScreen(TFT_BLACK);

        // autoOffsets()と同等の設定
        myIMU.setGyrDLPF(ICM20948_DLPF_6);
        myIMU.setGyrRange(ICM20948_GYRO_RANGE_250);
        myIMU.setAccRange(ICM20948_ACC_RANGE_2G);
        myIMU.setAccDLPF(ICM20948_DLPF_6);
        delay(100);

        // Phase 1: 安定化（300回空読み）
        drawCalStatus("[CAL] Stabilizing", 0, 300);
        for (int i = 0; i < 300; i++)
        {
            myIMU.readSensor();
            delay(1);
            if (i % 50 == 0)
                drawCalStatus("[CAL] Stabilizing", i, 300);
        }

        // Phase 2: 加速度サンプリング（1000回）
        xyzFloat accOfs = {0, 0, 0};
        xyzFloat accRaw;
        drawCalStatus("[CAL] Acc sampling", 0, 1000);
        for (int i = 0; i < 1000; i++)
        {
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
        accOfs.z -= 16384.0f; // 重力1G補正

        // Phase 3: ジャイロサンプリング（1000回）
        xyzFloat gyrOfs = {0, 0, 0};
        xyzFloat gyrRaw;
        drawCalStatus("[CAL] Gyr sampling", 0, 1000);
        for (int i = 0; i < 1000; i++)
        {
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
        drawCalStatus("[CAL] Done!");
        Serial.println("Calibration saved to NVS");
        delay(500);                // Done!を視認できるよう少し待つ
        tft.fillScreen(TFT_BLACK); // 通常描画に戻る前に画面クリア
    }

    bool isCalibrated()
    {
        return _calibrated;
    }

    void updata(float torim, double IAS, double rpm, double ALT)
    {
        if (!_calibrated)
        {
            static bool msgDrawn = false;
            if (!msgDrawn)
            {
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

        if (spritesOk)
        {
            updataSprites(torim, IAS, rpm, ALT);
        }
        else
        {
            canvas = nullptr;
            updataLegacy(torim, IAS, rpm, ALT);
        }

        if (shouldRedrawChar(torim, IAS, rpm, ALT))
        {
            Drawchar(torim, IAS, rpm, ALT);
        }
    }
};