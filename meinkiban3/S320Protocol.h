#pragma once
// 複製注意: meinkiban3 / soujyuukan_main / OpenLog_3.0 で同一に保つこと (不一致だと sizeof 違いで ESP-NOW パケットが破棄される)
#include <stdint.h>
#pragma pack(push, 1)
struct ControlData {
  uint32_t magic;
  uint8_t role;

  int16_t E_raw_adc, R_raw_adc;
  int16_t ele_param[4];
  int16_t rud_param[4];
  float E_stick_mapped, R_stick_mapped;
  int16_t E_krs,R_krs;
  float E_trim,E_angle, R_angle;
  float e_servo_temp, r_servo_temp;
  bool is_assisted;
  uint32_t ctrl_stk_t;
};
struct NavigationData {
  uint32_t magic;
  uint8_t role;
  float pitch;
  float pitch_rate;
};
struct FullTelemetryPacket {  //メイン基板からロガー C3に送る
  uint32_t magic;
  uint8_t role;
  int16_t E_raw_adc, R_raw_adc;
  int16_t ele_param[4];
  int16_t rud_param[4];
  float E_stick_mapped, R_stick_mapped;
  int16_t E_krs,R_krs;
  float E_trim, E_angle, R_angle;
  float e_servo_temp, r_servo_temp;
  bool is_assisted;
  float pitch;
  float roll;
  float pitch_rate;
  float roll_rate;
  float yaw_rate;
  float ax,ay,az;
  float front_rpm, rear_rpm;
  float air_speed, gnd_speed, Altitude, heading;
  float vel_down;   // GNSS NED 下向き速度 [m/s] (ドップラー由来。上昇率 Vz = -vel_down)
  double lat, lon;
  uint32_t epoch_time;
  uint32_t ctrl_stk_t;
  uint32_t main_bord_t;
  bool electrical_errors[12];
  // ★αβベーン(AS5600 ×2 via TCA9548A) ─ 3基板で同一に保つこと(未同期だと全パケット破棄/#1)
  float    vane_alpha_deg, vane_beta_deg;       // 正規化角[deg](中立基準, 約±180)
  uint16_t vane_alpha_raw, vane_beta_raw;       // 生角[0-4095](診断・脱落検出用)
  uint8_t  vane_alpha_health, vane_beta_health; // bit0:present 1:MD 2:ML 3:MH
  uint8_t  vane_alpha_agc, vane_beta_agc;       // AGC(3.3V:0-128, 中央~64狙い)
  uint32_t vane_t;                              // ベーン取得時刻[ms](信号整列用)
  uint32_t seq;                                 // テレメトリ連番(ESP-NOWロス検出用)
};
#pragma pack(pop)
static const uint8_t WIFI_CHANNEL = 1;
static uint8_t BROADCAST_MAC[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
#define MAGIC 0x53333230u
#define ROLE_MEINKIBAN3 1
#define ROLE_SOUJYUUKAN 2
#define ROLE_LOGGER 3
