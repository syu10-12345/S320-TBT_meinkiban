#ifndef PID_STATE_H
#define PID_STATE_H

// PID制御の状態
struct PidState {
  float kp, ki, kd;
  float integral;
  float integralMax;
  float lastError;
  unsigned long lastTime;
};

#endif
