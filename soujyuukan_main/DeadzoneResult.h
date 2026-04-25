#ifndef DEADZONE_RESULT_H
#define DEADZONE_RESULT_H

struct DeadzoneResult {
  float mappedValue;  // デッドゾーン処理後の値
  bool isCenter;      // 現在中央（デッドゾーン内）にいるかどうかの判定
};

DeadzoneResult detzoneMapping(int *ary, int x, float min, float med, float max);

#endif
