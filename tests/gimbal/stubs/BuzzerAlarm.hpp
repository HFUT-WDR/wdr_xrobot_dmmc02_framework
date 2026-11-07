#pragma once

#include <cstdint>

// 主机回归用的蜂鸣器桩：只满足 Gimbal/BMI088 的校准提示接口，不驱动硬件。
class BuzzerAlarm
{
 public:
  void StartTone(uint32_t, float = 0.005f) {}
  void StopTone() {}
  void Play(uint32_t, uint32_t, float = 0.005f) {}
};
