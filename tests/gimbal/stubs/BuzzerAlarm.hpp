#pragma once

#include <cstdint>

// 主机回归用的蜂鸣器桩：只满足 Gimbal/BMI088 的校准提示接口，不驱动硬件。
class BuzzerAlarm
{
 public:
  bool active{};
  uint32_t frequency{}, starts{}, stops{};
  float duty{};
  void StartTone(uint32_t freq, float value = 0.005f)
  {
    active = true;
    frequency = freq;
    duty = value;
    ++starts;
  }
  void StopTone()
  {
    active = false;
    ++stops;
  }
  void Play(uint32_t, uint32_t, float = 0.005f) {}
};
