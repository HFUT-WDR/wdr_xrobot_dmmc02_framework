#pragma once
#include <cstdint>
#include "imu.h"
namespace wdr_imu
{
struct Attitude
{
  float quaternion[4]{1, 0, 0, 0};
  float yaw{}, pitch{}, roll{}, gyro[3]{}, accel[3]{};
  uint64_t time_us{};
  uint32_t sequence{};
};
struct BoardState : Attitude
{
  float yaw_total{}, motion_accel_b[3]{}, motion_accel_n[3]{};
};
}  // namespace wdr_imu
