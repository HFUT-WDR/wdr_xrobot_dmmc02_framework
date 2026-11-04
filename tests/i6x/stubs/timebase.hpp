#pragma once
#include <cstdint>
namespace LibXR
{
struct Timebase
{
  static inline uint32_t now = 0;
  static uint32_t GetMilliseconds() { return now; }
};
}
