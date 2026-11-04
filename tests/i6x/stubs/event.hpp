#pragma once
#include <cstdint>
namespace LibXR
{
template <typename T> struct Callback
{
  template <typename F, typename P> static int Create(F, P) { return 0; }
};
struct Event
{
  void Register(uint32_t, int) {}
  void Active(uint32_t) {}
};
}
