#pragma once
namespace LibXR
{
struct Thread
{
  enum class Priority { HIGH };
  template <typename T, typename F> void Create(T*, F, const char*, int, Priority) {}
};
}
