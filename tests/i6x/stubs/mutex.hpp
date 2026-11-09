#pragma once
#include <mutex>
namespace LibXR
{
struct Mutex
{
  using LockGuard = std::lock_guard<Mutex>;
  std::mutex mutex;
  void lock() { mutex.lock(); }
  void unlock() { mutex.unlock(); }
};
}
