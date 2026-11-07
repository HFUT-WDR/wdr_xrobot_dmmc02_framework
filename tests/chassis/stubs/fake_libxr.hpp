#pragma once
#include <any>
#include <cassert>
#include <cstdint>
#include <cstring>
#include <deque>
#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <vector>
#define REQUIRE(x) assert(x)
#define ASSERT(x) assert(x)
#define XR_LOG_WARN(...) ((void)0)
#define UNUSED(x) static_cast<void>(x)
namespace LibXR
{
inline constexpr double STANDARD_GRAVITY = 9.80665;
constexpr double PI = 3.14159265358979323846;
constexpr double TWO_PI = 6.2831853071795864769;
enum class ErrorCode
{
  OK,
  ERROR,
  NO_RESPONSE
};
struct MicrosecondTimestamp
{
  uint64_t value;
  MicrosecondTimestamp(uint64_t v = 0) : value(v) {}
  operator uint64_t() const { return value; }
};
using MillisecondTimestamp = MicrosecondTimestamp;
struct Timebase
{
  static inline uint64_t now = 1000;
  static MicrosecondTimestamp GetMicroseconds() { return now; }
  static MillisecondTimestamp GetMilliseconds() { return now / 1000; }
};
struct Mutex
{
  using LockGuard = std::lock_guard<Mutex>;
  std::mutex mutex;
  void lock() { mutex.lock(); }
  void unlock() { mutex.unlock(); }
};
namespace Memory
{
inline void FastCopy(void* d, const void* s, size_t n) { std::memcpy(d, s, n); }
}  // namespace Memory
template <typename A>
struct Callback
{
  std::function<void(bool, A)> fn;
  template <typename F, typename T>
  static Callback Create(F f, T* self)
  {
    return {[=](bool isr, A a) { f(isr, self, a); }};
  }
};
struct Event
{
  std::map<uint32_t, std::vector<Callback<uint32_t>>> handlers;
  void Register(uint32_t id, Callback<uint32_t> cb) { handlers[id].push_back(cb); }
  void Active(uint32_t id)
  {
    for (auto cb : handlers[id])
    {
      cb.fn(false, id);
    }
  }
};
struct Topic
{
  std::string name;
  static inline std::map<std::string, std::any> values;
  struct Callback
  {
    std::function<void(const std::any&)> fn;
    template <typename O, typename T>
    static Callback Make(void (*f)(bool, O*, const T&), O* p)
    {
      return {[=](const std::any& a) { f(false, p, std::any_cast<const T&>(a)); }};
    }
    template <typename F, typename O>
    static Callback Create(F f, O* p)
    {
      return Make(+f, p);
    }
  };
  template <typename T>
  struct ASyncSubscriber
  {
    T data{};
    bool waiting = false, ready = false;
    MicrosecondTimestamp timestamp{};
    ASyncSubscriber(Topic topic)
    {
      topic.RegisterCallback(Callback::Create(
        [](bool, ASyncSubscriber* self, const T& value)
        {
          if (self->waiting)
          {
            self->data = value;
            self->timestamp = Timebase::GetMicroseconds();
            self->waiting = false;
            self->ready = true;
          }
        }, this));
    }
    void StartWaiting() { if (!ready) { waiting = true; } }
    bool Available() { return ready; }
    T& GetData() { ready = false; return data; }
    MicrosecondTimestamp GetTimestamp() { return timestamp; }
  };
  static inline std::map<std::string, std::vector<Callback>> callbacks;
  template <typename T>
  static Topic CreateTopic(const char* n, void* = nullptr, bool = false)
  {
    return {n};
  }
  template <typename T>
  static Topic FindOrCreate(const char* n)
  {
    return {n};
  }
  void RegisterCallback(Callback cb) { callbacks[name].push_back(cb); }
  template <typename T>
  void Publish(const T& value, MicrosecondTimestamp = {})
  {
    values[name] = value;
    for (auto cb : callbacks[name])
    {
      cb.fn(std::any(value));
    }
  }
};
struct CAN
{
  enum class Type
  {
    STANDARD,
    EXTENDED
  };
  enum class FilterMode
  {
    ID_RANGE
  };
  struct ClassicPack
  {
    uint32_t id{};
    Type type{Type::STANDARD};
    uint8_t dlc{};
    uint8_t data[8]{};
  };
  using Callback = LibXR::Callback<const ClassicPack&>;
  std::map<uint32_t, Callback> callbacks;
  std::deque<ClassicPack> sent;
  void Register(Callback cb, Type, FilterMode, uint32_t id, uint32_t)
  {
    callbacks[id] = cb;
  }
  ErrorCode AddMessage(const ClassicPack& pack)
  {
    sent.push_back(pack);
    return ErrorCode::OK;
  }
  void Emit(const ClassicPack& pack)
  {
    if (callbacks.contains(pack.id))
    {
      callbacks[pack.id].fn(true, pack);
    }
  }
};
struct Stop
{
};
struct Thread
{
  enum class Priority
  {
    MEDIUM,
    HIGH
  };
  static inline std::vector<Thread*> threads;
  static inline unsigned budget;
  static inline std::function<void()> before_sleep;
  std::function<void()> run;
  template <typename T, typename F>
  void Create(T* p, F f, const char*, int, Priority)
  {
    run = [=] { f(p); };
    threads.push_back(this);
  }
  static void SleepUntil(MillisecondTimestamp& t, uint32_t ms)
  {
    if (before_sleep)
    {
      before_sleep();
    }
    Timebase::now += ms * 1000;
    t = Timebase::GetMilliseconds();
    if (--budget == 0)
    {
      throw Stop{};
    }
  }
  void Run(unsigned n)
  {
    budget = n;
    try
    {
      run();
    }
    catch (Stop&)
    {
    }
  }
};
}  // namespace LibXR

#include "cycle_value.hpp"
