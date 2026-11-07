#pragma once
#include <any>
#include <atomic>
#include <cassert>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <functional>
#include <map>
#include <string>
#include <vector>
#define REQUIRE(x)  \
  do                \
  {                 \
    if (!(x))       \
    {               \
      std::abort(); \
    }               \
  } while (0)
namespace LibXR
{
inline constexpr double STANDARD_GRAVITY = 9.80665;
enum class ErrorCode
{
  OK,
  FULL,
  EMPTY,
  BUSY,
  ERROR
};
struct MicrosecondTimestamp
{
  uint64_t us;
  MicrosecondTimestamp(uint64_t v) : us(v) {}
  operator uint64_t() const { return us; }
};
struct Timebase
{
  static inline uint64_t now = 1000;
  static MicrosecondTimestamp GetMicroseconds() { return now; }
};
struct Stop
{
};
struct Semaphore
{
  static inline unsigned budget = 0;
  static inline std::function<void()> on_wait;
  explicit Semaphore(unsigned) {}
  void PostFromCallback(bool) {}
  void Wait(uint32_t)
  {
    if (on_wait) on_wait();
    Timebase::now += 1000;
    if (--budget == 0) throw Stop{};
  }
};
template <typename A>
struct Callback
{
  std::function<void(bool, A)> fn;
  template <typename F, typename T>
  static Callback Create(F f, T* ctx)
  {
    return {[f, ctx](bool isr, A arg) { f(isr, ctx, arg); }};
  }
};
struct Thread
{
  std::function<void()> run;
  enum class Priority
  {
    HIGH
  };
  template <typename T, typename F>
  void Create(T* self, F fn, const char*, unsigned, Priority)
  {
    run = [self, fn] { fn(self); };
    threads.push_back(this);
  }
  void Run(unsigned n)
  {
    Semaphore::budget = n;
    try
    {
      run();
    }
    catch (Stop&)
    {
    }
  }
  static inline std::vector<Thread*> threads;
};
template <typename T>
struct SlotPool
{
  std::deque<T> q;
  size_t capacity;
  explicit SlotPool(size_t n) : capacity(n) {}
  ErrorCode Put(const T& v)
  {
    if (q.size() == capacity) return ErrorCode::FULL;
    q.push_back(v);
    return ErrorCode::OK;
  }
  ErrorCode Get(T& v)
  {
    if (q.empty()) return ErrorCode::EMPTY;
    v = q.front();
    q.pop_front();
    return ErrorCode::OK;
  }
};
struct Topic
{
  static inline std::map<std::string, std::any> values;
  std::string name;
  template <typename T>
  static Topic CreateTopic(const char* n)
  {
    return {n};
  }
  template <typename T>
  void Publish(T& v)
  {
    values[name] = v;
  }
  template <typename T>
  void Publish(T& v, MicrosecondTimestamp)
  {
    Publish(v);
  }
};
struct CAN
{
  enum class Type
  {
    STANDARD,
    EXTENDED,
    REMOTE
  };
  enum class FilterMode
  {
    ID_RANGE
  };
  struct ClassicPack
  {
    uint32_t id{};
    Type type = Type::STANDARD;
    uint8_t dlc{};
    uint8_t data[8]{};
  };
  using Callback = LibXR::Callback<const ClassicPack&>;
  Callback cb;
  uint32_t id{};
  bool fail = false;
  std::deque<ClassicPack> sent;
  void Register(Callback c, Type, FilterMode, uint32_t first, uint32_t)
  {
    cb = c;
    id = first;
  }
  ErrorCode AddMessage(const ClassicPack& p)
  {
    if (fail) return ErrorCode::ERROR;
    sent.push_back(p);
    return ErrorCode::OK;
  }
  void Emit(const ClassicPack& p)
  {
    if (p.id == id && p.type == Type::STANDARD) cb.fn(true, p);
  }
};
struct Operation
{
  enum class OperationPollingStatus
  {
    READY,
    RUNNING,
    DONE,
    ERROR
  };
  std::atomic<OperationPollingStatus>* status;
  explicit Operation(std::atomic<OperationPollingStatus>& s) : status(&s) {}
};
using ReadOperation = Operation;
using WriteOperation = Operation;
struct RawData
{
  void* data;
  size_t size;
};
struct ConstRawData
{
  const void* data;
  size_t size;
};
struct UART
{
  enum class Parity
  {
    NO_PARITY
  };
  struct Configuration
  {
    uint32_t baud;
    Parity parity;
    uint8_t bits, stops;
  };
  struct Port
  {
    std::deque<uint8_t> q;
    size_t Size() const { return q.size(); }
  } port;
  Port* read_port_ = &port;
  uint32_t baud{};
  unsigned writes = 0;
  RawData rx{};
  ConstRawData tx{};
  Operation *rx_op{}, *tx_op{};
  std::vector<uint8_t> held;
  ErrorCode SetConfig(Configuration c)
  {
    baud = c.baud;
    return ErrorCode::OK;
  }
  ErrorCode Read(RawData d, ReadOperation& op)
  {
    assert(!rx_op);
    rx = d;
    rx_op = &op;
    op.status->store(Operation::OperationPollingStatus::RUNNING);
    CompleteRx();
    return ErrorCode::OK;
  }
  ErrorCode Write(ConstRawData d, WriteOperation& op)
  {
    assert(!tx_op);
    tx = d;
    tx_op = &op;
    held.assign((const uint8_t*)d.data, (const uint8_t*)d.data + d.size);
    ++writes;
    op.status->store(Operation::OperationPollingStatus::RUNNING);
    return ErrorCode::OK;
  }
  void CompleteRx()
  {
    if (rx_op && port.q.size() >= rx.size)
    {
      for (size_t i = 0; i < rx.size; ++i)
      {
        ((uint8_t*)rx.data)[i] = port.q.front();
        port.q.pop_front();
      }
      rx_op->status->store(Operation::OperationPollingStatus::DONE);
      rx_op = nullptr;
    }
  }
  void Feed(const uint8_t* p, size_t n)
  {
    for (size_t i = 0; i < n; ++i) port.q.push_back(p[i]);
    CompleteRx();
  }
  unsigned CompleteTx()
  {
    assert(tx_op);
    assert(std::memcmp(tx.data, held.data(), held.size()) == 0);
    unsigned rid = held[3];
    tx_op->status->store(Operation::OperationPollingStatus::DONE);
    tx_op = nullptr;
    return rid;
  }
};
}  // namespace LibXR
