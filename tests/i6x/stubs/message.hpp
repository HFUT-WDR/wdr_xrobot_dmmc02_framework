#pragma once
#include <any>
#include <functional>
#include <map>
#include <string>
namespace LibXR
{
struct Topic
{
  struct Callback
  {
    std::function<void(const std::any&)> call;
    template <typename Owner, typename T>
    static Callback Make(void (*fn)(bool, Owner*, const T&), Owner* owner)
    {
      return {[fn, owner](const std::any& value) { fn(false, owner, std::any_cast<const T&>(value)); }};
    }
    template <typename Fn, typename Owner>
    static Callback Create(Fn fn, Owner* owner) { return Make(+fn, owner); }
  };
  static inline std::map<std::string, std::any> values;
  static inline std::map<std::string, Callback> callbacks;
  std::string name;
  template <typename T> static Topic CreateTopic(const char* name, void* = nullptr, bool = false)
  {
    return {name};
  }
  void RegisterCallback(Callback cb) { callbacks[name] = cb; }
  template <typename T> void Publish(const T& value)
  {
    values[name] = value;
    if (callbacks.contains(name)) { callbacks.at(name).call(std::any(value)); }
  }
};
}
