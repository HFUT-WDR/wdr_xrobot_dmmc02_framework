#pragma once
#include <any>
#include <map>
#include <string>
namespace LibXR
{
struct Topic
{
  static inline std::map<std::string, std::any> values;
  std::string name;
  template <typename T> static Topic CreateTopic(const char* name, void* = nullptr, bool = false)
  {
    return {name};
  }
  template <typename T> void Publish(const T& value) { values[name] = value; }
};
}
