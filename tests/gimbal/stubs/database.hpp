#pragma once
#include <any>
#include <map>

#include "fake_libxr.hpp"
namespace LibXR
{
class Database
{
 public:
  std::map<std::string, std::any> values;
  template <typename T>
  struct Key
  {
    T data_;
    Database& db;
    std::string name;
    Key(Database& d, const char* n, T initial) : data_(initial), db(d), name(n)
    {
      if (d.values.contains(name)) data_ = std::any_cast<T>(d.values[name]);
    }
    ErrorCode Set(T value)
    {
      data_ = value;
      db.values[name] = value;
      return ErrorCode::OK;
    }
  };
};
}  // namespace LibXR
