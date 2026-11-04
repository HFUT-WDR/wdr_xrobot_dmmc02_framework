#pragma once

#include <array>
#include <cstring>

#include "database/interface.hpp"

// BMI088 的临时零偏存储；只在本次上电有效，不触碰内部 Flash。
class ImuRamDatabase : public LibXR::Database
{
 private:
  static bool Matches(KeyBase& key)
  {
    return std::strcmp(key.name_, "bmi088_gyro_data") == 0 &&
           key.raw_data_.size_ == sizeof(float) * 3;
  }

  LibXR::ErrorCode Get(KeyBase& key) override
  {
    if (!Matches(key))
    {
      return LibXR::ErrorCode::ARG_ERR;
    }
    if (!present_)
    {
      return LibXR::ErrorCode::NOT_FOUND;
    }
    std::memcpy(key.raw_data_.addr_, data_.data(), data_.size());
    return LibXR::ErrorCode::OK;
  }

  LibXR::ErrorCode Set(KeyBase& key, LibXR::RawData data) override
  {
    if (!Matches(key) || data.size_ != data_.size())
    {
      return LibXR::ErrorCode::ARG_ERR;
    }
    std::memcpy(data_.data(), data.addr_, data_.size());
    present_ = true;
    return LibXR::ErrorCode::OK;
  }

  LibXR::ErrorCode Add(KeyBase& key) override
  {
    return Set(key, key.raw_data_);
  }

  std::array<uint8_t, sizeof(float) * 3> data_{};
  bool present_ = false;
};
