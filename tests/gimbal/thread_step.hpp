#pragma once

#include "fake_libxr.hpp"

// 通过构造时注册的真实线程入口推进一拍，不为测试增加模块公开接口。
inline void run_thread_once(LibXR::Thread& thread)
{
  const auto now = LibXR::Timebase::now;
  const auto pending_hook = LibXR::Timebase::time_hook;
  // 线程入口先读取唤醒基准；拍内停止事件仍由后续控制阶段的时钟读取触发。
  LibXR::Timebase::time_hook = [pending_hook]
  { LibXR::Timebase::time_hook = pending_hook; };
  thread.Run(1);
  // 场景已显式推进时钟，撤销线程桩SleepUntil的额外增量，避免重复计时。
  LibXR::Timebase::now = now;
}
