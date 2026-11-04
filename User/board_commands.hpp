#pragma once

#include "ramfs.hpp"

// 启动时注册板级诊断；只读状态，不发送电机命令。
void register_board_commands(LibXR::RamFS& ramfs);
