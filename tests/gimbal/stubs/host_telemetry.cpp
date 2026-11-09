#include "HostData.hpp"
// 离线机构测试不创建USB遥测任务。
void HostData::StartTelemetry() { std::abort(); }
