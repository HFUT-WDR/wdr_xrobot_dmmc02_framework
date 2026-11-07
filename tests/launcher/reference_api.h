#pragma once
#include <stdint.h>
#ifdef __cplusplus
extern "C"
{
#endif
  /// 设置下一次原车虚拟热量更新读取的裁判冷却、上限、17mm热量与HAL_GetTick。
  void ref_heat_set(float cooling, float limit, uint16_t heat1, uint32_t tick);
  /// 调用原车Shoot_Virtual_Heat_Update一次，返回g_virtual_heat.current_heat。
  float ref_heat_update(float current_a, float current_b, float speed_a, float target);
  /// 设定裁判当前弹速并调用原车shoot_speed_control一次，返回basic_tarspeed。
  float ref_speed_update(float bullet_speed, uint32_t tick);
  float ref_speed_ema(void);
  /// 调用原车trigger_motor_turn_back一次（AUTO分支），返回Trigger_Actualspeed。
  float ref_jam(float target, float feedback);
  /// 调用原车LowPassFilter2p_Update一次；latest为Output[2]，delayed为原车读取的Output[0]。
  void ref_filter(float input, float* latest, float* delayed);
#ifdef __cplusplus
}
#endif
