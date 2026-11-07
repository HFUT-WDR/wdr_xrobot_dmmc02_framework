#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>
#include "ControlAlgorithms.hpp"
#include "tools/pid/pid.hpp"
#include "tools/low_pass_filter/low_pass_filter.hpp"
extern "C"
{
#include "PID.h"
#include "translation_filter.h"
}
static void near(float a, float b, float eps = 1e-5f)
{
  if (!(std::fabs(a - b) < eps)) { std::fprintf(stderr,"actual=%g expected=%g\n",a,b); }
  assert(std::fabs(a - b) < eps);
}
int main(int argc, char**)
{
  if (argc > 1)
  {
    PID_Info_TypeDef old{};
    PID_InitFull(&old,1,0,0,0,10,1,0);
    std::puts("entered frozen angle probe");
    std::fflush(stdout);
    PID_CalculateAngle(&old,0,std::numeric_limits<float>::infinity(),0);
    return 0;
  }
  // 直接运行冻结旧源码，证明其缺陷，而不是只在文档中推断。
  LowPassFilter1p_Info_TypeDef old_filter{};
  LowPassFilter1p_Init(&old_filter,.25f);
  near(LowPassFilter1p_Update(&old_filter,4),4);
  LowPassFilter1p_Init(&old_filter,.25f);
  const float old_restart=LowPassFilter1p_Update(&old_filter,4);
  near(old_restart,1); // 旧Init未复位Initialized，重启首拍行为与新对象不同。
  PID_Info_TypeDef old_pid{};
  float old_param[]{0,0,1,.5f,0,100,100};
  PID_Init(&old_pid,PID_POSITION,old_param);
  near(PID_Calculate(&old_pid,2,0),2);
  old_pid.PID_Calc_Clear(&old_pid);
  near(PID_Calculate(&old_pid,0,0),1); // 旧clear后残留的D滤波重新输出。
  float integral_param[]{0,1,0,0,0,1,10};
  old_pid={};PID_Init(&old_pid,PID_POSITION,integral_param);
  for (int i=0;i<1000;++i) { PID_Calculate(&old_pid,1,0); }
  near(PID_Calculate(&old_pid,-1,0),1); // 旧积分原始状态仍为999，反向不能立即退饱和。

  LowPassFilter2p_Info_TypeDef old_second{};
  float coeff[]{.5f,.25f,.25f};
  LowPassFilter2p_Init(&old_second,coeff);
  near(LowPassFilter2p_Update(&old_second,4),4);
  LowPassFilter2p_Init(&old_second,coeff);
  near(LowPassFilter2p_Update(&old_second,4),1);
  float dead_param[]{1,0,0,0,1,0,10};
  old_pid={};PID_Init(&old_pid,PID_POSITION,dead_param);
  near(PID_Calculate(&old_pid,2,0),2);
  near(PID_Calculate(&old_pid,.01f,0),2);

  wdr_control::LowPassFilter filter(.25f);
  sp::LowPassFilter sp_filter(.25f);
  for (int i=0;i<5000;++i)
  {
    const float input=std::sin(i*.013f);
    sp_filter.update(input);
    near(filter.Update(input),sp_filter.out);
  }
  filter.Reset();near(filter.GetOutput(),0);near(filter.Update(4),4);
  wdr_control::LowPassFilter hold(0), bypass(1);
  near(hold.Update(2),2);near(hold.Update(3),2);
  near(bypass.Update(2),2);near(bypass.Update(3),3);

  // 时间常数模式与冻结原车平移滤波对照，覆盖实际dt变化、清零和重新起步。
  wdr_control::LowPassFilter translation(
      1.0f, wdr_control::LowPassFilter::InitialState::ZERO);
  first_order_filter_type_t legacy{};
  const float tau[]{0.1666666667f};
  first_order_filter_init(&legacy, .002f, tau);
  for (unsigned i = 0; i < 10000; ++i)
  {
    const float dt = .001f + float(i % 7) * .0007f;
    const float input = i % 113 == 0 ? 0.0f : 3 * std::sin(i * .017f);
    legacy.frame_period = dt;
    first_order_filter_cali(&legacy, input);
    near(translation.Update(input, dt, tau[0]), legacy.out, 1e-6f);
    if (i % 113 == 0)
    {
      legacy.out = legacy.input = 0;
      translation.Reset();
    }
  }
  translation.Reset();
  near(translation.Update(4, .002f, tau[0]), 4 * .002f / (tau[0] + .002f));
  near(translation.Update(4, .002f, 0), 4);
  near(translation.Update(4, 0, tau[0]), 0);
  near(translation.Update(4, .002f, tau[0]), 4 * .002f / (tau[0] + .002f));

  // 无饱和且Ki=0时，与sp的微分先行、首拍抑制和新采样权重一致。
  wdr_control::PID filtered({.kp=2,.ki=0,.kd=.1f,.i_limit=0,.out_limit=1000,.d_alpha=.25f});
  sp::PID sp_pid(.002f,2,0,.1f,1000,0,.25f);
  for (int i=0;i<5000;++i)
  {
    const float target=std::cos(i*.01f), feedback=std::sin(i*.01f);
    sp_pid.calc(target,feedback);
    near(filtered.Calculate(target,feedback,.002f),sp_pid.out,3e-4f);
  }
  filtered.Reset();sp_pid.clear();
  near(filtered.Calculate(10,0,.002f),20); // 目标阶跃只有P，不产生D冲击。
  near(filtered.Calculate(10,0,.002f),20);
  filtered.Reset();near(filtered.GetOutput(),0);near(filtered.Calculate(0,0,.002f),0);

  assert(!wdr_control::PID::CheckParam({.out_limit=1e-8f}));
  assert(!wdr_control::PID::CheckParam({.ki=1e-8f,.i_limit=1,.out_limit=10}));
  assert(!wdr_control::PID::CheckParam({.ki=1e8f,.i_limit=.01f,.out_limit=10}));
  // 积分输出限幅和输出饱和门控；不得只限制显示的Iout而积累隐藏状态。
  wdr_control::PID integral({.kp=0,.ki=10,.kd=0,.i_limit=1,.out_limit=10});
  for (int i=0;i<1000;++i) { integral.Calculate(1,0,.01f); }
  near(integral.GetIntegralOutput(),1);
  near(integral.Calculate(-1,0,.01f),.9f);
  wdr_control::PID saturated({.kp=2,.ki=10,.kd=0,.i_limit=10,.out_limit=1});
  for (int i=0;i<1000;++i) { near(saturated.Calculate(10,0,.01f),1); }
  near(saturated.GetIntegralOutput(),0);
  near(saturated.Calculate(0,0,.01f),0);
  // 显式dt使积分在相同物理时长下不依赖采样率。
  wdr_control::PID fast({.ki=2,.i_limit=100,.out_limit=100});
  wdr_control::PID slow({.ki=2,.i_limit=100,.out_limit=100});
  for (int i=0;i<1000;++i) { fast.Calculate(1,0,.001f); }
  for (int i=0;i<500;++i) { slow.Calculate(1,0,.002f); }
  near(fast.GetOutput(),slow.GetOutput(),5e-5f);
  wdr_control::PID angle({.kp=1,.out_limit=10,.cycle=true});
  near(angle.Calculate(-3.13f,3.13f,.002f),.0231853f,2e-6f);
  wdr_control::PID deadband({.kp=1,.out_limit=10,.deadband=.1f});
  near(deadband.Calculate(1,0,.002f),1);
  near(deadband.Calculate(.01f,0,.002f),0);
  near(deadband.GetOutput(),0);
  for (float invalid:{std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::infinity()})
  {
    near(filtered.Calculate(invalid,0,.002f),0);
    near(filtered.Calculate(1,0,.002f),2);
    near(filter.Update(invalid),0);near(filter.Update(3),3);
  }
  near(filtered.Calculate(1,0,0),0);
  near(filtered.Calculate(1,0,-.002f),0);
  near(filtered.Calculate(std::numeric_limits<float>::max(),-std::numeric_limits<float>::max(),.002f),0);

  wdr_control::PID tracked({.kp=2, .ki=10, .kd=0, .i_limit=20, .out_limit=100});
  tracked.Calculate(1, 0, .1f);
  tracked.TrackIntegral(.5f);
  near(tracked.GetIntegralOutput(), -1.5f);
  near(tracked.Calculate(1, 0, .1f), 1.5f);
  tracked.Reset();
  near(tracked.GetIntegralOutput(), 0);

  // 二阶Butterworth：单位DC增益、截止频率约-3dB、两倍截止明显衰减，复位确定。
  constexpr float PI=3.14159265358979323846f;
  for (float frequency:{50.f,100.f,200.f})
  {
    wdr_control::SecondOrderLowPass lowpass(1000,100);
    double energy=0;
    for (int i=0;i<6000;++i)
    {
      const float output=lowpass.Update(std::sin(2*PI*frequency*i/1000));
      if (i>=1000) { energy+=double(output)*output; }
    }
    const float gain=std::sqrt(energy/5000*2);
    if (frequency==50) { assert(gain>.95f && gain<1); }
    if (frequency==100) { near(gain,std::sqrt(.5f),.002f); }
    if (frequency==200) { assert(gain<.21f); }
    lowpass.Reset();
    near(lowpass.GetOutput(),0);
    wdr_control::SecondOrderLowPass fresh(1000,100);
    near(lowpass.Update(1),fresh.Update(1));
    for (int i=0;i<1000;++i) { lowpass.Update(1); }
    near(lowpass.GetOutput(),1,2e-6f);
    near(lowpass.Update(std::numeric_limits<float>::infinity()),0);
  }
  puts("PASS: frozen defects reproduced; 5000 sp LPF + 5000 sp PID comparisons; dt, anti-windup, reset, derivative kick, cyclic error, invalid input, Butterworth frequency response");
}
