#pragma once
#ifdef __cplusplus
extern "C"
{
#endif
  struct ReferenceParam
  {
    float KP, KI, KD, KF, LimitIntegral, LimitOutput, IZone;
  };
  typedef struct ReferencePid
  {
    struct ReferenceParam Param;
    float Measure, Target, PrevTarget, Err[3], Pout, Iout, Dout, Integral, Output,
        Dbuf[3];
  } pid_type_def;
  float PID_CalculateByError(pid_type_def*, float, float);
  float PID_CalculateVelocity(pid_type_def*, float, float);
  void calc_gimbal_axis_cali(unsigned short, unsigned short, unsigned short*, float*,
                             float*);
  void reference_plan(float, float, float, float, float, float, float*, float*, float*);
  float reference_pitch(float, float, float, float, float, float*);
#ifdef __cplusplus
}
#endif
