/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : PID.c
  * @brief          : PID functions
  * @author         : GrassFan Wang
  * @date           : 2024/12/29
  * @version        : v1.1
  ******************************************************************************
  * @attention      : To be perfected
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include <stddef.h>
#include <string.h>
#include <math.h>
#include "PID.h"
/* Includes ------------------------------------------------------------------*/

/**
 * @brief 初始化PID参数.
 * @Param PID: 指向PID_Info_TypeDef结构的指针，包含PID控制器的信息.
 * @Param Param: 指向PID参数的浮点数指针，包含PID参数信息.
 * @retval PID错误状态.
 */
static PID_Status_e PID_Param_Init(PID_Info_TypeDef *PID,float Param[PID_PARAMETER_NUM])
{
    //判断PID类型和参数是否为空 若为空 返回PID_FAILED_INIT（初始化失败）
    if(PID->Type == PID_Type_None || Param == NULL)
    {
        return PID_FAILED_INIT;
    }

    //初始化PID参数
    PID->Param.KP = Param[0];
    PID->Param.KI = Param[1];
    PID->Param.KD = Param[2];
    PID->Param.Alpha = Param[3];
    if( PID->Param.Alpha > 0.f &&  PID->Param.Alpha < 1.f)
        LowPassFilter1p_Init(&PID->Dout_LPF,PID->Param.Alpha);

    PID->Param.Deadband = Param[4];
    PID->Param.LimitIntegral = Param[5];
    PID->Param.LimitOutput = Param[6];

    //清除PID错误计数
    PID->ERRORHandler.ErrorCount = 0;

    //返回PID_ERROR_NONE（无错误状态）
    return PID_ERROR_NONE;
}
//------------------------------------------------------------------------------


/**
 * @brief 清除PID计算值，将所有输出赋0.
 * @Param PID: 指向PID_Info_TypeDef结构的指针，包含PID控制器的信息.
 * @retval 无.
 */
static void PID_Calc_Clear(PID_Info_TypeDef *PID)
{
    //将所有输出赋0
    memset(PID->Err,0,sizeof(PID->Err));
    PID->Integral = 0;

    PID->Pout = 0;
    PID->Iout = 0;
    PID->Dout = 0;
    PID->Output = 0;

    /* 恢复错误状态，允许 PID 从 NaN/Inf 中自愈。
     * 不清除的话 PID_Calculate 会永久返回 0，只有复位才能恢复。 */
    PID->ERRORHandler.Status = PID_ERROR_NONE;
    PID->ERRORHandler.ErrorCount = 0;
}
//------------------------------------------------------------------------------


/**
 * @brief 初始化PID控制器.
 * @Param PID: 指向PID_Info_TypeDef结构的指针，包含PID控制器的信息.
 * @Param Type: PID控制器类型.
 * @Param Param:指向PID参数的浮点数指针，包含PID参数信息.
 * @retval PID错误状态.
 */
void PID_Init(PID_Info_TypeDef *PID,PID_Type_e Type,float Param[PID_PARAMETER_NUM])
{

    PID->Type = Type;

    PID->PID_Calc_Clear = PID_Calc_Clear;
    PID->PID_Param_Init = PID_Param_Init;

    PID->PID_Calc_Clear(PID);
    PID->ERRORHandler.Status = PID->PID_Param_Init(PID, Param);
}
//------------------------------------------------------------------------------


/**
  * @brief 判断PID错误状态
  * @Param PID: 指向PID_Info_TypeDef结构的指针，包含PID控制器的信息.
  * @retval 无.
  */
static void PID_ErrorHandle(PID_Info_TypeDef *PID)
{
    /* Judge NAN/INF */
    if(isnan(PID->Output) == true || isinf(PID->Output)==true)
    {
        PID->ERRORHandler.Status = PID_CALC_NANINF;
    }
}
//------------------------------------------------------------------------------

/**
  * @brief  PID控制器计算函数
  * @Param  *PID pointer to a PID_TypeDef_t structure that contains
  *              the configuration information for the specified PID.
  * @Param  Target  Target for the PID controller
  * @Param  Measure Measure for the PID controller
  * @retval the PID Output
  */
float PID_Calculate(PID_Info_TypeDef *PID, float Target,float Measure)
{
    /* 输入 NaN/Inf 保护：输入非法时直接清零避免污染 PID 内部状态。
     * 不清零的话 Err[0] = NaN，积分和输出会连锁变成 NaN，
     * 进而触发 PID_CALC_NANINF → 永久锁死。 */
    if (isnan(Target) || isnan(Measure) || isinf(Target) || isinf(Measure)) {
        PID->PID_Calc_Clear(PID);
        return 0.0f;
    }

    /* update the PID error status */
    PID_ErrorHandle(PID);
    if(PID->ERRORHandler.Status != PID_ERROR_NONE)
    {
        PID->PID_Calc_Clear(PID);
        return 0;
    }

    /* update the target/measure */
    PID->Target =  Target;
    PID->Measure = Measure;

    /* update the error */
    PID->Err[2] = PID->Err[1];
    PID->Err[1] = PID->Err[0];
    PID->Err[0] = PID->Target - PID->Measure;

    if(fabsf(PID->Err[0]) >= PID->Param.Deadband)
    {
        /* update the PID controller output */
        if(PID->Type == PID_POSITION)
        {
            /* Update the PID Integral */
            if(PID->Param.KI != 0)
                PID->Integral += PID->Err[0];
            else
                PID->Integral = 0;

            /* Update the Proportional Output, Integral Output, Derivative Output */
            PID->Pout = PID->Param.KP * PID->Err[0];
            PID->Iout = PID->Param.KI * PID->Integral;
            VAL_LIMIT(PID->Iout,-PID->Param.LimitIntegral,PID->Param.LimitIntegral);
            PID->Dout = PID->Param.KD * (PID->Err[0] - PID->Err[1]);
            if( PID->Param.Alpha > 0.f &&  PID->Param.Alpha < 1.f){

                PID->Dout_LPF.Alpha = PID->Param.Alpha;
                PID->Dout = LowPassFilter1p_Update(&PID->Dout_LPF, PID->Dout);

            }
            /* update the PID output */
            PID->Output = PID->Pout + PID->Iout + PID->Dout;
            VAL_LIMIT(PID->Output,-PID->Param.LimitOutput,PID->Param.LimitOutput);
        }
        else if(PID->Type == PID_VELOCITY)
        {
            /* Update the Proportional Output, Integral Output, Derivative Output */
            PID->Pout = PID->Param.KP * (PID->Err[0] - PID->Err[1]);
            PID->Iout = PID->Param.KI * (PID->Err[0]);
            PID->Dout = PID->Param.KD * (PID->Err[0] - 2.f*PID->Err[1] + PID->Err[2]);
            if( PID->Param.Alpha > 0.f &&  PID->Param.Alpha < 1.f){

                PID->Dout_LPF.Alpha = PID->Param.Alpha;
                PID->Dout = LowPassFilter1p_Update(&PID->Dout_LPF, PID->Dout);

            }
            /* update the PID output */
            PID->Output += PID->Pout + PID->Iout + PID->Dout;
            VAL_LIMIT(PID->Output,-PID->Param.LimitOutput,PID->Param.LimitOutput);
        }
    }

    return PID->Output;
}
//------------------------------------------------------------------------------

/**
 * @brief 将角度限制在 [-PI, PI] 范围内。
 */
static float angle_wrap(float ang)
{
    while (ang > (float)M_PI)  ang -= 2.0f * (float)M_PI;
    while (ang < -(float)M_PI) ang += 2.0f * (float)M_PI;
    return ang;
}

/**
 * @brief 绝对值限幅。
 */
static void abs_limit_f(float *num, float limit)
{
    if (*num >  limit) *num =  limit;
    if (*num < -limit) *num = -limit;
}

/**
 * @brief 角度环 PID 计算（云台位置控制）。
 *
 * @param[in,out] pid         PID 控制器对象。
 * @param[in]     measure     当前反馈值。
 * @param[in]     target      目标值。
 * @param[in]     error_delta 微分项输入（通常为角速度反馈）。
 * @return PID 输出结果。
 */
float PID_CalculateAngle(pid_type_def *pid, float measure, float target, float error_delta)
{
    if (pid == NULL) return 0.0f;

    pid->Measure = measure;
    pid->Target = target;

    float delta_set = target - pid->PrevTarget;
    pid->PrevTarget = target;
    pid->Err[0] = angle_wrap(target - measure);

    pid->Pout = pid->Param.KP * pid->Err[0];

    /* 积分分离：i_zone <= 0 表示始终积分，否则误差在 i_zone 内才积分 */
    if (pid->Param.IZone <= 0.0f || fabsf(pid->Err[0]) <= pid->Param.IZone) {
        pid->Integral += pid->Param.KI * pid->Err[0];
    }
    abs_limit_f(&pid->Integral, pid->Param.LimitIntegral);
    pid->Iout = pid->Integral;

    pid->Dout = pid->Param.KD * error_delta;

    pid->Output = pid->Pout + pid->Iout + pid->Dout + pid->Param.KF * delta_set;
    abs_limit_f(&pid->Output, pid->Param.LimitOutput);

    return pid->Output;
}

/**
 * @brief 基于外部预计算误差的角度环 PID。
 *
 * 与 PID_CalculateAngle 的区别：误差由调用方预先计算（如双 yaw 联动解算），
 * 内部仅做 angle_wrap 格式化后执行 P-I-D 计算。不包含 kf 前馈项。
 *
 * @param[in,out] pid         PID 控制器对象。
 * @param[in]     error       预计算的弧度角误差。
 * @param[in]     error_delta 微分项输入（通常为角速度反馈）。
 * @return PID 输出结果。
 */
float PID_CalculateByError(pid_type_def *pid, float error, float error_delta)
{
    if (pid == NULL) return 0.0f;

    pid->Err[0] = angle_wrap(error);

    pid->Pout = pid->Param.KP * pid->Err[0];

    if (pid->Param.IZone <= 0.0f || fabsf(pid->Err[0]) <= pid->Param.IZone) {
        pid->Integral += pid->Param.KI * pid->Err[0];
    }
    abs_limit_f(&pid->Integral, pid->Param.LimitIntegral);
    pid->Iout = pid->Integral;

    pid->Dout = pid->Param.KD * error_delta;

    pid->Output = pid->Pout + pid->Iout + pid->Dout;
    abs_limit_f(&pid->Output, pid->Param.LimitOutput);

    return pid->Output;
}

/**
 * @brief 速度环 PID 计算（带 Dbuf 微分缓存、抗积分饱和和 KF 前馈）。
 *
 * 用于云台 yaw / pitch 速度内环，替代原 fb_gimbal_PID_calc。
 * Dbuf[3] 保存最近三次误差差分用于微分计算，Integral 在输出被限幅后
 * 按 0.3 倍超额量回退以防止积分饱和。
 *
 * @param[in,out] pid     PID 控制器对象。
 * @param[in]     measure 当前速度反馈值。
 * @param[in]     target  目标速度值。
 * @return PID 输出结果。
 */
float PID_CalculateVelocity(pid_type_def *pid, float measure, float target)
{
    if (pid == NULL) return 0.0f;

    pid->Err[2] = pid->Err[1];
    pid->Err[1] = pid->Err[0];
    pid->Target = target;
    pid->Measure = measure;
    pid->Err[0] = target - measure;

    float delta_set = target - pid->PrevTarget;
    pid->PrevTarget = target;

    float ff = pid->Param.KF * delta_set;

    pid->Pout = pid->Param.KP * pid->Err[0];
    pid->Integral += pid->Param.KI * pid->Err[0];
    abs_limit_f(&pid->Integral, pid->Param.LimitIntegral);
    pid->Iout = pid->Integral;

    pid->Dbuf[2] = pid->Dbuf[1];
    pid->Dbuf[1] = pid->Dbuf[0];
    pid->Dbuf[0] = pid->Err[0] - pid->Err[1];
    pid->Dout = pid->Param.KD * pid->Dbuf[0];

    float out_pre = pid->Pout + pid->Iout + pid->Dout + ff;
    pid->Output = out_pre;
    abs_limit_f(&pid->Output, pid->Param.LimitOutput);

    /* 抗积分饱和：输出被限幅时，反向修正积分累加器 */
    pid->Integral -= (out_pre - pid->Output) * 0.3f;
    abs_limit_f(&pid->Integral, pid->Param.LimitIntegral);
    pid->Iout = pid->Integral;

    return pid->Output;
}

/**
 * @brief 清空 PID 全部内部状态（含 gimbal 扩展字段）。
 *
 * 除标准 Err[3] / Integral / Pout / Iout / Dout / Output 外，
 * 同时清零 gimbal 扩展字段 PrevTarget 和 Dbuf[3]。
 *
 * @param[in,out] pid PID 控制器对象指针。
 */
void PID_ClearExtended(pid_type_def *pid)
{
    if (pid == NULL) return;
    memset(pid->Err, 0, sizeof(pid->Err));
    pid->Integral = 0.0f;
    pid->Pout = 0.0f;
    pid->Iout = 0.0f;
    pid->Dout = 0.0f;
    pid->Output = 0.0f;
    pid->Target = 0.0f;
    pid->Measure = 0.0f;
    pid->PrevTarget = 0.0f;
    memset(pid->Dbuf, 0, sizeof(pid->Dbuf));
}

