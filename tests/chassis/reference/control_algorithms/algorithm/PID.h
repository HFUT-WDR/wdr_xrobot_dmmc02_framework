/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : pid.c
  * @brief          : pid functions
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : To be perfected
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CONTROLLER_PID_H
#define CONTROLLER_PID_H

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "LPF.h"
#include "struct_typedef.h"
/* Exported defines -----------------------------------------------------------*/
/**
 * @brief 将变量限制在给定范围内。
 *
 * @param[in,out] x 待限幅变量。
 * @param[in] min 最小允许值。
 * @param[in] max 最大允许值。
 */
#define VAL_LIMIT(x,min,max)  do{ \
                                    if ((x) > (max)) {(x) = (max);} \
                                    else if ((x) < (min)) {(x) = (min);} \
                                }while(0U)

/**
 * @brief PID 参数数组长度。
 */
#ifndef PID_PARAMETER_NUM
#define PID_PARAMETER_NUM 7
#endif

/* Exported types ------------------------------------------------------------*/
/**
 * @brief PID 错误状态。
 */
typedef enum
{
    PID_ERROR_NONE = 0x00U,  /*!< 无错误。 */
    PID_FAILED_INIT = 0x01U, /*!< 初始化失败。 */
    PID_CALC_NANINF = 0x02U, /*!< 计算结果出现 NaN 或无穷大。 */
    PID_Status_NUM,          /*!< PID 状态枚举数量。 */
}PID_Status_e;

/**
 * @brief PID 控制器类型。
 */
typedef enum
{
    PID_Type_None = 0x00U, /*!< 未指定类型。 */
    PID_POSITION = 0x01U,  /*!< 位置式 PID。 */
    PID_VELOCITY = 0x02U,  /*!< 速度式 PID。 */
    PID_TYPE_NUM,          /*!< PID 类型枚举数量。 */
}PID_Type_e;

/**
 * @brief PID 错误处理信息。
 */
typedef struct
{
    uint16_t ErrorCount; /*!< 错误累计次数。 */
    PID_Status_e Status; /*!< 当前错误状态。 */
}PID_ErrorHandler_Typedef;

/**
 * @brief PID 参数集合。
 */
typedef struct
{
    float KP;            /**< 比例系数。 */
    float KI;            /**< 积分系数。 */
    float KD;            /**< 微分系数。 */
    float Alpha;         /**< 微分一阶滤波器系数。 */
    float Deadband;      /**< 死区阈值，误差绝对值小于该值时停止计算。 */
    float LimitIntegral; /**< 积分限幅。 */
    float LimitOutput;   /**< 总输出限幅。 */
    float KF;            /**< 前馈系数（gimbal 扩展）。 */
    float IZone;         /**< 积分分离区，|err|>IZone 时停止积分（gimbal 扩展）。 */
}PID_Parameter_Typedef;

/**
 * @brief PID 控制器状态信息。
 */
typedef struct PID_TypeDef
{
    PID_Type_e Type; /**< PID 类型。 */

    float Target;  /**< 目标值。 */
    float Measure; /**< 实际测量值。 */

    float Err[3];   /**< 当前及过去两次误差。 */
    float Integral; /**< 误差积分值。 */
    float Pout;     /**< 比例输出。 */
    float Iout;     /**< 积分输出。 */
    float Dout;     /**< 微分输出。 */
    float Output;   /**< PID 总输出。 */

    float PrevTarget; /**< 上一拍目标值，用于前馈计算（gimbal 扩展）。 */
    float Dbuf[3];    /**< 微分缓存，速度环使用（gimbal 扩展）。 */

    LowPassFilter1p_Info_TypeDef Dout_LPF; /**< 微分输出一阶低通滤波器。 */

    PID_Parameter_Typedef Param;           /**< PID 参数结构体。 */
    PID_ErrorHandler_Typedef ERRORHandler; /**< PID 错误处理结构体。 */

    /**
     * @brief 装载 PID 参数到控制器实例。
     *
     * @param[in,out] PID PID 控制器对象。
     * @param[in] Param PID 参数数组。
     * @retval PID_ERROR_NONE 初始化成功。
     * @retval PID_FAILED_INIT 初始化失败。
     */
    PID_Status_e (*PID_Param_Init)(struct PID_TypeDef *PID,float *Param);

    /**
     * @brief 清空 PID 计算过程中的历史状态。
     *
     * @param[in,out] PID PID 控制器对象。
     */
    void (*PID_Calc_Clear)(struct PID_TypeDef *PID);

}PID_Info_TypeDef;

/**
 * @brief PID 控制器类型别名。
 */
typedef PID_Info_TypeDef pid_type_def;


/* Exported functions prototypes ---------------------------------------------*/
/**
 * @brief 初始化 PID 控制器。
 *
 * @param[out] Pid PID 控制器对象。
 * @param[in] type PID 类型。
 * @param[in] para PID 参数数组，长度为 `PID_PARAMETER_NUM`。
 */
extern void PID_Init(PID_Info_TypeDef *Pid,PID_Type_e type,float para[PID_PARAMETER_NUM]);
/**
  * @brief 执行一次 PID 计算。
  *
  * @param[in,out] PID PID 控制器对象。
  * @param[in] Target 目标值。
  * @param[in] Measure 实际测量值。
  * @return 本次 PID 计算输出。
  */
extern float PID_Calculate(PID_Info_TypeDef *PID, float Target,float Measure);

/**
 * @brief `PID_Calculate` 的兼容包装，参数顺序为测量值在前。
 *
 * @param[in,out] pid PID 控制器对象。
 * @param[in] measure 实际测量值。
 * @param[in] target 目标值。
 * @return 本次 PID 计算输出。
 */
static inline float PID_calc(pid_type_def *pid, float measure, float target)
{
    return PID_Calculate(pid, target, measure);
}

/**
 * @brief 使用 3 参数形式快速初始化 PID 控制器。
 *
 * @param[out] pid PID 控制器对象。
 * @param[in] type PID 类型。
 * @param[in] param3 依次为 KP、KI、KD 的参数数组。
 * @param[in] max_out 输出限幅。
 * @param[in] max_iout 积分限幅。
 */
static inline void PID_init(pid_type_def *pid, PID_Type_e type, const float param3[3], float max_out, float max_iout)
{
    float para[PID_PARAMETER_NUM] = {
        param3[0], param3[1], param3[2], 0.0f, 0.0f, max_iout, max_out,
    };
    PID_Init(pid, type, para);
}

/* =========================================================================
 * Gimbal 扩展 PID 函数（统一 PID 实现，消除 gimbal_PID_t 重复代码）
 * ========================================================================= */

/**
 * @brief 扩展初始化 PID 控制器（支持 KF 前馈和 IZone 积分分离）。
 *
 * 在标准 PID_Init 基础上额外设置 gimbal 扩展字段：
 * Param.KF（前馈系数）、Param.IZone（积分分离区）、PrevTarget、Dbuf[3]。
 * IZone <= 0 时始终积分，> 0 时仅当 |err| <= IZone 才累加积分。
 *
 * @param[out] pid     PID 控制器对象。
 * @param[in]  kp      比例系数。
 * @param[in]  ki      积分系数。
 * @param[in]  kd      微分系数。
 * @param[in]  kf      前馈系数。
 * @param[in]  max_out 输出限幅。
 * @param[in]  max_iout 积分限幅。
 * @param[in]  i_zone  积分分离区阈值。
 */
static inline void PID_InitFull(pid_type_def *pid, float kp, float ki, float kd, float kf,
                                float max_out, float max_iout, float i_zone)
{
    float para[PID_PARAMETER_NUM] = {kp, ki, kd, 0.0f, 0.0f, max_iout, max_out};
    PID_Init(pid, PID_POSITION, para);
    pid->Param.KF = kf;
    pid->Param.IZone = i_zone;
    pid->PrevTarget = 0.0f;
    memset(pid->Dbuf, 0, sizeof(pid->Dbuf));
}

/**
 * @brief 角度环 PID 计算（用于云台位置控制）。
 *
 * 支持 i_zone 积分分离、kf 目标变化前馈、角度格式化。
 *
 * @param[in,out] pid         PID 控制器对象。
 * @param[in]     measure     当前反馈值。
 * @param[in]     target      目标值。
 * @param[in]     error_delta 微分项输入（通常为角速度反馈）。
 * @return PID 输出结果。
 */
float PID_CalculateAngle(pid_type_def *pid, float measure, float target, float error_delta);

/**
 * @brief 基于外部误差的 PID 计算（角度环变体）。
 *
 * @param[in,out] pid         PID 控制器对象。
 * @param[in]     error       已计算的误差值。
 * @param[in]     error_delta 微分项输入。
 * @return PID 输出结果。
 */
float PID_CalculateByError(pid_type_def *pid, float error, float error_delta);

/**
 * @brief 速度环 PID 计算（带 Dbuf 微分缓存和抗积分饱和）。
 *
 * @param[in,out] pid     PID 控制器对象。
 * @param[in]     measure 当前速度反馈。
 * @param[in]     target  目标速度。
 * @return PID 输出结果。
 */
float PID_CalculateVelocity(pid_type_def *pid, float measure, float target);

/**
 * @brief 清空 PID 内部状态（包括 gimbal 扩展字段）。
 *
 * @param[in,out] pid PID 控制器对象。
 */
void PID_ClearExtended(pid_type_def *pid);

#endif //CONTROLLER_PID_H


