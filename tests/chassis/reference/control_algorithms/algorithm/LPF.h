/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : lowpass_filter.c
  * @brief          : lowpass filter
  * @author         : Yan Yuan bin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : To be perfected
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef LOWPASS_FILTER_H
#define LOWPASS_FILTER_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/


/* Exported types ------------------------------------------------------------*/
/**
 * @brief 一阶低通滤波器状态。
 */
typedef struct
{
    bool Initialized; /**< 初始化标志，`true` 表示已完成初始化。 */
    float Input;      /**< 当前输入值。 */
    float Output;     /**< 当前输出值。 */
    float Alpha;      /**< 一阶滤波系数。 */
}LowPassFilter1p_Info_TypeDef;

/**
 * @brief 二阶低通滤波器状态。
 */
typedef struct
{
    bool Initialized; /**< 初始化标志，`true` 表示已完成初始化。 */
    float Input;      /**< 当前输入值。 */
    float Output[3];  /**< 当前及过去两次输出值。 */
    float Alpha[3];   /**< 二阶滤波器系数数组。 */

}LowPassFilter2p_Info_TypeDef;

/**
 * @brief 二阶低通滤波器类型别名。
 */
typedef LowPassFilter2p_Info_TypeDef second_order_filter_type_t;

/* Extern Functions Prototypes ---------------------------------------------*/

/**
  * @brief 初始化一阶低通滤波器。
  *
  * @param[out] LPF 一阶低通滤波器对象。
  * @param[in] Alpha 一阶滤波系数。
  */
extern void LowPassFilter1p_Init(LowPassFilter1p_Info_TypeDef *LPF,float Alpha);
/**
  * @brief 输入新数据并更新一阶低通滤波器输出。
  *
  * @param[in,out] lpf 一阶低通滤波器对象。
  * @param[in] input 当前输入值。
  * @return 更新后的滤波输出。
  */
extern float LowPassFilter1p_Update(LowPassFilter1p_Info_TypeDef *lpf,float input);
/**
  * @brief 初始化二阶低通滤波器。
  *
  * @param[out] lpf 二阶低通滤波器对象。
  * @param[in] alpha 二阶滤波器系数数组。
  */
extern void  LowPassFilter2p_Init(LowPassFilter2p_Info_TypeDef *lpf,float alpha[3]);
/**
  * @brief 输入新数据并更新二阶低通滤波器输出。
  *
  * @param[in,out] lpf 二阶低通滤波器对象。
  * @param[in] input 当前输入值。
  * @return 更新后的滤波输出。
  */
extern float LowPassFilter2p_Update(LowPassFilter2p_Info_TypeDef *lpf,float input);

/**
 * @brief `LowPassFilter2p_Init` 的兼容包装接口。
 *
 * @param[out] filter 二阶低通滤波器对象。
 * @param[in] alpha 二阶滤波器系数数组。
 */
static inline void second_order_filter_init(second_order_filter_type_t *filter, const float alpha[3])
{
    LowPassFilter2p_Init(filter, (float *)alpha);
}

/**
 * @brief 输入新数据并更新二阶低通滤波器状态。
 *
 * @param[in,out] filter 二阶低通滤波器对象。
 * @param[in] input 当前输入值。
 */
static inline void second_order_filter_cali(second_order_filter_type_t *filter, float input)
{
    (void)LowPassFilter2p_Update(filter, input);
}

/**
 * @brief 获取输入值的符号。
 *
 * @param[in] input 输入值。
 * @return 大于 0 返回 1，小于 0 返回 -1，等于 0 返回 0。
 */
extern float sign(float input);
#ifdef __cplusplus
}
#endif
#endif //LOWPASS_FILTER_H
