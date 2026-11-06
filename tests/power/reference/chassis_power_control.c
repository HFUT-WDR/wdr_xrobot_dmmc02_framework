/**
  ****************************(C) COPYRIGHT 2023 jiewen_shen****************************
  * @file       chassis_power_control.c/h
  * @brief      chassis power control. 底盘功率控制(RLS自适应 + 超级电容 + 逐电机功率分配)
  * @note       移植自 D:\Learn\RobotMaster\chassis_power_control
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     16-4-2023       jiewen_shen     1. 缓冲能量线性比例限幅
  *  V2.0.0     2026-03-30      migrated        2. 适配MC02底盘闭环
  *  V3.0.0     2026-06-15      upgraded        3. PD能量环 + 混合权重二次方程功率分配
  *
  @verbatim
  ==============================================================================
  功率模型: P_i = tau_i * omega_i + k1*|omega_i| + k2*tau_i^2 + k3/4
  k1,k2 由RLS在线辨识, k3为静态损耗常数(RLS_K3_INIT)

  能量环:
  电容能量/缓冲能量 -> PD闭环(sqrt变换) -> 功率窗口[fullMaxPower, baseMaxPower]
  能量高时扩展功率区间, 能量低时迅速收缩功率使用

  功率分配:
  总需求超预算时, 按"跟踪误差 + 原始功率占比"混合权重分配,
  逐电机二次方程反解扭矩电流, 非简单等比例缩放
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2023 jiewen_shen****************************
  */
#include "chassis/chassis_power_control.h"

#include <math.h>

#include "CAN_CAPower.h"
#include "referee/referee.h"

static Power_RLS_t Power_RLS;
volatile power_ctrl_debug_t power_dbg = {0};

/* ==================== 能量环 PD 控制器 ==================== */
static PID_Info_TypeDef energy_pd_base;  /* 低能量保护 (setpoint=baseBuffSet), 确保系统不断电 */
static PID_Info_TypeDef energy_pd_full;  /* 高能量管理 (setpoint=fullBuffSet), 确保电容能量不过满 */
static uint8_t energy_loop_initialized = 0U;

void energy_loop_init(void)
{
    if (energy_loop_initialized) return;
    const float pd3[3] = {ENERGY_PD_KP, 0.0f, ENERGY_PD_KD};
    PID_init(&energy_pd_base, PID_POSITION, pd3, ENERGY_PD_MAX_OUT, ENERGY_PD_MAX_OUT);
    PID_init(&energy_pd_full, PID_POSITION, pd3, ENERGY_PD_MAX_OUT, ENERGY_PD_MAX_OUT);
    energy_loop_initialized = 1U;
}

/*
 * 将裁判允许功率转换成发给超级电容板的输入功率目标。
 * 这里不是简单直接取 max_power_limit，而是根据当前底盘实际输入功率 chassis_power
 * 在 85%~95% 功率区间内做一段线性过渡:
 * 1. 功率明显偏低时，鼓励电容继续放功，快速向上逼近上限。
 * 2. 功率已经接近上限时，减小补偿力度，避免在功率线附近来回抖动。
 */
static fp32 calculate_input_power(const fp32 max_power_limit, const fp32 chassis_power)
{
    const fp32 POWER_FILTER_GAIN = 0.3f;
    fp32 k = 0.0f;
    fp32 lower_bound = 0.85f * max_power_limit;
    fp32 upper_bound = 0.95f * max_power_limit;
    fp32 result = 0.0f;

    if (max_power_limit < 1e-3f) {
        return 0.0f;
    }

    if (chassis_power < lower_bound) {
        k = 1.0f - POWER_FILTER_GAIN;
    } else if (chassis_power > upper_bound) {
        k = POWER_FILTER_GAIN;
    } else {
        fp32 t = (chassis_power - lower_bound) / (upper_bound - lower_bound);
        k = (1.0f - POWER_FILTER_GAIN) - t * (1.0f - 2.0f * POWER_FILTER_GAIN);
    }

    result = chassis_power + k * (max_power_limit - chassis_power);
    return result > max_power_limit ? max_power_limit : result;
}

/*
 * 初始化 2 维 RLS 参数。
 * theta[0] -> 模型中的 k1，对应速度相关损耗。
 * theta[1] -> 模型中的 k2，对应扭矩平方损耗。
 * P 为协方差矩阵，初值越大说明初始阶段允许参数更新得更激进。
 */
void Power_RLS_init(Power_RLS_t *rls, float k1_init, float k2_init, float lambda)
{
    if (rls == NULL) {
        return;
    }

    rls->theta[0] = k1_init;
    rls->theta[1] = k2_init;
    rls->lambda = lambda;
    rls->P[0][0] = RLS_P;
    rls->P[0][1] = 0.0f;
    rls->P[1][0] = 0.0f;
    rls->P[1][1] = RLS_P;
    rls->estimated_Power = 0.0f;
}

/*
 * 标准递推最小二乘(RLS)更新。
 * 输入量 phi = [sum(|omega|), sum(tau^2)]，输出量 y 为"实测总功率扣掉有效机械功和静态损耗"后的部分。
 * 更新目标是让 k1、k2 在线贴合当前底盘状态，使后续功率分配更接近真实电机/底盘损耗。
 */
void Power_RLS_update(Power_RLS_t *rls, const float phi[2], float y)
{
    float P_phi[2];
    float denom = 0.0f;
    float K[2];
    float y_hat = 0.0f;
    float error = 0.0f;
    float P_new[2][2];

    if (rls == NULL || phi == NULL) {
        return;
    }

    P_phi[0] = rls->P[0][0] * phi[0] + rls->P[0][1] * phi[1];
    P_phi[1] = rls->P[1][0] * phi[0] + rls->P[1][1] * phi[1];

    // denom 过小意味着本次样本对参数不可辨识，继续更新会导致数值炸掉。
    denom = rls->lambda + phi[0] * P_phi[0] + phi[1] * P_phi[1];
    if (!isfinite(denom) || fabsf(denom) < 1e-6f || !isfinite(rls->lambda) || rls->lambda <= 1e-6f) {
        power_dbg.rls_bad_denom_cnt++;
        return;
    }

    K[0] = P_phi[0] / denom;
    K[1] = P_phi[1] / denom;

    y_hat = rls->theta[0] * phi[0] + rls->theta[1] * phi[1];
    error = y - y_hat;
    if (!isfinite(error)) {
        power_dbg.rls_bad_error_cnt++;
        return;
    }

    // 用当前残差修正 k1 / k2.
    rls->theta[0] += K[0] * error;
    rls->theta[1] += K[1] * error;

    for (uint8_t i = 0; i < 2U; i++) {
        for (uint8_t j = 0; j < 2U; j++) {
            P_new[i][j] = (rls->P[i][j] - K[i] * P_phi[j]) / rls->lambda;
        }
    }

    rls->P[0][0] = P_new[0][0];
    rls->P[0][1] = P_new[0][1];
    rls->P[1][0] = P_new[1][0];
    rls->P[1][1] = P_new[1][1];

    if (rls->theta[0] < RLS_K1_MIN_VAL) {
        rls->theta[0] = RLS_K1_MIN_VAL;
    }
    else if (rls->theta[0] > RLS_K1_MAX_VAL) {
        rls->theta[0] = RLS_K1_MAX_VAL;
    }
    if (rls->theta[1] < RLS_K2_MIN_VAL) {
        rls->theta[1] = RLS_K2_MIN_VAL;
    }
    else if (rls->theta[1] > RLS_K2_MAX_VAL) {
        rls->theta[1] = RLS_K2_MAX_VAL;
    }
}

Power_RLS_t *get_Power_RLS_point(void)
{
    return &Power_RLS;
}

void chassis_power_estimated(Power_RLS_t *rls)
{
    chassis_move_t *chassis = get_chassis_move_point();
    fp32 effectivePower = 0.0f;
    float samples[2] = {0.0f, 0.0f};

    if (rls == NULL || chassis == NULL) {
        return;
    }

    for (uint8_t i = 0U; i < 4U; i++) {
        const motor_3508_measure_t *measure = chassis->motor_chassis[i].chassis_motor_measure;
        fp32 tau = 0.0f;
        fp32 omega = 0.0f;

        if (measure == NULL) {
            continue;
        }

        tau = TORQUE_COEFF * (fp32)measure->given_current;
        omega = (fp32)measure->speed_rpm * RPM2RAD;

        // 有效机械功 tau*omega，对应真正推动底盘做功的部分。
        effectivePower += tau * omega;
        // RLS 的两个回归量分别累计总转速损耗和总铜耗/电流平方损耗。
        samples[0] += fabsf(omega);
        samples[1] += tau * tau;
    }

    rls->estimated_Power = effectivePower + rls->theta[0] * samples[0] + rls->theta[1] * samples[1] + RLS_K3_INIT;
    power_dbg.watch_power = rls->estimated_Power;
    power_dbg.watch_power_error = get_CAPower_measure_point()->Pout - effectivePower - RLS_K3_INIT;

    // 功率太小时噪声占比很高，不拿来更新 RLS，避免参数被小信号带偏。
    if (fabsf(get_CAPower_measure_point()->Pout) > 15.0f) {
        Power_RLS_update(rls, samples, get_CAPower_measure_point()->Pout - effectivePower - RLS_K3_INIT);
    }
}

/*
 * 底盘总功率控制主流程:
 * 1. 读取裁判功率限制与超级电容状态。
 * 2. PD能量环: 电容/缓冲能量 → sqrt变换 → PD闭环 → 功率窗口[fullMaxPower, baseMaxPower]。
 * 3. 逐电机估算功率需求，统计跟踪误差。
 * 4. 若总需求没超预算，保持原 PID 输出不动。
 * 5. 若超预算，按"跟踪误差权重 + 原始功率占比权重"混合分配给每个电机。
 * 6. 二次方程反解允许的最大扭矩电流，钳位后回写 PID Output。
 * 7. 反积分饱和。
 */
void chassis_power_control(chassis_move_t *chassis_power_ctrl, Power_RLS_t *rls)
{
    const fp32 DENOM_EPS = 1e-6f;
    float chassis_power = 0.0f;
    float max_power_limit = 0.0f;
    float chassis_max_power = 0.0f;
    float initial_give_power[4] = {0.0f};
    float initial_total_power = 0.0f;
    float error[4] = {0.0f};
    float error_sum = 0.0f;
    fp32 referee_chassis_power = 0.0f;
    fp32 referee_buffer = 0.0f;
    float allocatablePower = 0.0f;
    float cap_target = 0.0f;
    float sumPowerRequired = 0.0f;
    float errorConfidence = 0.0f;
    uint8_t i;

    if (chassis_power_ctrl == NULL || rls == NULL) {
        return;
    }

    energy_loop_init();

    get_chassis_power_and_buffer(&referee_chassis_power, &referee_buffer, &max_power_limit);
    if (fabsf(max_power_limit) <= 0.1f) {
        max_power_limit = 100.0f;
    }

    // 这里使用超电板测得的输入功率 Pin 作为当前底盘实际用的功率。
    chassis_power = get_CAPower_measure_point()->Pin;
    if (!isfinite(chassis_power) || chassis_power < 0.0f) {
        power_dbg.pout_invalid_cnt++;
        chassis_power = 0.0f;
    }

    // 向超级电容板发送目标功率。
    cap_target = calculate_input_power(max_power_limit, chassis_power);
    CAP_CAN_DataSend(cap_target - 5.0f, CHASSIS_CAP_ENABLE);

    /* ==================== PD 能量环 ====================
     * 电容/缓冲能量经过 sqrt 非线性变换后进入 PD 闭环。
     * sqrt 变换: 能量高区间被压缩 (不敏感), 能量低区间被扩展 (敏感),
     * 实现"能量高时扩展功率区间, 能量低时迅速收缩功率"的效果。
     * baseMaxPower: 上界, 能量低时压低防止断电。
     * fullMaxPower: 下界, 能量高时抬高允许爆发输出。
     */
    {
        float powerBuff, fullBuffSet, baseBuffSet;
        const CapDataTypedef *cap = get_CAPower_measure_point();

        // 能量反馈源选择: 电容优先, 裁判系统缓冲能量回退
        if (cap != NULL && cap->cap_energy > 10.0f) {
            powerBuff   = cap->cap_energy;
            fullBuffSet = CAP_FULL_BUFF_SET;
            baseBuffSet = CAP_BASE_BUFF_SET;
        } else {
            powerBuff   = referee_buffer;
            fullBuffSet = REFEREE_FULL_BUFF;
            baseBuffSet = REFEREE_BASE_BUFF;
        }

        // PD 闭环计算: sqrt 变换后进入 PD
        float pd_base_out = PID_Calculate(&energy_pd_base, sqrtf(baseBuffSet), sqrtf(powerBuff));
        float pd_full_out = PID_Calculate(&energy_pd_full, sqrtf(fullBuffSet), sqrtf(powerBuff));

        // 功率窗口: 从裁判功率上限减去 PD 修正量
        float baseMaxPower = fmaxf(max_power_limit - pd_base_out, MIN_MAX_POWER);
        float fullMaxPower = fmaxf(max_power_limit - pd_full_out, MIN_MAX_POWER);

        // 钳位: 用户期望功率限制在 [fullMaxPower, baseMaxPower] 窗口内
        chassis_max_power = cap_target;
        VAL_LIMIT(chassis_max_power, fullMaxPower, baseMaxPower);

        // 调试信息
        power_dbg.energy_pd_base_out = pd_base_out;
        power_dbg.energy_pd_full_out = pd_full_out;
        power_dbg.energy_power_buff  = powerBuff;
        power_dbg.energy_base_max    = baseMaxPower;
        power_dbg.energy_full_max    = fullMaxPower;
    }

    allocatablePower = chassis_max_power;

    /* ==================== 逐电机功率估算 + 误差统计 ==================== */
    for (i = 0U; i < 4U; i++) {
        const motor_3508_measure_t *measure = chassis_power_ctrl->motor_chassis[i].chassis_motor_measure;
        fp32 drive_omega = 0.0f;
        fp32 pid_out = chassis_power_ctrl->motor_speed_pid[i].Output;

        if (measure == NULL) {
            continue;
        }

        drive_omega = (fp32)measure->speed_rpm * RPM2RAD;
        initial_give_power[i] = pid_out * TORQUE_COEFF * drive_omega +
                                rls->theta[0] * fabsf(drive_omega) +
                                rls->theta[1] * pid_out * pid_out * TORQUE_COEFF * TORQUE_COEFF +
                                RLS_K3_INIT / 4.0f;

        // 跟踪误差 (转换到 rad/s 以匹配阈值常量)
        error[i] = fabsf(chassis_power_ctrl->motor_chassis[i].speed_set -
                         chassis_power_ctrl->motor_chassis[i].speed) * SPEED_TO_RADS;

        // 负功/零功率: 不参与分配, 保持原PID输出 (负功对总预算影响小, 处理反而增加虚根概率)
        if (initial_give_power[i] <= 0.0f) {
            continue;
        }

        initial_total_power += initial_give_power[i];
        error_sum += error[i];
        sumPowerRequired += initial_give_power[i];
    }

    // 理想总需求没超功率预算时，不做人为压缩，直接使用上层 PID 输出
    if (initial_total_power <= chassis_max_power) {
        power_dbg.error_confidence = 0.0f;
        return;
    }

    /* ==================== 混合权重功率分配 ====================
     * 误差置信度: 总跟踪误差大 → 偏误差权重 (照顾更需要动力的电机)
     *            总跟踪误差小 → 偏比例权重 (维持麦轮扭矩平衡)
     */
    if (error_sum > ERROR_POWER_DISTRIBUTION_SET) {
        errorConfidence = 1.0f;
    } else if (error_sum > PROP_POWER_DISTRIBUTION_SET) {
        errorConfidence = (error_sum - PROP_POWER_DISTRIBUTION_SET) /
                          (ERROR_POWER_DISTRIBUTION_SET - PROP_POWER_DISTRIBUTION_SET);
    } else {
        errorConfidence = 0.0f;
    }
    power_dbg.error_confidence = errorConfidence;

    /* 逐电机: 混合权重 → 目标功率 → 二次方程反解扭矩电流 */
    for (i = 0U; i < 4U; i++) {
        const motor_3508_measure_t *measure = chassis_power_ctrl->motor_chassis[i].chassis_motor_measure;

        if (measure == NULL || initial_give_power[i] <= 0.0f) {
            continue;
        }

        fp32 drive_omega = (fp32)measure->speed_rpm * RPM2RAD;
        fp32 pid_out = chassis_power_ctrl->motor_speed_pid[i].Output;

        // 混合权重: errorConfidence * 误差权重 + (1-errorConfidence) * 功率占比权重
        float powerWeight_Error = (error_sum > DENOM_EPS) ? error[i] / error_sum : 0.25f;
        float powerWeight_Prop  = (sumPowerRequired > DENOM_EPS) ? initial_give_power[i] / sumPowerRequired : 0.25f;
        float powerWeight = errorConfidence * powerWeight_Error + (1.0f - errorConfidence) * powerWeight_Prop;

        // 该电机分配到的目标功率
        float P_target = powerWeight * allocatablePower;

        /*
         * 二次方程反解扭矩电流:
         * P = tau*omega + k1*|omega| + k2*tau^2 + k3/4
         * 代入 tau = TORQUE_COEFF * I:
         * k2*TC^2*I^2 + TC*omega*I + (k1*|omega| + k3/4 - P_target) = 0
         *
         * I = (-omega ± sqrt(omega^2 - 4*k2*(k1*|omega| + k3/4 - P_target))) / (2*k2*TC)
         *    (TC 因子在分子分母中约掉)
         */
        float inner = drive_omega * drive_omega -
                      4.0f * rls->theta[1] * (rls->theta[0] * fabsf(drive_omega) + RLS_K3_INIT / 4.0f - P_target);
        float current;

        if (inner < 0.0f) {
            // 虚根: 取顶点 (最小功率点, 即功率最接近目标的点)
            current = -drive_omega / (2.0f * rls->theta[1] * TORQUE_COEFF);
        } else if (fabsf(inner) < DENOM_EPS) {
            // 重根
            current = -drive_omega / (2.0f * rls->theta[1] * TORQUE_COEFF);
        } else {
            // 两不等实根: 取与原 PID 输出符号一致的根
            float sqrt_inner = sqrtf(inner);
            float denom = 2.0f * rls->theta[1] * TORQUE_COEFF;
            if (pid_out > 0.0f) {
                current = (-drive_omega + sqrt_inner) / denom;
            } else {
                current = (-drive_omega - sqrt_inner) / denom;
            }
        }

        // 钳位到电机电流范围
        VAL_LIMIT(current, -MAX_MOTOR_CAN_CURRENT, MAX_MOTOR_CAN_CURRENT);

        chassis_power_ctrl->motor_speed_pid[i].Output = current;

        // 反积分饱和: 功率限幅后积分与输出不匹配,
        // 反算一个匹配的积分值写入 PID, 避免积分空转累积导致功率退出时电流跳变。
        {
            PID_Info_TypeDef *pid = &chassis_power_ctrl->motor_speed_pid[i];
            if (pid->Param.KI > 1e-6f) {
                fp32 iout_actual = current - pid->Pout - pid->Dout;
                VAL_LIMIT(iout_actual, -pid->Param.LimitIntegral, pid->Param.LimitIntegral);
                pid->Integral = iout_actual / pid->Param.KI;
            }
        }
    }
}
