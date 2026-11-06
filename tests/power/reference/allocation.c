// 原车分配函数尾段逐字提取；外层仅提供离线结构和输入。
#include <math.h>
#include <stdint.h>
#include <stddef.h>
typedef float fp32;
#define TORQUE_COEFF 1.90484e-5f
#define RLS_K3_INIT 3.77f
#define RPM2RAD (3.14159265358979323846f / 30.0f)
#define SPEED_TO_RADS 1.0f
#define ERROR_POWER_DISTRIBUTION_SET 20.0f
#define PROP_POWER_DISTRIBUTION_SET 15.0f
#define MAX_MOTOR_CAN_CURRENT 16000.0f
#define VAL_LIMIT(v, lo, hi) do { if ((v)<(lo)) (v)=(lo); else if ((v)>(hi)) (v)=(hi); } while(0)
typedef struct { float speed_rpm; } motor_3508_measure_t;
typedef struct { float Output, Pout, Dout, Integral; struct { float KI, LimitIntegral; } Param; } PID_Info_TypeDef;
typedef struct { motor_3508_measure_t* chassis_motor_measure; float speed_set, speed; } Wheel;
typedef struct { Wheel motor_chassis[4]; PID_Info_TypeDef motor_speed_pid[4]; } chassis_move_t;
typedef struct { float theta[2], P[2][2], lambda, estimated_Power; } Power_RLS_t;
static struct { float error_confidence; unsigned rls_bad_denom_cnt, rls_bad_error_cnt; } power_dbg;
static void allocate(chassis_move_t* chassis_power_ctrl, Power_RLS_t* rls, float chassis_max_power)
{
    const fp32 DENOM_EPS = 1e-6f;
    float initial_give_power[4] = {0};
    float initial_total_power = 0;
    float error[4] = {0};
    float error_sum = 0, allocatablePower = 0, sumPowerRequired = 0, errorConfidence = 0;
    uint8_t i;
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

void reference_allocate(const float rpm[4], const float error[4], const float requested[4],
                        float budget, float k1, float k2, float output[4])
{
  chassis_move_t chassis = {0};
  motor_3508_measure_t measure[4] = {0};
  Power_RLS_t rls = {{k1, k2}};
  for (unsigned i = 0; i < 4; ++i)
  {
    measure[i].speed_rpm = rpm[i];
    chassis.motor_chassis[i].chassis_motor_measure = &measure[i];
    chassis.motor_chassis[i].speed_set = error[i];
    chassis.motor_speed_pid[i].Output = requested[i];
  }
  allocate(&chassis, &rls, budget);
  for (unsigned i = 0; i < 4; ++i) output[i] = chassis.motor_speed_pid[i].Output;
}

#define RLS_K1_MIN_VAL 0.00011f
#define RLS_K1_MAX_VAL 0.03f
#define RLS_K2_MIN_VAL 440.0f
#define RLS_K2_MAX_VAL 600.0f
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


void reference_rls(float x0, float x1, float y, int reset, float output[2])
{
  static Power_RLS_t rls;
  if (reset)
  {
    rls = (Power_RLS_t){.theta={.01470f,476.3f}, .P={{.1f,0},{0,.1f}}, .lambda=.99999f};
  }
  float phi[2] = {x0,x1};
  Power_RLS_update(&rls,phi,y);
  output[0]=rls.theta[0]; output[1]=rls.theta[1];
}
