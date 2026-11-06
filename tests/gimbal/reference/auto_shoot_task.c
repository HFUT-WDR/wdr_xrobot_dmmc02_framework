#include <math.h>

#include "arm_math.h"

#include "gimbal/auto_shoot_task.h"
#include "chassis/chassis_task.h"
#include "gimbal_behaviour.h"
#include "referee/referee.h"
#include "shoot/shoot_task.h"
#include "usb_callback.h"
#include "user_lib.h"

// 本文件负责“自动模式下云台该往哪看、什么时候允许开火”。
// 核心逻辑包括：
// 1. 视觉跟踪时的开火门限判断；
// 2. 巡航扫描/受击回头逻辑；
// 3. 双 yaw 结构下的角度分配与限位处理。
#define LARGE_ARMOR_WIDTH 0.23f
#define SMALL_ARMOR_WIDTH (0.14f * 0.8f)
#define YAW_COMP_L_BARREL 0.10f
#define YAW_COMP_K 1.0f
#define MIN_BULLET_SPEED 5.0f

// 定义硬限位(绝对不能撞的地方)
#define LIMIT_HARD (45.0f * ANGLE_TO_RAD)
// 定义软限位起效点(从这里开始变软)
#define LIMIT_START (35.0f * ANGLE_TO_RAD)

static uint8_t get_attacked_angle(fp32 *attacked_angle);

uint32_t Reset_Tick = 0U; // 上次复位云台的时间
uint8_t attack_flag = 0U;
// attack_mask 用于给受击回头加一段屏蔽时间，避免同一次受击被重复响应。
static uint8_t attack_mask = 0U;
// 巡航扫描时的转向标志：0 正向扫，1 反向扫。
static uint8_t turn_flag = 0U;

/**
  * @brief          自动射击逻辑接口
  * @param[in]      void
  * @retval         1: 可以击发 0: 不可击发
  */
int Get_Shoot_Brief(void)
{
    const VisionToGimbal *vision = get_latest_vision_data();
    float distance = 0.0f;
    float current_yaw = 0.0f;
    float bullet_speed = 0.0f;
    float yaw_comp = 0.0f;
    float yaw_err_arc = 0.0f;
    float half_width = 0.0f;

    if (vision == NULL) {
        return 0;
    }

    // 正在执行一次击发结束流程时，直接禁止再次开火。
    if (get_interrupt_flag() == FIRE_END) {
        return 0;
    }

    // 必须处于自动开火模式且视觉稳定跟踪到目标，才进入后续判定。
    if (get_auto_fire_control() != FIRE_MODE || !vision->tracking) {
        return 0;
    }

    distance = vision->distance;
    if (distance < 0.1f) {
        return 0;
    }

    bullet_speed = get_bullet_speed();
    if (bullet_speed < MIN_BULLET_SPEED) {
        bullet_speed = MIN_BULLET_SPEED;
    }

    // 优先使用达妙 IMU 的姿态与角速度；若不可用，则退回到云台电机反馈。
    if (gimbal_control.gimbal_INT_angle_point_DM != NULL) {
        current_yaw = rad_format(gimbal_control.gimbal_INT_angle_point_DM->yaw * ANGLE_TO_RAD);
        yaw_comp = atan2f(gimbal_control.gimbal_INT_angle_point_DM->gyro[2] * YAW_COMP_L_BARREL, bullet_speed) *
                   YAW_COMP_K;
    } else {
        current_yaw = gimbal_control.gimbal_yaw_motor[INNER_YAW].absolute_angle;
        yaw_comp = atan2f(gimbal_control.gimbal_yaw_motor[INNER_YAW].motor_gyro * YAW_COMP_L_BARREL, bullet_speed) *
                   YAW_COMP_K;
    }

    // yaw_comp 目前保留接口但未真正参与开火门限，后续若要加入前视补偿可直接接入。
    (void)yaw_comp;
    // 把 yaw 角误差换算成目标平面上的弧长误差，再和装甲板等效半宽做比较。
    // armor_yaw 越偏，装甲板在视线方向上的有效宽度越小，因此要乘 cos 衰减。
    // yaw_err_arc = fabsf(rad_format(vision->target_yaw - current_yaw)) * distance;
    // half_width = (vision->id == 1U) ? (LARGE_ARMOR_WIDTH * 0.5f) : (SMALL_ARMOR_WIDTH * 0.5f);
  //  return (yaw_err_arc < half_width * fabsf(arm_cos_f32(vision->armor_yaw))) ? 1 : 0;
    return 0;
}

const uint8_t *get_auto_mode_flag(void)
{
    return (uint8_t *)&pc_flag;
}

// 2024.2.2 自瞄函数
void Auto_Track(yaw_set_t *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    static VisionToGimbal gimbal_control_from_upper;
    fp32 current_total = 0.0f;
    fp32 increment = 0.0f;
    fp32 target_absolute_angle = 0.0f;

    get_vision_data(&gimbal_control_from_upper); // copy内存
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL) {
        return;
    }

    // pitch 由上位机直接给绝对角目标；当前双 yaw 结构下，这里的 *pitch 不参与额外分配。
    gimbal_control_set->gimbal_pitch_motor.absolute_angle_set = gimbal_control_from_upper.pitch;
    target_absolute_angle = gimbal_control_from_upper.yaw;
    *pitch = 0.0f;

    if (gimbal_control_set->yaw_conut > 1U) {
        // 目标给的是“总绝对 yaw”，因此先算出总增量，再按双 yaw 机构能力分配给大小 yaw。
        current_total = rad_format(gimbal_control_set->gimbal_yaw_motor[INNER_YAW].absolute_angle_set +
                                   gimbal_control_set->gimbal_yaw_motor[OUTER_YAW].absolute_angle_set);
        increment = rad_format(target_absolute_angle - current_total);
        DualYawAbsoluteAngleAllocation(increment, &yaw->outer_yaw, &yaw->inner_yaw, gimbal_control_set);
    } else {
        yaw->inner_yaw = 0.0f;
        yaw->outer_yaw = 0.0f;
    }
}

// 巡航检测敌人
void Auto_Detect(yaw_set_t *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    fp32 current_yaw = 0.0f;
    static fp32 pit_remap_angle = 0.0f;
    static fp32 pitch_pos = 0.0f;

    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL) {
        return;
    }

    current_yaw = gimbal_control_set->gimbal_yaw_motor[OUTER_YAW].absolute_angle;
    // pitch是yaw轴绝对角度三倍频后的正弦映射
    pit_remap_angle = loop_fp32_constrain(3.0f * current_yaw + 0.2f * PI, 0.0f, 2.0f * PI);
    pitch_pos = 0.075f * sinf(pit_remap_angle) + 0.075f;

    // 被击打后拉起 attack_flag，在一段时间内允许上层切到受击回头逻辑。
    if (ref_get_attacked_cb() == 1U && attack_mask == 0U) {
        // 检测到被击打并且不处于屏蔽时间内
        // detect_attacked_angle(yaw, pitch, gimbal_control_set);
        if (HAL_GetTick() - ref_get_hit_tick() > 0U && attack_flag == 0U) {
            attack_flag = 1U;
        }
        if (HAL_GetTick() - ref_get_hit_tick() > 5000U) {
            // 最长5s响应时间 屏蔽匀速旋转的累加
            ref_clear_attacked_cb();
            attack_mask = 1U;
            attack_flag = 0U;
        }
    }

    if (HAL_GetTick() - Reset_Tick > 2000U) {
        // 最后一次复位云台时间超过2s
        attack_mask = 0U;
    }

    gimbal_control_set->gimbal_pitch_motor.absolute_angle_set = pitch_pos;
    *pitch = 0.0f;

    // 外 yaw 扫到机械限位时反向，实现往返巡航。
    if (fabsf(gimbal_control_set->gimbal_yaw_motor[OUTER_YAW].relative_angle -
              gimbal_control_set->gimbal_yaw_motor[OUTER_YAW].max_relative_angle) < 0.01f) {
        if (turn_flag == 0U) {
            turn_flag = 1U;
        }
    } else if (fabsf(gimbal_control_set->gimbal_yaw_motor[OUTER_YAW].relative_angle -
                     gimbal_control_set->gimbal_yaw_motor[OUTER_YAW].min_relative_angle) < 0.01f) {
        if (turn_flag == 1U) {
            turn_flag = 0U;
        }
    }

    // 巡航模式默认只让大 yaw 做主扫描，小 yaw 基本保持不动。
    yaw->outer_yaw = (turn_flag == 0U) ? GIMBAL_YAW_SPEED : -GIMBAL_YAW_SPEED;
    yaw->inner_yaw = -0.0001f;
}

// 受击反馈
static uint8_t get_attacked_angle(fp32 *attacked_angle)
{
    uint8_t type = 0U;
    uint8_t id = 0U;

    if (attacked_angle == NULL) {
        return 0U;
    }

    get_attacked_type(&id, &type);
    if (type == 0U && attack_flag == 1U) {
        // 装甲编号映射成车体坐标系方向，再叠加底盘角度与旋转补偿，得到世界系受击方向。
        *attacked_angle = loop_fp32_constrain(chassis_move.chassis_yaw + ((fp32)id * PI / 2.0f) - PI / 4.0f -
                                              (B_K * chassis_move.wz + B_E),
                                              -PI,
                                              PI);
        return 1U;
    }

    *attacked_angle = 0.0f;
    return 0U;
}

void detect_attacked_angle(yaw_set_t *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    fp32 attacked_angle = 0.0f;
    fp32 target_absolute_angle = 0.0f;
    fp32 current_total = 0.0f;
    fp32 increment = 0.0f;

    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL) {
        return;
    }

    if (get_attacked_angle(&attacked_angle)) {
        target_absolute_angle = attacked_angle;
        if (gimbal_control_set->yaw_conut > 1U) {
            // 受击响应同样按“目标总绝对角 -> 双 yaw 分配”的流程处理。
            current_total = gimbal_control_set->gimbal_yaw_motor[INNER_YAW].absolute_angle +
                            gimbal_control_set->gimbal_yaw_motor[OUTER_YAW].absolute_angle;
            increment = rad_format(target_absolute_angle - current_total);
            DualYawAbsoluteAngleAllocation(increment, &yaw->outer_yaw, &yaw->inner_yaw, gimbal_control_set);
            gimbal_control_set->gimbal_yaw_motor[INNER_YAW].absolute_angle_set =
                yaw->inner_yaw + gimbal_control_set->gimbal_yaw_motor[INNER_YAW].absolute_angle;
            gimbal_control_set->gimbal_yaw_motor[OUTER_YAW].absolute_angle_set =
                yaw->outer_yaw + gimbal_control_set->gimbal_yaw_motor[OUTER_YAW].absolute_angle;
        } else {
            yaw->inner_yaw = 0.0f;
            yaw->outer_yaw = 0.0f;
            return;
        }
        gimbal_control_set->gimbal_pitch_motor.absolute_angle_set = 0.15f; // pitch轴低头固定角度
        *pitch = 0.0f;
        // 记录一次“受击回头复位时间”，用于后续屏蔽窗口控制。
        Reset_Tick = HAL_GetTick();
    } else {
        yaw->inner_yaw = 0.0f;
        yaw->outer_yaw = 0.0f;
        *pitch = 0.0f;
    }
}

void Auto_Wait(yaw_set_t *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    (void)gimbal_control_set;
    if (yaw == NULL || pitch == NULL) {
        return;
    }
    yaw->outer_yaw = 0.0f;
    yaw->inner_yaw = 0.0f;
    *pitch = 0.0f;
}

void Auto_Move(yaw_set_t *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    (void)gimbal_control_set;
    if (yaw == NULL || pitch == NULL) {
        return;
    }
    *pitch = 0.0f;
    yaw->inner_yaw = 0.0f;
    yaw->outer_yaw = 0.0f;
}

/**
  * @brief          双Yaw绝对角度智能分配
  * @param[in/out]  yaw_increment: yaw角度增量（输入原始值，输出分配后值）
  * @param[in]      gimbal_control_set: 云台控制数据
  * @retval         none
  */
void DualYawAbsoluteAngleAllocation(fp32 yaw_increment, fp32 *outer_yaw, fp32 *inner_yaw,
                                    gimbal_control_t *gimbal_control_set)
{
    fp32 limit_bias = 0.10f; // 限位缓冲区间
    fp32 max_outer = 0.0f;
    fp32 min_outer = 0.0f;
    fp32 current_setpoint = 0.0f;
    fp32 upper_threshold = 0.0f;
    fp32 lower_threshold = 0.0f;
    fp32 k = 0.6f;

    if (outer_yaw == NULL || inner_yaw == NULL || gimbal_control_set == NULL) {
        return;
    }

    max_outer = gimbal_control_set->gimbal_yaw_motor[OUTER_YAW].max_relative_angle;
    min_outer = gimbal_control_set->gimbal_yaw_motor[OUTER_YAW].min_relative_angle;
    current_setpoint = gimbal_control_set->gimbal_yaw_motor[OUTER_YAW].relative_angle + yaw_increment;
    upper_threshold = max_outer - limit_bias;
    lower_threshold = min_outer + limit_bias;

    *outer_yaw = 0.0f;
    *inner_yaw = 0.0f;

    // 默认优先由外 yaw 吃下全部角度增量；
    // 只有当外 yaw 即将逼近限位时，才把一部分增量分给内 yaw 接手。
    if (yaw_increment > 0.0f) {
        if (current_setpoint > upper_threshold) {
            gimbal_control_set->gimbal_yaw_motor[OUTER_YAW].is_limited = 1;
            *outer_yaw = yaw_increment * k + 0.13f;
            *inner_yaw = yaw_increment * (1.0f - k);
        } else {
            gimbal_control_set->gimbal_yaw_motor[OUTER_YAW].is_limited = 0;
            *outer_yaw = yaw_increment;
            *inner_yaw = 0.0f;
        }
    } else if (yaw_increment < 0.0f) {
        if (current_setpoint < lower_threshold) {
            gimbal_control_set->gimbal_yaw_motor[OUTER_YAW].is_limited = 1;
            *outer_yaw = yaw_increment * k - 0.13f;
            *inner_yaw = yaw_increment * (1.0f - k);
        } else {
            gimbal_control_set->gimbal_yaw_motor[OUTER_YAW].is_limited = 0;
            *outer_yaw = yaw_increment;
            *inner_yaw = 0.0f;
        }
    }
}

// 在坐标系统一的前提下，二者应当具有相同的增量
void new_dual_yaw_absolute_angle_allocation(fp32 yaw_increment, fp32 *outer_yaw, fp32 *inner_yaw,
                                            gimbal_control_t *gimbal_control_set)
{
    fp32 original_increment = 0.0f;
    fp32 inner_increment = 0.0f;
    fp32 outer_increment = 0.0f;
    fp32 current_rel = 0.0f;
    fp32 max_rel = 0.0f;
    fp32 min_rel = 0.0f;
    fp32 comfort_zone = 0.4f;
    fp32 weight = 0.6f;

    if (outer_yaw == NULL || inner_yaw == NULL || gimbal_control_set == NULL) {
        return;
    }

    original_increment = yaw_increment;
    current_rel = gimbal_control_set->gimbal_yaw_motor[OUTER_YAW].relative_angle;
    max_rel = gimbal_control_set->gimbal_yaw_motor[OUTER_YAW].max_relative_angle;
    min_rel = gimbal_control_set->gimbal_yaw_motor[OUTER_YAW].min_relative_angle;

    // 相比上面的硬切分版本，这里会随着外 yaw 接近限位逐渐降低其承担比例，
    // 再把剩余角度平滑转移给内 yaw。
    if (original_increment > 0.0f) {
        if (current_rel > comfort_zone) {
            fp32 dist = max_rel - current_rel;
            fp32 transition_width = max_rel - comfort_zone;
            if (dist <= 0.0f) {
                weight = 0.0f;
            } else if (dist < transition_width) {
                weight = dist / transition_width;
            }
        }
    } else if (current_rel < -comfort_zone) {
        fp32 dist = current_rel - min_rel;
        fp32 transition_width = -comfort_zone - min_rel;
        if (dist <= 0.0f) {
            weight = 0.0f;
        } else if (dist < transition_width) {
            weight = dist / transition_width;
        }
    }

    weight = weight * weight * (3.0f - 2.0f * weight);
    outer_increment = original_increment * weight;

    // 外 yaw 的最终输出仍需再做一次机械限位裁剪。
    if (current_rel + outer_increment > max_rel) {
        outer_increment = max_rel - current_rel;
    } else if (current_rel + outer_increment < min_rel) {
        outer_increment = min_rel - current_rel;
    }

    inner_increment = original_increment - outer_increment;
    *outer_yaw = outer_increment;
    *inner_yaw = inner_increment;
}

/**
  * @brief          smoothstep限幅函数，软化临界
  * @param[in]      ideal_angle: 目标相对角增加量 ,limit_start: 限幅起效点 limit_hard: 硬限位点
  * @retval         限幅后的相对角增加量
  */
float fast_smooth_limit(float ideal_angle, fp32 limit_start, fp32 limit_hard)
{
    float abs_input = fabsf(ideal_angle);
    float sign = (ideal_angle > 0.0f) ? 1.0f : -1.0f;
    float t = 0.0f;
    float factor = 0.0f;
    float smooth_output = 0.0f;

    if (abs_input <= limit_start) {
        // 软限位区之外保持线性，不改变手感。
        return ideal_angle;
    }

    if (abs_input >= limit_hard) {
        // 到达硬限位后直接夹死，绝不允许继续逼近机械极限。
        return sign * limit_hard;
    }

    // 在软限位区内用 smoothstep 逐渐压缩增量，让手感连续且不突兀。
    t = (abs_input - limit_start) / (limit_hard - limit_start);
    factor = t * t * (3.0f - 2.0f * t);
    smooth_output = limit_start + (limit_hard - limit_start) * factor;
    return sign * smooth_output;
}
