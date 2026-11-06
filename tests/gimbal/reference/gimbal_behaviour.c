#include "gimbal_behaviour.h"

#include <math.h>

#include "gimbal/auto_shoot_task.h"
#include "detect.h"
#include "remote_control.h"
#include "usb_callback.h"

#define GIMBAL_ZERO_FORCE_TO_INIT_HYST_MS 100U  // 进入 INIT 前需连续 ZERO_FORCE 至少 100ms（1ms/周期）

#define rc_deadband_limit(input, output, deadline) \
    do { \
        if ((input) > (deadline) || (input) < -(deadline)) { \
            (output) = (input); \
        } else { \
            (output) = 0; \
        } \
    } while (0)

#define gimbal_cali_gyro_judge(gyro, cmd_time, angle_set, angle, ecd_set, ecd, step) \
    do { \
        if (fabsf(gyro) < GIMBAL_CALI_GYRO_LIMIT) { \
            (cmd_time)++; \
            if ((cmd_time) > GIMBAL_CALI_STEP_TIME) { \
                (cmd_time) = 0U; \
                (angle_set) = (angle); \
                (ecd_set) = (ecd); \
                (step)++; \
            } \
        } else { \
            (cmd_time) = 0U; \
        } \
    } while (0)

gimbal_behaviour_e gimbal_behaviour = GIMBAL_ZERO_FORCE;
gimbal_behaviour_e last_gimbal_behaviour = GIMBAL_ZERO_FORCE;

static void gimbal_behaviour_set(gimbal_control_t *gimbal_mode_set);
static void gimbal_zero_force_control(yaw_set_t *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);
static void gimbal_init_control(yaw_set_t *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);
static void gimbal_absolute_angle_control(yaw_set_t *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);
static void gimbal_relative_angle_control(yaw_set_t *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);
static void gimbal_motionless_control(yaw_set_t *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);
static void gimbal_auto_move_upper_control(yaw_set_t *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);
static void gimbal_upper_control(yaw_set_t *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);
static void gimbal_autoangle_control(yaw_set_t *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);
static void gimbal_lower_control(yaw_set_t *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);
static void gimbal_cali_control(yaw_set_t *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);

void gimbal_behaviour_mode_set(gimbal_control_t *gimbal_mode_set)
{
    if (gimbal_mode_set == NULL) {
        return;
    }

    gimbal_behaviour_set(gimbal_mode_set);

     //accoring to gimbal_behaviour, set motor control mode
    //根据云台行为状态机设置电机状态机
    if (gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        gimbal_mode_set->gimbal_yaw_motor[INNER_YAW].gimbal_motor_mode = GIMBAL_MOTOR_RAW;
        gimbal_mode_set->gimbal_yaw_motor[OUTER_YAW].gimbal_motor_mode = GIMBAL_MOTOR_RAW;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    }
    else if (gimbal_behaviour == GIMBAL_INIT)
    {
        gimbal_mode_set->gimbal_yaw_motor[INNER_YAW].gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
        gimbal_mode_set->gimbal_yaw_motor[OUTER_YAW].gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    }
    else if (gimbal_behaviour == GIMBAL_CALI)
    {
        gimbal_mode_set->gimbal_yaw_motor[INNER_YAW].gimbal_motor_mode = GIMBAL_MOTOR_RAW;
        gimbal_mode_set->gimbal_yaw_motor[OUTER_YAW].gimbal_motor_mode = GIMBAL_MOTOR_RAW;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    }
    else if (gimbal_behaviour == GIMBAL_ABSOLUTE_ANGLE)
    {
        gimbal_mode_set->gimbal_yaw_motor[INNER_YAW].gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
        gimbal_mode_set->gimbal_yaw_motor[OUTER_YAW].gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE; //26哨兵选择在遥控时让pitch电机使用其自带MIT控制，角度来源为编码器，故选encode
    }
    else if (gimbal_behaviour == GIMBAL_RELATIVE_ANGLE)
    {
        gimbal_mode_set->gimbal_yaw_motor[INNER_YAW].gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
        gimbal_mode_set->gimbal_yaw_motor[OUTER_YAW].gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    }
    else if (gimbal_behaviour == GIMBAL_MOTIONLESS)
    {
        gimbal_mode_set->gimbal_yaw_motor[INNER_YAW].gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
        gimbal_mode_set->gimbal_yaw_motor[OUTER_YAW].gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    }
    else if (gimbal_behaviour == GIMBAL_AUTO)
	{
		gimbal_mode_set->gimbal_yaw_motor[INNER_YAW].gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
		gimbal_mode_set->gimbal_yaw_motor[OUTER_YAW].gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
        const uint8_t auto_mode = get_auto_mode();
        if (auto_mode == SCAN_MODE || auto_mode == NAVI_WITH_SCAN_MODE || get_force_flag())
        {
            gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
        }
        else gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
	}
    else if (gimbal_behaviour == GIMBAL_AUTO_TEST)
		{
			gimbal_mode_set->gimbal_yaw_motor[INNER_YAW].gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
			gimbal_mode_set->gimbal_yaw_motor[OUTER_YAW].gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
			gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
		}
}

void gimbal_behaviour_control_set(yaw_set_t *add_yaw, fp32 *add_pitch, gimbal_control_t *gimbal_control_set)
{
    if (add_yaw == NULL || add_pitch == NULL || gimbal_control_set == NULL) {
        return;
    }

    if (gimbal_behaviour == GIMBAL_ZERO_FORCE) {
        gimbal_zero_force_control(add_yaw, add_pitch, gimbal_control_set);
    } else if (gimbal_behaviour == GIMBAL_INIT) {
        gimbal_init_control(add_yaw, add_pitch, gimbal_control_set);
    } else if (gimbal_behaviour == GIMBAL_CALI) {
        gimbal_cali_control(add_yaw, add_pitch, gimbal_control_set);
    } else if (gimbal_behaviour == GIMBAL_RELATIVE_ANGLE) {
        gimbal_relative_angle_control(add_yaw, add_pitch, gimbal_control_set);
    } else if (gimbal_behaviour == GIMBAL_MOTIONLESS) {
        gimbal_motionless_control(add_yaw, add_pitch, gimbal_control_set);
    } else if (gimbal_behaviour == GIMBAL_AUTO) {
        if (get_auto_mode() == NAVI_MODE || get_auto_mode() == PASSING_MODE || get_auto_mode() == NAVI_WITH_SCAN_MODE) {
            gimbal_auto_move_upper_control(add_yaw, add_pitch, gimbal_control_set);
        } else {
            gimbal_upper_control(add_yaw, add_pitch, gimbal_control_set);
        }
    } else if (gimbal_behaviour == GIMBAL_AUTO_TEST) {
        gimbal_lower_control(add_yaw, add_pitch, gimbal_control_set);
    } else {
        gimbal_absolute_angle_control(add_yaw, add_pitch, gimbal_control_set);
    }
}

bool_t gimbal_cmd_to_chassis_stop(void)
{
    return (bool_t)(gimbal_behaviour == GIMBAL_INIT || gimbal_behaviour == GIMBAL_CALI ||
                    gimbal_behaviour == GIMBAL_MOTIONLESS || gimbal_behaviour == GIMBAL_ZERO_FORCE);
}

bool_t gimbal_cmd_to_shoot_stop(void)
{
    return (bool_t)(gimbal_behaviour == GIMBAL_INIT || gimbal_behaviour == GIMBAL_CALI ||
                    gimbal_behaviour == GIMBAL_ZERO_FORCE);
}

static void gimbal_behaviour_set(gimbal_control_t *gimbal_mode_set)
{
    static uint16_t init_time = 0U;
    static uint16_t init_stop_time = 0U;
    static uint16_t last_key_v = 0U;
    static uint8_t auto_test_enable = 0U;
    static uint16_t zero_force_cnt = 0U;  // ZERO_FORCE 持续时长（ms），用于去抖 INIT 过渡
    uint16_t key_v = 0U;
    uint8_t at_change_pressed = 0U;

    if (gimbal_mode_set == NULL || gimbal_mode_set->gimbal_rc_ctrl == NULL) {
        gimbal_behaviour = GIMBAL_ZERO_FORCE;
        return;
    }

    last_gimbal_behaviour = gimbal_behaviour;
    key_v = gimbal_mode_set->gimbal_rc_ctrl->key.v;
    at_change_pressed = (uint8_t)(((key_v & AT_CHANGE_KEY) != 0U) && ((last_key_v & AT_CHANGE_KEY) == 0U));
    last_key_v = key_v;

    if (gimbal_behaviour == GIMBAL_CALI &&
        gimbal_mode_set->gimbal_cali.step != GIMBAL_CALI_END_STEP &&
        gimbal_mode_set->gimbal_cali.step != 0U) {
        return;
    }

    if (gimbal_mode_set->gimbal_cali.step == GIMBAL_CALI_START_STEP &&
        !toe_is_error(DBUS_TOE)) {
        // 校准任务已请求开始扫描，此处强制保持在 GIMBAL_CALI，直到双 yaw 机械扫描走完。
        gimbal_behaviour = GIMBAL_CALI;
        return;
    }

#if USE_I6X_RECEIVER
    // I6X 拨杆方向与 switch_* 宏的物理含义相反:
    // switch_is_up   表示拨杆实际下拨
    // switch_is_mid  表示拨杆实际中位
    // switch_is_down 表示拨杆实际上拨
    //
    // I6X 云台模式切换说明(GIMBAL_MODE_CHANNEL = 2，按当前分支逻辑):
    // 1. 云台模式拨杆上拨: 云台乏力，进入 GIMBAL_ZERO_FORCE。
    // 2. 云台模式拨杆中位: 云台绝对角度控制，进入 GIMBAL_ABSOLUTE_ANGLE。
    // 3. 云台模式拨杆下拨: 进入自瞄/上位机 AUTO 分支。
    //    默认进入 GIMBAL_AUTO；按下 AT_CHANGE_KEY 可在 GIMBAL_AUTO 与 GIMBAL_AUTO_TEST 间切换。
    // 4. 离开 AUTO 分支或遥控异常时: 清除 AUTO_TEST 记忆，回到默认 AUTO 分支。
    // 5. 从 GIMBAL_ZERO_FORCE 切回任意非乏力模式时: 先进入 GIMBAL_INIT，初始化完成后再切目标模式。
#endif



    if (gimbal_behaviour == GIMBAL_INIT) {
        init_time++;

        if ( ( fabsf(gimbal_mode_set->gimbal_yaw_motor[INNER_YAW].relative_angle - INIT_YAW_SET) < GIMBAL_INIT_ANGLE_ERROR || toe_is_error(INNER_YAW_GIMBAL_MOTOR_TOE) ) &&
           ( fabsf(gimbal_mode_set->gimbal_yaw_motor[OUTER_YAW].relative_angle - INIT_YAW_SET) < GIMBAL_INIT_ANGLE_ERROR || toe_is_error(OUTER_YAW_GIMBAL_MOTOR_TOE) )&&
            fabsf(gimbal_mode_set->gimbal_pitch_motor.relative_angle - INIT_PITCH_SET) < GIMBAL_INIT_ANGLE_ERROR) {
            if (init_stop_time < GIMBAL_INIT_STOP_TIME) {
                init_stop_time++;
            }
        } else if (init_time < GIMBAL_INIT_TIME) {
            init_time++;
        }

        if (init_time < GIMBAL_INIT_TIME &&
            init_stop_time < GIMBAL_INIT_STOP_TIME &&
            !switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL]) &&
            !toe_is_error(DBUS_TOE)) {
            return;
        }

        init_stop_time = 0U;
        init_time = 0U;
    }

    if (toe_is_error(DBUS_TOE) || switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL])) {
        zero_force_cnt++;
        gimbal_behaviour = GIMBAL_ZERO_FORCE;
    } else if (switch_is_up(gimbal_mode_set->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL])) {
        gimbal_behaviour = GIMBAL_AUTO;
    } else if (switch_is_mid(gimbal_mode_set->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL])) {
        gimbal_behaviour = GIMBAL_ABSOLUTE_ANGLE;
    } else {
        zero_force_cnt = 0U;
        gimbal_behaviour = GIMBAL_ZERO_FORCE;
    }

    // if (last_gimbal_behaviour == GIMBAL_ZERO_FORCE &&
    //     gimbal_behaviour != GIMBAL_ZERO_FORCE &&
    //     zero_force_cnt >= GIMBAL_ZERO_FORCE_TO_INIT_HYST_MS) {
    //     // 从乏力恢复时先走初始化——仅当乏力持续超过阈值才触发，防止单帧干扰引起归中。
    //     gimbal_behaviour = GIMBAL_INIT;
    // }

    /* 离开 ZERO_FORCE 后清零计数（在 INIT 判断之后） */
    if (gimbal_behaviour != GIMBAL_ZERO_FORCE) {
        zero_force_cnt = 0U;
    }
}

// 自瞄模式控制
static void gimbal_autoangle_control(yaw_set_t *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL) {
        return;
    }

    yaw->inner_yaw = 0.0f;
    yaw->outer_yaw = 0.0f;
    *pitch = 0.0f;
    Auto_Track(yaw, pitch, gimbal_control_set);
}

// 自瞄模式控制
static void gimbal_lower_control(yaw_set_t *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    static uint32_t init_tick = 0U;

    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL) {
        return;
    }

    switch (gimbal_control_set->auto_state_machine) {
        case DETECT_STATUS:
        {
            // 导航运行时云台不锁敌，继续使用导航/扫描控制。
            if (get_auto_mode() == NAVI_MODE || get_auto_mode() == PASSING_MODE ||
                get_auto_mode() == NAVI_WITH_SCAN_MODE) {
                Auto_Move(yaw, pitch, gimbal_control_set);
            } else {
                Auto_Detect(yaw, pitch, gimbal_control_set);
            }

            if (Find_IS_NOT()) {
                if (get_auto_mode() == NAVI_MODE || get_auto_mode() == PASSING_MODE ||
                    get_auto_mode() == NAVI_WITH_SCAN_MODE) {
                    gimbal_control_set->auto_state_machine = DETECT_STATUS;
                } else {
                    gimbal_control_set->auto_state_machine = TRACK_STATUS;
                }
            } else {
                gimbal_control_set->auto_state_machine = DETECT_STATUS;
            }
        } break;

        case TRACK_STATUS:
        {
            gimbal_autoangle_control(yaw, pitch, gimbal_control_set);
            if (Find_IS_NOT()) {
                if (get_auto_mode() == NAVI_MODE || get_auto_mode() == PASSING_MODE ||
                    get_auto_mode() == NAVI_WITH_SCAN_MODE) {
                    gimbal_control_set->auto_state_machine = DETECT_STATUS;
                } else {
                    gimbal_control_set->auto_state_machine = TRACK_STATUS;
                }
            } else {
                init_tick = HAL_GetTick();
                gimbal_control_set->auto_state_machine = CONFIRM_STATUS;
            }
        } break;

        case CONFIRM_STATUS:
        {
            Auto_Wait(yaw, pitch, gimbal_control_set);
            // 自瞄超时
            if (HAL_GetTick() - init_tick < 1000U) {
                if (Find_IS_NOT()) {
                    gimbal_control_set->auto_state_machine = TRACK_STATUS;
                }
            } else {
                gimbal_control_set->auto_state_machine = DETECT_STATUS;
            }
        } break;

        default:
        {
            gimbal_control_set->auto_state_machine = DETECT_STATUS;
            Auto_Wait(yaw, pitch, gimbal_control_set);
        }
        break;
    }
}

static void gimbal_zero_force_control(yaw_set_t *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    (void)gimbal_control_set;
    yaw->inner_yaw = 0.0f;
    yaw->outer_yaw = 0.0f;
    *pitch = 0.0f;
}

static void gimbal_init_control(yaw_set_t *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (gimbal_control_set == NULL) {
        return;
    }

    if (!toe_is_error(INNER_YAW_GIMBAL_MOTOR_TOE))
    {
        if (fabsf(INIT_YAW_SET - gimbal_control_set->gimbal_yaw_motor[INNER_YAW].relative_angle) > GIMBAL_INIT_ANGLE_ERROR) {
            *pitch = (INIT_PITCH_SET - gimbal_control_set->gimbal_pitch_motor.absolute_angle) * GIMBAL_INIT_PITCH_SPEED;
            yaw->inner_yaw =
                (INIT_YAW_SET - gimbal_control_set->gimbal_yaw_motor[INNER_YAW].relative_angle_set) * GIMBAL_INIT_YAW_SPEED;
            yaw->outer_yaw = 0.0f;
        } else {
            *pitch = (INIT_PITCH_SET - gimbal_control_set->gimbal_pitch_motor.absolute_angle) * GIMBAL_INIT_PITCH_SPEED;
            yaw->inner_yaw =
                (INIT_YAW_SET - gimbal_control_set->gimbal_yaw_motor[INNER_YAW].relative_angle) * GIMBAL_INIT_YAW_SPEED;
            yaw->outer_yaw =
                (INIT_YAW_SET - gimbal_control_set->gimbal_yaw_motor[OUTER_YAW].relative_angle) * GIMBAL_INIT_YAW_SPEED;
        }
    }
    else if (!toe_is_error(OUTER_YAW_GIMBAL_MOTOR_TOE))
    {
        if (fabsf(INIT_YAW_SET - gimbal_control_set->gimbal_yaw_motor[OUTER_YAW].relative_angle) > GIMBAL_INIT_ANGLE_ERROR) {
            *pitch = (INIT_PITCH_SET - gimbal_control_set->gimbal_pitch_motor.absolute_angle) * GIMBAL_INIT_PITCH_SPEED;
            yaw->inner_yaw = 0.0f;
            yaw->outer_yaw =
                (INIT_YAW_SET - gimbal_control_set->gimbal_yaw_motor[OUTER_YAW].relative_angle_set) * GIMBAL_INIT_YAW_SPEED;
        } else {
            *pitch = (INIT_PITCH_SET - gimbal_control_set->gimbal_pitch_motor.absolute_angle) * GIMBAL_INIT_PITCH_SPEED;
            yaw->inner_yaw =
                (INIT_YAW_SET - gimbal_control_set->gimbal_yaw_motor[INNER_YAW].relative_angle) * GIMBAL_INIT_YAW_SPEED;
            yaw->outer_yaw =
                (INIT_YAW_SET - gimbal_control_set->gimbal_yaw_motor[OUTER_YAW].relative_angle) * GIMBAL_INIT_YAW_SPEED;
        }
    }
}

/*
 * 2026 赛季哨兵云台校准的机械扫描顺序如下:
 * 1. GIMBAL_CALI_PITCH_MAX_STEP -> 扫 OUTER_YAW 正向极限。
 * 2. GIMBAL_CALI_PITCH_MIN_STEP -> 扫 OUTER_YAW 反向极限。
 * 3. GIMBAL_CALI_YAW_MAX_STEP   -> 扫 INNER_YAW 正向极限。
 * 4. GIMBAL_CALI_YAW_MIN_STEP   -> 扫 INNER_YAW 反向极限。
 *
 * 注意这里继续沿用原工程的 step 命名，所谓 pitch step 在本赛季哨兵里
 * 实际对应的是 OUTER_YAW(小 yaw) 的扫描，不是真正的 pitch 电机校准。
 * pitch 电机在整个自动校准过程中保持 0 输出。
 */
static void gimbal_cali_control(yaw_set_t *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    static uint16_t cali_time = 0U;

    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL) {
        return;
    }

    yaw->inner_yaw = 0.0f;
    yaw->outer_yaw = 0.0f;
    *pitch = 0.0f;

    if (gimbal_control_set->gimbal_cali.outer_yaw_only == GIMBAL_CALI_INNER_YAW_ONLY_MODE &&
        gimbal_control_set->gimbal_cali.step >= GIMBAL_CALI_PITCH_MAX_STEP &&
        gimbal_control_set->gimbal_cali.step < GIMBAL_CALI_YAW_MAX_STEP) {
        // 小 yaw 掉线时只执行 inner_yaw 扫描，先跳过 outer_yaw 步骤。
        cali_time = 0U;
        gimbal_control_set->gimbal_cali.step = GIMBAL_CALI_YAW_MAX_STEP;
        return;
    }

    if (gimbal_control_set->gimbal_cali.outer_yaw_only == GIMBAL_CALI_OUTER_YAW_ONLY_MODE &&
        gimbal_control_set->gimbal_cali.step >= GIMBAL_CALI_YAW_MAX_STEP &&
        gimbal_control_set->gimbal_cali.step < GIMBAL_CALI_END_STEP) {
        // 大 yaw 掉线时只允许完成 outer_yaw 扫描，后续 inner_yaw 步骤直接跳过。
        cali_time = 0U;
        gimbal_control_set->gimbal_cali.step = GIMBAL_CALI_END_STEP;
        return;
    }

    if (gimbal_control_set->gimbal_cali.step == GIMBAL_CALI_PITCH_MAX_STEP) {
        if (gimbal_control_set->gimbal_yaw_motor[OUTER_YAW].gimbal_motor_measure != NULL &&
            gimbal_control_set->gimbal_yaw_motor[OUTER_YAW].gimbal_motor_measure->motor_yaw_measure != NULL) {
            yaw->outer_yaw = GIMBAL_CALI_MOTOR_SET;
            gimbal_cali_gyro_judge(gimbal_control_set->gimbal_yaw_motor[OUTER_YAW].motor_gyro,
                                   cali_time,
                                   gimbal_control_set->gimbal_cali.max_pitch,
                                   gimbal_control_set->gimbal_yaw_motor[OUTER_YAW].absolute_angle,
                                   gimbal_control_set->gimbal_cali.max_pitch_ecd,
                                   gimbal_control_set->gimbal_yaw_motor[OUTER_YAW].gimbal_motor_measure->motor_yaw_measure->ecd,
                                   gimbal_control_set->gimbal_cali.step);
        } else {
            cali_time = 0U;
        }
    } else if (gimbal_control_set->gimbal_cali.step == GIMBAL_CALI_PITCH_MIN_STEP) {
        if (gimbal_control_set->gimbal_yaw_motor[OUTER_YAW].gimbal_motor_measure != NULL &&
            gimbal_control_set->gimbal_yaw_motor[OUTER_YAW].gimbal_motor_measure->motor_yaw_measure != NULL) {
            yaw->outer_yaw = -GIMBAL_CALI_MOTOR_SET;
            gimbal_cali_gyro_judge(gimbal_control_set->gimbal_yaw_motor[OUTER_YAW].motor_gyro,
                                   cali_time,
                                   gimbal_control_set->gimbal_cali.min_pitch,
                                   gimbal_control_set->gimbal_yaw_motor[OUTER_YAW].absolute_angle,
                                   gimbal_control_set->gimbal_cali.min_pitch_ecd,
                                   gimbal_control_set->gimbal_yaw_motor[OUTER_YAW].gimbal_motor_measure->motor_yaw_measure->ecd,
                                   gimbal_control_set->gimbal_cali.step);
        } else {
            cali_time = 0U;
        }
    } else if (gimbal_control_set->gimbal_cali.step == GIMBAL_CALI_YAW_MAX_STEP) {
        if (gimbal_control_set->gimbal_yaw_motor[INNER_YAW].gimbal_motor_measure != NULL &&
            gimbal_control_set->gimbal_yaw_motor[INNER_YAW].gimbal_motor_measure->motor_yaw_measure != NULL) {
            yaw->inner_yaw = GIMBAL_CALI_MOTOR_SET;
            gimbal_cali_gyro_judge(gimbal_control_set->gimbal_yaw_motor[INNER_YAW].motor_gyro,
                                   cali_time,
                                   gimbal_control_set->gimbal_cali.max_yaw,
                                   gimbal_control_set->gimbal_yaw_motor[INNER_YAW].absolute_angle,
                                   gimbal_control_set->gimbal_cali.max_yaw_ecd,
                                   gimbal_control_set->gimbal_yaw_motor[INNER_YAW].gimbal_motor_measure->motor_yaw_measure->ecd,
                                   gimbal_control_set->gimbal_cali.step);
        } else {
            cali_time = 0U;
        }
    } else if (gimbal_control_set->gimbal_cali.step == GIMBAL_CALI_YAW_MIN_STEP) {
        if (gimbal_control_set->gimbal_yaw_motor[INNER_YAW].gimbal_motor_measure != NULL &&
            gimbal_control_set->gimbal_yaw_motor[INNER_YAW].gimbal_motor_measure->motor_yaw_measure != NULL) {
            yaw->inner_yaw = -GIMBAL_CALI_MOTOR_SET;
            gimbal_cali_gyro_judge(gimbal_control_set->gimbal_yaw_motor[INNER_YAW].motor_gyro,
                                   cali_time,
                                   gimbal_control_set->gimbal_cali.min_yaw,
                                   gimbal_control_set->gimbal_yaw_motor[INNER_YAW].absolute_angle,
                                   gimbal_control_set->gimbal_cali.min_yaw_ecd,
                                   gimbal_control_set->gimbal_yaw_motor[INNER_YAW].gimbal_motor_measure->motor_yaw_measure->ecd,
                                   gimbal_control_set->gimbal_cali.step);
        } else {
            cali_time = 0U;
        }
    } else {
        cali_time = 0U;
    }
}

static void gimbal_absolute_angle_control(yaw_set_t *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    int16_t yaw_channel = 0;
    int16_t pitch_channel = 0;

    if (gimbal_control_set == NULL || gimbal_control_set->gimbal_rc_ctrl == NULL) {
        return;
    }

    rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[YAW_CHANNEL], yaw_channel, RC_DEADBAND);
    rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[PITCH_CHANNEL], pitch_channel, RC_DEADBAND);

    yaw->inner_yaw = (fp32)yaw_channel * YAW_RC_SEN;
    yaw->outer_yaw = (fp32)yaw_channel * YAW_RC_SEN;
    *pitch = -((fp32)pitch_channel * PITCH_RC_SEN + (fp32)gimbal_control_set->gimbal_rc_ctrl->mouse.y * PITCH_MOUSE_SEN);
}

static void gimbal_relative_angle_control(yaw_set_t *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    static int16_t yaw_channel = 0;
    static int16_t pitch_channel = 0;

    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL || gimbal_control_set->gimbal_rc_ctrl == NULL) {
        return;
    }

    rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[YAW_CHANNEL], yaw_channel, RC_DEADBAND);
    rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[PITCH_CHANNEL], pitch_channel, RC_DEADBAND);

    yaw->inner_yaw = (fp32)yaw_channel * YAW_RC_SEN;
    yaw->outer_yaw = 0.0f;
    *pitch = (fp32)pitch_channel * PITCH_RC_SEN + (fp32)gimbal_control_set->gimbal_rc_ctrl->mouse.y * PITCH_MOUSE_SEN;
}

static void gimbal_auto_move_upper_control(yaw_set_t *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    fp32 target_yaw = 0.0f;

    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL) {
        return;
    }

    if (!(get_auto_mode() == NAVI_MODE || get_auto_mode() == NAVI_WITH_SCAN_MODE || get_auto_mode() == PASSING_MODE)) {
        return;
    }

    target_yaw = get_latest_vision_data()->big_yaw_angle;
    gimbal_control_set->gimbal_yaw_motor[INNER_YAW].absolute_angle_set = target_yaw;
    gimbal_control_set->gimbal_yaw_motor[OUTER_YAW].absolute_angle_set = target_yaw;
    gimbal_control_set->gimbal_pitch_motor.absolute_angle_set = 0.15f;
    yaw->inner_yaw = 0.0f;
    yaw->outer_yaw = 0.0f;
    *pitch = 0.0f;
}

static void gimbal_upper_control(yaw_set_t *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    static VisionToGimbal gimbal_control_from_upper;
    fp32 target_absolute_angle = 0.0f;

    get_vision_data(&gimbal_control_from_upper);

    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL) {
        return;
    }

    gimbal_control_set->gimbal_pitch_motor.absolute_angle_set = gimbal_control_from_upper.pitch;
    target_absolute_angle = gimbal_control_from_upper.yaw;

    if (gimbal_control_set->yaw_conut > 1U) {
        gimbal_control_set->gimbal_yaw_motor[OUTER_YAW].absolute_angle_set = target_absolute_angle;
        gimbal_control_set->gimbal_yaw_motor[INNER_YAW].absolute_angle_set = target_absolute_angle;
    }

    yaw->inner_yaw = 0.0f;
    yaw->outer_yaw = 0.0f;
    *pitch = 0.0f;
}

static void gimbal_motionless_control(yaw_set_t *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    (void)gimbal_control_set;
    yaw->inner_yaw = 0.0f;
    yaw->outer_yaw = 0.0f;
    *pitch = 0.0f;
}
