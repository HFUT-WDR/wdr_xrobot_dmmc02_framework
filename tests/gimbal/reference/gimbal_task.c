#include "gimbal/gimbal_task.h"

#include <ctype.h>
#include <math.h>
#include <string.h>
#include "dm_imu_l1.h"
#include "arm_math.h"
#include "calibrate_task.h"
#include "cmsis_os.h"
#include "detect.h"
#include "dm_4310.h"
#include "gimbal_behaviour.h"
#include "imu.h"
#include "shoot/shoot_task.h"
#include "usb_callback.h"
#include "user_lib.h"
#include "referee/referee.h"
#include "line_adrc.h"
#include "gimbal/gravity_comp_lut.h"
#include "pitch_mit_sweep.h"
#include "gimbal/dual_yaw_ctrl.h"
#include "pitch_ctrl.h" /* 提供 control_motor / gimbal_*_angle_limit 等声明 */


#ifndef PI
#define PI 3.14159265358979323846f
#endif

#define GIMBAL_YAW_CURRENT_LIMIT 30000.0f
#define GIMBAL_PITCH_TORQUE_LIMIT 5.0f
#define GIMBAL_DM_ENABLE_PERIOD_MS 500U
#define P_SCAN_LOW_FREQ 1.0f
#define BIG_YAW_SCAN_SPD 2.0f
#define BIG_YAW_SCAN_LOW_SPD 1.0f
#define BIG_YAW_RAMP_RATE 4.0f
#define P_RAMP_STEP 0.001f
#define P_CENTER_MOVE_SPD 0.002f

#define P_SCAN_HIGH_CENTER (-0.3f)
#define P_SCAN_HIGH_AMP 0.2f


typedef struct {
    fp32 center, range, freq_min, freq_max, sweep_period, amp_scale_min, amp_scale_max;
    uint8_t sweep_type;
} pitch_varfreq_scan_cfg_t;

static pitch_varfreq_scan_cfg_t pitch_varfreq_scan_cfg = {-0.2f, 0.2f, 3.0f, 8.0f, 50.0f, 0.35f, 1.0f, 1U};

gimbal_control_t gimbal_control;
pitch_protection_debug_t pitch_prot_dbg = {0};

static void gimbal_init(gimbal_control_t *init);
static void gimbal_set_mode(gimbal_control_t *set_mode);
static void gimbal_feedback_update(gimbal_control_t *feedback_update);
static void gimbal_mode_change_control_transit(gimbal_control_t *gimbal_mode_change);
static void gimbal_set_control(gimbal_control_t *set_control);
static void gimbal_control_loop(gimbal_control_t *control_loop);
static void gimbal_output(gimbal_control_t *ctrl);
static void gimbal_watch_update(const gimbal_control_t *ctrl);
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);
/* calc_gimbal_axis_cali / calc_gimbal_cali / cmd_cali_gimbal_hook / set_cali_gimbal_hook
 * 已迁移至 User/application/module/calibrate/gimbal_cali.c。 */
static void gimbal_total_pid_clear(gimbal_control_t *gimbal_clear);
static void auto_pid_change(gimbal_control_t *auto_change);
static void CAN_cmd_gimbal(int16_t inner_yaw, int16_t outer_yaw, int16_t left_trigger, int16_t right_trigger);

/* ===== 兼容包装函数声明（实现由 PID.c 提供，此处为内联包装） ===== */
/*
  * @brief          双yaw特殊的pid控制，包含内外yaw的联动控制，采用主从跟随，大yaw跟小yaw，小yaw防撞限位的算法也放在这里
  * @param[out]     gimbal_control_loop 总的结构体指针，方便直接调用内外yaw电机数据
  * @retval         none
  */
/*
  * @brief          双yaw特殊的pid控制，包含内外yaw的联动控制，采用主从跟随,小yaw跟大yaw
  * @param[out]     gimbal_control_loop 总的结构体指针，方便直接调用内外yaw电机数据
  * @retval         none
  */
//双yaw扫描模式
/**
 * @brief Pitch扫描函数
 *
 * @param control_loop 总结构体
 */
static fp32 sign_value(fp32 value);
static int16_t current_limit_i16(fp32 current, fp32 limit);
static fp32 outer_yaw_td_update_error(gimbal_motor_t *outer_yaw, fp32 error, fp32 dt);
static fp32 outer_yaw_td_accel_ff_update(gimbal_motor_t *outer_yaw);

/**
 * @brief 云台主任务，周期执行反馈更新、模式切换、控制解算和电机输出。
 *
 * @param pvParameters FreeRTOS 任务参数，当前未使用。
 */
/* ===== 调试观测变量（static 限制作用域，消除全局命名空间污染） ===== */
static fp32 dm_yaw,dm_pitch,dm_roll;
static fp32 pitch_relative_angle_watch,pitch_absolute_angle_watch,pitch_gyro_watch,pitch_absolute_angle_set_watch,pitch_gyro_set_watch;
static fp32 pitch_relative_angle_set_watch;
static fp32 A_fric_speed = 0,B_fric_speed ,fric_tar_speed = 0,trigger_target_speed=0,trigger_fb_speed= 0;
static fp32 small_yaw_absolute_angle_watch,small_yaw_angle_set_watch;
static fp32 small_yaw_relative_angle_watch;
static fp32 small_yaw_gyro_watch,trigger_speed_set,trigger_fb_speed;
static fp32 small_yaw_gyro_set_watch;
static fp32 small_yaw_accel_watch;
static fp32 big_yaw_absolute_angle_watch,big_yaw_gyro_watch;
static fp32 big_yaw_current_set;
static fp32 small_yaw_hz,pitch_hz;
static uint32_t lost_event_count,peak_level,fail_count;
static fp32 yaw_relative_angle;
static fp32 TD_angle_set_watch,TD_gyro_watch,TD_accel_watch;
static fp32 roll_angle_watch;
static fp32 pitch_torque,pitch_gyro,hal_tick,pitch_gyro_watch,pitch_gyro_set_watch,pitch_filtered_gyro_watch;
static fp32 pitch_accel_set;
static fp32 accel_x,accel_y,accel_z;
static fp32 fric_L_RPM,fric_R_RPM,bullet_speed;
static fp32 upper_yaw_set,upper_pitch_set;
static fp32 mode;
static fp32 big_yaw_error_end=0,big_yaw_relative_angle;
static fp32 big_yaw_ecd_watch;
static fp32 upper_mode_watch;
static fp32 gimbal_behaviour_watch =0;
static fp32 dbus_toe_err_watch,inner_yaw_toe_err_watch,outer_yaw_toe_err_watch;
static fp32 distance_watch;
static fp32 big_yaw_hz;
static fp32 upper_big_yaw_watch;
static fp32 id_watch = 0;
static fp32 trigger_T_watch = 0;
static fp32 trigger_current_watch = 0;
static fp32 fire_watch = 0;
/**
 * @brief 云台控制输出与离线保护。
 *
 * 将控制流水线解算出的 given_current 通过 CAN 总线发送至各电机，
 * 同时处理离线检测和自瞄 STOP 模式的零输出保护。
 *
 * 输出顺序:
 *   1. Pitch 离线 → 尝试重新使能 DM4310
 *   2. 关键链路离线（DBUS / inner yaw / outer yaw）→ 全部零输出
 *   3. 自瞄 STOP 模式 → 全部零输出
 *   4. 正常 → 发送 yaw 电流 + shoot 摩擦轮 + pitch MIT 指令
 *
 * @param[in,out] ctrl 云台总控制结构体指针。
 */
static void gimbal_output(gimbal_control_t *ctrl)
{
    uint8_t allow_outer_offline = 0U;
    uint8_t allow_inner_offline = 0U;

    if (ctrl == NULL) return;

    allow_outer_offline = (uint8_t)(gimbal_behaviour == GIMBAL_CALI &&
                                     ctrl->gimbal_cali.outer_yaw_only == GIMBAL_CALI_INNER_YAW_ONLY_MODE);
    allow_inner_offline = (uint8_t)(gimbal_behaviour == GIMBAL_CALI &&
                                     ctrl->gimbal_cali.outer_yaw_only == GIMBAL_CALI_OUTER_YAW_ONLY_MODE);

    /* Pitch 离线则尝试重新使能 */
    if (toe_is_error(PITCH_GIMBAL_MOTOR_TOE)) {
        DM4310_MotorEnable(Pitch_Index);
    }

    /* 离线保护：关键链路断联 → 零输出 */
    if (toe_is_error(DBUS_TOE) ||
        (toe_is_error(OUTER_YAW_GIMBAL_MOTOR_TOE) && !allow_outer_offline) ||
        (toe_is_error(INNER_YAW_GIMBAL_MOTOR_TOE) && !allow_inner_offline)) {

        dbus_toe_err_watch   = toe_is_error(DBUS_TOE);
        inner_yaw_toe_err_watch = toe_is_error(INNER_YAW_GIMBAL_MOTOR_TOE);
        outer_yaw_toe_err_watch = toe_is_error(OUTER_YAW_GIMBAL_MOTOR_TOE);

        CAN_cmd_gimbal(0, 0, 0, 0);
        CAN_cmd_shoot(0, 0, 0, 0);
        gimbal_pitch_output(NULL);
        DM4310_MotorEnable(1);
        return;
    }

    /* 自瞄 STOP 模式 → 零输出 */
    if ((gimbal_behaviour == GIMBAL_AUTO || gimbal_behaviour == GIMBAL_AUTO_TEST) &&
        get_auto_mode() == STOP_MODE) {
        CAN_cmd_gimbal(0, 0, 0, 0);
        CAN_cmd_shoot(0, 0, 0, 0);
        gimbal_pitch_output(NULL);
        return;
    }

    /* 正常输出 */
    CAN_cmd_gimbal(ctrl->gimbal_yaw_motor[INNER_YAW].given_current,
                   ctrl->gimbal_yaw_motor[OUTER_YAW].given_current,
                   get_shoot_trigger_current(), 0);
    // CAN_cmd_gimbal(ctrl->gimbal_yaw_motor[INNER_YAW].given_current,
    //               ctrl->gimbal_yaw_motor[OUTER_YAW].given_current,
    //               0, 0);
    CAN_cmd_shoot(shoot_control_L.Fric_current_A,
                  shoot_control_L.Fric_current_B, 0, 0);
    // CAN_cmd_shoot(0,
    //               0, 0, 0);
    gimbal_pitch_output(&ctrl->gimbal_pitch_motor);
}

/**
 * @brief 调试观测变量采集。
 *
 * 每控制周期将云台、发射、视觉等模块的关键状态复制到 static watch 变量，
 * 供 VOFA / 调试器通过变量地址实时观测。所有变量均为只读 const 指针访问，
 * 不会修改控制状态。
 *
 * 可通过 DEBUG_WATCH_ENABLE=0 编译期完全移除此函数调用以节省 CPU 时间。
 *
 * @param[in] ctrl 云台总控制结构体指针（只读）。
 */
static void gimbal_watch_update(const gimbal_control_t *ctrl)
{
    if (ctrl == NULL) return;

    dm_pitch = ctrl->gimbal_INT_angle_point_DM->pitch;
    dm_roll  = ctrl->gimbal_INT_angle_point_DM->roll;
    dm_yaw   = ctrl->gimbal_INT_angle_point_DM->yaw;

    pitch_relative_angle_watch      = ctrl->gimbal_pitch_motor.relative_angle;
    pitch_absolute_angle_watch      = ctrl->gimbal_pitch_motor.absolute_angle;
    pitch_gyro_watch                = ctrl->gimbal_pitch_motor.motor_gyro;
    pitch_absolute_angle_set_watch  = ctrl->gimbal_pitch_motor.absolute_angle_set;
    pitch_relative_angle_set_watch  = ctrl->gimbal_pitch_motor.relative_angle_set;
    pitch_gyro_set_watch            = ctrl->gimbal_pitch_motor.motor_gyro_set;

    A_fric_speed = (shoot_control_L.Fric_FBspeed_A + 21.0f) * 3000;
    B_fric_speed = shoot_control_L.Fric_FBspeed_B;

    trigger_fb_speed    = (shoot_control_L.Trigger_FBspeed_filtered + 20.0f) * (-3000);
    trigger_target_speed = shoot_control_L.Trigger_Actualspeed;

    small_yaw_absolute_angle_watch = ctrl->gimbal_yaw_motor[OUTER_YAW].absolute_angle;
    small_yaw_angle_set_watch      = ctrl->gimbal_yaw_motor[OUTER_YAW].absolute_angle_set;
    small_yaw_gyro_watch           = ctrl->gimbal_yaw_motor[OUTER_YAW].motor_gyro;
    trigger_speed_set              = shoot_control_L.Trigger_Actualspeed;
    big_yaw_absolute_angle_watch   = ctrl->gimbal_yaw_motor[INNER_YAW].absolute_angle;
    big_yaw_gyro_watch             = ctrl->gimbal_yaw_motor[INNER_YAW].motor_gyro;
    big_yaw_current_set            = ctrl->gimbal_yaw_motor[INNER_YAW].given_current;
    small_yaw_accel_watch          = ctrl->gimbal_INT_angle_point_DM->accel[IMU_GYRO_INDEX_YAW];

    small_yaw_hz = get_error_list_point()[OUTER_YAW_GIMBAL_MOTOR_TOE].frequency;
    pitch_hz     = get_error_list_point()[PITCH_GIMBAL_MOTOR_TOE].frequency;
    big_yaw_hz   = get_error_list_point()[INNER_YAW_GIMBAL_MOTOR_TOE].frequency;

    lost_event_count = get_can3_rx_stats()->rx_lost_event_count;
    peak_level       = get_can3_rx_stats()->rx_fifo_peak_level;
    fail_count       = get_can3_rx_stats()->rx_get_fail_count;

    yaw_relative_angle          = ctrl->gimbal_yaw_motor[OUTER_YAW].relative_angle;
    small_yaw_gyro_set_watch    = ctrl->gimbal_yaw_motor[OUTER_YAW].motor_gyro_set;
    roll_angle_watch            = ctrl->gimbal_INT_angle_point_DM->roll * ANGLE_TO_RAD;
    pitch_torque                = ctrl->gimbal_pitch_motor.gimbal_motor_measure->motor_pitch_measure->torque;
    pitch_gyro                  = ctrl->gimbal_pitch_motor.motor_gyro;
    hal_tick                    = HAL_GetTick();
    pitch_gyro_watch            = ctrl->gimbal_pitch_motor.motor_gyro;
    pitch_gyro_set_watch        = ctrl->gimbal_pitch_motor.motor_gyro_set;
    pitch_accel_set             = get_latest_vision_data()->pitch_acc;
    pitch_filtered_gyro_watch   = ctrl->gimbal_pitch_motor.filtered_gyro;

    accel_x = ctrl->gimbal_INT_angle_point_DM->accel[IMU_GYRO_INDEX_ROLL];
    accel_y = ctrl->gimbal_INT_angle_point_DM->accel[IMU_GYRO_INDEX_PITCH];
    accel_z = ctrl->gimbal_INT_angle_point_DM->accel[IMU_GYRO_INDEX_YAW];

    fric_tar_speed = shoot_control_L.Fric_Targetspeed;
    fric_L_RPM = shoot_control_L.fric_motor_measure[0]
                 ? shoot_control_L.fric_motor_measure[0]->motor_fric_measure->speed_rpm : 0.0f;
    fric_R_RPM = shoot_control_L.fric_motor_measure[1]
                 ? shoot_control_L.fric_motor_measure[1]->motor_fric_measure->speed_rpm : 0.0f;

    bullet_speed         = get_bullet_speed();
    upper_yaw_set        = get_latest_vision_data()->yaw;
    upper_pitch_set      = get_latest_vision_data()->pitch;
    mode                 = get_latest_vision_data()->mode;
    small_yaw_relative_angle_watch = ctrl->gimbal_yaw_motor[OUTER_YAW].relative_angle;
    big_yaw_relative_angle        = ctrl->gimbal_yaw_motor[INNER_YAW].relative_angle;
    big_yaw_ecd_watch    = ctrl->gimbal_yaw_motor[INNER_YAW].gimbal_motor_measure->motor_yaw_measure->ecd;
    upper_mode_watch     = get_latest_vision_data()->mode;
    dbus_toe_err_watch   = toe_is_error(DBUS_TOE);
    gimbal_behaviour_watch = gimbal_behaviour;
    distance_watch       = get_latest_vision_data()->distance;
    upper_big_yaw_watch  = get_latest_vision_data()->big_yaw_angle;
    id_watch             = get_latest_vision_data()->id;
    trigger_T_watch      = shoot_control_L.trigger_motor_measure->motor_trigger_measure->temperature;
    trigger_current_watch = shoot_control_L.trigger_motor_measure->motor_trigger_measure->given_current;
    fire_watch           = get_latest_vision_data()->fire_control;
}


_Noreturn void gimbal_task(void *pvParameters)
{
    uint16_t enable_counter = 0U;
    (void)pvParameters;
    osDelay(GIMBAL_TASK_INIT_TIME);
    gimbal_init(&gimbal_control);
    cali_apply_saved_gimbal_cali();
    osDelay(1000); // 等待校准数据生效
    DM4310_MotorEnable(1);
    shoot_init();

    while (toe_is_error(DBUS_TOE) || toe_is_error(OUTER_YAW_GIMBAL_MOTOR_TOE) || toe_is_error(INNER_YAW_GIMBAL_MOTOR_TOE)) {
        DM4310_MotorEnable(Pitch_Index);
        gimbal_feedback_update(&gimbal_control);
        osDelay(50);
    }

    while (1) {
        /* ── 复活处理 ── */
        if (tell_revive()) {
            vTaskDelay(2000);
            for (int i = 0; i < 100; i++) {
                DM4310_MotorEnable(Pitch_Index);
                vTaskDelay(10);
            }
            gimbal_PID_clear(&gimbal_control.gimbal_pitch_motor.gimbal_motor_gyro_pid);
            gimbal_PID_clear(&gimbal_control.gimbal_pitch_motor.gimbal_motor_absolute_angle_pid);
            gimbal_PID_clear(&gimbal_control.gimbal_pitch_motor.gimbal_motor_relative_angle_pid);
        }


        gimbal_set_mode(&gimbal_control);
        gimbal_mode_change_control_transit(&gimbal_control);
        gimbal_feedback_update(&gimbal_control);
        gimbal_set_control(&gimbal_control);
        gimbal_control_loop(&gimbal_control);
        shoot_control_loop(&shoot_control_L);

        /*输出（含离线保护）*/
        gimbal_output(&gimbal_control);

        /*调试变量采集*/
        gimbal_watch_update(&gimbal_control);

        gimbal_control.last_auto_state_machine = gimbal_control.auto_state_machine;
        osDelay(GIMBAL_CONTROL_TIME);
    }
}

/**
 * @brief 初始化云台控制对象、传感器指针、限位和各级 PID 参数。
 *
 * @param init 待初始化的云台总控制结构体。
 */
static void gimbal_init(gimbal_control_t *init)
{
    if (init == NULL) return;
    init->gimbal_rc_ctrl = get_remote_control_point();
    init->gimbal_INT_angle_point = get_INS_angle_point();
    init->gimbal_INT_gyro_point = get_gyro_data_point();
    init->gimbal_INT_angle_point_DM = get_dm_imu_point(0);
    init->chassis_INT_angle_point_DM = get_dm_imu_point(1);
    init->gimbal_yaw_motor[INNER_YAW].gimbal_motor_measure = get_inner_yaw_gimbal_motor_measure_point();
    init->gimbal_yaw_motor[OUTER_YAW].gimbal_motor_measure = get_outer_yaw_gimbal_motor_measure_point();
    init->gimbal_pitch_motor.gimbal_motor_measure = get_pitch_gimbal_motor_measure_point();
    init->gimbal_yaw_motor[INNER_YAW].id = YAW_MOTOR;
    init->gimbal_yaw_motor[OUTER_YAW].id = YAW_MOTOR;
    init->gimbal_pitch_motor.id = PITCH_MOTOR;
    init->gimbal_yaw_motor[INNER_YAW].gimbal_motor_mode = init->gimbal_yaw_motor[INNER_YAW].last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    init->gimbal_yaw_motor[OUTER_YAW].gimbal_motor_mode = init->gimbal_yaw_motor[OUTER_YAW].last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    init->gimbal_pitch_motor.gimbal_motor_mode = init->gimbal_pitch_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    init->gimbal_pitch_motor.max_relative_angle = PITCH_MAX_RELATIVE_ANGLE;
    init->gimbal_pitch_motor.min_relative_angle = PITCH_MIN_RELATIVE_ANGLE;

    gimbal_PID_init(&init->gimbal_yaw_motor[INNER_YAW].gimbal_motor_absolute_angle_pid, INNER_YAW_GYRO_ABSOLUTE_PID_MAX_OUT, INNER_YAW_GYRO_ABSOLUTE_PID_MAX_IOUT, YAW_MOTOR, INNER_YAW_GYRO_ABSOLUTE_PID_KP, INNER_YAW_GYRO_ABSOLUTE_PID_KI, INNER_YAW_GYRO_ABSOLUTE_PID_KD, INNER_YAW_GYRO_ABSOLUTE_PID_KF);
    gimbal_PID_init(&init->gimbal_yaw_motor[INNER_YAW].gimbal_motor_relative_angle_pid, INNER_YAW_ENCODE_RELATIVE_PID_MAX_OUT, INNER_YAW_ENCODE_RELATIVE_PID_MAX_IOUT, YAW_MOTOR, INNER_YAW_ENCODE_RELATIVE_PID_KP, INNER_YAW_ENCODE_RELATIVE_PID_KI, INNER_YAW_ENCODE_RELATIVE_PID_KD, INNER_YAW_ENCODE_RELATIVE_PID_KF);
    gimbal_PID_init(&init->gimbal_yaw_motor[INNER_YAW].gimbal_motor_gyro_pid, INNER_YAW_SPEED_PID_MAX_OUT, INNER_YAW_SPEED_PID_MAX_IOUT, YAW_MOTOR, INNER_YAW_SPEED_PID_KP, INNER_YAW_SPEED_PID_KI, INNER_YAW_SPEED_PID_KD, INNER_YAW_SPEED_PID_KF);
    gimbal_PID_init(&init->gimbal_yaw_motor[INNER_YAW].gimbal_motor_soft_gyro_pid,INNER_YAW_SPEED_MIN_PID_MAX_OUT,INNER_YAW_SPEED_MIN_PID_MAX_IOUT,YAW_MOTOR,INNER_YAW_SPEED_MIN_PID_KP,INNER_YAW_SPEED_MIN_PID_KI,INNER_YAW_SPEED_MIN_PID_KD,INNER_YAW_SPEED_MIN_PID_KF);
    gimbal_PID_init(&init->gimbal_yaw_motor[OUTER_YAW].gimbal_motor_absolute_angle_pid, OUTER_YAW_GYRO_ABSOLUTE_PID_MAX_OUT, OUTER_YAW_GYRO_ABSOLUTE_PID_MAX_IOUT, YAW_MOTOR, OUTER_YAW_GYRO_ABSOLUTE_PID_KP, OUTER_YAW_GYRO_ABSOLUTE_PID_KI, OUTER_YAW_GYRO_ABSOLUTE_PID_KD, OUTER_YAW_GYRO_ABSOLUTE_PID_KF);
    gimbal_PID_init(&init->gimbal_yaw_motor[OUTER_YAW].gimbal_motor_relative_angle_pid, OUTER_YAW_ENCODE_RELATIVE_PID_MAX_OUT, OUTER_YAW_ENCODE_RELATIVE_PID_MAX_IOUT, YAW_MOTOR, OUTER_YAW_ENCODE_RELATIVE_PID_KP, OUTER_YAW_ENCODE_RELATIVE_PID_KI, OUTER_YAW_ENCODE_RELATIVE_PID_KD, OUTER_YAW_ENCODE_RELATIVE_PID_KF);
    gimbal_PID_init(&init->gimbal_yaw_motor[OUTER_YAW].gimbal_motor_gyro_pid, OUTER_YAW_SPEED_PID_MAX_OUT, OUTER_YAW_SPEED_PID_MAX_IOUT, YAW_MOTOR, OUTER_YAW_SPEED_PID_KP, OUTER_YAW_SPEED_PID_KI, OUTER_YAW_SPEED_PID_KD, OUTER_YAW_SPEED_PID_KF);
    gimbal_PID_init(&init->gimbal_yaw_motor[OUTER_YAW].gimbal_motor_soft_gyro_pid, OUTER_YAW_SPEED_MIN_PID_MAX_OUT, OUTER_YAW_SPEED_MIN_PID_MAX_IOUT, YAW_MOTOR, OUTER_YAW_SPEED_MIN_PID_KP, OUTER_YAW_SPEED_MIN_PID_KI, OUTER_YAW_SPEED_MIN_PID_KD, OUTER_YAW_SPEED_MIN_PID_KF);
    gimbal_PID_init(&init->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid, PITCH_GYRO_ABSOLUTE_PID_MAX_OUT, PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT, PITCH_MOTOR, PITCH_GYRO_ABSOLUTE_PID_KP, PITCH_GYRO_ABSOLUTE_PID_KI, PITCH_GYRO_ABSOLUTE_PID_KD, PITCH_GYRO_ABSOLUTE_PID_KF);
    gimbal_PID_init(&init->gimbal_pitch_motor.gimbal_motor_relative_angle_pid, PITCH_ENCODE_RELATIVE_PID_MAX_OUT, PITCH_ENCODE_RELATIVE_PID_MAX_IOUT, PITCH_MOTOR, PITCH_ENCODE_RELATIVE_PID_KP, PITCH_ENCODE_RELATIVE_PID_KI, PITCH_ENCODE_RELATIVE_PID_KD, PITCH_ENCODE_RELATIVE_PID_KF);
    gimbal_PID_init(&init->gimbal_pitch_motor.gimbal_motor_gyro_pid, PITCH_SPEED_PID_MAX_OUT, PITCH_SPEED_PID_MAX_IOUT, PITCH_MOTOR, PITCH_SPEED_PID_KP, PITCH_SPEED_PID_KI, PITCH_SPEED_PID_KD, PITCH_SPEED_PID_KF);
    init->gimbal_yaw_motor[INNER_YAW].gimbal_motor_absolute_angle_pid.Param.IZone = INNER_YAW_ANGLE_PID_I_ZONE;
    init->gimbal_yaw_motor[OUTER_YAW].gimbal_motor_absolute_angle_pid.Param.IZone = OUTER_YAW_ANGLE_PID_I_ZONE;
    init->gimbal_yaw_motor[OUTER_YAW].k_gyro = OUTER_YAW_GYRO_FB_K;
    init->gimbal_yaw_motor[OUTER_YAW].kp_plan_gyro = OUTER_YAW_K_PLAN_GYRO;
    init->gimbal_yaw_motor[OUTER_YAW].kp_plan_accel = OUTER_YAW_K_PLAN_ACCEL;
    init->gimbal_yaw_motor[INNER_YAW].kp_plan_gyro = INNER_YAW_K_PLAN_GYRO;
    init->yaw_conut = YAW_NUM;
    init->gimbal_cali.step = 0U;
    init->gimbal_cali.outer_yaw_only = 0U;
    init->offset_is_ready = 1U;
    init->yaw_align_offset = 0.0f;
    LADRC_1st_Init(&init->gimbal_pitch_motor.ladrc_controler, PITCH_LADRC_WO, PITCH_LADRC_WC, PITCH_LADRC_b0, (fp32)GIMBAL_CONTROL_TIME * 0.001f);
    gimbal_total_pid_clear(init);
    gimbal_feedback_update(init);
    init->big_yaw_drag_target = init->gimbal_yaw_motor[INNER_YAW].absolute_angle;
    init->gimbal_yaw_motor[INNER_YAW].absolute_angle_set = init->gimbal_yaw_motor[INNER_YAW].absolute_angle;
    init->gimbal_yaw_motor[INNER_YAW].relative_angle_set = init->gimbal_yaw_motor[INNER_YAW].relative_angle;
    init->gimbal_yaw_motor[OUTER_YAW].absolute_angle_set = init->gimbal_yaw_motor[OUTER_YAW].absolute_angle;
    init->gimbal_yaw_motor[OUTER_YAW].relative_angle_set = init->gimbal_yaw_motor[OUTER_YAW].relative_angle;
    init->gimbal_yaw_motor[OUTER_YAW].td_angle = 0.0f;
    init->gimbal_yaw_motor[OUTER_YAW].td_gyro = 0.0f;
    init->gimbal_yaw_motor[OUTER_YAW].td_accel = 0.0f;
    init->gimbal_yaw_motor[OUTER_YAW].td_accel_ff = 0.0f;
    init->gimbal_yaw_motor[OUTER_YAW].td_inited = 0U;
    init->gimbal_pitch_motor.absolute_angle_set = init->gimbal_pitch_motor.absolute_angle;
    init->gimbal_pitch_motor.relative_angle_set = init->gimbal_pitch_motor.relative_angle;
    gravity_comp_lut_init_cos(-PITCH_G,
                              init->gimbal_pitch_motor.min_relative_angle,
                              init->gimbal_pitch_motor.max_relative_angle);
    gravity_comp_lut_online_set_enable(1U);
    init->gimbal_pitch_motor.td_angle = init->gimbal_pitch_motor.absolute_angle;
    init->gimbal_pitch_motor.td_gyro = 0.0f;
    init->gimbal_pitch_motor.td_accel = 0.0f;
    init->gimbal_pitch_motor.td_inited = 0U;
    init->gimbal_pitch_motor.filtered_gyro = 0.0f;
    LowPassFilter1p_Init(&init->gimbal_pitch_motor.vel_err_lpf, 0.24f);
    LowPassFilter1p_Init(&init->gimbal_pitch_motor.torque_lpf, 0.24f);
    pitch_mit_sweep_init();
}

/**
 * @brief 根据行为层状态刷新各轴控制模式。
 *
 * @param set_mode 云台总控制结构体。
 */
static void gimbal_set_mode(gimbal_control_t *set_mode) { if (set_mode != NULL) gimbal_behaviour_mode_set(set_mode); }

/**
 * @brief 更新双 yaw 与 pitch 的编码器、IMU 角度和角速度反馈。
 *
 * @param feedback_update 云台总控制结构体。
 */
static void gimbal_feedback_update(gimbal_control_t *feedback_update)
{
    fp32 inner_abs = 0.0f, outer_rel = 0.0f;
    if (feedback_update == NULL) return;

    if (feedback_update->gimbal_INT_angle_point) inner_abs = feedback_update->gimbal_INT_angle_point[IMU_ANGLE_INDEX_YAW];

    if (feedback_update->gimbal_yaw_motor[INNER_YAW].gimbal_motor_measure && feedback_update->gimbal_yaw_motor[INNER_YAW].gimbal_motor_measure->motor_yaw_measure)
        feedback_update->gimbal_yaw_motor[INNER_YAW].relative_angle = motor_ecd_to_angle_change(feedback_update->gimbal_yaw_motor[INNER_YAW].gimbal_motor_measure->motor_yaw_measure->ecd, feedback_update->gimbal_yaw_motor[INNER_YAW].offset_ecd);

    if (feedback_update->gimbal_yaw_motor[OUTER_YAW].gimbal_motor_measure && feedback_update->gimbal_yaw_motor[OUTER_YAW].gimbal_motor_measure->motor_yaw_measure) {
        outer_rel = motor_ecd_to_angle_change(feedback_update->gimbal_yaw_motor[OUTER_YAW].gimbal_motor_measure->motor_yaw_measure->ecd, feedback_update->gimbal_yaw_motor[OUTER_YAW].offset_ecd);
        feedback_update->gimbal_yaw_motor[OUTER_YAW].relative_angle = outer_rel;
    }
    {
        // 对齐后的大 yaw: 用在线估计的偏置补偿两个 IMU 参考系差异,统一为小yaw的dm陀螺仪的坐标系
        fp32 inner_aligned = inner_abs - feedback_update->yaw_align_offset;
        feedback_update->gimbal_yaw_motor[INNER_YAW].raw_absolute_angle = inner_aligned;
        feedback_update->gimbal_yaw_motor[INNER_YAW].absolute_angle = rad_format(inner_aligned);
    }
    if (feedback_update->gimbal_INT_angle_point_DM)
    {
        feedback_update->gimbal_yaw_motor[OUTER_YAW].absolute_angle = rad_format((feedback_update->gimbal_INT_angle_point_DM->yaw) * ANGLE_TO_RAD);
    }

    if (feedback_update->gimbal_INT_gyro_point) {
        feedback_update->gimbal_yaw_motor[INNER_YAW].motor_gyro = feedback_update->gimbal_INT_gyro_point[IMU_GYRO_INDEX_YAW];
    }


    if (feedback_update->gimbal_pitch_motor.gimbal_motor_measure && feedback_update->gimbal_pitch_motor.gimbal_motor_measure->motor_pitch_measure) {
        feedback_update->gimbal_pitch_motor.relative_angle = feedback_update->gimbal_pitch_motor.gimbal_motor_measure->motor_pitch_measure->position;
    }

    if (feedback_update->gimbal_INT_angle_point_DM)
    {
        feedback_update->gimbal_yaw_motor[OUTER_YAW].motor_gyro = arm_cos_f32(feedback_update->gimbal_pitch_motor.relative_angle) * (feedback_update->gimbal_INT_angle_point_DM->gyro[IMU_GYRO_INDEX_YAW]) \
    - arm_sin_f32(feedback_update->gimbal_pitch_motor.relative_angle) * (feedback_update->gimbal_INT_angle_point_DM->gyro[IMU_GYRO_INDEX_ROLL]);
        //这里要的是对地绝对角速度，当前双yaw控制思路不要相对角速度，解耦更易于理解和控制;
    }

    if (feedback_update->gimbal_INT_angle_point_DM)
    {
        feedback_update->gimbal_pitch_motor.absolute_angle = feedback_update->gimbal_INT_angle_point_DM->pitch * ANGLE_TO_RAD;
        feedback_update->gimbal_pitch_motor.motor_gyro = feedback_update->gimbal_INT_angle_point_DM->gyro[IMU_GYRO_INDEX_PITCH];
    }
    //feedback_update->gimbal_pitch_motor.relative_angle = fp32_constrain(feedback_update->gimbal_pitch_motor.relative_angle, feedback_update->gimbal_pitch_motor.min_relative_angle, feedback_update->gimbal_pitch_motor.max_relative_angle);

    // 小 yaw 电机在线时，利用编码器相对角持续估计双 IMU 对齐偏置
    if (!toe_is_error(OUTER_YAW_GIMBAL_MOTOR_TOE) &&
        feedback_update->gimbal_INT_angle_point_DM != NULL) {
        fp32 outer_abs_raw = feedback_update->gimbal_INT_angle_point_DM->yaw * ANGLE_TO_RAD;
        fp32 raw_offset = rad_format( inner_abs - outer_abs_raw + outer_rel);
        feedback_update->yaw_align_offset +=
            YAW_ALIGN_LPF_ALPHA * (raw_offset - feedback_update->yaw_align_offset); //本质低通滤波
    }

    feedback_update->yaw_error = rad_format(feedback_update->gimbal_yaw_motor[INNER_YAW].absolute_angle - rad_format(feedback_update->gimbal_yaw_motor[OUTER_YAW].absolute_angle - feedback_update->gimbal_yaw_motor[OUTER_YAW].relative_angle));

    feedback_update->gimbal_pitch_motor.filtered_gyro = LowPassFilter1p_Update(&feedback_update->gimbal_pitch_motor.vel_err_lpf, feedback_update->gimbal_pitch_motor.motor_gyro);

}

/**
 * @brief 在自动与非自动模式切换时重载对应的角度环 PID 参数。
 *
 * @param auto_change 云台总控制结构体。
 */
static void auto_pid_change(gimbal_control_t *auto_change)
{
    if (auto_change == NULL) return;
    if ((last_gimbal_behaviour != GIMBAL_AUTO && gimbal_behaviour == GIMBAL_AUTO) || (last_gimbal_behaviour != GIMBAL_AUTO_TEST && gimbal_behaviour == GIMBAL_AUTO_TEST)) {
        gimbal_PID_init(&auto_change->gimbal_yaw_motor[INNER_YAW].gimbal_motor_absolute_angle_pid, AT1_INNER_YAW_GYRO_ABSOLUTE_PID_MAX_OUT, AT1_INNER_YAW_GYRO_ABSOLUTE_PID_MAX_IOUT, YAW_MOTOR, AT1_INNER_YAW_GYRO_ABSOLUTE_PID_KP, AT1_INNER_YAW_GYRO_ABSOLUTE_PID_KI, AT1_INNER_YAW_GYRO_ABSOLUTE_PID_KD, AT1_INNER_YAW_GYRO_ABSOLUTE_PID_KF);
        gimbal_PID_init(&auto_change->gimbal_yaw_motor[OUTER_YAW].gimbal_motor_absolute_angle_pid, AT2_OUTER_YAW_GYRO_ABSOLUTE_PID_MAX_OUT, AT2_OUTER_YAW_GYRO_ABSOLUTE_PID_MAX_IOUT, YAW_MOTOR, AT2_OUTER_YAW_GYRO_ABSOLUTE_PID_KP, AT2_OUTER_YAW_GYRO_ABSOLUTE_PID_KI, AT2_OUTER_YAW_GYRO_ABSOLUTE_PID_KD, AT2_OUTER_YAW_GYRO_ABSOLUTE_PID_KF);
        gimbal_PID_init(&auto_change->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid, AT1_PITCH_GYRO_ABSOLUTE_PID_MAX_OUT, AT1_PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT, PITCH_MOTOR, AT1_PITCH_GYRO_ABSOLUTE_PID_KP, AT1_PITCH_GYRO_ABSOLUTE_PID_KI, AT1_PITCH_GYRO_ABSOLUTE_PID_KD, AT1_PITCH_GYRO_ABSOLUTE_PID_KF);
        auto_change->gimbal_yaw_motor[INNER_YAW].gimbal_motor_absolute_angle_pid.Param.IZone = AT_INNER_YAW_ANGLE_PID_I_ZONE;
        auto_change->gimbal_yaw_motor[OUTER_YAW].gimbal_motor_absolute_angle_pid.Param.IZone = AT_OUTER_YAW_ANGLE_PID_I_ZONE;
    } else if ((last_gimbal_behaviour == GIMBAL_AUTO && gimbal_behaviour != GIMBAL_AUTO) || (last_gimbal_behaviour == GIMBAL_AUTO_TEST && gimbal_behaviour != GIMBAL_AUTO_TEST)) {
        gimbal_PID_init(&auto_change->gimbal_yaw_motor[INNER_YAW].gimbal_motor_absolute_angle_pid, INNER_YAW_GYRO_ABSOLUTE_PID_MAX_OUT, INNER_YAW_GYRO_ABSOLUTE_PID_MAX_IOUT, YAW_MOTOR, INNER_YAW_GYRO_ABSOLUTE_PID_KP, INNER_YAW_GYRO_ABSOLUTE_PID_KI, INNER_YAW_GYRO_ABSOLUTE_PID_KD, INNER_YAW_GYRO_ABSOLUTE_PID_KF);
        gimbal_PID_init(&auto_change->gimbal_yaw_motor[OUTER_YAW].gimbal_motor_absolute_angle_pid, OUTER_YAW_GYRO_ABSOLUTE_PID_MAX_OUT, OUTER_YAW_GYRO_ABSOLUTE_PID_MAX_IOUT, YAW_MOTOR, OUTER_YAW_GYRO_ABSOLUTE_PID_KP, OUTER_YAW_GYRO_ABSOLUTE_PID_KI, OUTER_YAW_GYRO_ABSOLUTE_PID_KD, OUTER_YAW_GYRO_ABSOLUTE_PID_KF);
        gimbal_PID_init(&auto_change->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid, PITCH_GYRO_ABSOLUTE_PID_MAX_OUT, PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT, PITCH_MOTOR, PITCH_GYRO_ABSOLUTE_PID_KP, PITCH_GYRO_ABSOLUTE_PID_KI, PITCH_GYRO_ABSOLUTE_PID_KD, PITCH_GYRO_ABSOLUTE_PID_KF);
        auto_change->gimbal_yaw_motor[INNER_YAW].gimbal_motor_absolute_angle_pid.Param.IZone = INNER_YAW_ANGLE_PID_I_ZONE;
        auto_change->gimbal_yaw_motor[OUTER_YAW].gimbal_motor_absolute_angle_pid.Param.IZone = OUTER_YAW_ANGLE_PID_I_ZONE;
    }
}

/**
 * @brief 处理模式切换时的状态过渡，清零控制量并重置自动模式状态机。
 *
 * @param gimbal_mode_change 云台总控制结构体。
 */
static void gimbal_mode_change_control_transit(gimbal_control_t *gimbal_mode_change)
{
    gimbal_motor_t *m[3];
    if (gimbal_mode_change == NULL) return;
    auto_pid_change(gimbal_mode_change);
    m[0] = &gimbal_mode_change->gimbal_yaw_motor[INNER_YAW];
    m[1] = &gimbal_mode_change->gimbal_yaw_motor[OUTER_YAW];
    m[2] = &gimbal_mode_change->gimbal_pitch_motor;
    for (uint8_t i = 0; i < 3U; i++) if ((m[i]->gimbal_motor_mode != m[i]->last_gimbal_motor_mode) ||(gimbal_behaviour == GIMBAL_AUTO && get_auto_mode() != get_auto_last_mode()))
    {

        m[i]->absolute_angle_set = m[i]->absolute_angle;
        m[i]->relative_angle_set = m[i]->relative_angle;
        m[i]->motor_gyro_set = 0.0f;
        m[i]->raw_cmd_current = 0.0f;
        m[i]->current_set = 0.0f;
        m[i]->given_current = 0;
        gimbal_PID_clear(&m[i]->gimbal_motor_absolute_angle_pid);
        gimbal_PID_clear(&m[i]->gimbal_motor_relative_angle_pid);
        gimbal_PID_clear(&m[i]->gimbal_motor_gyro_pid);
        gimbal_PID_clear(&m[i]->gimbal_motor_soft_gyro_pid);
        m[i]->last_gimbal_motor_mode = m[i]->gimbal_motor_mode;

        //LADRC部分数据清除
        LADRC_1st_Reset(&m[i]->ladrc_controler, m[i]->motor_gyro);
        m[i]->filtered_gyro = m[i]->motor_gyro;
        m[i]->tor = 0;
        m[i]->td_angle = m[i]->absolute_angle;
        m[i]->td_gyro = 0.0f;
        m[i]->td_accel = 0.0f;
        m[i]->td_accel_ff = 0.0f;
        m[i]->td_inited = 0U;
    }

    if ((gimbal_behaviour == GIMBAL_AUTO || gimbal_behaviour == GIMBAL_AUTO_TEST) && get_auto_mode() != get_auto_last_mode()) {
        gimbal_mode_change->gimbal_pitch_motor.absolute_angle_set = gimbal_mode_change->gimbal_pitch_motor.absolute_angle;
        gimbal_mode_change->gimbal_pitch_motor.tor = 0.0f;
        gimbal_PID_clear(&gimbal_mode_change->gimbal_yaw_motor[INNER_YAW].gimbal_motor_absolute_angle_pid);
        gimbal_PID_clear(&gimbal_mode_change->gimbal_yaw_motor[INNER_YAW].gimbal_motor_relative_angle_pid);
        gimbal_PID_clear(&gimbal_mode_change->gimbal_yaw_motor[INNER_YAW].gimbal_motor_gyro_pid);
        gimbal_PID_clear(&gimbal_mode_change->gimbal_yaw_motor[OUTER_YAW].gimbal_motor_absolute_angle_pid);
        gimbal_PID_clear(&gimbal_mode_change->gimbal_yaw_motor[OUTER_YAW].gimbal_motor_relative_angle_pid);
        gimbal_PID_clear(&gimbal_mode_change->gimbal_yaw_motor[OUTER_YAW].gimbal_motor_gyro_pid);
    }

    if ((last_gimbal_behaviour != GIMBAL_AUTO && gimbal_behaviour == GIMBAL_AUTO) ||
        (last_gimbal_behaviour != GIMBAL_AUTO_TEST && gimbal_behaviour == GIMBAL_AUTO_TEST)) {
        gimbal_mode_change->auto_state_machine = DETECT_STATUS;
        gimbal_mode_change->last_auto_state_machine = DETECT_STATUS;
        shoot_control_L.shoot_mode = SHOOT_STOP;
        shoot_control_L.open_fric = 0.0f;
        shoot_control_R.shoot_mode = SHOOT_STOP;
        shoot_control_R.open_fric = 0.0f;
        gimbal_mode_change->gimbal_pitch_motor.absolute_angle_set = gimbal_mode_change->gimbal_pitch_motor.absolute_angle;
        gimbal_mode_change->gimbal_pitch_motor.relative_angle_set = gimbal_mode_change->gimbal_pitch_motor.relative_angle;
        gimbal_mode_change->gimbal_yaw_motor[INNER_YAW].absolute_angle_set = gimbal_mode_change->gimbal_yaw_motor[INNER_YAW].absolute_angle;
        gimbal_mode_change->gimbal_yaw_motor[OUTER_YAW].absolute_angle_set = gimbal_mode_change->gimbal_yaw_motor[OUTER_YAW].absolute_angle;
        gimbal_mode_change->big_yaw_state = 0U;
        gimbal_mode_change->small_yaw_state = 0U;
    } else if (((last_gimbal_behaviour == GIMBAL_AUTO && gimbal_behaviour != GIMBAL_AUTO) ||
                (last_gimbal_behaviour == GIMBAL_AUTO_TEST && gimbal_behaviour != GIMBAL_AUTO_TEST))) {
        shoot_control_L.shoot_mode = SHOOT_STOP;
        shoot_control_L.open_fric = 0.0f;
        shoot_control_R.shoot_mode = SHOOT_STOP;
        shoot_control_R.open_fric = 0.0f;
        gimbal_mode_change->gimbal_pitch_motor.absolute_angle_set = gimbal_mode_change->gimbal_pitch_motor.absolute_angle;
        gimbal_mode_change->gimbal_pitch_motor.relative_angle_set = gimbal_mode_change->gimbal_pitch_motor.relative_angle;
        gimbal_mode_change->gimbal_yaw_motor[INNER_YAW].absolute_angle_set = gimbal_mode_change->gimbal_yaw_motor[INNER_YAW].absolute_angle;
        gimbal_mode_change->gimbal_yaw_motor[OUTER_YAW].absolute_angle_set = gimbal_mode_change->gimbal_yaw_motor[OUTER_YAW].absolute_angle;
        gimbal_mode_change->big_yaw_state = 0U;
        gimbal_mode_change->small_yaw_state = 0U;
        gimbal_mode_change->gimbal_pitch_motor.tor = 0.0f;
    }



}

/**
 * @brief 将行为层给出的增量控制量写入各电机目标值或原始电流指令。
 *
 * @param set_control 云台总控制结构体。
 */
static void gimbal_set_control(gimbal_control_t *set_control)
{
    yaw_set_t y = {0.0f, 0.0f};
    fp32 p = 0.0f;
    if (set_control == NULL) return;
    gimbal_behaviour_control_set(&y, &p, set_control);
    if (set_control->gimbal_yaw_motor[INNER_YAW].gimbal_motor_mode == GIMBAL_MOTOR_RAW) set_control->gimbal_yaw_motor[INNER_YAW].raw_cmd_current = y.inner_yaw;
    else if (set_control->gimbal_yaw_motor[INNER_YAW].gimbal_motor_mode == GIMBAL_MOTOR_GYRO) gimbal_absolute_angle_limit(&set_control->gimbal_yaw_motor[INNER_YAW], y.inner_yaw);
    else gimbal_relative_angle_limit(&set_control->gimbal_yaw_motor[INNER_YAW], y.inner_yaw);
    if (set_control->gimbal_yaw_motor[OUTER_YAW].gimbal_motor_mode == GIMBAL_MOTOR_RAW) set_control->gimbal_yaw_motor[OUTER_YAW].raw_cmd_current = y.outer_yaw;
    else if (set_control->gimbal_yaw_motor[OUTER_YAW].gimbal_motor_mode == GIMBAL_MOTOR_GYRO) gimbal_absolute_angle_limit(&set_control->gimbal_yaw_motor[OUTER_YAW], y.outer_yaw);
    else gimbal_relative_angle_limit(&set_control->gimbal_yaw_motor[OUTER_YAW], y.outer_yaw);
    if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW) set_control->gimbal_pitch_motor.raw_cmd_current = p;
    else if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO) gimbal_absolute_angle_limit_pitch(&set_control->gimbal_pitch_motor, p);
    else gimbal_relative_angle_limit(&set_control->gimbal_pitch_motor, p);
}

/**
 * @brief 执行单个电机的串级控制，输出限幅后的给定电流。
 *
 * @param gimbal_motor 待控制的电机对象。
 * @param use_relative 非零时使用相对角度环，否则使用绝对角度环。
 */

/**
 * @brief 根据当前行为模式选择不同控制策略，完成 yaw 与 pitch 的闭环解算。
 *
 * @param control_loop 云台总控制结构体。
 */
static void gimbal_control_loop(gimbal_control_t *control_loop)
{
    uint8_t auto_control = 0U;
    uint8_t auto_mode = STOP_MODE;

    if (control_loop == NULL) return;

    auto_control = (uint8_t)(gimbal_behaviour == GIMBAL_AUTO || gimbal_behaviour == GIMBAL_AUTO_TEST);
    if (auto_control) {
        auto_mode = get_auto_mode();
    }

    /**************************************yaw轴电机*************************************/
    if (control_loop->gimbal_yaw_motor[INNER_YAW].gimbal_motor_mode == GIMBAL_MOTOR_RAW) {
        control_motor(&control_loop->gimbal_yaw_motor[INNER_YAW], 0U);
        control_motor(&control_loop->gimbal_yaw_motor[OUTER_YAW], 0U);
    }
    else if (control_loop->gimbal_yaw_motor[INNER_YAW].gimbal_motor_mode == GIMBAL_MOTOR_GYRO) {
        if (auto_control)
        {
            if (auto_mode == NAVI_MODE || auto_mode == PASSING_MODE) {
                TwoYAW_gimbal_absolute_angle_control_big_yaw_first(control_loop);
            } else if (auto_mode == SCAN_MODE || auto_mode == NAVI_WITH_SCAN_MODE) {
                Dual_Yaw_Scan_Mode(control_loop);
            } else if (!get_force_flag()) {
                TwoYAW_gimbal_absolute_angle_control_small_yaw_first(control_loop);
            } else {
                TwoYAW_gimbal_absolute_angle_control_big_yaw_first(control_loop);
            }
        } else {
            TwoYAW_gimbal_absolute_angle_control_big_yaw_first(control_loop);
        }
    }
    else if (control_loop->gimbal_yaw_motor[INNER_YAW].gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE) {
        control_motor(&control_loop->gimbal_yaw_motor[INNER_YAW], 1U);
        control_motor(&control_loop->gimbal_yaw_motor[OUTER_YAW], 1U);
    }

    /**************************************pitch轴电机*************************************/
    if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW) {
        control_motor(&control_loop->gimbal_pitch_motor, 0U);
    }
    else if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO) {
        if (auto_control) {
            if (auto_mode == SCAN_MODE || auto_mode == NAVI_WITH_SCAN_MODE) {
                Pitch_Scan_Mode(control_loop);
            } else {
                if (get_force_flag()) {
                    Pitch_Scan_Mode(control_loop);
                } else {
                    NormalControlStrategy(&control_loop->gimbal_pitch_motor);
                }
            }
        }
    }
    else if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE) {
        if (gimbal_behaviour == GIMBAL_ABSOLUTE_ANGLE) //26哨兵在遥控时选择使用MIT，且使用了速度环，故通过TD规划一个角速度(就让没用的TD再有用一次吧)
        {
#if PITCH_IDENTIFY
            pitch_mit_sweep_update(&control_loop->gimbal_pitch_motor);
#else
            TD_MIT_Control(&control_loop->gimbal_pitch_motor,1);
#endif
        }
        else if (auto_control)
        {
            if (auto_mode == SCAN_MODE || auto_mode == NAVI_WITH_SCAN_MODE) {
                Pitch_Scan_Mode(control_loop);
            } else
            {
                if (get_force_flag()) {
                    Pitch_Scan_Mode(control_loop);
                }
            }
        }
        else control_motor(&control_loop->gimbal_pitch_motor, 1U);
    }
}

/**
 * @brief 将 pitch 轴控制结果转换为 DM4310 MIT 控制指令并发送。
 *
 * @param pitch pitch 电机控制对象。
 */
fp32 ab_set_watch = 0.0f;


/**
 * @brief 通过底盘总线发送双 yaw 和扳机电流指令。
 *
 * @param inner_yaw 内 yaw 给定电流。
 * @param outer_yaw 外 yaw 给定电流。
 * @param left_trigger 左扳机给定电流。
 * @param right_trigger 右扳机给定电流。
 */
static void CAN_cmd_gimbal(int16_t inner_yaw, int16_t outer_yaw, int16_t left_trigger, int16_t right_trigger)
{
    uint8_t data[8] = {(uint8_t)(inner_yaw >> 8), (uint8_t)inner_yaw, (uint8_t)(outer_yaw >> 8), (uint8_t)outer_yaw, (uint8_t)(left_trigger >> 8), (uint8_t)left_trigger, (uint8_t)(right_trigger >> 8), (uint8_t)right_trigger};
#if USE_FDCAN
    FDCAN_TxHeaderTypeDef tx = {.Identifier = CAN_GIMBAL_ALL_ID, .IdType = FDCAN_STANDARD_ID, .TxFrameType = FDCAN_DATA_FRAME, .DataLength = FDCAN_DLC_BYTES_8, .FDFormat = FDCAN_CLASSIC_CAN, .BitRateSwitch = FDCAN_BRS_OFF, .TxEventFifoControl = FDCAN_NO_TX_EVENTS, .ErrorStateIndicator = FDCAN_ESI_ACTIVE, .MessageMarker = 0};
    if (HAL_FDCAN_GetTxFifoFreeLevel(BSP_CAN_GIMBAL_HANDLE) > 0U)
    {
        HAL_FDCAN_AddMessageToTxFifoQ(BSP_CAN_GIMBAL_HANDLE, &tx, data);
    }
#else
    CAN_TxHeaderTypeDef tx = {.StdId = CAN_GIMBAL_ALL_ID, .IDE = CAN_ID_STD, .RTR = CAN_RTR_DATA, .DLC = 8};
    uint32_t mailbox = 0U;
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) > 0U) HAL_CAN_AddTxMessage(&hcan2, &tx, data, &mailbox);
#endif
}

/**
 * @brief 对绝对角目标做增量更新并完成角度回环。
 *
 * @param gimbal_motor 目标电机。
 * @param add 角度增量。
 */

/**
 * @brief 更新 pitch 绝对角目标，并结合机械限位抑制越界。
 *
 * @param gimbal_motor pitch 电机对象。
 * @param add 角度增量。
 */

/**
 * @brief 对相对角目标做增量更新并限制在机械允许范围内。
 *
 * @param gimbal_motor 目标电机。
 * @param add 角度增量。
 */

/**
 * @brief 将编码器读数转换为相对零位的角度变化量。
 *
 * @param ecd 当前编码器值。
 * @param offset_ecd 零位编码器值。
 *
 * @return 相对角度，单位 rad。
 */
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
    int32_t rel = (int32_t)ecd - (int32_t)offset_ecd;
    if (rel > HALF_ECD_RANGE) rel -= (ECD_RANGE + 1);
    else if (rel < -HALF_ECD_RANGE) rel += (ECD_RANGE + 1);
    return (fp32)rel * MOTOR_ECD_TO_RAD;
}

/**
 * @brief 初始化单个 PID 控制器的参数和运行状态。
 *
 * @param pid 待初始化的 PID 对象。
 * @param maxout 输出限幅。
 * @param max_iout 积分限幅。
 * @param mode 电机类型模式。
 * @param kp 比例系数。
 * @param ki 积分系数。
 * @param kd 微分系数。
 * @param kf 前馈系数。
 */
/* ===== 兼容包装函数：委托至 components/algorithm/PID/PID.c 统一实现 ===== */

/* gimbal_PID_* 内联包装已迁移至 gimbal_task.h */

/**
 * @brief 清空云台所有 yaw 与 pitch 电机的 PID 状态。
 *
 * @param gimbal_clear 云台总控制结构体。
 */
static void gimbal_total_pid_clear(gimbal_control_t *gimbal_clear)
{
    if (gimbal_clear == NULL) return;
    for (uint8_t i = 0; i < YAW_NUM; i++) {
        gimbal_PID_clear(&gimbal_clear->gimbal_yaw_motor[i].gimbal_motor_absolute_angle_pid);
        gimbal_PID_clear(&gimbal_clear->gimbal_yaw_motor[i].gimbal_motor_relative_angle_pid);
        gimbal_PID_clear(&gimbal_clear->gimbal_yaw_motor[i].gimbal_motor_gyro_pid);
        gimbal_PID_clear(&gimbal_clear->gimbal_yaw_motor[i].gimbal_motor_soft_gyro_pid);
    }
    gimbal_PID_clear(&gimbal_clear->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid);
    gimbal_PID_clear(&gimbal_clear->gimbal_pitch_motor.gimbal_motor_relative_angle_pid);
    gimbal_PID_clear(&gimbal_clear->gimbal_pitch_motor.gimbal_motor_gyro_pid);
}

/* PID_clear_for_gimbal 已通过内联包装委托至 PID_ClearExtended。 */

/**
 * @brief 获取指定索引的 yaw 电机状态指针。
 *
 * @param i yaw 电机索引。
 *
 * @return 有效索引返回对应电机指针，否则返回 NULL。
 */
const gimbal_motor_t *get_yaw_motor_point(int i) { return (i >= 0 && i < (int)YAW_NUM) ? &gimbal_control.gimbal_yaw_motor[i] : NULL; }

/**
 * @brief 获取 pitch 电机状态指针。
 *
 * @return pitch 电机指针。
 */
const gimbal_motor_t *get_pitch_motor_point(void) { return &gimbal_control.gimbal_pitch_motor; }

/**
 * @brief 获取指定 yaw 电机的相对角反馈地址。
 *
 * @param i yaw 电机索引。
 *
 * @return 有效索引返回相对角地址，否则返回 NULL。
 */
const fp32 *get_relative_angle(int i) { return (i >= 0 && i < (int)YAW_NUM) ? &gimbal_control.gimbal_yaw_motor[i].relative_angle : NULL; }

/* ===== P2-11: gimbal_control 上下文注入 getter 实现 ===== */

void gimbal_ctrl_get_quaternion(const gimbal_control_t *ctrl, fp32 q[4])
{
    if (ctrl == NULL || q == NULL) return;
    if (ctrl->gimbal_INT_angle_point_DM != NULL) {
        q[0] = ctrl->gimbal_INT_angle_point_DM->quat[0];
        q[1] = ctrl->gimbal_INT_angle_point_DM->quat[1];
        q[2] = ctrl->gimbal_INT_angle_point_DM->quat[2];
        q[3] = ctrl->gimbal_INT_angle_point_DM->quat[3];
    } else {
        q[0] = q[1] = q[2] = q[3] = 0.0f;
    }
}

fp32 gimbal_ctrl_get_yaw_angle(const gimbal_control_t *ctrl, uint8_t idx)
{
    return (ctrl != NULL && idx < YAW_NUM) ? ctrl->gimbal_yaw_motor[idx].absolute_angle : 0.0f;
}

fp32 gimbal_ctrl_get_yaw_gyro(const gimbal_control_t *ctrl, uint8_t idx)
{
    return (ctrl != NULL && idx < YAW_NUM) ? ctrl->gimbal_yaw_motor[idx].motor_gyro : 0.0f;
}

fp32 gimbal_ctrl_get_pitch_angle(const gimbal_control_t *ctrl)
{
    return (ctrl != NULL) ? ctrl->gimbal_pitch_motor.absolute_angle : 0.0f;
}

fp32 gimbal_ctrl_get_pitch_gyro(const gimbal_control_t *ctrl)
{
    return (ctrl != NULL) ? ctrl->gimbal_pitch_motor.motor_gyro : 0.0f;
}

/**
 * @brief 导出当前云台零位和限位参数，供校准流程读取。
 *
 * @param yaw_offset yaw 零位编码器。
 * @param pitch_offset pitch 零位编码器。
 * @param max_yaw yaw 最大角度。
 * @param min_yaw yaw 最小角度。
 * @param max_pitch pitch 最大角度。
 * @param min_pitch pitch 最小角度。
 *
 * @return 参数指针有效时返回 1，否则返回 0。
 */
/* calc_gimbal_axis_cali 已迁移至 gimbal_cali.c */

/* calc_gimbal_cali / cmd_cali_gimbal_hook / set_cali_gimbal_hook 已迁移至 gimbal_cali.c */

/**
 * @brief 计算速度环 PID 输出，并加入阻力前馈与简单抗积分饱和。
 *
 * @param pid PID 对象。
 * @param get 当前反馈值。
 * @param set 目标值。
 *
 * @return PID 输出结果。
 */
/* fb_gimbal_PID_calc 委托至统一的 PID_CalculateVelocity。
 * 注意：原 yaw 摩擦前馈因 KF=0 始终为零，已移除。
 * Resistance_feedforward_count 保留供未来非零 KF 场景使用。 */
fp32 fb_gimbal_PID_calc(gimbal_PID_t *pid, fp32 get, fp32 set)
{
    return PID_CalculateVelocity(pid, get, set);
}

/**
 * @brief 基于经验模型计算 yaw 阻力补偿前馈。
 *
 * @param x 目标速度或等效输入。
 * @param a0 多项式常数项。
 * @param a1 多项式一次项系数。
 * @param a2 多项式二次项系数。
 * @param x_max 前馈上限拐点。
 * @param x_min 前馈线性段拐点。
 *
 * @return 前馈补偿值。
 */
fp32 Resistance_feedforward_count(fp32 x, float a0, float a1, float a2, fp32 x_max, fp32 x_min)
{
    fp32 limited_x = x, min_t = (a0 + a1 * x_min + a2 * x_min * x_min) * sign_value(x), k = 0.0f;
    k = min_t / (x_min - 0.1f);
    if (fabsf(x) < x_min) return k * x;
    if (fabsf(x) > x_max) limited_x = x_max * sign_value(x);
    return (a0 + a1 * limited_x + a2 * limited_x * limited_x) * sign_value(x);
}

/**
 * @brief 使用速度环直接控制单个电机并输出限幅电流。
 *
 * @param gimbal_motor 目标电机。
 */
void gimbal_motor_speed_control(gimbal_motor_t *gimbal_motor)
{
    if (gimbal_motor != NULL) {
        gimbal_motor->current_set = fb_gimbal_PID_calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->motor_gyro, 1.0f);
        gimbal_motor->given_current = current_limit_i16(gimbal_motor->current_set, GIMBAL_YAW_CURRENT_LIMIT);
    }
}
/**
 * @brief yaw 轴失联时的预留处理接口。
 */
void gimbal_yaw_lost_fun(void) {}

/**
 * @brief pitch 轴失联时的预留处理接口。
 */
void gimbal_pitch_lost_fun(void) {}

/**
 * @brief 按最大步长平滑插值目标角，避免指令突变。
 *
 * @param current_angle 当前角度，当前实现未使用。
 * @param current_target 当前目标角指针。
 * @param final_target 最终目标角。
 * @param max_step 单次允许的最大步长。
 */
void motor_interplotion_update(float current_angle, float *current_target, float final_target, float max_step)
{
    const float pi = 3.1415926f, two_pi = 6.2831852f;
    float d = 0.0f, wd = 0.0f;
    (void)current_angle;
    if (current_target == NULL) return;
    d = final_target - *current_target;
    if (d > pi) d -= two_pi; else if (d < -pi) d += two_pi;
    if (fabsf(d) < max_step * 0.1f) { *current_target = final_target; return; }
    *current_target += (d > 0.0f) ? max_step : -max_step;
    *current_target = loop_fp32_constrain(*current_target, -pi, pi);
    wd = final_target - *current_target;
    if (wd > pi) wd -= two_pi; else if (wd < -pi) wd += two_pi;
    if ((wd > 0.0f && *current_target > final_target) || (wd < 0.0f && *current_target < final_target)) *current_target = final_target;
}

/**
 * @brief 生成平滑权重，用于降低大 yaw 在小角度范围内的跟随强度。
 *
 * @param mech_angle_abs 机械相对角绝对值。
 * @param comfort_zone 舒适区阈值。
 * @param active_zone 全激活阈值。
 *
 * @return 0 到 1 的平滑权重。
 */

/**
 * @brief 对输入做平滑软限幅，避免靠近硬限位时出现突变。
 *
 * @param value 输入值。
 * @param comfort_limit 软限位阈值。
 * @param hard_limit 硬限位阈值。
 *
 * @return 限幅后的值。
 */

/**
 * @brief 常规控制策略，yaw 走串级 PID，pitch 走目标角与前馈力矩控制。
 *
 * @param control 待控制的电机对象。
 */
fp32 i_out_watch = 0.0f;


/*
  * @brief          双yaw特殊的pid控制，包含内外yaw的联动控制，采用主从跟随,小yaw跟大yaw
  * @param[out]     gimbal_control_loop 总的结构体指针，方便直接调用内外yaw电机数据
  * @retval         none
  */

/**
 * @brief 双 yaw 扫描模式，小 yaw 锁中位，大 yaw 以恒定角速度巡航。
 *
 * @param gimbal_control_loop 云台总控制结构体。
 */

/**
 * @brief 设置 pitch 变频扫描的中心、幅值和频率范围参数。
 *
 * @param center 扫描中心角。
 * @param range 扫描幅值范围。
 * @param freq_min 最小扫描频率。
 * @param freq_max 最大扫描频率。
 * @param sweep_period 频率扫变周期。
 */
void pitch_varfreq_scan_set_config(fp32 center, fp32 range, fp32 freq_min, fp32 freq_max, fp32 sweep_period)
{
    if (range < 0.0f) range = -range;
    if (freq_min < 0.05f) freq_min = 0.05f;
    if (freq_max < 0.05f) freq_max = 0.05f;
    if (freq_max < freq_min) { fp32 t = freq_min; freq_min = freq_max; freq_max = t; }
    if (sweep_period < 0.2f) sweep_period = 0.2f;
    pitch_varfreq_scan_cfg.center = center; pitch_varfreq_scan_cfg.range = range; pitch_varfreq_scan_cfg.freq_min = freq_min; pitch_varfreq_scan_cfg.freq_max = freq_max; pitch_varfreq_scan_cfg.sweep_period = sweep_period;
}

/**
 * @brief 读取当前 pitch 变频扫描配置。
 *
 * @param center 输出扫描中心角。
 * @param range 输出扫描幅值范围。
 * @param freq_min 输出最小扫描频率。
 * @param freq_max 输出最大扫描频率。
 * @param sweep_period 输出频率扫变周期。
 */
void pitch_varfreq_scan_get_config(fp32 *center, fp32 *range, fp32 *freq_min, fp32 *freq_max, fp32 *sweep_period)
{
    if (center) *center = pitch_varfreq_scan_cfg.center;
    if (range) *range = pitch_varfreq_scan_cfg.range;
    if (freq_min) *freq_min = pitch_varfreq_scan_cfg.freq_min;
    if (freq_max) *freq_max = pitch_varfreq_scan_cfg.freq_max;
    if (sweep_period) *sweep_period = pitch_varfreq_scan_cfg.sweep_period;
}

/**
 * @brief 设置 pitch 扫描幅值缩放的上下限。
 *
 * @param amp_scale_min 幅值缩放下限。
 * @param amp_scale_max 幅值缩放上限。
 */
void pitch_varfreq_scan_set_amp_scale_limit(fp32 amp_scale_min, fp32 amp_scale_max)
{
    pitch_varfreq_scan_cfg.amp_scale_min = fp32_constrain(amp_scale_min, 0.0f, 1.0f);
    pitch_varfreq_scan_cfg.amp_scale_max = fp32_constrain(amp_scale_max, 0.0f, 1.0f);
    if (pitch_varfreq_scan_cfg.amp_scale_max < pitch_varfreq_scan_cfg.amp_scale_min) {
        fp32 t = pitch_varfreq_scan_cfg.amp_scale_min;
        pitch_varfreq_scan_cfg.amp_scale_min = pitch_varfreq_scan_cfg.amp_scale_max;
        pitch_varfreq_scan_cfg.amp_scale_max = t;
    }
}

/**
 * @brief 读取 pitch 扫描幅值缩放的上下限。
 *
 * @param amp_scale_min 输出幅值缩放下限。
 * @param amp_scale_max 输出幅值缩放上限。
 */
void pitch_varfreq_scan_get_amp_scale_limit(fp32 *amp_scale_min, fp32 *amp_scale_max)
{
    if (amp_scale_min) *amp_scale_min = pitch_varfreq_scan_cfg.amp_scale_min;
    if (amp_scale_max) *amp_scale_max = pitch_varfreq_scan_cfg.amp_scale_max;
}

/**
 * @brief 设置 pitch 扫描类型。
 *
 * @param sweep_type 扫描类型编号，当前仅支持 0 和 1。
 */
void pitch_varfreq_scan_set_type(uint8_t sweep_type) { pitch_varfreq_scan_cfg.sweep_type = (sweep_type > 1U) ? 0U : sweep_type; }

/**
 * @brief 获取当前 pitch 扫描类型。
 *
 * @return 扫描类型编号。
 */
uint8_t pitch_varfreq_scan_get_type(void) { return pitch_varfreq_scan_cfg.sweep_type; }

/**
 * @brief 生成 pitch 正弦扫描轨迹，并同步计算角速度与力矩前馈。
 *
 * @param control_loop 云台总控制结构体。
 */

/**
 * @brief 对 pitch 电机执行角度、速度和力矩保护，避免越界和过冲。
 *
 * @param pitch_motor pitch 电机对象。
 */

/**
 * @brief 返回输入值的符号。
 *
 * @param value 输入值。
 *
 * @return 非负返回 1.0f，负数返回 -1.0f。
 */
static fp32 sign_value(fp32 value) { return (value >= 0.0f) ? 1.0f : -1.0f; }

/**
 * @brief 将浮点电流值按限幅裁剪并转换为 int16_t。
 *
 * @param current 输入电流。
 * @param limit 电流限幅值。
 *
 * @return 限幅后的整型电流。
 */
static int16_t current_limit_i16(fp32 current, fp32 limit) { current = fp32_constrain(current, -limit, limit); return (int16_t)current; }





