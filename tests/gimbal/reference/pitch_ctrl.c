#include "pitch_ctrl.h"

#include "arm_math.h"
#include "detect.h"
#include "dm_4310.h"
#include "line_adrc.h"
#include "gimbal_behaviour.h"
#include "gimbal_params.h"
#include "gravity_comp_lut.h"
#include "referee/referee.h"
#include "shoot/shoot_task.h"
#include "usb_callback.h"
#include "user_lib.h"

#include <math.h>

/* 本地工具函数（与 dual_yaw_ctrl.c 中相同的副本） */
static int16_t current_limit_i16(fp32 current, fp32 limit) { current = fp32_constrain(current, -limit, limit); return (int16_t)current; }

void control_motor(gimbal_motor_t *gimbal_motor, uint8_t use_relative)
{
    if (gimbal_motor == NULL) return;
    if (gimbal_motor->gimbal_motor_mode == GIMBAL_MOTOR_RAW) {
        gimbal_motor->current_set = gimbal_motor->raw_cmd_current;
    } else {
        if (!use_relative &&
            gimbal_motor == &gimbal_control.gimbal_yaw_motor[OUTER_YAW] &&
            gimbal_behaviour == GIMBAL_ABSOLUTE_ANGLE) {
            fp32 yaw_error = rad_format(gimbal_motor->absolute_angle_set - gimbal_motor->absolute_angle);
            fp32 true_error = yaw_error;
            const fp32 dt = (fp32)GIMBAL_CONTROL_TIME * 0.001f;



            gimbal_motor->motor_gyro_set = gimbal_PID_calc_by_error(&gimbal_motor->gimbal_motor_absolute_angle_pid,
                                                                     true_error,
                                                                     gimbal_motor->motor_gyro);
            gimbal_motor->current_set = fb_gimbal_PID_calc(&gimbal_motor->gimbal_motor_gyro_pid,
                                                            gimbal_motor->motor_gyro,
                                                            gimbal_motor->motor_gyro_set);

        } else  if (gimbal_motor != &gimbal_control.gimbal_pitch_motor)
        {
            gimbal_motor->motor_gyro_set = use_relative ? gimbal_PID_calc(&gimbal_motor->gimbal_motor_relative_angle_pid, gimbal_motor->relative_angle, gimbal_motor->relative_angle_set, gimbal_motor->motor_gyro) : gimbal_PID_calc(&gimbal_motor->gimbal_motor_absolute_angle_pid, gimbal_motor->absolute_angle, gimbal_motor->absolute_angle_set, gimbal_motor->motor_gyro);
            if (gimbal_motor == &gimbal_control.gimbal_yaw_motor[OUTER_YAW]) gimbal_motor->current_set = fb_gimbal_PID_calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);
            else gimbal_motor->current_set = fb_gimbal_PID_calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);
        }
    }
    if (gimbal_motor->id == PITCH_MOTOR) gimbal_motor->tor = fp32_constrain(gimbal_motor->tor, -GIMBAL_PITCH_TORQUE_LIMIT, GIMBAL_PITCH_TORQUE_LIMIT);
    gimbal_motor->given_current = current_limit_i16(gimbal_motor->current_set, GIMBAL_YAW_CURRENT_LIMIT);
}

void gimbal_pitch_output(gimbal_motor_t *pitch)
{
    if (pitch == NULL || toe_is_error(DBUS_TOE) || gimbal_behaviour == GIMBAL_ZERO_FORCE) {
        DM4310_MIT_CtrlMotor(DM_CAN, FDCAN_DM4310_M1_SLAVE_ID, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        return;
    }
    //pitch_motor_protection(pitch);
    pitch->tor = fp32_constrain(pitch->tor, -GIMBAL_PITCH_TORQUE_LIMIT, GIMBAL_PITCH_TORQUE_LIMIT);
    if (gimbal_behaviour == GIMBAL_INIT ||
        ((gimbal_behaviour == GIMBAL_AUTO || gimbal_behaviour == GIMBAL_AUTO_TEST) &&
         (get_auto_mode() == NAVI_MODE || get_auto_mode() == STOP_MODE))) {
        DM4310_MIT_CtrlMotor(DM_CAN,
                             FDCAN_DM4310_M1_SLAVE_ID,
                             INIT_PITCH_SET,
                             0.5f,
                             PITCH_4310_MIT_CONTROL_KP,
                             PITCH_4310_MIT_CONTROL_KD,
                             0.0f);
        return;
    }
    if (gimbal_behaviour == GIMBAL_AUTO || gimbal_behaviour == GIMBAL_AUTO_TEST) {
        if (get_auto_mode() == SCAN_MODE || get_auto_mode() == NAVI_WITH_SCAN_MODE || get_force_flag()) {
            DM4310_MIT_CtrlMotor(DM_CAN,
                                 FDCAN_DM4310_M1_SLAVE_ID,
                                 pitch->absolute_angle_set,
                                 pitch->motor_gyro_set,
                                 PITCH_4310_MIT_CONTROL_KP,
                                 1.0f,
                                 pitch->tor);
        } else {
            DM4310_MIT_CtrlMotor(DM_CAN, FDCAN_DM4310_M1_SLAVE_ID, 0.0f, pitch->motor_gyro_set, 0.0f, AT1_PITCH_GYRO_ABSOLUTE_PID_MIT_KD, pitch->tor);

            //fp32 pitch_tar_relative_set = get_latest_vision_data()->pitch - gimbal_control.gimbal_pitch_motor.absolute_angle + gimbal_control.gimbal_pitch_motor.relative_angle;
            //DM4310_MIT_CtrlMotor(DM_CAN, FDCAN_DM4310_M1_SLAVE_ID, pitch_tar_relative_set, get_latest_vision_data()->pitch_vel, 50.0f, 1.5f, pitch->tor);
            //调yaw时暂时锁定pitch
            // DM4310_MIT_CtrlMotor(DM_CAN,
            //                  FDCAN_DM4310_M1_SLAVE_ID,
            //                  INIT_PITCH_SET,
            //                  0.5f,
            //                  PITCH_4310_MIT_CONTROL_KP,
            //                  PITCH_4310_MIT_CONTROL_KD,
            //                  0.0f);

        }
        return;
    }
    if (gimbal_behaviour == GIMBAL_ABSOLUTE_ANGLE)
    {
#if PITCH_IDENTIFY
       // DM4310_MIT_CtrlMotor(DM_CAN, FDCAN_DM4310_M1_SLAVE_ID, 0, 0, 0, 0, pitch->tor);
        DM4310_MIT_CtrlMotor(DM_CAN,
                             FDCAN_DM4310_M1_SLAVE_ID,
                             pitch->absolute_angle_set,
                             pitch->motor_gyro_set,
                             PITCH_4310_MIT_CONTROL_KP,
                             PITCH_4310_MIT_CONTROL_KD,
                             pitch->tor);
#else
        DM4310_MIT_CtrlMotor(DM_CAN,
                             FDCAN_DM4310_M1_SLAVE_ID,
                             pitch->relative_angle_set,
                             pitch->motor_gyro_set,
                             PITCH_4310_MIT_CONTROL_KP,
                             PITCH_4310_MIT_CONTROL_KD,
                             0);
        /* ab_set_watch 保留在 gimbal_task.c（static 作用域） */
        //DM4310_MIT_CtrlMotor(DM_CAN, FDCAN_DM4310_M1_SLAVE_ID, 0, 0, 0, 0, pitch->tor);
#endif
    }

}

void gimbal_absolute_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add)
{
    if (gimbal_motor != NULL) gimbal_motor->absolute_angle_set = rad_format(gimbal_motor->absolute_angle_set + add);
}

void gimbal_absolute_angle_limit_pitch(gimbal_motor_t *gimbal_motor, fp32 add)
{
    fp32 bias = 0.0f;
    if (gimbal_motor == NULL) return;
    bias = rad_format(gimbal_motor->absolute_angle_set - gimbal_motor->absolute_angle);
    if (gimbal_motor->relative_angle + bias + add > gimbal_motor->max_relative_angle && add > 0.0f) add = gimbal_motor->max_relative_angle - gimbal_motor->relative_angle - bias;
    else if (gimbal_motor->relative_angle + bias + add < gimbal_motor->min_relative_angle && add < 0.0f) add = gimbal_motor->min_relative_angle - gimbal_motor->relative_angle - bias;
    gimbal_motor->absolute_angle_set = fp32_constrain(gimbal_motor->absolute_angle_set + add, gimbal_motor->min_relative_angle, gimbal_motor->max_relative_angle);
}

void gimbal_relative_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add)
{
    if (gimbal_motor != NULL) gimbal_motor->relative_angle_set = fp32_constrain(gimbal_motor->relative_angle_set + add, gimbal_motor->min_relative_angle, gimbal_motor->max_relative_angle);
}

/* Pitch 扰动补偿策略: 是否开启角度ESO */
#define PITCH_USE_ESO 0

/* Pitch ESO 门控阈值（仅 PITCH_USE_ESO=1 时生效） */
#define PITCH_ESO_OMEGA_GATE  5.0f    /**< 角速度门控 (rad/s)，超过此值关闭补偿 */
#define PITCH_ESO_ALPHA_GATE  20.0f   /**< 角加速度门控 (rad/s²)，超过此值关闭补偿 */

// Pitch I-Zone 积分（传统 I 项）
static inline void pitch_i_update(pid_type_def *pid, fp32 error_rad,
                                  fp32 i_max, fp32 i_min, uint8_t do_reset)
{
    if (fabsf(error_rad) <= 0.05f && fabsf(error_rad) > 0.004f) {
        pid->Integral += PITCH_GYRO_ABSOLUTE_PID_KI * error_rad;
        if (pid->Integral > i_max) pid->Integral = i_max;
        else if (pid->Integral < i_min) pid->Integral = i_min;
    }
    if (error_rad * pid->Integral < 0.0f) pid->Integral *= 0.85f;
    if (do_reset) pid->Integral = 0.0f;
}

// Pitch 独立 ESO — 从角度误差观测总扰动（重力残差/摩擦/外力/发射后座力）
static fp32 pitch_eso_update(fp32 actual_angle, fp32 actual_gyro,
                             fp32 torque, fp32 alpha_ref, fp32 dt,
                             uint8_t do_reset, uint8_t *comp_active)
{
    static ESO_2nd_t eso;
    static uint8_t inited = 0U;
    const fp32 wo      = 60.0f;    // 观测器带宽 (rad/s)
    const fp32 b0      = 1.0f / PITCH_J;
    const fp32 damping = PITCH_B / PITCH_J;

    if (!inited || do_reset) {
        ESO_2nd_Init(&eso, wo, b0, damping,
                     PITCH_ESO_OMEGA_GATE, PITCH_ESO_ALPHA_GATE, dt);
        ESO_2nd_Reset(&eso, actual_angle, actual_gyro);
        inited = 1U;
        if (comp_active != NULL) *comp_active = 0U;
        return 0.0f;
    }

    return ESO_2nd_Update(&eso, actual_angle, actual_gyro,
                          torque, alpha_ref, comp_active);
}

static float PitchCoulombComp(float omega_ref)
{
    if (PITCH_Fc == 0.0f) return 0.0f;
    return PITCH_Fc * tanhf(omega_ref / PITCH_COULOMB_SMOOTH);
}


void NormalControlStrategy(gimbal_motor_t *control)
{
    const VisionToGimbal *vision = get_latest_vision_data();
    fp32 target_pitch = 0.0f;
    fp32 target_pitch_vel = 0.0f;
    fp32 target_pitch_acc = 0.0f;

    if (control == NULL) return;

    /* 模式切换时重置扰动补偿状态 + 力矩 */
    if ((last_gimbal_behaviour != gimbal_behaviour) || get_auto_last_mode() != get_auto_mode()) {
        control->gimbal_motor_absolute_angle_pid.Integral = 0.0f;
        control->tor = 0.0f;
    }

    /* Yaw 轴路径：使用标准串级 PID */
    if (control->id == YAW_MOTOR) {
        control->motor_gyro_set = gimbal_PID_calc(&control->gimbal_motor_absolute_angle_pid, control->absolute_angle, control->absolute_angle_set, control->motor_gyro);
        if (control == &gimbal_control.gimbal_yaw_motor[OUTER_YAW]) control->current_set = fb_gimbal_PID_calc(&control->gimbal_motor_soft_gyro_pid, control->motor_gyro, control->motor_gyro_set);
        else control->current_set = fb_gimbal_PID_calc(&control->gimbal_motor_gyro_pid, control->motor_gyro, control->motor_gyro_set);
        control->last_gyro = control->motor_gyro;
        control->given_current = current_limit_i16(control->current_set, GIMBAL_YAW_CURRENT_LIMIT);
        return;
    }

    /* ── Pitch 轴：MIT 力矩控制（P-D + ESO + J/B/G 前馈）── */

    if (vision != NULL) {
        target_pitch = vision->pitch;
        target_pitch_vel = vision->pitch_vel;
        target_pitch_acc = vision->pitch_acc;
    } else {
        target_pitch = get_pitch_angle();
        target_pitch_vel = get_pitch_vel();
    }
    /*限幅*/
    control->absolute_angle_set = fp32_constrain(target_pitch, PITCH_MIN_ABSOLUTE_ANGLE, PITCH_MAX_ABSOLUTE_ANGLE);
    control->motor_gyro_set = (target_pitch> PITCH_MAX_ABSOLUTE_ANGLE || target_pitch < PITCH_MIN_ABSOLUTE_ANGLE) ? 0.0f : target_pitch_vel;
    target_pitch_acc = (target_pitch> PITCH_MAX_ABSOLUTE_ANGLE || target_pitch < PITCH_MIN_ABSOLUTE_ANGLE) ? 0.0f : target_pitch_acc;

    fp32 error_rad = rad_format(control->absolute_angle_set  - gimbal_control.gimbal_pitch_motor.absolute_angle);
    fp32 vel_err = control->motor_gyro_set - gimbal_control.gimbal_pitch_motor.motor_gyro;

    /* P-D 计算 — 固定使用 PITCH_GYRO_ABSOLUTE 参数，不跟随 auto_pid_change 切换 */
    fp32 pid_out = PITCH_GYRO_ABSOLUTE_PID_KP * error_rad + PITCH_GYRO_ABSOLUTE_PID_KD * vel_err;

    /* 扰动补偿: I-Zone 或 独立ESO */
    const uint8_t do_reset = (uint8_t)(fabs(control->absolute_angle_set  - gimbal_control.gimbal_pitch_motor.absolute_angle) > 0.1f);
    fp32 disturbance_torque=0.0f;
#if PITCH_USE_ESO
    /* 用电机实际力矩反馈驱动 ESO（而非指令力矩），含阻尼项 + 门控 */
    {
        fp32 actual_torque = 0.0f;
        uint8_t eso_comp_active = 0U;
        if (control->gimbal_motor_measure != NULL && control->gimbal_motor_measure->motor_pitch_measure != NULL) {
            actual_torque = control->gimbal_motor_measure->motor_pitch_measure->torque;
        }
        disturbance_torque = pitch_eso_update(
            gimbal_control.gimbal_pitch_motor.absolute_angle,
            gimbal_control.gimbal_pitch_motor.motor_gyro,
            actual_torque, target_pitch_acc,
            (fp32)GIMBAL_CONTROL_TIME * 0.001f, do_reset, &eso_comp_active);
        /* 门控关闭时撤出补偿，避免大动态下 z3 滞后帮倒忙 */
        if (!eso_comp_active) {
            disturbance_torque = 0.0f;
        }
    }
#endif
    {
        pid_type_def *pid = &control->gimbal_motor_absolute_angle_pid;
        pitch_i_update(pid, error_rad, 0.2f, -0.3f, do_reset);
        disturbance_torque += pid->Integral;
    }

    /* J/B/G 模型前馈 */
    fp32 accel_ff = PITCH_J * target_pitch_acc;
    fp32 gyro_ff = PITCH_B * control->motor_gyro_set;
    fp32 gravity_ff = -PITCH_G * cosf(gimbal_control.gimbal_pitch_motor.absolute_angle);
    fp32 friction_ff = PitchCoulombComp(control->motor_gyro_set);
#if PITCH_DATA_COLLECT
    J_out = accel_ff;
    B_out = gyro_ff;
    G_out = gravity_ff;
    i_out = disturbance_torque;
    f_out = friction_ff;
#endif

    control->tor = fp32_constrain((float)(PITCH_J * pid_out + accel_ff + gyro_ff + gravity_ff + disturbance_torque+friction_ff),
                                  -GIMBAL_PITCH_TORQUE_LIMIT,
                                  GIMBAL_PITCH_TORQUE_LIMIT);
#if PITCH_DATA_COLLECT
    total_tor_out = control->tor;
#endif
}

void Pitch_Scan_Mode(gimbal_control_t *control_loop)
{
    float current_amp = 0.0f;
    float omega = 0.0f;
    float sin_val = 0.0f;
    float cos_val = 0.0f;
    float ff_acc = 0.0f;
    gimbal_motor_t *pPitch = NULL;
    static float phase = 0.0f;
    static float current_center = 0.0f; // 当前实际执行的中心点（会慢慢变）
    static float amp_scale = 0.0f;
    static uint32_t last_call_tick = 0U;

    float scan_center = 0.0f;
    float scan_amp =0.0f;
    float scan_freq = 0.0f;

    if (control_loop == NULL) return;
    pPitch = &control_loop->gimbal_pitch_motor;


    if (get_game_progress()== PROGRESS_BATTLE &&  !enemy_outpost_is_destroyed() ) //比赛时且地方前哨站没掉
    {
        scan_center = P_SCAN_HIGH_CENTER;
        scan_amp = P_SCAN_HIGH_AMP;
        scan_freq = P_SCAN_LOW_FREQ;
    }
    else
    {
        scan_center = P_SCAN_CENTER;
        scan_amp = P_SCAN_AMP;
        scan_freq = P_SCAN_FREQ;
    }

    // scan_center = P_SCAN_HIGH_CENTER;
    // scan_amp = P_SCAN_HIGH_AMP;
    // scan_freq = P_SCAN_LOW_FREQ;

    // scan_center = P_SCAN_HIGH_CENTER;
    // scan_amp = P_SCAN_HIGH_AMP;
    // scan_freq = P_SCAN_FREQ;
    // --- 2. 自动复位逻辑 ---
    // 检测是否刚切入该模式 (间隔 > 20ms)
    if (HAL_GetTick() - last_call_tick > 20U) {
        // [关键] 初始时刻，把“执行中心”锁死在“当前电机角度”
        // 这样无论目标中心在哪，起始瞬间误差都为0，绝对不跳变
        current_center = pPitch->absolute_angle;
        phase = 0.0f;
        amp_scale = 0.0f; // 从0开始渐显
    }
    last_call_tick = HAL_GetTick();

    // --- 3. 中心点平滑归位 (核心修改) ---
    if (current_center < scan_center - P_CENTER_MOVE_SPD) current_center += P_CENTER_MOVE_SPD;
    else if (current_center > scan_center + P_CENTER_MOVE_SPD) current_center -= P_CENTER_MOVE_SPD;
    else current_center = scan_center;

    // --- 4. 波形计算 ---
    if (amp_scale < 1.0f) {
        amp_scale += P_RAMP_STEP;
        if (amp_scale > 1.0f) amp_scale = 1.0f;
    }

    current_amp = scan_amp * amp_scale;
    omega = 6.2831853f * scan_freq; // w = 2*PI*f

    // 相位更新
    phase += omega * 0.001f; // dt = 1ms
    if (phase > 6.2831853f) phase -= 6.2831853f;

    // --- 5. 核心输出赋值 ---
    sin_val = arm_sin_f32(phase);
    cos_val = arm_cos_f32(phase);
    pPitch->absolute_angle_set = fp32_constrain(current_center + current_amp * sin_val, PITCH_MIN_ABSOLUTE_ANGLE, PITCH_MAX_ABSOLUTE_ANGLE);
    pPitch->motor_gyro_set = current_amp * omega * cos_val;
    ff_acc = -1.0f * current_amp * omega * omega * sin_val;
    pPitch->tor = fp32_constrain((PITCH_J * ff_acc) - (PITCH_G * arm_cos_f32(pPitch->absolute_angle_set)),
                                 -GIMBAL_PITCH_TORQUE_LIMIT,
                                 GIMBAL_PITCH_TORQUE_LIMIT);
}

void pitch_motor_protection(gimbal_motor_t *pitch_motor)
{
#if PITCH_PROTECTION_ENABLE
    fp32 last_angle_set = 0.0f, last_tor = 0.0f;
    static uint8_t braking = 0U, inited = 0U;
    fp32 angle_delta = 0.0f, abs_speed = 0.0f, margin_upper = 0.0f, margin_lower = 0.0f, tor_delta = 0.0f;
    if (pitch_motor == NULL) return;
    memset(&pitch_prot_dbg, 0, sizeof(pitch_prot_dbg));
    if (!inited) { last_angle_set = pitch_motor->absolute_angle; inited = 1U; }
    angle_delta = pitch_motor->absolute_angle_set - last_angle_set;
    if (angle_delta > PITCH_ANGLE_SET_MAX_DELTA) { pitch_motor->absolute_angle_set = last_angle_set + PITCH_ANGLE_SET_MAX_DELTA; pitch_prot_dbg.angle_rate_limited = 1U; }
    else if (angle_delta < -PITCH_ANGLE_SET_MAX_DELTA) { pitch_motor->absolute_angle_set = last_angle_set - PITCH_ANGLE_SET_MAX_DELTA; pitch_prot_dbg.angle_rate_limited = 1U; }
    last_angle_set = pitch_motor->absolute_angle_set;
    if (pitch_motor->absolute_angle_set > pitch_motor->max_relative_angle) { pitch_motor->absolute_angle_set = pitch_motor->max_relative_angle; pitch_prot_dbg.angle_clamped = 1U; }
    else if (pitch_motor->absolute_angle_set < pitch_motor->min_relative_angle) { pitch_motor->absolute_angle_set = pitch_motor->min_relative_angle; pitch_prot_dbg.angle_clamped = 1U; }
    abs_speed = fabsf(pitch_motor->motor_gyro);
    if (abs_speed > PITCH_MAX_SPEED) braking = 1U;
    if (braking) { pitch_prot_dbg.overspeed_braking = 1U; pitch_motor->tor = 0.0f; pitch_motor->motor_gyro_set = 0.0f; if (abs_speed < PITCH_SPEED_RECOVER) braking = 0U; last_tor = 0.0f; pitch_prot_dbg.any_triggered = 1U; return; }
    if (pitch_motor->motor_gyro_set > PITCH_MAX_VEL_CMD) { pitch_motor->motor_gyro_set = PITCH_MAX_VEL_CMD; pitch_prot_dbg.vel_cmd_limited = 1U; }
    else if (pitch_motor->motor_gyro_set < -PITCH_MAX_VEL_CMD) { pitch_motor->motor_gyro_set = -PITCH_MAX_VEL_CMD; pitch_prot_dbg.vel_cmd_limited = 1U; }
    margin_upper = pitch_motor->max_relative_angle - PITCH_SOFT_LIMIT_MARGIN;
    margin_lower = pitch_motor->min_relative_angle + PITCH_SOFT_LIMIT_MARGIN;
    if (pitch_motor->relative_angle > margin_upper) { pitch_prot_dbg.soft_limit_upper = 1U; if (pitch_motor->tor > 0.0f) pitch_motor->tor = 0.0f; if (pitch_motor->motor_gyro_set > 0.0f) pitch_motor->motor_gyro_set = 0.0f; }
    else if (pitch_motor->relative_angle < margin_lower) { pitch_prot_dbg.soft_limit_lower = 1U; if (pitch_motor->tor < 0.0f) pitch_motor->tor = 0.0f; if (pitch_motor->motor_gyro_set < 0.0f) pitch_motor->motor_gyro_set = 0.0f; }
    tor_delta = pitch_motor->tor - last_tor;
    if (tor_delta > PITCH_TORQUE_MAX_DELTA) { pitch_motor->tor = last_tor + PITCH_TORQUE_MAX_DELTA; pitch_prot_dbg.torque_rate_limited = 1U; }
    else if (tor_delta < -PITCH_TORQUE_MAX_DELTA) { pitch_motor->tor = last_tor - PITCH_TORQUE_MAX_DELTA; pitch_prot_dbg.torque_rate_limited = 1U; }
    last_tor = pitch_motor->tor;
    if (pitch_motor->tor > PITCH_MAX_TORQUE) { pitch_motor->tor = PITCH_MAX_TORQUE; pitch_prot_dbg.torque_clamped = 1U; }
    else if (pitch_motor->tor < -PITCH_MAX_TORQUE) { pitch_motor->tor = -PITCH_MAX_TORQUE; pitch_prot_dbg.torque_clamped = 1U; }
    pitch_prot_dbg.any_triggered = pitch_prot_dbg.angle_rate_limited | pitch_prot_dbg.angle_clamped | pitch_prot_dbg.overspeed_braking | pitch_prot_dbg.vel_cmd_limited | pitch_prot_dbg.soft_limit_upper | pitch_prot_dbg.soft_limit_lower | pitch_prot_dbg.hard_limit_pullback | pitch_prot_dbg.torque_rate_limited | pitch_prot_dbg.torque_clamped;
#else
    (void)pitch_motor;
#endif
}



//此函数原本思路是使用TD观测器+DM4310自带MIT速度环进行跟踪，然而效果不好，现在已经退化为遥控模式下的纯MIT跟踪
void TD_MIT_Control(gimbal_motor_t *gimbal_motor,const uint8_t using_relative_angle)
{
    const fp32 dt = (fp32)GIMBAL_CONTROL_TIME * 0.001f;
    fp32 tor_pid = 0.0f;

    if (gimbal_motor == NULL) {
        return;
    }
    fp32 angle_set=0;
    fp32 angle_feedback=0;
    if (using_relative_angle)
    {
        angle_set = gimbal_motor->relative_angle_set;
        angle_feedback = gimbal_motor->relative_angle;
    }
    else
    {
        angle_set = gimbal_motor->absolute_angle_set;
        angle_feedback = gimbal_motor->absolute_angle;
    }
    if (gimbal_motor->td_inited == 0U) {
        gimbal_motor->td_angle = angle_feedback;
        gimbal_motor->td_gyro = gimbal_motor->motor_gyro;
        gimbal_motor->td_accel = 0.0f;
        gimbal_motor->td_inited = 1U;
    }

    gimbal_motor->td_accel = (PITCH_TD_W * PITCH_TD_W) * (angle_set - gimbal_motor->td_angle) -
                             (2.0f * PITCH_TD_ZETA * PITCH_TD_W) * gimbal_motor->td_gyro;
    gimbal_motor->td_gyro += gimbal_motor->td_accel * dt;
    gimbal_motor->td_angle += gimbal_motor->td_gyro * dt;

    if (using_relative_angle)
    {
        gimbal_motor->td_angle = fp32_constrain(gimbal_motor->td_angle,
                                            gimbal_motor->min_relative_angle,
                                            gimbal_motor->max_relative_angle);
    }


    gimbal_motor->motor_gyro_set = gimbal_motor->td_gyro;
    if (using_relative_angle)
    {
        tor_pid = gimbal_PID_calc(&gimbal_motor->gimbal_motor_relative_angle_pid,
                              angle_feedback,
                              gimbal_motor->td_angle,
                              gimbal_motor->motor_gyro);
    }
    else
    {
        tor_pid = gimbal_PID_calc(&gimbal_motor->gimbal_motor_absolute_angle_pid,
                              angle_feedback,
                              gimbal_motor->td_angle,
                              gimbal_motor->motor_gyro);
    }


    // 重力补偿使用当前反馈角度，避免姿态误差时前馈相位错误。
    gimbal_motor->gravity_compensation = gravity_comp_lut_get_compensation(gimbal_motor->absolute_angle);
    gimbal_motor->tor = PITCH_J * gimbal_motor->td_accel + tor_pid + gimbal_motor->gravity_compensation;
    gimbal_motor->tor = fp32_constrain(gimbal_motor->tor,
                                       -GIMBAL_PITCH_TORQUE_LIMIT,
                                       GIMBAL_PITCH_TORQUE_LIMIT);
}

