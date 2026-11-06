#include "gimbal/dual_yaw_ctrl.h"
#include "arm_math.h"
#include "detect.h"
#include "gimbal_behaviour.h"
#include "usb_callback.h"
#include "gimbal_params.h"
#include "referee/referee.h"
#include "shoot/shoot_task.h"

/* ===== 本地工具函数 ===== */

static fp32 sign_value(fp32 value) { return (value >= 0.0f) ? 1.0f : -1.0f; }
static int16_t current_limit_i16(fp32 current, fp32 limit) { current = fp32_constrain(current, -limit, limit); return (int16_t)current; }

/* fast_smooth_limit: 与 auto_shoot_task.c 版本逻辑略有差异（多 edge case 处理），保留为 static */

/* ===== 工具函数 ===== */

static fp32 calc_big_yaw_weight(fp32 mech_angle_abs, fp32 comfort_zone, fp32 active_zone)
{
    if (mech_angle_abs <= comfort_zone) return 0.0f;
    if (mech_angle_abs >= active_zone) return 1.0f;
    mech_angle_abs = (mech_angle_abs - comfort_zone) / (active_zone - comfort_zone);
    return mech_angle_abs * mech_angle_abs * (3.0f - 2.0f * mech_angle_abs);
}

static fp32 fast_smooth_limit(fp32 value, fp32 comfort_limit, fp32 hard_limit)
{
    fp32 abs_value = fabsf(value);
    fp32 limited = 0.0f;

    if (hard_limit <= comfort_limit) return fp32_constrain(value, -hard_limit, hard_limit);
    if (abs_value <= comfort_limit) return value;
    if (abs_value >= hard_limit) return sign_value(value) * hard_limit;

    abs_value = (abs_value - comfort_limit) / (hard_limit - comfort_limit);
    limited = comfort_limit + (hard_limit - comfort_limit) * (abs_value * abs_value * (3.0f - 2.0f * abs_value));
    return sign_value(value) * limited;
}


/*
  * @brief          双yaw特殊的pid控制，包含内外yaw的联动控制，采用主从跟随，大yaw跟小yaw，小yaw防撞限位的算法也放在这里
  * @param[out]     gimbal_control_loop 总的结构体指针，方便直接调用内外yaw电机数据
  * @retval         none
  */

fp32 small_iout = 0.0f;
fp32 small_pout = 0.0f;

/**
 *
 * @param gimbal_control_loop 总的结构体指针，方便直接调用内外yaw电机数据
 * @param b_yaw_error 大yaw电机最终送入Pid的误差值
 * @param s_yaw_error 小yaw电机最终送入Pid的误差值
 * @brief 双yaw特殊的控制规划，计算出最终的error值供pid调用，只包含联动的轨迹计算逻辑，独立于控制优化部分。
 *       大yaw拖拽目标存储在 gimbal_control_loop->big_yaw_drag_target 中。
 */
void Two_Yaw_small_yaw_first_final_err_calculate(gimbal_control_t* gimbal_control_loop,fp32* b_yaw_error,fp32* s_yaw_error)
{


    /**获取数据**/
    fp32 rel_cur = gimbal_control_loop->gimbal_yaw_motor[OUTER_YAW].relative_angle;
    fp32 small_abs_cur = gimbal_control_loop->gimbal_yaw_motor[OUTER_YAW].absolute_angle;
    fp32 big_abs_cur = gimbal_control_loop->gimbal_yaw_motor[INNER_YAW].absolute_angle;
    fp32 small_yaw_target = gimbal_control_loop->gimbal_yaw_motor[OUTER_YAW].absolute_angle_set;

    /**************************************小yaw：最短路径 + 软限位防撞**************************************/
    fp32 small_abs_err = rad_format(small_yaw_target - small_abs_cur);
    fp32 target_rel = rel_cur + small_abs_err;

    //软限位边界,限位对称故合并处理，不需要区分正负
    fp32 soft_max = gimbal_control_loop->gimbal_yaw_motor[OUTER_YAW].max_relative_angle-0.1f;
    fp32 soft_min = gimbal_control_loop->gimbal_yaw_motor[OUTER_YAW].min_relative_angle+0.1f;
    fp32 safe_target_rel = fp32_constrain(target_rel, soft_min, soft_max);
    fp32 small_yaw_error = safe_target_rel-rel_cur;

    /**************************************大yaw：增量预测（避开双IMU坐标系偏差）**************************************/
    fp32 deadzone = soft_max * 0.4f;

    // 计算大Yaw在它自己的坐标系下，目前想要走多远
    fp32 big_abs_err = rad_format(gimbal_control_loop->big_yaw_drag_target - big_abs_cur);

    //预测如果两个Yaw都到达了各自的目标，最终的物理相对角度
    // 相对角未来值 = 当前相对角 + (小yaw要走的距离) - (大yaw要走的距离)
    fp32 expected_rel = rel_cur + small_abs_err - big_abs_err;

    //我们只关心未来相对角有没有越过死区
    if (expected_rel > deadzone) {
        // 预期超出了正向死区，把大Yaw目标往前拖拽
        gimbal_control_loop->big_yaw_drag_target += (expected_rel - deadzone);
        gimbal_control_loop->big_yaw_drag_target = rad_format(gimbal_control_loop->big_yaw_drag_target);
    }
    else if (expected_rel < -deadzone) {
        // 预期超出了负向死区，把大Yaw目标往后拖拽
        gimbal_control_loop->big_yaw_drag_target += (expected_rel + deadzone);
        gimbal_control_loop->big_yaw_drag_target = rad_format(gimbal_control_loop->big_yaw_drag_target);
    }
    //如果在死区内，big_yaw_drag_target 保持上一拍的绝对静止

    // 大Yaw闭环：在它自己的坐标系下做绝对位置控制
    fp32 big_yaw_error = rad_format(gimbal_control_loop->big_yaw_drag_target - big_abs_cur);

    //传出
    *b_yaw_error = big_yaw_error;
    *s_yaw_error = small_yaw_error;

}


/**
 *
 * @param gimbal_control_loop 总的结构体指针，方便直接调用内外yaw电机数据
 * @brief 双yaw特殊的pid控制，包含内外yaw的联动控制，采用主从跟随,小yaw跟大yaw
 */
void TwoYAW_gimbal_absolute_angle_control_small_yaw_first(gimbal_control_t *gimbal_control_loop)
{
    fp32 err_low = 0.15f;
    fp32 err_high = 0.2f;
    fp32 err_abs = 0.0f;
    fp32 t = 0.0f;
    fp32 comfort_zone = 0.7f;
    fp32 active_zone = 0.8f;
    fp32 small_yaw_state_zone = 0.4f;
    fp32 big_yaw_state_judge = 0.0f;
    fp32 small_yaw_state_judge = 0.0f;
    fp32 ff_vel = 0.0f;
    fp32 ff_accel = 0.0f;
    fp32 small_yaw_pid_error = 0.0f;

    gimbal_PID_t *outer_abs_pid = NULL;
    gimbal_PID_t *outer_gyro_pid = NULL;
    if (gimbal_control_loop == NULL) return;

    // 自追踪模式切换：首次进入或行为模式/自瞄模式变化时，同步拖拽目标到当前大yaw角度
    {
        static gimbal_behaviour_e prev_behaviour = (gimbal_behaviour_e)-1;
        static uint8_t prev_auto_mode = 0xFFU;
        if (prev_behaviour != gimbal_behaviour || prev_auto_mode != get_auto_mode())
        {
            gimbal_control_loop->big_yaw_drag_target = gimbal_control_loop->gimbal_yaw_motor[INNER_YAW].absolute_angle;
            prev_behaviour = gimbal_behaviour;
            prev_auto_mode = get_auto_mode();
        }
    }

    fp32 small_yaw_error=0.0f;
    fp32 big_yaw_error = 0.0f;

    //通过运动轨迹规划计算最终需要的误差值
    Two_Yaw_small_yaw_first_final_err_calculate(gimbal_control_loop,&big_yaw_error,&small_yaw_error);

    // 基于误差角大小线性切换小yaw角度环PID: |error|<0.15 用原PID, |error|>0.2 用MIN PID
    err_abs = fabsf(small_yaw_error);
    if (err_abs >= err_high) t = 1.0f;
    else if (err_abs > err_low) t = (err_abs - err_low) / (err_high - err_low);

    outer_abs_pid = &gimbal_control_loop->gimbal_yaw_motor[OUTER_YAW].gimbal_motor_absolute_angle_pid;
    outer_gyro_pid = &gimbal_control_loop->gimbal_yaw_motor[OUTER_YAW].gimbal_motor_gyro_pid;
    outer_abs_pid->Param.KP = OUTER_YAW_GYRO_ABSOLUTE_PID_KP + (OUTER_YAW_GYRO_ABSOLUTE_MIN_PID_KP - OUTER_YAW_GYRO_ABSOLUTE_PID_KP) * t;
    outer_abs_pid->Param.KD = OUTER_YAW_GYRO_ABSOLUTE_PID_KD + (OUTER_YAW_GYRO_ABSOLUTE_MIN_PID_KD - OUTER_YAW_GYRO_ABSOLUTE_PID_KD) * t;
    outer_abs_pid->Param.LimitIntegral = OUTER_YAW_GYRO_ABSOLUTE_PID_MAX_IOUT + (OUTER_YAW_GYRO_ABSOLUTE_MIN_PID_MAX_IOUT - OUTER_YAW_GYRO_ABSOLUTE_PID_MAX_IOUT) * t;
    outer_gyro_pid->Param.KP = OUTER_YAW_SPEED_PID_KP + (OUTER_YAW_SPEED_MIN_PID_KP - OUTER_YAW_SPEED_PID_KP) * t;
    outer_gyro_pid->Param.KD = OUTER_YAW_SPEED_PID_KD + (OUTER_YAW_SPEED_MIN_PID_KD - OUTER_YAW_SPEED_PID_KD) * t;
    // gyro PID 的 ki/max_iout 不随 t 变，保持初始化值

  //state状态机判断，用于判断是否使用前馈,积分
    big_yaw_state_judge = fabsf(big_yaw_error);
    small_yaw_state_judge = fabsf(small_yaw_error);
    if (gimbal_control_loop->big_yaw_state == 0U) {
        if (big_yaw_state_judge < active_zone - 0.1f) {
            gimbal_control_loop->big_yaw_state = 1U;
            /* 进入大角度模式时清零大yaw角度环PID，防止state 0累积的I项导致缓慢漂移 */
            PID_clear_for_gimbal(&gimbal_control_loop->gimbal_yaw_motor[INNER_YAW].gimbal_motor_absolute_angle_pid);
        }
    } else {
        if (big_yaw_state_judge > active_zone + 0.05f) gimbal_control_loop->big_yaw_state = 0U;
    }


    if (gimbal_control_loop->small_yaw_state == 0U) {
        if (small_yaw_state_judge < small_yaw_state_zone) gimbal_control_loop->small_yaw_state = 1U;
    } else {
        if (small_yaw_state_judge > active_zone) gimbal_control_loop->small_yaw_state = 0U;
    }

    // if (gimbal_control_loop->big_yaw_state == 0U) {
    //     big_yaw_error_final = big_yaw_error; // 全力跟随
    // } else {
    //     big_yaw_error_final = big_yaw_error; // 直接跟踪绝对目标，连续过零无跳变，P项自然刹车
    //     if (fabsf(relative_error) < comfort_zone &&
    //         (gimbal_behaviour == GIMBAL_AUTO || gimbal_behaviour == GIMBAL_AUTO_TEST)) {
    //         big_yaw_error_final = get_enemy_center_yaw() + relative_error - gimbal_control_loop->gimbal_yaw_motor[OUTER_YAW].absolute_angle; // 自瞄时用小yaw相对角补偿瞄准敌人中心
    //     }
    // }
    /* big_yaw_error_end watch 变量保留在 gimbal_task.c（static 作用域） */
    small_yaw_pid_error = small_yaw_error;


    ff_vel = get_yaw_vel() * gimbal_control_loop->gimbal_yaw_motor[OUTER_YAW].kp_plan_gyro;
    ff_accel = get_yaw_accel() * gimbal_control_loop->gimbal_yaw_motor[OUTER_YAW].kp_plan_accel;


    /*************************************************PID计算****************************************************/
    //小yaw角度环，速度环串级pid计算
    gimbal_control_loop->gimbal_yaw_motor[OUTER_YAW].motor_gyro_set =
        gimbal_PID_calc_by_error(&gimbal_control_loop->gimbal_yaw_motor[OUTER_YAW].gimbal_motor_absolute_angle_pid,
                                 small_yaw_pid_error,
                                 gimbal_control_loop->gimbal_yaw_motor[OUTER_YAW].motor_gyro);

    //PS:角速度前馈在效果上可以增加响应的剧烈程度，可以起到减小相位的作用。
    //这里不选择直接加角速度，而是在速度环前引入角速度前馈，一方面符合物理直觉，一方面速度环本身能起到滤波效果，可以减小前馈值噪声导致的扰动
    if (gimbal_control_loop->small_yaw_state == 1U) { //小幅度摆动才用前馈
        gimbal_control_loop->gimbal_yaw_motor[OUTER_YAW].motor_gyro_set += ff_vel;
    }
    gimbal_control_loop->gimbal_yaw_motor[OUTER_YAW].current_set =
        fb_gimbal_PID_calc(&gimbal_control_loop->gimbal_yaw_motor[OUTER_YAW].gimbal_motor_gyro_pid,
                           gimbal_control_loop->gimbal_yaw_motor[OUTER_YAW].motor_gyro,
                           gimbal_control_loop->gimbal_yaw_motor[OUTER_YAW].motor_gyro_set);
    gimbal_control_loop->gimbal_yaw_motor[OUTER_YAW].last_gyro = gimbal_control_loop->gimbal_yaw_motor[OUTER_YAW].motor_gyro;
    gimbal_control_loop->gimbal_yaw_motor[OUTER_YAW].given_current =
        current_limit_i16(gimbal_control_loop->gimbal_yaw_motor[OUTER_YAW].current_set, GIMBAL_YAW_CURRENT_LIMIT);

    //大yaw角度环，速度环串级pid计算
    gimbal_control_loop->gimbal_yaw_motor[INNER_YAW].motor_gyro_set =
        gimbal_PID_calc_by_error(&gimbal_control_loop->gimbal_yaw_motor[INNER_YAW].gimbal_motor_absolute_angle_pid,
                                 big_yaw_error,
                                 gimbal_control_loop->gimbal_yaw_motor[INNER_YAW].motor_gyro);
    /* Sqrt-Law 近端制动：大角度全力模式下限制速度指令上限，防止动能过冲导致超调 */
    if (gimbal_control_loop->big_yaw_state == 0U) {
        const fp32 inner_yaw_brake_decel = 20.0f;
        fp32 max_gyro_braking = 0.0f;
        arm_sqrt_f32(2.0f * inner_yaw_brake_decel * fabsf(big_yaw_error), &max_gyro_braking);
        if (fabsf(gimbal_control_loop->gimbal_yaw_motor[INNER_YAW].motor_gyro_set) > max_gyro_braking) {
            gimbal_control_loop->gimbal_yaw_motor[INNER_YAW].motor_gyro_set =
                sign_value(big_yaw_error) * max_gyro_braking;
        }
    }
    gimbal_control_loop->gimbal_yaw_motor[INNER_YAW].current_set =
        fb_gimbal_PID_calc(&gimbal_control_loop->gimbal_yaw_motor[INNER_YAW].gimbal_motor_soft_gyro_pid,
                           gimbal_control_loop->gimbal_yaw_motor[INNER_YAW].motor_gyro,
                           gimbal_control_loop->gimbal_yaw_motor[INNER_YAW].motor_gyro_set);
    gimbal_control_loop->gimbal_yaw_motor[INNER_YAW].last_gyro = gimbal_control_loop->gimbal_yaw_motor[INNER_YAW].motor_gyro;
    gimbal_control_loop->gimbal_yaw_motor[INNER_YAW].given_current =
        current_limit_i16(gimbal_control_loop->gimbal_yaw_motor[INNER_YAW].current_set, GIMBAL_YAW_CURRENT_LIMIT);

    /************************************对小yaw尝试使用角加速度前馈**************************************************/
    //角加速度前馈在效果上类似于增大阻尼（用导数效果理解，让导数变化灵敏，修正原值更快，则原值变化可以更平稳），可以起到抑制超调的作用
    if (gimbal_control_loop->small_yaw_state == 1U) { //小幅度摆动才用前馈抵消，否则大角度转动小yaw可能反向一下，很不好
        gimbal_control_loop->gimbal_yaw_motor[OUTER_YAW].given_current =
            current_limit_i16((fp32)gimbal_control_loop->gimbal_yaw_motor[OUTER_YAW].given_current + ff_accel,
                              GIMBAL_YAW_CURRENT_LIMIT);
    }
    /***********************************************************************************************************/
}

void TwoYAW_gimbal_absolute_angle_control_big_yaw_first(gimbal_control_t *gimbal_control_loop)
{
    static const fp32 max_yaw_step= 0.5f;
    fp32 big_yaw_error = 0.0f;
    fp32 small_yaw_error = 0.0f;
    gimbal_motor_t *pSmallYaw = NULL;
    gimbal_motor_t *pBigYaw = NULL;
    fp32 big_yaw_target = 0.0f;

    if (gimbal_control_loop == NULL) return;

    // 1. 获取指针
    pSmallYaw = &gimbal_control_loop->gimbal_yaw_motor[OUTER_YAW];
    pBigYaw = &gimbal_control_loop->gimbal_yaw_motor[INNER_YAW];

    /**************************************计算偏差*****************************************/
    if (gimbal_behaviour == GIMBAL_AUTO && get_auto_last_mode() == SCAN_MODE) {
        PID_clear_for_gimbal(&pSmallYaw->gimbal_motor_absolute_angle_pid);
        PID_clear_for_gimbal(&pSmallYaw->gimbal_motor_gyro_pid);
        PID_clear_for_gimbal(&pBigYaw->gimbal_motor_absolute_angle_pid);
        PID_clear_for_gimbal(&pBigYaw->gimbal_motor_gyro_pid);
        PID_clear_for_gimbal(&pSmallYaw->gimbal_motor_soft_gyro_pid);
    }

    // --- 大Yaw逻辑 ---
    // 直接计算绝对误差，不需要考虑小Yaw在哪里
    // 注意：导航模式下，必须保证大Yaw的 absolute_angle 是有意义的(已校准)
    if (gimbal_behaviour == GIMBAL_AUTO)
    {
        if (get_force_flag() && get_mode() == OPEN_FRIC_MODE)
        {
            big_yaw_target = get_latest_vision_data()->enemy_center_yaw;
        }
        else
        {
                big_yaw_target = get_latest_vision_data()->big_yaw_angle;
        }

        big_yaw_error = rad_format(big_yaw_target - pBigYaw->absolute_angle);
        if (big_yaw_error > max_yaw_step) big_yaw_error = max_yaw_step;
        else if (big_yaw_error < -max_yaw_step) big_yaw_error = -max_yaw_step;
    }
    else
    {
        big_yaw_error = rad_format(pBigYaw->absolute_angle_set - pBigYaw->absolute_angle);
    }

    // --- 小Yaw逻辑 (从动/归中) ---
    // 目标永远是机械零点 (0.0f)
    // 误差 = 目标 - 当前相对角
    small_yaw_error = 0.0f - pSmallYaw->relative_angle;

    /*************************************************PID计算****************************************************/
    // --- 小Yaw PID ---
    // 任务：锁死在中位
    pSmallYaw->motor_gyro_set =
        gimbal_PID_calc_by_error(&pSmallYaw->gimbal_motor_absolute_angle_pid, small_yaw_error, pSmallYaw->motor_gyro);
    pSmallYaw->current_set = fb_gimbal_PID_calc(&pSmallYaw->gimbal_motor_soft_gyro_pid, pSmallYaw->motor_gyro, pSmallYaw->motor_gyro_set);
    pSmallYaw->last_gyro = pSmallYaw->motor_gyro;
    pSmallYaw->given_current = current_limit_i16(pSmallYaw->current_set, GIMBAL_YAW_CURRENT_LIMIT);

    // --- 大Yaw PID ---
    // 任务：执行导航角度
    pBigYaw->motor_gyro_set =
        gimbal_PID_calc_by_error(&pBigYaw->gimbal_motor_absolute_angle_pid, big_yaw_error, pBigYaw->motor_gyro);
    pBigYaw->current_set = fb_gimbal_PID_calc(&pBigYaw->gimbal_motor_soft_gyro_pid, pBigYaw->motor_gyro, pBigYaw->motor_gyro_set);
    pBigYaw->last_gyro = pBigYaw->motor_gyro;
    pBigYaw->given_current = current_limit_i16(pBigYaw->current_set, GIMBAL_YAW_CURRENT_LIMIT);
}

void Dual_Yaw_Scan_Mode(gimbal_control_t *gimbal_control_loop)
{
    fp32 big_yaw_set_speed = BIG_YAW_SCAN_SPD;
    fp32 big_yaw_spd = BIG_YAW_SCAN_SPD;
    fp32 big_yaw_target_spd = BIG_YAW_SCAN_SPD;
    gimbal_motor_t *pSmallYaw = NULL;
    gimbal_motor_t *pBigYaw = NULL;

    if (gimbal_control_loop == NULL) return;

    pSmallYaw = &gimbal_control_loop->gimbal_yaw_motor[OUTER_YAW];
    pBigYaw = &gimbal_control_loop->gimbal_yaw_motor[INNER_YAW];
    gimbal_control_loop->small_yaw_scan_ref = 0.0f;

    if (get_game_progress() == PROGRESS_BATTLE && get_stage_remain_time() > (60 * (7-3)) )
    {
        big_yaw_set_speed = BIG_YAW_SCAN_LOW_SPD;
    }
    else
    {
        big_yaw_set_speed = BIG_YAW_SCAN_SPD;
    }
    //big_yaw_set_speed = BIG_YAW_SCAN_LOW_SPD;
    // --------------------------------------------------------
    // 2. 小Yaw逻辑 (锁中位，给大Yaw让出扫描空间)
    // --------------------------------------------------------
    pSmallYaw->motor_gyro_set =
        gimbal_PID_calc_by_error(&pSmallYaw->gimbal_motor_absolute_angle_pid, 0.0f - pSmallYaw->relative_angle, pSmallYaw->motor_gyro);
    pSmallYaw->current_set = fb_gimbal_PID_calc(&pSmallYaw->gimbal_motor_soft_gyro_pid, pSmallYaw->motor_gyro, pSmallYaw->motor_gyro_set);

    // --------------------------------------------------------
    // 大Yaw逻辑 (以上位机目标角为中心 ±90° 范围扫描)
    // --------------------------------------------------------
    fp32 center = get_latest_vision_data()->big_yaw_angle;
    fp32 diff   = rad_format(center - pBigYaw->absolute_angle);

    // if (get_auto_mode() == NAVI_WITH_SCAN_MODE)
    // {
    //     if (diff > PI / 2.0f) {
    //         big_yaw_target_spd = big_yaw_set_speed;              // 超出右边界→向左转
    //     } else if (diff < -PI / 2.0f) {
    //         big_yaw_target_spd = -big_yaw_set_speed;             // 超出左边界→向右转
    //     } /* 在范围内则保持当前方向 */
    // }

    // 斜坡限速：平滑过渡到目标速度，避免急转向
    fp32 delta = big_yaw_target_spd - big_yaw_spd;
    fp32 max_step = BIG_YAW_RAMP_RATE * (fp32)GIMBAL_CONTROL_TIME * 0.001f;
    big_yaw_spd += fp32_constrain(delta, -max_step, max_step);

    pBigYaw->motor_gyro_set = big_yaw_spd;
    pBigYaw->current_set = fb_gimbal_PID_calc(&pBigYaw->gimbal_motor_gyro_pid, pBigYaw->motor_gyro, pBigYaw->motor_gyro_set);

    // --------------------------------------------------------
    // 输出赋值
    // --------------------------------------------------------
    pSmallYaw->last_gyro = pSmallYaw->motor_gyro;
    pBigYaw->last_gyro = pBigYaw->motor_gyro;
    pSmallYaw->given_current = current_limit_i16(pSmallYaw->current_set, GIMBAL_YAW_CURRENT_LIMIT);
    pBigYaw->given_current = current_limit_i16(pBigYaw->current_set, GIMBAL_YAW_CURRENT_LIMIT);
}

