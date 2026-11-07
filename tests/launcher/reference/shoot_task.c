#include "shoot/shoot_task.h"

#include "arm_math.h"
#include <math.h>
#include <string.h>

#include "gimbal/auto_shoot_task.h"
#include "detect.h"
#include "gimbal_behaviour.h"
#include "shoot/heat_control.h"
#include "referee/referee.h"
#include "usb_callback.h"

#define LARGE_ARMOR_WIDTH 0.23f
#define SMALL_ARMOR_WIDTH (0.14f * 0.8f)
#define YAW_COMP_L_BARREL 0.10f
#define YAW_COMP_K 1.0f
#define MIN_BULLET_SPEED 5.0f
#define Fric_STARTUP_TARGET_SPEED 21.0f

/*******下位机火控开关********/
#define  USE_LOWER_FIRE_CONTROL 0
/***************************/
static uint8_t shoot_initialized = 0U;
static uint8_t fire_check_flag = 0U;
static float shooter1_speed = 0.0f;
static float shooter2_speed = 0.0f;
static fp32 basic_tarspeed = Fric_STARTUP_TARGET_SPEED;

/** ── 弹速管理状态（裁判反馈低频，用 EMA 评估基准弹速） ── */
static fp32 spd_ema_speed = Fric_STARTUP_TARGET_SPEED; /**< 弹速 EMA 基准值 (m/s)。 */
static fp32 spd_last_fb = 0.0f;                        /**< 上一拍反馈弹速 (m/s)，用于新样本检测。 */
static uint32_t spd_last_adjust = 0U;                  /**< 上次慢速调节时间戳 (ms)。 */
static uint8_t spd_sample_fresh = 0U;                  /**< 自上次调节后是否有新反馈样本。 */

static void shoot_set_mode(shoot_control_t *shoot_mode_set);
static void shoot_auto_set_mode(shoot_control_t *shoot_mode_set);
static void shoot_auto_test_set_mode(shoot_control_t *shoot_mode_set);
static void shoot_feedback_update(shoot_control_t *shoot_update);
static void shoot_speed_control(shoot_control_t *shoot_control);
static void trigger_motor_turn_back(shoot_control_t *trigger_back);
static void shoot_bullet_control(shoot_control_t *bullet_control);
static void interrupt_shoot_control(shoot_control_t *shoot_control);
static uint8_t local_fire_control_check(void);
static void shoot_record_fire_command(shoot_control_t *shoot_control);

shoot_control_t shoot_control_L;
shoot_control_t shoot_control_R;

void Left_shoot_init(void)
{
    static const fp32 Trigger_speed_pid[3] = {TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI, TRIGGER_ANGLE_PID_KD};
    static const fp32 Fric_speed_pid[3] = {SHOOT_SPEED_PID_KP, SHOOT_SPEED_PID_KI, SHOOT_SPEED_PID_KD};
    static const fp32 filter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};
    memset(&shoot_control_L, 0, sizeof(shoot_control_L));
    shoot_control_L.shoot_mode = SHOOT_STOP;
    shoot_control_L.trigger_motor_measure = get_L_trigger_motor_measure_point();
    shoot_control_L.fric_motor_measure[0] = get_shoot_LA_data_point1();
    shoot_control_L.fric_motor_measure[1] = get_shoot_LB_data_point2();
    shoot_control_L.shoot_id = LEFT_SHOOT;
    PID_init(&shoot_control_L.trigger_motor_pid, PID_POSITION, Trigger_speed_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);
    PID_init(&shoot_control_L.FricSpeed_PID[0], PID_POSITION, Fric_speed_pid, SHOOT_SPEED_PID_MAX_OUT, SHOOT_SPEED_PID_MAX_IOUT);
    PID_init(&shoot_control_L.FricSpeed_PID[1], PID_POSITION, Fric_speed_pid, SHOOT_SPEED_PID_MAX_OUT, SHOOT_SPEED_PID_MAX_IOUT);
    second_order_filter_init(&shoot_control_L.trigger_speed_filter, filter_num);
    shoot_control_L.LimitSwitchPort = GPIOE;
    shoot_control_L.LimitSwitchPin = GPIO_PIN_13;
    shoot_control_L.ReadLimitSwitch = (GPIO_PinState (*)(GPIO_TypeDef *, uint16_t))HAL_GPIO_ReadPin;
    shoot_control_L.interrupt_flag = FIRE_PREPARING;
}

void Right_shoot_init(void)
{
    static const fp32 Trigger_speed_pid[3] = {TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI, TRIGGER_ANGLE_PID_KD};
    static const fp32 Fric_speed_pid[3] = {SHOOT_SPEED_PID_KP, SHOOT_SPEED_PID_KI, SHOOT_SPEED_PID_KD};
    static const fp32 filter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};
    memset(&shoot_control_R, 0, sizeof(shoot_control_R));
    shoot_control_R.shoot_mode = SHOOT_STOP;
    shoot_control_R.trigger_motor_measure = get_R_trigger_motor_measure_point();
    shoot_control_R.fric_motor_measure[0] = get_shoot_RA_data_point3();
    shoot_control_R.fric_motor_measure[1] = get_shoot_RB_data_point4();
    shoot_control_R.shoot_id = RIGHT_SHOOT;
    PID_init(&shoot_control_R.trigger_motor_pid, PID_POSITION, Trigger_speed_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);
    PID_init(&shoot_control_R.FricSpeed_PID[0], PID_POSITION, Fric_speed_pid, SHOOT_SPEED_PID_MAX_OUT, SHOOT_SPEED_PID_MAX_IOUT);
    PID_init(&shoot_control_R.FricSpeed_PID[1], PID_POSITION, Fric_speed_pid, SHOOT_SPEED_PID_MAX_OUT, SHOOT_SPEED_PID_MAX_IOUT);
    second_order_filter_init(&shoot_control_R.trigger_speed_filter, filter_num);
    shoot_control_R.LimitSwitchPort = GPIOE;
    shoot_control_R.LimitSwitchPin = GPIO_PIN_14;
    shoot_control_R.ReadLimitSwitch = (GPIO_PinState (*)(GPIO_TypeDef *, uint16_t))HAL_GPIO_ReadPin;
    shoot_control_R.interrupt_flag = FIRE_PREPARING;
}

void basic_shoot_init(shoot_control_t *basic_init)
{
    if (basic_init == NULL) return;
    basic_init->shoot_rc = get_remote_control_point();
    basic_init->Trigger_current = 0;
    basic_init->Trigger_FBspeed_filtered = 0.0f;
    basic_init->Trigger_Actualspeed = 0.0f;
    basic_init->Fric_FBspeed_A = 0.0f;
    basic_init->Fric_FBspeed_B = 0.0f;
    basic_init->Fric_current_A = 0;
    basic_init->Fric_current_B = 0;
    basic_init->fric_flag = 0U;
    get_shoot_speed_limit(&basic_init->last_shoot_speed_limit, basic_init->shoot_id);
    get_shoot_speed_limit(&basic_init->shoot_speed_limit, basic_init->shoot_id);
}

void shoot_init(void)
{
    if (shoot_initialized != 0U) return;
    Left_shoot_init();
    Right_shoot_init();
    basic_shoot_init(&shoot_control_L);
    basic_shoot_init(&shoot_control_R);
    shoot_feedback_update(&shoot_control_L);
    shoot_feedback_update(&shoot_control_R);
    shoot_initialized = 1U;
}

void shoot_control_loop(shoot_control_t *shoot_control)
{
    shoot_mode_e last_shoot_mode;
    float trigger_out = 0.0f;
    if (shoot_control == NULL) return;
    shoot_feedback_update(shoot_control);
    last_shoot_mode = shoot_control->shoot_mode;
    if (gimbal_behaviour == GIMBAL_AUTO) shoot_auto_set_mode(shoot_control);
    else if (gimbal_behaviour == GIMBAL_AUTO_TEST) shoot_auto_test_set_mode(shoot_control);
    else shoot_set_mode(shoot_control);
    if (last_shoot_mode != SHOOT_STOP && shoot_control->shoot_mode == SHOOT_STOP) {
        shoot_control->FricSpeed_PID[0].PID_Calc_Clear(&shoot_control->FricSpeed_PID[0]);
        shoot_control->FricSpeed_PID[1].PID_Calc_Clear(&shoot_control->FricSpeed_PID[1]);
    }
    if (shoot_control->fric_motor_measure[0] && shoot_control->fric_motor_measure[0]->motor_fric_measure &&
        shoot_control->fric_motor_measure[1] && shoot_control->fric_motor_measure[1]->motor_fric_measure) {
        Shoot_Virtual_Heat_Update(
            (float)shoot_control->fric_motor_measure[0]->motor_fric_measure->given_current / (16384.0f / 20.0f),
            (float)shoot_control->fric_motor_measure[1]->motor_fric_measure->given_current / (16384.0f / 20.0f),
            shoot_control->Fric_FBspeed_A,
            (shoot_control->shoot_mode == SHOOT_STOP) ? 0.0f : shoot_control->Fric_Targetspeed);
    }
    if (shoot_control->shoot_mode == SHOOT_STOP) {
        shoot_control->Trigger_Actualspeed = 0.0f;
        shoot_control->Fric_Targetspeed = 0.0f;
        shoot_control->Trigger_Targetspeed = 0.0f;
    } else if (shoot_control->shoot_mode == SHOOT_READY_FRIC || shoot_control->shoot_mode == SHOOT_READY_BULLET || shoot_control->shoot_mode == SHOOT_READY) {
        shoot_control->Trigger_Actualspeed = 0.0f;
        shoot_control->Trigger_Targetspeed = 0.0f;
    } else if (shoot_control->shoot_mode == SHOOT_BULLET) {
        shoot_bullet_control(shoot_control);
    } else if (shoot_control->shoot_mode == SHOOT_CONTINUE_BULLET) {
        // shoot_control->Trigger_Targetspeed = -heat_control();
        // trigger_motor_turn_back(shoot_control);
        shoot_bullet_control(shoot_control);
    }
    if ((last_shoot_mode == SHOOT_BULLET || last_shoot_mode == SHOOT_CONTINUE_BULLET) &&
        (shoot_control->shoot_mode == SHOOT_READY_BULLET || shoot_control->shoot_mode == SHOOT_READY)) {
        shoot_control->trigger_motor_pid.PID_Calc_Clear(&shoot_control->trigger_motor_pid);
    }
    if (shoot_control->shoot_mode == SHOOT_STOP) {
        shoot_control->Trigger_current = 0;
        if (fabsf(shoot_control->Fric_FBspeed_A) < 0.1f && fabsf(shoot_control->Fric_FBspeed_B) < 0.1f) {
            shoot_control->Fric_current_A = 0;
            shoot_control->Fric_current_B = 0;
        } else if (shoot_control->shoot_id == LEFT_SHOOT) {
            //3508电机+c620电调通过int值电流控制，不用考虑这里的浮点位数舍去，影响很小
            shoot_control->Fric_current_A = PID_calc(&shoot_control->FricSpeed_PID[0], shoot_control->Fric_FBspeed_A, 0.0f);
            shoot_control->Fric_current_B = -PID_calc(&shoot_control->FricSpeed_PID[1], shoot_control->Fric_FBspeed_B, 0.0f);
        } else {
            shoot_control->Fric_current_A = PID_calc(&shoot_control->FricSpeed_PID[0], shoot_control->Fric_FBspeed_A, 0.0f);
            shoot_control->Fric_current_B = -PID_calc(&shoot_control->FricSpeed_PID[1], shoot_control->Fric_FBspeed_B, 0.0f);
        }
    } else {
        trigger_out = PID_calc(&shoot_control->trigger_motor_pid, shoot_control->Trigger_FBspeed_filtered, shoot_control->Trigger_Actualspeed);
        if (trigger_out < -16384.0f) trigger_out = -16384.0f;
        if (trigger_out > 16384.0f) trigger_out = 16384.0f;
        shoot_control->Trigger_current = -(int16_t)trigger_out;
        if (shoot_control->shoot_id == LEFT_SHOOT) {
            shoot_control->Fric_current_A = PID_calc(&shoot_control->FricSpeed_PID[0], shoot_control->Fric_FBspeed_A, shoot_control->Fric_Targetspeed);
            shoot_control->Fric_current_B = -PID_calc(&shoot_control->FricSpeed_PID[1], shoot_control->Fric_FBspeed_B, shoot_control->Fric_Targetspeed);
        } else {
            shoot_control->Fric_current_A = PID_calc(&shoot_control->FricSpeed_PID[0], shoot_control->Fric_FBspeed_A, -shoot_control->Fric_Targetspeed);
            shoot_control->Fric_current_B = -PID_calc(&shoot_control->FricSpeed_PID[1], shoot_control->Fric_FBspeed_B, -shoot_control->Fric_Targetspeed);
        }
    }
    //interrupt_shoot_control(shoot_control);
}

static void shoot_set_mode(shoot_control_t *shoot_mode_set)
{
    shoot_mode_e last_mode = SHOOT_STOP;
    if (shoot_mode_set == NULL || shoot_mode_set->shoot_rc == NULL) return;
    last_mode = shoot_mode_set->shoot_mode;
    if ((switch_is_up(shoot_mode_set->shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_up(shoot_mode_set->last_s) && shoot_mode_set->shoot_mode == SHOOT_STOP))
    {
        shoot_mode_set->shoot_mode = SHOOT_READY_FRIC;
    }
    else if ((switch_is_up(shoot_mode_set->shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_up(shoot_mode_set->last_s) && shoot_mode_set->shoot_mode != SHOOT_STOP))
    {
        shoot_mode_set->shoot_mode = SHOOT_STOP;
    }
#if USE_I6X_RECEIVER
    if (switch_is_down(shoot_mode_set->shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && switch_is_up(shoot_mode_set->shoot_rc->rc.s[PC_CONTROL_SHOOT_CHANNEL]) && (shoot_mode_set->shoot_rc->key.v & SHOOT_ON_KEYBOARD) && shoot_mode_set->shoot_mode == SHOOT_STOP)
#else
    if (switch_is_mid(shoot_mode_set->shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && (shoot_mode_set->shoot_rc->key.v & SHOOT_ON_KEYBOARD) && shoot_mode_set->shoot_mode == SHOOT_STOP)
#endif
        shoot_mode_set->shoot_mode = SHOOT_READY_FRIC;
#if USE_I6X_RECEIVER
    else if (switch_is_down(shoot_mode_set->shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && switch_is_up(shoot_mode_set->shoot_rc->rc.s[PC_CONTROL_SHOOT_CHANNEL]) && (shoot_mode_set->shoot_rc->key.v & SHOOT_OFF_KEYBOARD) && shoot_mode_set->shoot_mode != SHOOT_STOP)
#else
    else if (switch_is_mid(shoot_mode_set->shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && (shoot_mode_set->shoot_rc->key.v & SHOOT_OFF_KEYBOARD) && shoot_mode_set->shoot_mode != SHOOT_STOP)
#endif
        shoot_mode_set->shoot_mode = SHOOT_STOP;
    if (shoot_mode_set->shoot_mode == SHOOT_READY_FRIC && fabsf(shoot_mode_set->Fric_FBspeed_A - shoot_mode_set->Fric_Targetspeed) < 0.3f && fabsf(shoot_mode_set->Fric_FBspeed_B - shoot_mode_set->Fric_Targetspeed) < 0.3f)
    {
        shoot_mode_set->shoot_mode = SHOOT_READY_BULLET;
    }
    else if (shoot_mode_set->shoot_mode == SHOOT_READY_BULLET)
    {
        shoot_mode_set->shoot_mode = SHOOT_READY;
    }
#if USE_I6X_RECEIVER
    else if (shoot_mode_set->shoot_mode == SHOOT_READY && (switch_is_up(shoot_mode_set->shoot_rc->rc.s[SHOOT_CONTROL_CHANNEL]) || shoot_mode_set->press_l) )
#else
    else if (shoot_mode_set->shoot_mode == SHOOT_READY && shoot_mode_set->press_l)
#endif
        shoot_mode_set->shoot_mode = SHOOT_BULLET;

    if (last_mode != SHOOT_BULLET && shoot_mode_set->shoot_mode == SHOOT_BULLET) {
        shoot_record_fire_command(shoot_mode_set);
    }

    if (shoot_mode_set->shoot_mode == SHOOT_BULLET && switch_is_down(shoot_mode_set->shoot_rc->rc.s[SHOOT_CONTROL_CHANNEL]))
    {
        shoot_mode_set ->shoot_mode =SHOOT_READY;
    }

    if (shoot_mode_set->shoot_mode == SHOOT_CONTINUE_BULLET) shoot_mode_set->shoot_mode = SHOOT_READY_BULLET;
    get_shoot_heat_limit_and_heat(&shoot_mode_set->heat_limit, &shoot_mode_set->heat, shoot_mode_set->shoot_id);
    shoot_mode_set->heat = get_shoot_heat(shoot_mode_set->shoot_id);
    if (gimbal_cmd_to_shoot_stop()) shoot_mode_set->shoot_mode = SHOOT_STOP;
    shoot_mode_set->fric_flag = (shoot_mode_set->shoot_mode >= SHOOT_READY_FRIC) ? 1U : 0U;
    shoot_mode_set->last_s = shoot_mode_set->shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL];
}

static void shoot_auto_set_mode(shoot_control_t *shoot_mode_set)
{
    if (shoot_mode_set == NULL || shoot_mode_set->shoot_rc == NULL) return;
    if (get_auto_mode() == OPEN_FRIC_MODE && shoot_mode_set->open_fric == 0.0f) {
        shoot_mode_set->shoot_mode = SHOOT_READY_FRIC;
        shoot_mode_set->open_fric = 1.0f;
        return;
    } else if (get_auto_mode() != OPEN_FRIC_MODE) {
        shoot_mode_set->shoot_mode = SHOOT_STOP;
        shoot_mode_set->open_fric = 0.0f;
    }

    if (shoot_mode_set->shoot_mode == SHOOT_READY_FRIC &&
        fabsf(shoot_mode_set->Fric_Targetspeed) - fabsf(shoot_mode_set->Fric_FBspeed_A) < 0.1f &&
        fabsf(shoot_mode_set->Fric_Targetspeed) - fabsf(shoot_mode_set->Fric_FBspeed_B) < 0.1f &&
        shoot_mode_set->Fric_Targetspeed != 0.0f) {
        shoot_mode_set->shoot_mode = SHOOT_READY_BULLET;
    } else if (shoot_mode_set->shoot_mode == SHOOT_READY_BULLET) {
        shoot_mode_set->shoot_mode = SHOOT_READY;
    }
    #if USE_LOWER_FIRE_CONTROL
    else if (shoot_mode_set->shoot_mode == SHOOT_READY && local_fire_control_check())
    #else
    else if (shoot_mode_set->shoot_mode == SHOOT_READY && get_auto_fire_control())
    #endif
    {
        shoot_mode_set->shoot_mode = SHOOT_BULLET;
    } else if (shoot_mode_set->shoot_mode == SHOOT_BULLET) {
        shoot_mode_set->shoot_mode = SHOOT_CONTINUE_BULLET;
    } else if (shoot_mode_set->shoot_mode == SHOOT_CONTINUE_BULLET) {
        #if USE_LOWER_FIRE_CONTROL
        if (!local_fire_control_check())
        #else
        if (!get_auto_fire_control())
        #endif
        {
            shoot_mode_set->shoot_mode = SHOOT_READY_BULLET;
        }
        else {
            shoot_mode_set->shoot_mode = SHOOT_CONTINUE_BULLET;
        }
    }

    get_shoot_heat_limit_and_heat(&shoot_mode_set->heat_limit, &shoot_mode_set->heat, shoot_mode_set->shoot_id);
    shoot_mode_set->heat = get_shoot_heat(shoot_mode_set->shoot_id);
    shoot_mode_set->fric_flag = (shoot_mode_set->shoot_mode >= SHOOT_READY_FRIC) ? 1U : 0U;
    shoot_mode_set->last_s = shoot_mode_set->shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL];
}

static void shoot_auto_test_set_mode(shoot_control_t *shoot_mode_set)
{
    if (shoot_mode_set == NULL || shoot_mode_set->shoot_rc == NULL) return;

    // Legacy AUTO_TEST opened friction on DETECT->TRACK and closed it on TRACK/CONFIRM->DETECT.
    if (gimbal_control.auto_state_machine == TRACK_STATUS &&
        gimbal_control.last_auto_state_machine == DETECT_STATUS &&
        shoot_mode_set->shoot_mode == SHOOT_STOP) {
        shoot_mode_set->shoot_mode = SHOOT_READY_FRIC;
    } else if (gimbal_control.auto_state_machine == DETECT_STATUS &&
               gimbal_control.last_auto_state_machine != DETECT_STATUS &&
               shoot_mode_set->shoot_mode != SHOOT_STOP) {
        shoot_mode_set->shoot_mode = SHOOT_STOP;
    }

    if (shoot_mode_set->shoot_mode == SHOOT_READY_FRIC &&
        fabsf(shoot_mode_set->Fric_Targetspeed) - fabsf(shoot_mode_set->Fric_FBspeed_A) < 0.1f &&
        fabsf(shoot_mode_set->Fric_Targetspeed) - fabsf(shoot_mode_set->Fric_FBspeed_B) < 0.1f &&
        shoot_mode_set->Fric_Targetspeed != 0.0f) {
        shoot_mode_set->shoot_mode = SHOOT_READY_BULLET;
    } else if (shoot_mode_set->shoot_mode == SHOOT_READY_BULLET && Get_Shoot_Brief() == 1) {
        shoot_mode_set->shoot_mode = SHOOT_READY;
    } else if (shoot_mode_set->shoot_mode == SHOOT_READY) {
        if (Get_Shoot_Brief() == 0) {
            shoot_mode_set->shoot_mode = SHOOT_READY_BULLET;
        } else {
            shoot_mode_set->shoot_mode = SHOOT_READY;
        }
    }

    get_shoot_heat_limit_and_heat(&shoot_mode_set->heat_limit, &shoot_mode_set->heat, shoot_mode_set->shoot_id);
    shoot_mode_set->heat = get_shoot_heat(shoot_mode_set->shoot_id);
    shoot_mode_set->fric_flag = (shoot_mode_set->shoot_mode >= SHOOT_READY_FRIC) ? 1U : 0U;
    shoot_mode_set->last_s = shoot_mode_set->shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL];
}

static void shoot_feedback_update(shoot_control_t *shoot_update)
{
    if (shoot_update == NULL || shoot_update->shoot_rc == NULL) return;
    if (shoot_update->shoot_mode < SHOOT_BULLET) {
        shoot_update->fire_delay_pending = 0U;
    }
    if (shoot_update->trigger_motor_measure && shoot_update->trigger_motor_measure->motor_trigger_measure)
        second_order_filter_cali(&shoot_update->trigger_speed_filter, shoot_update->trigger_motor_measure->motor_trigger_measure->speed_rpm * -MOTOR_RPM_TO_SPEED);
    shoot_update->Trigger_FBspeed_filtered = shoot_update->trigger_speed_filter.Output[0];
    shoot_update->Fric_FBspeed_A = (shoot_update->fric_motor_measure[0] && shoot_update->fric_motor_measure[0]->motor_fric_measure) ? shoot_update->fric_motor_measure[0]->motor_fric_measure->speed_rpm * FRICTION_ALL_COEFFICIENT : 0.0f;
    shoot_update->Fric_FBspeed_B = (shoot_update->fric_motor_measure[1] && shoot_update->fric_motor_measure[1]->motor_fric_measure) ? -(shoot_update->fric_motor_measure[1]->motor_fric_measure->speed_rpm) * FRICTION_ALL_COEFFICIENT : 0.0f;
    shoot_update->LimitState = shoot_update->ReadLimitSwitch ? shoot_update->ReadLimitSwitch(shoot_update->LimitSwitchPort, shoot_update->LimitSwitchPin) : SWITCH_TRIGGER_OFF;
    shoot_update->last_press_l = shoot_update->press_l;
    shoot_update->press_l = shoot_update->shoot_rc->mouse.press_l;
    if (shoot_update->press_l) { if (shoot_update->press_l_time < PRESS_LONG_TIME) shoot_update->press_l_time++; } else shoot_update->press_l_time = 0;
    if (shoot_update->shoot_mode != SHOOT_STOP && switch_is_down(shoot_update->shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL])) { if (shoot_update->rc_s_time < RC_S_LONG_TIME) shoot_update->rc_s_time++; } else shoot_update->rc_s_time = 0;
    get_shoot_speed_limit(&shoot_update->shoot_speed_limit, shoot_update->shoot_id);

        shoot_speed_control(shoot_update);               /* 先跑弹速管理，更新 basic_tarspeed */
        shoot_update->Fric_Targetspeed = basic_tarspeed; /* 取最新设定量级 */
        shoot_update->Fric_Targetspeed *= -1; //反一下实际摩擦轮速度
    if (shoot_update->shoot_id == LEFT_SHOOT) get_shooter_now_speed(&shooter1_speed, 1);
    else if (shoot_update->shoot_id == RIGHT_SHOOT) get_shooter_now_speed(&shooter2_speed, 2);
    if (shoot_update->shoot_speed_limit != shoot_update->last_shoot_speed_limit && shoot_update->shoot_mode >= SHOOT_READY_FRIC) shoot_update->shoot_mode = SHOOT_READY_FRIC;
    shoot_update->last_shoot_speed_limit = shoot_update->shoot_speed_limit;
    if (shoot_update->fric_motor_measure[0] && shoot_update->fric_motor_measure[0]->motor_fric_measure &&
        shoot_update->fric_motor_measure[1] && shoot_update->fric_motor_measure[1]->motor_fric_measure) {
        shoot_update->shooter_speed =
            (shoot_update->fric_motor_measure[0]->motor_fric_measure->speed_rpm +
             shoot_update->fric_motor_measure[1]->motor_fric_measure->speed_rpm) * 0.5f;
    } else {
        shoot_update->shooter_speed = 0.0f;
    }
}

static void trigger_motor_turn_back(shoot_control_t *trigger_back)
{
    if (trigger_back == NULL || trigger_back->shoot_rc == NULL) return;
    if (gimbal_behaviour != GIMBAL_AUTO && gimbal_behaviour != GIMBAL_AUTO_TEST) {
        if (trigger_back->shoot_mode >= SHOOT_BULLET && !switch_is_down(trigger_back->shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && trigger_back->press_l == 0U) {
            trigger_back->shoot_mode = SHOOT_READY_BULLET;
            return;
        }
    }
    if (fabsf(trigger_back->Trigger_Targetspeed) <= 0.0f) {
        trigger_back->Trigger_Actualspeed = trigger_back->Trigger_Targetspeed;
        trigger_back->block_time = 0U;
        trigger_back->reverse_time = 0U;
        return;
    }
    if (trigger_back->block_time < BLOCK_TIME) trigger_back->Trigger_Actualspeed = trigger_back->Trigger_Targetspeed;
    else trigger_back->Trigger_Actualspeed = -trigger_back->Trigger_Targetspeed;
    if (fabsf(trigger_back->Trigger_FBspeed_filtered) < BLOCK_TRIGGER_SPEED && trigger_back->block_time < BLOCK_TIME) {
        trigger_back->block_time++;
        trigger_back->reverse_time = 0U;
    } else if (trigger_back->block_time == BLOCK_TIME && trigger_back->reverse_time < REVERSE_TIME) {
        trigger_back->reverse_time++;
    } else {
        trigger_back->block_time = 0U;
    }
}

static void shoot_bullet_control(shoot_control_t *bullet_control)
{
    if (bullet_control == NULL) return;
    if (gimbal_behaviour != GIMBAL_AUTO && gimbal_behaviour != GIMBAL_AUTO_TEST) {
        if (bullet_control->shoot_mode == SHOOT_BULLET && !switch_is_down(bullet_control->shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && bullet_control->press_l == 0U) {
            bullet_control->shoot_mode = SHOOT_READY_BULLET;
            return;
        }
    }
    {
        float heat = Get_Virtual_Heat();
        float heat_limit = Get_Virtual_Heat_Limit();
        if (heat_limit > SHOOT_HEAT_STOP_OFFSET
            && heat >= heat_limit - SHOOT_HEAT_STOP_OFFSET) {
            bullet_control->Trigger_Targetspeed = 0.0f;
        }
        // else if (heat >=heat_limit - SHOOT_HEAT_SOFT) {
        //     bullet_control->Trigger_Targetspeed = -10.0f;
        // }
        else
        {
            bullet_control->Trigger_Targetspeed = -20.0f;
        }

    }
   // bullet_control->Trigger_Actualspeed = bullet_control->Trigger_Targetspeed;
    trigger_motor_turn_back(bullet_control);
}

static void interrupt_shoot_control(shoot_control_t *shoot_control)
{
    if (shoot_control == NULL) return;
    if (shoot_control->interrupt_flag == FIRE_PREPARING && shoot_control->shoot_mode == SHOOT_READY_BULLET) {
        shoot_control->interrupt_flag = FIRE_READY;
    } else if (shoot_control->interrupt_flag == FIRE_END && HAL_GetTick() - shoot_control->interrupt_tick >= 5U) {
        shoot_control->interrupt_flag = FIRE_PREPARING;
    }
}

static void shoot_record_fire_command(shoot_control_t *shoot_control)
{
    if (shoot_control == NULL) return;
    shoot_control->interrupt_start_tick = (uint16_t)HAL_GetTick();
    shoot_control->interrupt_delta_tick = 0U;
    shoot_control->fire_delay_pending = 1U;
}

// static uint8_t local_fire_control_check(void)
// {
//     const VisionToGimbal *vision = get_latest_vision_data();
//     float distance = 0.0f;
//     float current_yaw = 0.0f;
//     float bullet_speed = 0.0f;
//     float yaw_comp = 0.0f;
//     float yaw_err_arc = 0.0f;
//     float half_width = 0.0f;
//     float pitch_angle = 0.0f;
//     uint8_t pitch_valid = 0;
//
//
//     if (vision == NULL) {
//         fire_check_flag = 0U;
//         return 0U;
//     }
//
//     if (get_interrupt_flag() == FIRE_END) {
//         fire_check_flag = 0U;
//         return 0U;
//     }
//
//     if (get_auto_fire_control() != FIRE_MODE) {
//         fire_check_flag = 0U;
//         return 0U;
//     }
//
//     distance = vision->distance;
//     if (distance < 0.1f) {
//         fire_check_flag = 0U;
//         return 0U;
//     }
//
//     bullet_speed = get_bullet_speed();
//     if (bullet_speed < MIN_BULLET_SPEED) {
//         bullet_speed = MIN_BULLET_SPEED;
//     }
//
//     if (gimbal_control.gimbal_INT_angle_point_DM != NULL) {
//         current_yaw = rad_format(gimbal_control.gimbal_INT_angle_point_DM->yaw * ANGLE_TO_RAD);
//         yaw_comp = atan2f(gimbal_control.gimbal_yaw_motor[OUTER_YAW].motor_gyro * YAW_COMP_L_BARREL, bullet_speed) * YAW_COMP_K;
//     }
//
//     (void)yaw_comp;
//     yaw_err_arc = fabsf(rad_format(vision->target_yaw - current_yaw)) * distance;
//     half_width = (vision->id == 1U) ? (LARGE_ARMOR_WIDTH * 0.5f) : (SMALL_ARMOR_WIDTH * 0.5f);
//     pitch_angle = gimbal_control.gimbal_pitch_motor.absolute_angle;
//     fire_check_flag = (uint8_t)(yaw_err_arc < half_width * fabsf(arm_cos_f32(vision->armor_yaw))) ;
//     return fire_check_flag;
// }

void CAN_cmd_shoot(int16_t left_a, int16_t left_b, int16_t right_a, int16_t right_b)
{
    uint8_t data[8] = {(uint8_t)(left_a >> 8), (uint8_t)left_a, (uint8_t)(left_b >> 8), (uint8_t)left_b, (uint8_t)(right_a >> 8), (uint8_t)right_a, (uint8_t)(right_b >> 8), (uint8_t)right_b};
#if USE_FDCAN
    FDCAN_TxHeaderTypeDef tx = {.Identifier = CAN_SHOOT_ALL_ID, .IdType = FDCAN_STANDARD_ID, .TxFrameType = FDCAN_DATA_FRAME, .DataLength = FDCAN_DLC_BYTES_8, .FDFormat = FDCAN_CLASSIC_CAN, .BitRateSwitch = FDCAN_BRS_OFF, .TxEventFifoControl = FDCAN_NO_TX_EVENTS, .ErrorStateIndicator = FDCAN_ESI_ACTIVE, .MessageMarker = 0};
    if (HAL_FDCAN_GetTxFifoFreeLevel(BSP_CAN_SHOOT_HANDLE) > 0U) HAL_FDCAN_AddMessageToTxFifoQ(BSP_CAN_SHOOT_HANDLE, &tx, data);
#else
    CAN_TxHeaderTypeDef tx = {.StdId = CAN_SHOOT_ALL_ID, .IDE = CAN_ID_STD, .RTR = CAN_RTR_DATA, .DLC = 8};
    uint32_t mailbox = 0U;
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) > 0U) HAL_CAN_AddTxMessage(&hcan2, &tx, data, &mailbox);
#endif
}

uint8_t *get_fric_flag(void) { return &shoot_control_L.fric_flag; }
void get_shoot_speed_limit(uint16_t *speed_limit, shoot_id_e id)
{
    if (speed_limit == NULL) return;
    if (id == LEFT_SHOOT) get_shooter_id1_speed_limit(speed_limit);
    else if (id == RIGHT_SHOOT) get_shooter_id2_speed_limit(speed_limit);
}

void get_shoot_heat_limit_and_heat(uint16_t *heat_limit, uint16_t *heat, shoot_id_e id)
{
    if (id == LEFT_SHOOT) get_shoot_heat1_limit_and_heat1(heat_limit, heat);
    else if (id == RIGHT_SHOOT) get_shoot_heat2_limit_and_heat2(heat_limit, heat);
}

uint16_t get_shoot_heat(shoot_id_e id)
{
    if (id == LEFT_SHOOT) return get_shoot_heat1();
    if (id == RIGHT_SHOOT) return get_shoot_heat2();
    return 0U;
}
static void shoot_speed_control(shoot_control_t *shoot_control)
{
    fp32 fb_speed = 0.0f;
    fp32 limit = 0.0f;
    fp32 err = 0.0f;
    uint32_t now = 0U;

    if (shoot_control == NULL) return;

    get_shooter_now_speed(&fb_speed, 1);

    /* ── 新样本检测 + 低通滤波 更新基准弹速 ── */
    if (fb_speed != spd_last_fb && fb_speed > SPEED_MIN_VALID_FB) {
        spd_ema_speed += SPEED_LPF_ALPHA * (fb_speed - spd_ema_speed);//等价LPF
        spd_last_fb = fb_speed;
        spd_sample_fresh = 1U;
    }

    /* 读取本侧弹速限制，裁判数据未就绪时兜底为默认限制。 */
    limit = (fp32)shoot_control->shoot_speed_limit;
    if (limit < 1.0f) limit = 25.0f;

    now = HAL_GetTick();

    /* ── 最新弹速即将超限，立刻下调设定值，不受时间锁限制 ── */
    if (spd_sample_fresh && fb_speed >= (limit - SPEED_URGENT_MARGIN)) {
        basic_tarspeed -= SPEED_URGENT_STEP;
        /* 双重保险：设定量级不允许超过安全上限，避免步进单次降得不够。 */
        if (basic_tarspeed > (limit - SPEED_URGENT_MARGIN)) {
            basic_tarspeed = limit - SPEED_URGENT_STEP;
        }
        if (basic_tarspeed < SPEED_SETPOINT_MIN) {
            basic_tarspeed = SPEED_SETPOINT_MIN;
        }
        spd_last_adjust = now;   /* 重置时间锁，防止慢速通道立刻反向调节 */
        spd_sample_fresh = 0U;
        return;
    }

    /* ── 慢速通道：时间锁 + 死区，缓慢把基准弹速拉回参考目标 ── */
    if (spd_sample_fresh && ((now - spd_last_adjust) >= SPEED_ADJUST_INTERVAL_MS)) {
        err = SHOOT_REFERENCE_SPEED - spd_ema_speed;
        if (err > SPEED_DEADBAND) {
            basic_tarspeed += SPEED_ADJUST_STEP;
        } else if (err < -SPEED_DEADBAND) {
            basic_tarspeed -= SPEED_ADJUST_STEP;
        }
        if (basic_tarspeed > SPEED_SETPOINT_MAX) basic_tarspeed = SPEED_SETPOINT_MAX;
        if (basic_tarspeed < SPEED_SETPOINT_MIN) basic_tarspeed = SPEED_SETPOINT_MIN;

        if (err > SPEED_DEADBAND || err < -SPEED_DEADBAND) {
            spd_last_adjust = now;   /* 仅在真正调节时刷新时间锁 */
        }
        spd_sample_fresh = 0U;
    }
}
void set_interrupt_flag_end_L(void)
{
    if (shoot_control_L.interrupt_flag == FIRE_READY) {
        shoot_control_L.interrupt_flag = FIRE_END;
        shoot_control_L.interrupt_tick = (uint16_t)HAL_GetTick();
    }
}

fp32 delta_tick=0;

void record_fire_detect_L(void)
{
    if (shoot_control_L.fire_delay_pending == 0U) {
        return;
    }
    shoot_control_L.interrupt_delta_tick = (uint16_t)((uint16_t)HAL_GetTick() - shoot_control_L.interrupt_start_tick);
    delta_tick = shoot_control_L.interrupt_delta_tick;
    shoot_control_L.fire_delay_pending = 0U;
}

fire_state_e get_interrupt_flag(void) { return shoot_control_L.interrupt_flag; }
uint8_t get_fire_check_flag(void) { return fire_check_flag; }
int16_t get_shoot_trigger_current(void) { return shoot_control_L.Trigger_current; }
