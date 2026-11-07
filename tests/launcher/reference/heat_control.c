#include "shoot/heat_control.h"

#include <math.h>

#include "referee/referee.h"
#include "shoot/shoot_task.h"

// 热量控制分两层：
// 1. 用摩擦轮电流构造“虚拟热量”，在本地快速估计每次开火的热量增长；
// 2. 再用裁判系统热量做低频校正，避免长期漂移。
#define SIM_HEAT_MAX 260.0f
#define SIM_COOLING_RATE 30.0f
#define SIM_HEAT_PER_SHOT 10.0f

#define FRIC_CURRENT_THRESHOLD 3.0f
#define SHOOT_CONFIRM_TIME_MS 10U
#define CONTROL_PERIOD_MS 1U
#define SHOOT_MIN_INTERVAL_MS 10U

#define REFEREE_SETTLE_PERIOD_S 0.1f
#define HEAT_CALC_DIVIDER 10U
#define DEFAULT_BURST_N 20U
#define HEAT_STOP_MARGIN 20.0f
#define MAX_MECH_FREQ_HZ 30.0f
#define RPM_PER_HZ 0.78539f
#define HEAT_SYNC_PERIOD_MS 200U
#define HEAT_SYNC_GAIN 1.0f
#define HEAT_SYNC_MAX_STEP 20.0f

typedef enum {
    HEAT_FSM_IDLE = 0,   // 摩擦轮未就绪，不做击发判定
    HEAT_FSM_READY,      // 摩擦轮已稳定，等待一次“疑似开火”电流脉冲
    HEAT_FSM_SUSPECT,    // 已检测到电流脉冲，等待持续时间确认
    HEAT_FSM_CONFIRM     // 已确认开火，等待最小发射间隔结束
} HeatFSMState_e;

typedef struct {
    float current_heat;        // 本地估算的当前热量
    HeatFSMState_e state;      // 虚拟热量状态机
    uint32_t suspect_tick;     // 最近一次疑似开火起始时刻
    fp32 cooling_value;        // 当前裁判系统下发的冷却值
    fp32 limit_heat;           // 当前裁判系统下发的热量上限
} VirtualHeat_t;

static VirtualHeat_t g_virtual_heat = {0.0f, HEAT_FSM_IDLE, 0U, 0.0f, 0.0f};
static uint32_t shoot_interrupt_tick = 0U;
// 输出给拨弹机构的目标转速，本质是热量约束后的机械发射频率映射值。
static fp32 real_trriger_speed = 0.0f;
static uint8_t heat_calc_counter = 0U;

static float Feedforward_Freq_Calc(float q_res, float a, float d, uint16_t n, float max_freq);
static void shoot_interrupt_update(void);

fp32 shoot_heat_watch=0.0f;
fp32 max_fric_current_watch = 0.0f;

void Shoot_Virtual_Heat_Update(float fric_current_A, float fric_current_B, float fric_speed_A, float fric_target)
{
    static uint16_t settle_counter_ms = 0U;
    static float cooling_residual = 0.0f;
    static uint32_t heat_sync_tick = 0U;
    const uint16_t settle_period_ms = (uint16_t)(REFEREE_SETTLE_PERIOD_S * 1000.0f);
    float max_fric_current = fmaxf(fabsf(fric_current_A), fabsf(fric_current_B));
    max_fric_current_watch = max_fric_current;
    uint8_t is_fric_ready = (uint8_t)((fabsf(fric_speed_A) > fabsf(fric_target) * 0.9f) && (fric_target != 0.0f));
    uint32_t now_tick = HAL_GetTick();

    // 强化进攻姿态下裁判反馈热量上限无限、当前热量恒为 0。
    // 若虚拟热量模型照常累积，退出强化后会因虚拟热量高位而长时间锁死发射，
    // 因此这里整体旁路热量模型，保持虚拟热量恒为 0，热量停止判断自然放行。
    // 同时维持冷却/同步节拍，避免退出强化姿态瞬间误触发一次拉偏。
    if (get_sentry_state() == ENHANCED_ATTACK_STATE) {
        g_virtual_heat.current_heat = 0.0f;
        g_virtual_heat.state = HEAT_FSM_IDLE;
        heat_sync_tick = now_tick;
        settle_counter_ms = 0U;
        cooling_residual = 0.0f;
        shoot_heat_watch = g_virtual_heat.current_heat;
        return;
    }

    // 摩擦轮未拉起时不允许进入击发判定状态，避免停转/起转瞬间误加热量。
    if (!is_fric_ready) {
        g_virtual_heat.state = HEAT_FSM_IDLE;
    }

    // 优先读取裁判系统真实限值；若裁判数据未反馈，则退回到本地默认值。
    g_virtual_heat.cooling_value = get_shoot_cooling_value();
    g_virtual_heat.limit_heat = get_shoot_heat_limit();
    if (g_virtual_heat.limit_heat <= 0.0f) {
        g_virtual_heat.limit_heat = SIM_HEAT_MAX;
    }

    //热量冷却按裁判结算周期离散化，残余小数累积到下一周期，减少积分误差。
    settle_counter_ms += CONTROL_PERIOD_MS;
    if (settle_counter_ms >= settle_period_ms) {
        settle_counter_ms -= settle_period_ms;

        if (g_virtual_heat.current_heat > 0.0f) {
            float cool_rate = (g_virtual_heat.cooling_value > 0.0f) ? g_virtual_heat.cooling_value : SIM_COOLING_RATE;
            float cool_total = cool_rate * REFEREE_SETTLE_PERIOD_S + cooling_residual;
            uint16_t cool_step = (uint16_t)cool_total;
            cooling_residual = cool_total - (float)cool_step;

            g_virtual_heat.current_heat -= (float)cool_step;
            if (g_virtual_heat.current_heat < 0.0f) {
                g_virtual_heat.current_heat = 0.0f;
            }
        }
    }

    // 用摩擦轮电流脉冲识别一次实际发射：
    // READY 等待脉冲，SUSPECT 确认脉冲持续时间，CONFIRM 做最小间隔锁定。
    switch (g_virtual_heat.state) {
    case HEAT_FSM_IDLE:
        if (is_fric_ready && max_fric_current < FRIC_CURRENT_THRESHOLD) {
            g_virtual_heat.state = HEAT_FSM_READY;
        }
        break;

    case HEAT_FSM_READY:
        if (max_fric_current > FRIC_CURRENT_THRESHOLD) {
            g_virtual_heat.state = HEAT_FSM_SUSPECT;
            g_virtual_heat.suspect_tick = HAL_GetTick();
            // 发弹延迟统计取首次脉冲到达时刻，不等待后面的热量确认延迟。
            record_fire_detect_L();
        }
        break;

    case HEAT_FSM_SUSPECT:
        if (max_fric_current < FRIC_CURRENT_THRESHOLD) {
            g_virtual_heat.state = HEAT_FSM_READY;
        } else if (HAL_GetTick() - g_virtual_heat.suspect_tick > SHOOT_CONFIRM_TIME_MS) {
            g_virtual_heat.state = HEAT_FSM_CONFIRM;
            g_virtual_heat.current_heat += SIM_HEAT_PER_SHOT;
            if (g_virtual_heat.current_heat > g_virtual_heat.limit_heat) {
                g_virtual_heat.current_heat = g_virtual_heat.limit_heat;
            }
            // 与现有开火中断/结束标志复用，告诉发射流程“已经确认完成一次击发”。
            set_interrupt_flag_end_L();
        }
        break;

    case HEAT_FSM_CONFIRM:
        if (HAL_GetTick() - g_virtual_heat.suspect_tick > SHOOT_MIN_INTERVAL_MS) {
            if (max_fric_current < FRIC_CURRENT_THRESHOLD) {
                g_virtual_heat.state = HEAT_FSM_READY;
            }
        }
        break;

    default:
        g_virtual_heat.state = HEAT_FSM_IDLE;
        break;
    }

    //周期性把本地虚拟热量向裁判系统真实热量缓慢贴齐，避免长期累计误差。
    if (now_tick - heat_sync_tick >= HEAT_SYNC_PERIOD_MS) {
        float referee_heat = (float)get_shoot_heat1();
        float err = referee_heat - g_virtual_heat.current_heat;
        float step = err * HEAT_SYNC_GAIN;

        heat_sync_tick = now_tick;
        if (step > HEAT_SYNC_MAX_STEP) step = HEAT_SYNC_MAX_STEP;
        if (step < -HEAT_SYNC_MAX_STEP) step = -HEAT_SYNC_MAX_STEP;

        g_virtual_heat.current_heat += step;

        if (g_virtual_heat.current_heat < 0.0f) {
            g_virtual_heat.current_heat = 0.0f;
        }
        if (g_virtual_heat.current_heat > g_virtual_heat.limit_heat) {
            g_virtual_heat.current_heat = g_virtual_heat.limit_heat;
        }
    }

    shoot_heat_watch = g_virtual_heat.current_heat;
}

float Get_Virtual_Heat(void)
{
    return g_virtual_heat.current_heat;
}

float Get_Virtual_Heat_Limit(void)
{
    return g_virtual_heat.limit_heat;
}

float Get_Virtual_Cooling_Value(void)
{
    return g_virtual_heat.cooling_value;
}

static float Feedforward_Freq_Calc(float q_res, float a, float d, uint16_t n, float max_freq)
{
    float f_ref = 1.0f / REFEREE_SETTLE_PERIOD_S;
    float balance_freq = a / d;
    float target_freq = 0.0f;

    // q_res 是剩余热量余量，a 是冷却能力，d 是单发热量。
    // 公式本质是在“热量不超限”的约束下，求一个尽量高的连续发射频率前馈值。
    if (q_res <= HEAT_STOP_MARGIN || n == 0U || d <= 0.0f) return 0.0f;
    if (balance_freq >= max_freq) return max_freq;

    target_freq = balance_freq + (f_ref * q_res - a) / (d * (float)n);

    if (target_freq > max_freq) target_freq = max_freq;
    if (target_freq < 0.0f) target_freq = 0.0f;

    return target_freq;
}

float heat_control(void)
{
    heat_calc_counter++;
    if (heat_calc_counter >= HEAT_CALC_DIVIDER) {
        float q_res = Get_Virtual_Heat_Limit() - Get_Virtual_Heat();
        float a = Get_Virtual_Cooling_Value();
        float freq = 0.0f;

        heat_calc_counter = 0U;
        if (q_res < 0.0f) q_res = 0.0f;
        if (a <= 0.0f) a = SIM_COOLING_RATE;

        // 不必每 1ms 都重算一次频率，按分频节拍更新即可减少输出抖动。
        freq = Feedforward_Freq_Calc(q_res, a, SIM_HEAT_PER_SHOT, DEFAULT_BURST_N, MAX_MECH_FREQ_HZ);
        real_trriger_speed = freq * RPM_PER_HZ;
    }
    return real_trriger_speed;
}

static void shoot_interrupt_update(void)
{
    // 预留的“最近一次击发时间”记录接口，目前仅维护时间戳。
    shoot_interrupt_tick = HAL_GetTick();
}

uint8_t shoot_interrupt_check(void)
{
    // 距离最近一次击发时间过短时返回 0，用于屏蔽过密触发。
    if (HAL_GetTick() - shoot_interrupt_tick < 5U) {
        return 0U;
    }
    return 1U;
}
