#ifndef SHOOT_TASK_H
#define SHOOT_TASK_H

#include "CAN_receive.h"
#include "LPF.h"
#include "PID.h"
#include "gimbal/gimbal_task.h"
#include "remote_control.h"
#include "struct_typedef.h"
#include "user_lib.h"

#ifndef USE_I6X_RECEIVER
/**
 * @brief 根据 `RC_TYPE` 推导当前是否使用 I6X 接收机链路。
 */
#define USE_I6X_RECEIVER (RC_TYPE == 0)
#endif

/**
 * @brief 发射模式与控制通道配置。
 */
#define SHOOT_RC_MODE_CHANNEL 1
#define SHOOT_CONTROL_CHANNEL 0
#define SHOOT_CONTROL_TIME GIMBAL_CONTROL_TIME
#define SHOOT_FRIC_PWM_ADD_VALUE 100.0f

#if USE_I6X_RECEIVER
/**
 * @brief I6X 遥控模式下的 PC 发射控制通道号。
 */
#define PC_CONTROL_SHOOT_CHANNEL 3
#define SHOOT_CONTROL_CHANNEL 0
#endif

/**
 * @brief 发射键位与判定时间参数。
 */
#define SHOOT_ON_KEYBOARD KEY_PRESSED_OFFSET_Q
#define SHOOT_OFF_KEYBOARD KEY_PRESSED_OFFSET_E
#define SHOOT_UP_KEYBOARD KEY_PRESSED_OFFSET_R
#define SHOOT_DONE_KEY_OFF_TIME 15
#define PRESS_LONG_TIME 400
#define RC_S_LONG_TIME 2000
#define UP_ADD_TIME 80

/**
 * @brief 编码器与运动学换算常量。
 */
#define HALF_ECD_RANGE 4096
#define ECD_RANGE 8191
#define MOTOR_RPM_TO_SPEED 0.00290888208665721596153948461415f
#define MOTOR_ECD_TO_ANGLE 0.000021305288720633905968306772076277f
#define FULL_COUNT 18


/**
 * @brief 发射目标速度配置。
 */
#define READY_BULLET_SPEED -4.0f
#define TRIGGER_SPEED -5.0f
#define CONTINUE_TRIGGER_SPEED1 -15.0f
#define CONTINUE_TRIGGER_SPEED2 -25.0f

/**
 * @brief 拨弹开关逻辑定义。
 */
#define SWITCH_TRIGGER_ON 0
#define SWITCH_TRIGGER_OFF 1

/**
 * @brief 卡弹检测与反转恢复配置。
 */
#define BLOCK_TRIGGER_SPEED 1.0f
#define BLOCK_TIME 1000
#define REVERSE_TIME 500
#define REVERSE_SPEED_LIMIT 13.0f

/**
 * @brief 发射角度相关常量。
 */
#define PI_FOUR 0.78539816339744830961566084581988f
#define PI_TEN 0.314f

/**
 * @brief 拨弹电机位置环 PID 参数。
 */
#define TRIGGER_ANGLE_PID_KP 1000.0f
#define TRIGGER_ANGLE_PID_KI 2.0f
#define TRIGGER_ANGLE_PID_KD 0.0f

/**
 * @brief 单发拨弹速度环输出限制。
 */
#define TRIGGER_BULLET_PID_MAX_OUT 10000.0f
#define TRIGGER_BULLET_PID_MAX_IOUT 9000.0f

/**
 * @brief 预装弹阶段拨弹速度环输出限制。
 */
#define TRIGGER_READY_PID_MAX_OUT 10000.0f
#define TRIGGER_READY_PID_MAX_IOUT 7000.0f

/**
 * @brief 发射热量安全余量。
 */
#define SHOOT_HEAT_REMAIN_VALUE 40

/**
 * @brief 拨盘热量停止偏移量：当前热量 >= (上限-此值) 时停止拨盘。
 */
#define SHOOT_HEAT_STOP_OFFSET 80

/**
 * @brief 摩擦轮速度环 PID 参数。
 */
#define SHOOT_SPEED_PID_KP 8000.0f //7000.0f
#define SHOOT_SPEED_PID_KI 5.0f //10.0f
#define SHOOT_SPEED_PID_KD 0.0f
#define SHOOT_SPEED_PID_MAX_OUT 16000.0f
#define SHOOT_SPEED_PID_MAX_IOUT 1000.0f

/**
 * @brief 弹速管理：参考目标弹速 (m/s)，慢速通道据此调节设定值。
 *        与 Fric_STARTUP_TARGET_SPEED (21.5f) 保持一致。
 */
#define SHOOT_REFERENCE_SPEED 23.5f

/**
 * @brief 弹速管理：慢速调节最小时间间隔 (ms)，即“时间锁”。
 *        除“即将超限”的紧急判断外，其余调节更新间隔不得低于此值。
 */
#define SPEED_ADJUST_INTERVAL_MS 5000U

/**
 * @brief 弹速管理：慢速调节步长 (m/s / 步)。
 */
#define SPEED_ADJUST_STEP 0.2f

/**
 * @brief 弹速管理：调节死区 (m/s)。|基准弹速 - 参考目标| < 此值时不调节，防止弹速跳变。
 */
#define SPEED_DEADBAND 0.5f

/**
 * @brief 弹速管理：超限预警余量 (m/s)。最新弹速 >= 限制 - 余量 时触发紧急下调。
 */
#define SPEED_URGENT_MARGIN 0.5f

/**
 * @brief 弹速管理：紧急下调步长 (m/s / 步)，不受时间锁限制。
 */
#define SPEED_URGENT_STEP 0.3f

/**
 * @brief 弹速管理：摩擦轮设定值下限 (m/s 等效)。
 */
#define SPEED_SETPOINT_MIN 15.0f

/**
 * @brief 弹速管理：摩擦轮设定值上限 (m/s 等效)。
 */
#define SPEED_SETPOINT_MAX 23.5f

/**
 * @brief 弹速管理：基准弹速 LPF 平滑系数 (0~1)，仅在新反馈样本时更新。
 */
#define SPEED_LPF_ALPHA 0.3f

/**
 * @brief 弹速管理：有效反馈弹速下限 (m/s)，低于此值视为无效样本，不参与统计。
 */
#define SPEED_MIN_VALID_FB 5.0f

/**
 * @brief 电机转速换算与摩擦轮系数。
 */
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define FRICTION_ALL_COEFFICIENT 0.00359375f

/**
 * @brief 发射状态机模式。
 */
typedef enum
{
    SHOOT_STOP = 0,        /**< 发射机构停机。 */
    SHOOT_READY_FRIC,      /**< 先启动摩擦轮准备。 */
    SHOOT_READY_BULLET,    /**< 拨弹机构预装弹。 */
    SHOOT_READY,           /**< 已完成准备，等待发射。 */
    SHOOT_BULLET,          /**< 单发状态。 */
    SHOOT_CONTINUE_BULLET, /**< 连发状态。 */
} shoot_mode_e;

/**
 * @brief 发射机构左右编号。
 */
typedef enum
{
    LEFT_SHOOT = 0, /**< 左侧发射机构。 */
    RIGHT_SHOOT,    /**< 右侧发射机构。 */
} shoot_id_e;

/**
 * @brief 自动开火中断状态机。
 */
typedef enum
{
    FIRE_PREPARING = 0, /**< 正在等待满足开火条件。 */
    FIRE_READY = 1,     /**< 已满足条件，允许开火。 */
    FIRE_END = 2,       /**< 当前开火流程结束。 */
} fire_state_e;

/**
 * @brief 单侧发射机构控制总状态。
 */
typedef struct
{
    shoot_mode_e shoot_mode;        /**< 当前发射状态机模式。 */
    int8_t last_s;                  /**< 上一拍拨杆状态。 */
    const RC_ctrl_t *shoot_rc;      /**< 遥控输入指针。 */
    shoot_id_e shoot_id;            /**< 当前发射机构编号。 */
    const motor_measure_t *trigger_motor_measure; /**< 拨弹电机反馈指针。 */
    const motor_measure_t *fric_motor_measure[2]; /**< 两个摩擦轮电机反馈指针。 */
    pid_type_def trigger_motor_pid; /**< 拨弹位置/速度控制 PID。 */
    second_order_filter_type_t trigger_speed_filter; /**< 拨弹速度滤波器。 */
    pid_type_def FricSpeed_PID[2];  /**< 双摩擦轮速度环 PID。 */
    fp32 Fric_FBspeed_A;            /**< A 摩擦轮反馈速度。 */
    fp32 Fric_FBspeed_B;            /**< B 摩擦轮反馈速度。 */
    fp32 Fric_Targetspeed;          /**< 摩擦轮目标速度。 */
    int16_t Fric_current_A;         /**< A 摩擦轮输出电流。 */
    int16_t Fric_current_B;         /**< B 摩擦轮输出电流。 */
    fp32 Trigger_Actualspeed;       /**< 拨弹实际速度。 */
    fp32 Trigger_Targetspeed;       /**< 拨弹目标速度。 */
    fp32 Trigger_FBspeed_filtered;  /**< 滤波后的拨弹速度反馈。 */
    int16_t Trigger_current;        /**< 拨弹电机输出电流。 */
    bool_t press_l;                 /**< 当前左键状态。 */
    bool_t press_r;                 /**< 当前右键状态。 */
    bool_t last_press_l;            /**< 上一拍左键状态。 */
    bool_t last_press_r;            /**< 上一拍右键状态。 */
    uint16_t press_l_time;          /**< 左键累计按下时间。 */
    uint16_t press_r_time;          /**< 右键累计按下时间。 */
    uint16_t rc_s_time;             /**< 拨杆维持时间。 */
    uint16_t block_time;            /**< 卡弹持续时间。 */
    uint16_t reverse_time;          /**< 反转退弹持续时间。 */
    uint8_t key_time;               /**< 键控去抖/计时状态。 */
    uint16_t heat_limit;            /**< 当前热量上限。 */
    uint16_t heat;                  /**< 当前已用热量。 */
    uint16_t shoot_speed_limit;     /**< 当前射速上限。 */
    uint16_t last_shoot_speed_limit; /**< 上一拍射速上限。 */
    float shooter_speed;            /**< 当前射手速度设定或反馈值。 */
    uint8_t fric_flag;              /**< 摩擦轮使能标志。 */
    uint8_t shoot_flag;             /**< 发射允许标志。 */
    bool_t LimitState;              /**< 限位开关当前状态。 */
    GPIO_TypeDef *LimitSwitchPort;  /**< 限位开关 GPIO 端口。 */
    uint16_t LimitSwitchPin;        /**< 限位开关 GPIO 引脚。 */
    GPIO_PinState (*ReadLimitSwitch)(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin); /**< 限位开关读取回调。 */
    fire_state_e interrupt_flag;    /**< 自动开火中断状态。 */
    uint16_t interrupt_tick;        /**< 最近一次中断相关时间戳。 */
    uint16_t interrupt_start_tick;  /**< 中断开始时间戳。 */
    uint16_t interrupt_delta_tick;  /**< 中断持续时间。 */
    uint8_t fire_delay_pending;     /**< 是否存在待执行的开火延时。 */
    float open_fric;                /**< 开环摩擦轮控制量。 */
} shoot_control_t;

extern shoot_control_t shoot_control_L;
extern shoot_control_t shoot_control_R;

/**
 * @brief 初始化整套发射机构。
 */
void shoot_init(void);

/**
 * @brief 单周期执行一侧发射机构状态机与控制输出。
 *
 * @param[in,out] shoot_control 当前侧发射机构控制结构。
 */
void shoot_control_loop(shoot_control_t *shoot_control);

/**
 * @brief 发送左右发射机构的 CAN 电流指令。
 *
 * @param[in] left_a 左侧 A 电机电流。
 * @param[in] left_b 左侧 B 电机电流。
 * @param[in] right_a 右侧 A 电机电流。
 * @param[in] right_b 右侧 B 电机电流。
 */
void CAN_cmd_shoot(int16_t left_a, int16_t left_b, int16_t right_a, int16_t right_b);

/**
 * @brief 获取摩擦轮使能标志位地址。
 *
 * @return 标志位指针。
 */
uint8_t *get_fric_flag(void);

/**
 * @brief 初始化左侧发射机构。
 */
void Left_shoot_init(void);

/**
 * @brief 初始化右侧发射机构。
 */
void Right_shoot_init(void);

/**
 * @brief 初始化发射机构公共字段。
 *
 * @param[in,out] basic_init 发射机构控制结构。
 */
void basic_shoot_init(shoot_control_t *basic_init);

/**
 * @brief 获取当前射速上限。
 *
 * @param[out] speed_limit 射速上限输出。
 * @param[in] id 发射机构编号。
 */
void get_shoot_speed_limit(uint16_t *speed_limit, shoot_id_e id);

/**
 * @brief 获取当前热量上限与已用热量。
 *
 * @param[out] heat_limit 热量上限。
 * @param[out] heat 当前热量。
 * @param[in] id 发射机构编号。
 */
void get_shoot_heat_limit_and_heat(uint16_t *heat_limit, uint16_t *heat, shoot_id_e id);

/**
 * @brief 获取指定发射机构当前热量。
 *
 * @param[in] id 发射机构编号。
 * @return 当前热量值。
 */
uint16_t get_shoot_heat(shoot_id_e id);

/**
 * @brief 将左侧自动开火中断状态置为结束。
 */
void set_interrupt_flag_end_L(void);

/**
 * @brief 记录一次左侧开火检测事件。
 */
void record_fire_detect_L(void);

/**
 * @brief 获取当前自动开火中断状态。
 *
 * @return 自动开火中断状态枚举值。
 */
fire_state_e get_interrupt_flag(void);

/**
 * @brief 获取本地火控检查标志。
 *
 * @return 火控检查标志。
 */
uint8_t get_fire_check_flag(void);

/**
 * @brief 获取当前拨弹电机输出电流。
 *
 * @return 拨弹电机输出电流。
 */
int16_t get_shoot_trigger_current(void);

#endif
