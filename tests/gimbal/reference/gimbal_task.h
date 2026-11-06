#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H

#include "CAN_receive.h"
#include "dm_imu_l1.h"
#include "gimbal/gimbal_cali.h"
#include "line_adrc.h"
#include "remote_control.h"
#include "struct_typedef.h"
#include "LPF.h"
#include "PID.h"
#include "gimbal_params.h" /* User_Config 目录已在 CMake include path 中 */

typedef struct MI_Motor MI_Motor_t;

/**
 * @brief 双 yaw 机构目标角集合。
 */
typedef struct
{
    fp32 inner_yaw; /**< 内层 yaw 目标角，单位 rad。 */
    fp32 outer_yaw; /**< 外层 yaw 目标角，单位 rad。 */
} yaw_set_t;

/***********辨识控制开关*****************/
#define  PITCH_IDENTIFY 0

/**************************************/

/******************采集pitch各种信息开关********************/
#define PITCH_DATA_COLLECT 0

#if PITCH_DATA_COLLECT
fp32 J_out,B_out,G_out,total_tor_out,i_out;
#endif


/* PID 与控制参数已迁移至 User_Config/gimbal_params.h，请到该文件调参。 */

/**
 * @brief 云台任务启动前的初始化等待时间，单位 ms。
 */
#define GIMBAL_TASK_INIT_TIME 201

/**
 * @brief yaw 控制通道号。
 */
#define YAW_CHANNEL 2

/**
 * @brief pitch 控制通道号。
 */
#define PITCH_CHANNEL 3

#if USE_I6X_RECEIVER
/**
 * @brief I6X 遥控下云台模式切换通道号。
 */
#define GIMBAL_MODE_CHANNEL 2
#else
/**
 * @brief 非 I6X 遥控下云台模式切换通道号。
 */
#define GIMBAL_MODE_CHANNEL 0
#endif

#define TURN_KEYBOARD KEY_PRESSED_OFFSET_F
#define TURN_SPEED 0.04f
#define TEST_KEYBOARD KEY_PRESSED_OFFSET_R
#define RC_DEADBAND 10

#define PITCH_MAX_RELATIVE_ANGLE 0.57f //0.46frad
#define PITCH_MIN_RELATIVE_ANGLE (-0.47f)

#define YAW_RC_SEN -0.000006f
#define PITCH_RC_SEN -0.000003f
#define YAW_MOUSE_SEN 0.00005f
#define PITCH_MOUSE_SEN 0.00005f

#define AUTO_MOUSE_SEN 0.00015f
#define AUTO_PITCH_MAXLIM 0.39f
#define AUTO_PITCH_MINLIM -0.33f
#define AUTO_PITCH_OFFSET 0.23f
#define AUTO_YAW_OFFSET 0.0f
#define AUTO_PITCH_SEN 1.0f
#define AUTO_YAW_SEN 1.0f

#define YAW_ENCODE_SEN 0.01f
#define PITCH_ENCODE_SEN 0.01f

#define GIMBAL_CONTROL_TIME 1
#define GIMBAL_TEST_MODE 0

#define PITCH_TURN 0
#define YAW_TURN 0

#define MI_HALF_ECD_RANGE 8192
#define MI_ECD_RANGE 16384
#define MI_MOTOR_ECD_TO_RAD 3.8349519697141030742952189737299e-4f

#define MS_HALF_ECD_RANGE 32768
#define MS_ECD_RANGE 65536
#define MS_MOTOR_ECD_TO_RAD 9.5873799242852576857380474343247e-5f
#define MF_MOTOR_OFFECD 27989
#define MF_MOTOR_MAXECD (27989 + 8192)
#define MF_MOTOR_MINECD (27989 - 8192)
#define MF_MOTOR_MAX_RELATIVE_ANGLE (8192 * MS_MOTOR_ECD_TO_RAD)
#define MF_MOTOR_MIN_RELATIVE_ANGLE ((-8192) * MS_MOTOR_ECD_TO_RAD)

#define HALF_ECD_RANGE 4096
#define ECD_RANGE 8191
#define GIMBAL_INIT_ANGLE_ERROR 0.1f
#define GIMBAL_INIT_STOP_TIME 1000
#define GIMBAL_INIT_TIME 6000
#define GIMBAL_CALI_REDUNDANT_ANGLE 0.1f
#define GIMBAL_INIT_PITCH_SPEED 0.004f
#define GIMBAL_INIT_YAW_SPEED 0.005f

#define INIT_YAW_SET 0.0f
#define INIT_PITCH_SET 0.0f

#define GIMBAL_CALI_MOTOR_SET 8000
#define GIMBAL_MS_YAW_CALI_MOTOR_SET 600
#define GIMBAL_CALI_STEP_TIME 2000
#define GIMBAL_CALI_GYRO_LIMIT 0.1f

#define GIMBAL_CALI_PITCH_MAX_STEP 1
#define GIMBAL_CALI_PITCH_MIN_STEP 2
#define GIMBAL_CALI_YAW_MAX_STEP 3
#define GIMBAL_CALI_YAW_MIN_STEP 4

#define GIMBAL_CALI_START_STEP GIMBAL_CALI_PITCH_MAX_STEP
#define GIMBAL_CALI_END_STEP 5

#define GIMBAL_MOTIONLESS_RC_DEADLINE 10
#define GIMBAL_MOTIONLESS_TIME_MAX 3000

#ifndef MOTOR_ECD_TO_RAD
#define MOTOR_ECD_TO_RAD 0.000766990394f
#endif

#define RECORD_CENTER_KEY KEY_PRESSED_OFFSET_X
#define RECORD_RADIUS_KEY KEY_PRESSED_OFFSET_C
#define AT_CHANGE_KEY KEY_PRESSED_OFFSET_CTRL

#define GIMBAL_YAW_RECOUNT_A0 2528.92869f
#define GIMBAL_YAW_RECOUNT_A1 802.9869f
#define GIMBAL_YAW_RECOUNT_A2 -106.54695f
#define GIMBAL_YAW_COUNT_MAX 3.77f
#define GIMBAL_YAW_COUNT_MIN 0.4f

#define ANGLE_TO_RAD 0.017453292519943296f
#define YAW_NUM 2

/* 机械模型、AT PID、前馈、I-Zone 等参数已迁移至 User_Config/gimbal_params.h。 */

/**
 * @brief 自动瞄准跟踪状态机。
 */
typedef enum
{
    DETECT_STATUS = 0, /**< 搜索/检测目标阶段。 */
    TRACK_STATUS,      /**< 稳定跟踪目标阶段。 */
    CONFIRM_STATUS,    /**< 目标确认阶段。 */
} track_state_machine_e;

/**
 * @brief 云台电机逻辑编号。
 */
typedef enum
{
    YAW_MOTOR = 0, /**< yaw 轴电机。 */
    PITCH_MOTOR,   /**< pitch 轴电机。 */
} gimbal_motor_id_e;

/**
 * @brief 云台 PID 控制器状态结构（统一使用 pid_type_def）。
 *
 * 原 gimbal_PID_t 已合并至 components/algorithm/PID/PID.h 的 pid_type_def，
 * 扩展字段: Param.KF(前馈)、Param.IZone(积分分离)、PrevTarget、Dbuf[3]。
 * 字段对应关系:
 *   gimbal.kp/max_out/max_iout → pid.Param.KP/LimitOutput/LimitIntegral
 *   gimbal.set/get/err/out     → pid.Target/Measure/Err[0]/Output
 *   gimbal.error[3]/Dbuf[3]    → pid.Err[3]/Dbuf[3]
 *   gimbal.i_zone/kf           → pid.Param.IZone/KF
 */
typedef pid_type_def gimbal_PID_t;

/* ===== 兼容包装函数（内联委托至 PID.c 统一实现） ===== */
static inline void gimbal_PID_init(gimbal_PID_t *pid, fp32 maxout, fp32 max_iout, uint8_t mode, fp32 kp, fp32 ki, fp32 kd, fp32 kf)
{
    (void)mode;
    PID_InitFull(pid, kp, ki, kd, kf, maxout, max_iout, 0.0f);
}
static inline void gimbal_PID_clear(gimbal_PID_t *pid) { PID_ClearExtended(pid); }
static inline fp32 gimbal_PID_calc(gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta) { return PID_CalculateAngle(pid, get, set, error_delta); }
static inline fp32 gimbal_PID_calc_by_error(gimbal_PID_t *pid, fp32 error, fp32 error_delta) { return PID_CalculateByError(pid, error, error_delta); }
static inline void PID_clear_for_gimbal(gimbal_PID_t *pid) { PID_ClearExtended(pid); }

/**
 * @brief 单个云台电机的控制与状态封装。
 */
typedef struct
{
    const motor_measure_t *gimbal_motor_measure; /**< 电机反馈数据指针。 */
    gimbal_motor_id_e id;                       /**< 电机逻辑编号。 */
    gimbal_PID_t gimbal_motor_absolute_angle_pid; /**< 绝对角 PID。 */
    gimbal_PID_t gimbal_motor_relative_angle_pid; /**< 相对角 PID。 */
    gimbal_PID_t gimbal_motor_gyro_pid;           /**< 陀螺仪角速度 PID。 */
    gimbal_PID_t gimbal_motor_soft_gyro_pid;      /**< 软限幅角速度 PID。 */
    gimbal_motor_mode_e gimbal_motor_mode;        /**< 当前控制模式。 */
    gimbal_motor_mode_e last_gimbal_motor_mode;   /**< 上一拍控制模式。 */
    uint16_t offset_ecd;                          /**< 编码器零位偏移。 */
    fp32 max_relative_angle;                      /**< 相对角上限。 */
    fp32 min_relative_angle;                      /**< 相对角下限。 */
    fp32 relative_angle;                          /**< 当前相对角。 */
    fp32 relative_angle_set;                      /**< 当前相对角目标。 */
    fp32 absolute_angle;                          /**< 当前绝对角。 */
    fp32 raw_absolute_angle;                      /**< 原始绝对角。 */
    fp32 absolute_angle_set;                      /**< 当前绝对角目标。 */
    fp32 motor_gyro;                              /**< 当前角速度反馈。 */
    fp32 motor_gyro_set;                          /**< 当前角速度目标。 */
    fp32 motor_speed;                             /**< 当前速度反馈。 */
    fp32 raw_cmd_current;                         /**< 原始电流指令。 */
    fp32 current_set;                             /**< 当前目标电流。 */
    int16_t given_current;                        /**< 最终给定电流。 */
    fp32 last_gyro_set;                           /**< 上一拍角速度目标。 */
    int is_limited;                               /**< 当前是否触发限幅。 */
    fp32 kp_plan_gyro;                            /**< 陀螺补偿规划系数。 */
    fp32 kp_plan_accel;                           /**< 加速度补偿规划系数。 */
    fp32 k_gyro;                                  /**< 角速度补偿系数。 */
    fp32 last_gyro;                               /**< 上一拍角速度。 */
    fp32 tor;                                     /**< 力矩估计值。 */
    fp32 gravity_compensation;                    /**< 重力补偿量。 */
    fp32 td_angle;                                /**< TD 规划角度输出。 */
    fp32 td_gyro;                                 /**< TD 规划角速度输出。 */
    fp32 td_accel;                                /**< TD 规划角加速度输出。 */
    fp32 td_accel_ff;                             /**< 仅用于前馈的加速度滤波状态。 */
    uint8_t td_inited;                            /**< TD 状态是否已按当前反馈初始化。 */
    LADRC_1st_t ladrc_controler;                  /**< 一阶 LADRC 控制器。 */
    fp32 filtered_gyro;                           /**< 滤波后的角速度。 */
    LowPassFilter1p_Info_TypeDef vel_err_lpf;     /**< 速度误差 LPF。 */
    LowPassFilter1p_Info_TypeDef torque_lpf;      /**< 力矩输出LPF */
} gimbal_motor_t;

/**
 * @brief 云台校准流程状态记录结构。
 */
/* gimbal_step_cali_t 和 GIMBAL_CALI_* 宏已迁移至 gimbal_cali.h。 */

/**
 * @brief 云台总控制状态结构。
 */
typedef struct
{
    const RC_ctrl_t *gimbal_rc_ctrl;    /**< 遥控输入指针。 */
    const fp32 *gimbal_INT_angle_point; /**< 姿态角反馈指针。 */
    const fp32 *gimbal_INT_gyro_point;  /**< 角速度反馈指针。 */
    gimbal_motor_t gimbal_yaw_motor[2]; /**< 双 yaw 电机控制结构。 */
    gimbal_motor_t gimbal_pitch_motor;  /**< pitch 电机控制结构。 */
    dm_imu_t *gimbal_INT_angle_point_DM;   /**< 云台 DM IMU 指针。 */
    dm_imu_t *chassis_INT_angle_point_DM;  /**< 底盘 DM IMU 指针。 */
    uint8_t yaw_conut;                     /**< yaw 计数/状态记录。 */
    gimbal_step_cali_t gimbal_cali;       /**< 校准流程状态。 */
    MI_Motor_t *MI_gimbal_pitch_motor;    /**< pitch 电机对象指针。 */
    fp32 MI_set_ecd;                      /**< pitch 目标编码器值。 */
    fp32 MI_set_angle;                    /**< pitch 目标角。 */
    fp32 MI_set_torque;                   /**< pitch 目标力矩。 */
    fp32 kp;                              /**< pitch MIT 控制 kp。 */
    fp32 kd;                              /**< pitch MIT 控制 kd。 */
    fp32 ki;                              /**< 预留积分或扩展参数。 */
    track_state_machine_e auto_state_machine; /**< 自动瞄状态机。 */
    uint8_t last_auto_state_machine;          /**< 上一拍自动瞄状态。 */
    float yaw_offset;                         /**< 自瞄 yaw 偏移。 */
    uint8_t offset_is_ready;                  /**< 偏移量是否已就绪。 */
    float yaw_error;                          /**< 当前 yaw 误差。 */
    float dual_yaw_target_angle;              /**< 双 yaw 目标总角度。 */
    float small_yaw_scan_ref;                 /**< 小 yaw 扫描参考角。 */
    uint8_t big_yaw_state;                    /**< 大 yaw 当前状态。 */
    uint8_t small_yaw_state;                  /**< 小 yaw 当前状态。 */
    fp32 yaw_align_offset;                    /**< BMI088 yaw 向 DM IMU yaw 对齐的在线估计偏置。 */
    fp32 big_yaw_drag_target;                 /**< 大yaw增量预测拖拽框的持久绝对目标（板载IMU坐标系）。 */
} gimbal_control_t;

/**
 * @brief pitch 保护调试标志结构。
 */
typedef struct
{
    uint8_t angle_rate_limited; /**< 角度变化率是否限幅。 */
    uint8_t angle_clamped;      /**< 目标角是否钳位。 */
    uint8_t overspeed_braking;  /**< 是否触发超速制动。 */
    uint8_t vel_cmd_limited;    /**< 速度指令是否限幅。 */
    uint8_t soft_limit_upper;   /**< 是否触发上软限位。 */
    uint8_t soft_limit_lower;   /**< 是否触发下软限位。 */
    uint8_t hard_limit_pullback; /**< 是否触发硬限位回拉。 */
    uint8_t torque_rate_limited; /**< 力矩变化率是否限幅。 */
    uint8_t torque_clamped;      /**< 力矩是否钳位。 */
    uint8_t any_triggered;       /**< 是否存在任一保护触发。 */
} pitch_protection_debug_t;

/**
 * @brief pitch 保护功能使能开关。
 */
#define PITCH_PROTECTION_ENABLE 1

/**
 * @brief pitch 目标角单周期最大变化量，单位 rad。
 */
#define PITCH_ANGLE_SET_MAX_DELTA 0.01f

/**
 * @brief pitch 目标力矩单周期最大变化量。
 */
#define PITCH_TORQUE_MAX_DELTA 0.5f

/**
 * @brief pitch 保护允许的最大转速，单位 rad/s。
 */
#define PITCH_MAX_SPEED 15.0f

/**
 * @brief pitch 超速恢复阈值，单位 rad/s。
 */
#define PITCH_SPEED_RECOVER 8.0f

/**
 * @brief pitch 软限位保护边距，单位 rad。
 */
#define PITCH_SOFT_LIMIT_MARGIN 0.05f

/**
 * @brief pitch 限位回拉比例系数。
 */
#define PITCH_LIMIT_KP 40.0f

/**
 * @brief pitch 限位回拉微分系数。
 */
#define PITCH_LIMIT_KD 2.0f

/**
 * @brief pitch 保护允许的最大输出力矩。
 */
#define PITCH_MAX_TORQUE 6.0f

/**
 * @brief pitch 保护允许的最大速度指令。
 */
#define PITCH_MAX_VEL_CMD 10.0f

extern gimbal_control_t gimbal_control;
extern pitch_protection_debug_t pitch_prot_dbg;

/**
 * @brief 获取指定 yaw 电机控制结构指针。
 *
 * @param[in] i yaw 电机下标。
 * @return 电机控制结构地址。
 */
const gimbal_motor_t *get_yaw_motor_point(int i);

/**
 * @brief 获取 pitch 电机控制结构指针。
 *
 * @return pitch 电机控制结构地址。
 */
const gimbal_motor_t *get_pitch_motor_point(void);

/**
 * @brief 获取指定 yaw 轴当前相对角指针。
 *
 * @param[in] i yaw 电机下标。
 * @return 相对角数据地址。
 */
const fp32 *get_relative_angle(int i);

/**
 * @brief 读取当前校准结果并判断云台是否已完成校准。
 *
 * @param[out] yaw_offset yaw 零位偏移。
 * @param[out] pitch_offset pitch 零位偏移。
 * @param[out] max_yaw yaw 最大机械角。
 * @param[out] min_yaw yaw 最小机械角。
 * @param[out] max_pitch pitch 最大机械角。
 * @param[out] min_pitch pitch 最小机械角。
 * @retval 1 校准数据有效。
 * @retval 0 校准数据无效。
 */
/* cmd_cali_gimbal_hook / set_cali_gimbal_hook 已迁移至 gimbal_cali.h。 */

/**
 * @brief 计算云台 PID 输出。
 *
 * @param[in,out] pid PID 控制器结构。
 * @param[in] get 当前反馈值。
 * @param[in] set 当前目标值。
 * @return PID 输出结果。
 */
fp32 fb_gimbal_PID_calc(gimbal_PID_t *pid, fp32 get, fp32 set);

/**
 * @brief 计算阻力补偿前馈值。
 *
 * @param[in] x 当前输入量。
 * @param[in] a0 多项式常数项系数。
 * @param[in] a1 多项式一次项系数。
 * @param[in] a2 多项式二次项系数。
 * @param[in] x_max 输入上限。
 * @param[in] x_min 输入下限。
 * @return 前馈补偿结果。
 */
fp32 Resistance_feedforward_count(fp32 x, float a0, float a1, float a2, fp32 x_max, fp32 x_min);

/**
 * @brief 执行单个云台电机的速度环控制。
 *
 * @param[in,out] gimbal_motor 云台电机控制结构。
 */
void gimbal_motor_speed_control(gimbal_motor_t *gimbal_motor);

/**
 * @brief yaw 电机离线处理函数。
 */
void gimbal_yaw_lost_fun(void);

/**
 * @brief pitch 电机离线处理函数。
 */
void gimbal_pitch_lost_fun(void);

/**
 * @brief 按最大步长将当前目标插值逼近最终目标。
 *
 * @param[in] current_angle 当前反馈角度。
 * @param[in,out] current_target 当前目标角地址。
 * @param[in] final_target 最终目标角。
 * @param[in] max_step 单周期最大步长。
 */
void motor_interplotion_update(float current_angle, float *current_target, float final_target, float max_step);

/**
 * @brief 配置 pitch 变频扫描参数。
 *
 * @param[in] center 扫描中心角。
 * @param[in] range 扫描幅值范围。
 * @param[in] freq_min 最低扫描频率。
 * @param[in] freq_max 最高扫描频率。
 * @param[in] sweep_period 扫频周期。
 */
void pitch_varfreq_scan_set_config(fp32 center, fp32 range, fp32 freq_min, fp32 freq_max, fp32 sweep_period);

/**
 * @brief 读取 pitch 变频扫描参数。
 *
 * @param[out] center 扫描中心角。
 * @param[out] range 扫描幅值范围。
 * @param[out] freq_min 最低扫描频率。
 * @param[out] freq_max 最高扫描频率。
 * @param[out] sweep_period 扫频周期。
 */
void pitch_varfreq_scan_get_config(fp32 *center, fp32 *range, fp32 *freq_min, fp32 *freq_max, fp32 *sweep_period);

/**
 * @brief 配置 pitch 变频扫描幅值缩放限制。
 *
 * @param[in] amp_scale_min 幅值缩放下限。
 * @param[in] amp_scale_max 幅值缩放上限。
 */
void pitch_varfreq_scan_set_amp_scale_limit(fp32 amp_scale_min, fp32 amp_scale_max);

/**
 * @brief 读取 pitch 变频扫描幅值缩放限制。
 *
 * @param[out] amp_scale_min 幅值缩放下限。
 * @param[out] amp_scale_max 幅值缩放上限。
 */
void pitch_varfreq_scan_get_amp_scale_limit(fp32 *amp_scale_min, fp32 *amp_scale_max);

/**
 * @brief 设置 pitch 变频扫描类型。
 *
 * @param[in] sweep_type 扫描类型编号。
 */
void pitch_varfreq_scan_set_type(uint8_t sweep_type);

/**
 * @brief 获取当前 pitch 变频扫描类型。
 *
 * @return 扫描类型编号。
 */
uint8_t pitch_varfreq_scan_get_type(void);

/**
 * @brief 对 pitch 电机执行限位与超速保护。
 *
 * @param[in,out] pitch_motor pitch 电机控制结构。
 */
/* pitch_motor_protection 已迁移至 pitch_ctrl.h */

/* =========================================================================
 * P2-11: gimbal_control 上下文注入 — 外部模块只读访问接口
 * ========================================================================= */

/**
 * @brief 获取 DM IMU 四元数（用于 USB 遥测）。
 * @param[in]  ctrl 云台控制结构体指针。
 * @param[out] q    四元数数组 q[0..3]，ctrl 为 NULL 时填充 0。
 */
void gimbal_ctrl_get_quaternion(const gimbal_control_t *ctrl, fp32 q[4]);

/**
 * @brief 获取指定 yaw 电机的绝对角度。
 * @param[in] ctrl 云台控制结构体指针。
 * @param[in] idx  YAW_NUM 范围内的索引。
 * @return 绝对角度 (rad)，无效输入返回 0。
 */
fp32 gimbal_ctrl_get_yaw_angle(const gimbal_control_t *ctrl, uint8_t idx);

/**
 * @brief 获取指定 yaw 电机的角速度。
 * @param[in] ctrl 云台控制结构体指针。
 * @param[in] idx  YAW_NUM 范围内的索引。
 * @return 角速度 (rad/s)，无效输入返回 0。
 */
fp32 gimbal_ctrl_get_yaw_gyro(const gimbal_control_t *ctrl, uint8_t idx);

/**
 * @brief 获取 pitch 电机的绝对角度。
 * @param[in] ctrl 云台控制结构体指针。
 * @return 绝对角度 (rad)，ctrl 为 NULL 时返回 0。
 */
fp32 gimbal_ctrl_get_pitch_angle(const gimbal_control_t *ctrl);

/**
 * @brief 获取 pitch 电机的角速度。
 * @param[in] ctrl 云台控制结构体指针。
 * @return 角速度 (rad/s)，ctrl 为 NULL 时返回 0。
 */
fp32 gimbal_ctrl_get_pitch_gyro(const gimbal_control_t *ctrl);

/**
 * @brief FreeRTOS 云台主任务。
 *
 * @param pvParameters 任务参数，当前未使用。
 */
_Noreturn void gimbal_task(void *pvParameters);

#endif
