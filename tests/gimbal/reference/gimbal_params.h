/**
 * @file    gimbal_params.h
 * @brief   云台 PID 与控制参数集中配置。
 *
 * 将原散布在 gimbal_task.h 中的 200+ 行 #define 参数集中到此文件，
 * 调参时不再翻找 gimbal_task.h，保持配置与逻辑分离。
 *
 * 注意：本文件仅包含编译期常量；校准数据等运行时参数仍走 Flash 持久化路径。
 */

#ifndef GIMBAL_PARAMS_H
#define GIMBAL_PARAMS_H

/* =========================================================================
 * 一、Pitch 轴速度环 PID 参数
 * ========================================================================= */
#define PITCH_SPEED_PID_KP      13000.0f
#define PITCH_SPEED_PID_KI      30.0f
#define PITCH_SPEED_PID_KD      0.0f
#define PITCH_SPEED_PID_KF      0.0f
#define PITCH_SPEED_PID_MAX_OUT 30000.0f
#define PITCH_SPEED_PID_MAX_IOUT 3000.0f

/* =========================================================================
 * 二、Inner Yaw 速度环 PID 参数
 * ========================================================================= */
#define INNER_YAW_SPEED_PID_KP      22000.0f    //30000.0f
#define INNER_YAW_SPEED_PID_KI      500.0f
#define INNER_YAW_SPEED_PID_KD      0.0f
#define INNER_YAW_SPEED_PID_KF      0.0f
#define INNER_YAW_SPEED_PID_MAX_OUT 30000.0f
#define INNER_YAW_SPEED_PID_MAX_IOUT 10000.0f

#define INNER_YAW_SPEED_MIN_PID_KP      20000.0f
#define INNER_YAW_SPEED_MIN_PID_KI      0.0f
#define INNER_YAW_SPEED_MIN_PID_KD      0.0f
#define INNER_YAW_SPEED_MIN_PID_KF      0.0f
#define INNER_YAW_SPEED_MIN_PID_MAX_OUT 30000.0f
#define INNER_YAW_SPEED_MIN_PID_MAX_IOUT 10000.0f

/* =========================================================================
 * 三、Outer Yaw 速度环 PID 参数
 * ========================================================================= */
#define OUTER_YAW_SPEED_PID_KP      12000.0f //12000.0f
#define OUTER_YAW_SPEED_PID_KI      5.0f
#define OUTER_YAW_SPEED_PID_KD      0.0f
#define OUTER_YAW_SPEED_PID_KF      0.0f
#define OUTER_YAW_SPEED_PID_MAX_OUT 30000.0f
#define OUTER_YAW_SPEED_PID_MAX_IOUT 2000.0f

#define OUTER_YAW_SPEED_MIN_PID_KP      7000.0f
#define OUTER_YAW_SPEED_MIN_PID_KI      0.0f
#define OUTER_YAW_SPEED_MIN_PID_KD      0.0f
#define OUTER_YAW_SPEED_MIN_PID_KF      0.0f
#define OUTER_YAW_SPEED_MIN_PID_MAX_OUT 30000.0f
#define OUTER_YAW_SPEED_MIN_PID_MAX_IOUT 0.0f

/* =========================================================================
 * 四、Gyro 绝对角 PID 参数（位置外环）
 * ========================================================================= */
#define PITCH_GYRO_ABSOLUTE_PID_KP      3000.0f
#define PITCH_GYRO_ABSOLUTE_PID_KI      10.0f
#define PITCH_GYRO_ABSOLUTE_PID_KD      150.0f
#define PITCH_GYRO_ABSOLUTE_PID_KF      0.0f
#define PITCH_GYRO_ABSOLUTE_PID_MAX_OUT 1000.0f
#define PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT 0.0f

#define INNER_YAW_GYRO_ABSOLUTE_PID_KP      30.0f  //30.0f
#define INNER_YAW_GYRO_ABSOLUTE_PID_KI      0.0f
#define INNER_YAW_GYRO_ABSOLUTE_PID_KD      0.0f
#define INNER_YAW_GYRO_ABSOLUTE_PID_KF      0.0f
#define INNER_YAW_GYRO_ABSOLUTE_PID_MAX_OUT 10.0f
#define INNER_YAW_GYRO_ABSOLUTE_PID_MAX_IOUT 0.0f

#define OUTER_YAW_GYRO_ABSOLUTE_PID_KP      40.0f
#define OUTER_YAW_GYRO_ABSOLUTE_PID_KI      0.0f //0.05f
#define OUTER_YAW_GYRO_ABSOLUTE_PID_KD      0.0f
#define OUTER_YAW_GYRO_ABSOLUTE_PID_KF      0.0f
#define OUTER_YAW_GYRO_ABSOLUTE_PID_MAX_OUT 10.0f
#define OUTER_YAW_GYRO_ABSOLUTE_PID_MAX_IOUT 1.0f
#define OUTER_YAW_GYRO_FB_K                0.0f

#define OUTER_YAW_GYRO_ABSOLUTE_MIN_PID_KP      10.0f
#define OUTER_YAW_GYRO_ABSOLUTE_MIN_PID_KI      0.0f
#define OUTER_YAW_GYRO_ABSOLUTE_MIN_PID_KD      0.0f
#define OUTER_YAW_GYRO_ABSOLUTE_MIN_PID_KF      0.0f
#define OUTER_YAW_GYRO_ABSOLUTE_MIN_PID_MAX_OUT 10.0f
#define OUTER_YAW_GYRO_ABSOLUTE_MIN_PID_MAX_IOUT 0.0f

/* =========================================================================
 * 五、编码器相对角 PID 参数
 * ========================================================================= */
#define PITCH_ENCODE_RELATIVE_PID_KP      26.0f
#define PITCH_ENCODE_RELATIVE_PID_KI      0.002f
#define PITCH_ENCODE_RELATIVE_PID_KD      0.0002f
#define PITCH_ENCODE_RELATIVE_PID_KF      0.0f
#define PITCH_ENCODE_RELATIVE_PID_MAX_OUT 10.0f
#define PITCH_ENCODE_RELATIVE_PID_MAX_IOUT 0.0f

#define INNER_YAW_ENCODE_RELATIVE_PID_KP      25.0f
#define INNER_YAW_ENCODE_RELATIVE_PID_KI      0.0f
#define INNER_YAW_ENCODE_RELATIVE_PID_KD      0.0f
#define INNER_YAW_ENCODE_RELATIVE_PID_KF      0.0f
#define INNER_YAW_ENCODE_RELATIVE_PID_MAX_OUT 10.0f
#define INNER_YAW_ENCODE_RELATIVE_PID_MAX_IOUT 0.0f

#define OUTER_YAW_ENCODE_RELATIVE_PID_KP      20.0f
#define OUTER_YAW_ENCODE_RELATIVE_PID_KI      1.0f
#define OUTER_YAW_ENCODE_RELATIVE_PID_KD      0.0f
#define OUTER_YAW_ENCODE_RELATIVE_PID_KF      0.0f
#define OUTER_YAW_ENCODE_RELATIVE_PID_MAX_OUT 10.0f
#define OUTER_YAW_ENCODE_RELATIVE_PID_MAX_IOUT 1.0f

/* =========================================================================
 * 六、Pitch DM4310 MIT 控制参数
 * ========================================================================= */
#define PITCH_4310_MIT_CONTROL_KP 15.0f
#define PITCH_4310_MIT_CONTROL_KD 1.4f

/* TD_MIT 跟踪微分器参数 */
#define PITCH_TD_W    40.0f
#define PITCH_TD_ZETA 1.0f

/* =========================================================================
 * 七、Pitch 轴机械模型参数（J/B/G/C 辨识结果）
 * ========================================================================= */
#define PITCH_J       0.01116936f //0.00836342f
#define PITCH_B       0.02214229f  //0.11827910f
#define PITCH_G       0.88508810f //0.87343722f
#define PITCH_Fc      0.14099603f
#define PITCH_COULOMB_SMOOTH 0.5f

#define PITCH_VEL_ERR_DAMP_K 0.0f

/* =========================================================================
 * 八、自动瞄准（AT）PID 参数
 * ========================================================================= */
#define AT1_PITCH_GYRO_ABSOLUTE_PID_KP      30.0f //3200 25
#define AT1_PITCH_GYRO_ABSOLUTE_PID_KI      20.f
#define AT1_PITCH_GYRO_ABSOLUTE_PID_KD      0.0f   //38.0f
#define AT1_PITCH_GYRO_ABSOLUTE_PID_MIT_KD  1.0f
#define AT1_PITCH_GYRO_ABSOLUTE_PID_KF      0.0f
#define AT1_PITCH_GYRO_ABSOLUTE_PID_MAX_OUT 10.0f
#define AT1_PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT 3.0f

#define AT1_INNER_YAW_GYRO_ABSOLUTE_PID_KP      20.0f
#define AT1_INNER_YAW_GYRO_ABSOLUTE_PID_KI      0.0f
#define AT1_INNER_YAW_GYRO_ABSOLUTE_PID_KD      0.0f
#define AT1_INNER_YAW_GYRO_ABSOLUTE_PID_KF      0.0f
#define AT1_INNER_YAW_GYRO_ABSOLUTE_PID_MAX_OUT 10.0f
#define AT1_INNER_YAW_GYRO_ABSOLUTE_PID_MAX_IOUT 0.0f

#define AT2_OUTER_YAW_GYRO_ABSOLUTE_PID_KP      30.0f
#define AT2_OUTER_YAW_GYRO_ABSOLUTE_PID_KI      0.05f
#define AT2_OUTER_YAW_GYRO_ABSOLUTE_PID_KD      0.0f
#define AT2_OUTER_YAW_GYRO_ABSOLUTE_PID_KF      0.0f
#define AT2_OUTER_YAW_GYRO_ABSOLUTE_PID_MAX_OUT 10.0f
#define AT2_OUTER_YAW_GYRO_ABSOLUTE_PID_MAX_IOUT 1.0f

/* =========================================================================
 * 九、前馈规划系数
 * ========================================================================= */
#define OUTER_YAW_K_PLAN_GYRO   0.9f    //0.9f
#define OUTER_YAW_K_PLAN_ACCEL  120.0f  //80.0f
#define INNER_YAW_K_PLAN_GYRO   0.0f

/* =========================================================================
 * 十、Outer Yaw TD（跟踪微分器）参数
 * ========================================================================= */


/* =========================================================================
 * 十一、积分分离区（I-Zone）
 * ========================================================================= */
#define INNER_YAW_ANGLE_PID_I_ZONE    0.1f
#define OUTER_YAW_ANGLE_PID_I_ZONE    0.08f
#define AT_INNER_YAW_ANGLE_PID_I_ZONE 0.10f
#define AT_OUTER_YAW_ANGLE_PID_I_ZONE 0.08f

/* =========================================================================
 * 十二、自瞄 I/D 范围
 * ========================================================================= */
#define AT_YAW_I_scope   100.0f
#define AT_YAW_D_scope   0.0f
#define AT_PITCH_I_scope 100.0f
#define AT_PITCH_D_scope 0.0f

/* =========================================================================
 * 十三、双 IMU Yaw 对齐 LPF 系数
 * ========================================================================= */
#define YAW_ALIGN_LPF_ALPHA 0.001f

/* =========================================================================
 * 十四、Pitch 绝对角机械限位
 * ========================================================================= */
#define PITCH_MAX_ABSOLUTE_ANGLE 0.5f
#define PITCH_MIN_ABSOLUTE_ANGLE (-0.31f)

/* =========================================================================
 * 十五、双 yaw 扫描与限幅参数（从 gimbal_task.c 局部宏迁出）
 * ========================================================================= */
#define GIMBAL_YAW_CURRENT_LIMIT   30000.0f
#define GIMBAL_PITCH_TORQUE_LIMIT  5.0f
#define BIG_YAW_SCAN_SPD           1.5f
#define BIG_YAW_SCAN_LOW_SPD       1.0f
#define BIG_YAW_RAMP_RATE          4.0f
#define P_SCAN_FREQ                1.0f
#define P_SCAN_LOW_FREQ            1.0f
#define P_RAMP_STEP                0.001f
#define P_CENTER_MOVE_SPD          0.002f
#define P_SCAN_CENTER              0.1f
#define P_SCAN_AMP                 0.35f
#define P_SCAN_HIGH_CENTER         (-0.3f)
#define P_SCAN_HIGH_AMP            0.2f

#endif /* GIMBAL_PARAMS_H */
