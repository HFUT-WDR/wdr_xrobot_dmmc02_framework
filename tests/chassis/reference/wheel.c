#include <math.h>
typedef float fp32;
#define MOTOR_DISTANCE_TO_CENTER .18951f
void reference_wheel(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set,
                                                  fp32 wheel_speed[4])
{
    const float k = 0.7071067811865475f; // sin(45) 或 cos(45)
    // 标准机器人坐标系全向轮逆解算矩阵 (X前, Y左, Z逆时针)

    wheel_speed[0] = ( vy_set + vx_set) * k + MOTOR_DISTANCE_TO_CENTER * wz_set; // 0: 右前轮
    wheel_speed[1] = ( vy_set - vx_set) * k + MOTOR_DISTANCE_TO_CENTER * wz_set; // 1: 左前轮
    wheel_speed[2] = (-vy_set - vx_set) * k + MOTOR_DISTANCE_TO_CENTER * wz_set; // 2: 左后轮
    wheel_speed[3] = (-vy_set + vx_set) * k + MOTOR_DISTANCE_TO_CENTER * wz_set; // 3: 右后轮
}

