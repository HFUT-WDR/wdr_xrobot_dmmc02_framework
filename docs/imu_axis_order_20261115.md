# IMU 姿态量与三分量轴向统一（2026-11-15）

本文记录 WDR-Xrobot-org 组织内所有 IMU 相关模块的统一轴向契约。适用于 `AttitudeEkf`、
`ImuEkfFusion`、`ESKF`、`DmImu`、`BMI088`，以及按 `wdr_imu::Attitude` 消费姿态的
`Gimbal`、`Chassis`、`HostData`。

## 契约

| 量 | 顺序 | 单位 | 说明 |
| --- | --- | --- | --- |
| 姿态角 | **roll, pitch, yaw** | rad | 成员声明顺序即 Topic 载荷布局 |
| 角速度 `gyro[3]` | roll(x), pitch(y), yaw(z) | rad/s | 已减零偏、已施加安装变换 |
| 线加速度 `accel[3]` | roll(x), pitch(y), yaw(z) | m/s² | 含重力 |
| 运动加速度 `motion_accel_b/n[3]` | roll(x), pitch(y), yaw(z) | m/s² | 去重力，仅输出 |
| 四元数 | w, x, y, z | — | Hamilton 乘法，机体系到导航系 |
| `DmMeasurement::euler_deg[3]` | roll, pitch, yaw | ° | 设备原始单位，未换算 |

坐标系为右手系，符合右手定则；静止且 Z 朝上时加速度为 `[0,0,+g]`。

## 三分量数组取用

使用 `wdr_imu::IMU_GYRO_INDEX_ROLL/PITCH/YAW`（值为 0/1/2），不写 `gyro[0]` 之类的魔数。
姿态角按字段名取用（`attitude.yaw`），不依赖声明顺序。

## 角加速度

当前**没有任何模块输出角加速度**。`motion_accel_b/n` 是线加速度，不是角加速度。
后续新增该量时同样按 roll,pitch,yaw 排列，并在此处更新契约。

## 实施边界

- `Attitude` 成员换序改变了 Topic 载荷布局：`yaw` 与 `roll` 的偏移互换。仓内消费者已按字段名
  或轴向常量取用；**仓外按旧布局解析 `board_attitude`/老 `dm_attitude` 的脚本必须同步修改**。
- 冻结的 DM 参考源 `AttitudeEkf/tests/reference/dm_imu_l1_attitude.c` 保持原有 yaw,pitch,roll
  顺序不改。生产实现与参考源的逐位对照在 `tests/test_dm_oracle.c` 中按 `{2,1,0}` 换位比较，
  数值必须完全一致，只允许下标换位。
- `CMD::GimbalCMD` 与 `HostData/host_protocol.hpp` 的云台目标字段（yaw, pit, rol）属于控制指令
  与上位机线格式，与 `27_Sentry_Host` 的上位机软件共用契约，不属于 IMU 姿态输出，本次未改。
- `xrobot-org/MadgwickAHRS` 为第三方模块，保留原样：其 `ahrs_euler` Topic 数据本身已是
  roll,pitch,yaw 顺序（`LibXR::EulerAngle` 为 roll,pitch,yaw；`ToEulerAngle()` 返回该序），
  仅 `show` 命令打印成 `pitch, roll, yaw`。这是上游行为，本地不分叉。

## 模块清单与改动

| 仓库 | 改动 |
| --- | --- |
| `WDR-Xrobot-org/AttitudeEkf` | `Attitude` 换序；C 核 `euler_rad/deg` 换序并加下标常量；`show` 改打印顺序；新增 static_assert 与轴向断言 |
| `WDR-Xrobot-org/ImuEkfFusion` | 同上（与 AttitudeEkf 同源同布局） |
| `WDR-Xrobot-org/ESKF` | `Attitude` 换序；`show` 改打印顺序；新增 static_assert 与轴向断言 |
| `WDR-Xrobot-org/DmImu` | 仅注释：`DmMeasurement` 三分量顺序说明（原本已符合） |
| `WDR-Xrobot-org/BMI088` | 仅注释/README：体轴 x/y/z 与 roll/pitch/yaw 的对应（原本已符合） |
| `WDR-Xrobot-org/Gimbal` | `CheckFaults` 的三分量取轴改用 `IMU_GYRO_INDEX_*`，取值不变 |
| `WDR-Xrobot-org/Chassis` | 仅 README |
| `WDR-Xrobot-org/HostData` | 仅注释：`AimState.gyro/accel` 数组顺序 |
| `WDR-Xrobot-org/CMD`、`DR16`、`I6x`、`Referee` | 无改动（控制指令字段，非 IMU 输出） |
| `WDR-Xrobot-org/HostLink`、`27_Sentry_Host` | 无改动（线协议已为 roll,pitch,yaw） |
| 其余仓库 | 无 IMU 姿态输出 |

## 验证

主机回归：

```sh
python3 Modules/WDR-Xrobot-org/AttitudeEkf/tests/run_regression.py
python3 Modules/WDR-Xrobot-org/AttitudeEkf/tests/run_regression.py   # ImuEkfFusion 同名脚本
python3 Modules/WDR-Xrobot-org/DmImu/tests/run_tests.py
python3 tests/board_support/run_tests.py
python3 tests/imu_contract/check_layout.py
```

`check_layout.py` 分别编译三份 `imu_types.hpp`，比对 `Attitude` 的 `sizeof` 与各成员
`offsetof`，并断言 roll < pitch < yaw。这是目前唯一能发现多份重复类型定义不同步的手段。

构建：`cmake --preset Debug`/`Release` 加 `cmake --build --preset ... -j4`。

**未验证**：未烧录、未板测。实机轴向符号、外部上位机采集脚本的列顺序、仓外消费者的载荷
兼容性均未验证。
