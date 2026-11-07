# 具名常量复用与参数整理（2026-11-07）

本轮依据数值实际含义整理16个自维护模块。第三方vendor目录、原车冻结对照源和测试中的独立期望值未修改。

## 定义归属

- RMMotor继续持有MOTOR_ENC_RES、MOTOR_CUR_RES；DualYaw的换算、取模、校准和零位校验共同复用编码器分辨率。
- RMMotor统一声明GM6020电压模式协议上限，DualYaw和Gimbal引用；Motor提供两个具体驱动共用的rpm换算单位。
- DualYaw定义大小Yaw角色索引，反馈、控制、目标和校准输出统一引用。
- CMD提供PC有效期与轻量遥控输入尺度头；Gimbal、I6x、DR16和Referee按已有依赖复用。
- ControlAlgorithms声明原车PID参数的1ms解释基准；该值与Gimbal任务标称周期分别命名，避免误改离散增益语义。
- ImuFusion/dm_imu_protocol.hpp统一保存请求/连续流帧长、子包长度、头尾、CRC、字段量程与时钟换算；采集、会话和组帧器共用。
- DMMotor统一声明控制命令码、反馈ID偏移和MIT字段位宽；协议数值和打包顺序不变。
- BoardAttitude复用LibXR标准重力，先转换到float再参与乘法；板载融合复用已有RADIANS_TO_DEGREES。
- PowerControl集中定义超电定点尺度和编码边界；不同数值判据按RLS、分配和二次方程分别命名。

## 新开放的机器人参数

Pitch增加积分区间、复位阈值、积分限幅、换向衰减和扫描阻尼；PowerControl增加输入功率分段调节参数及预留功率。原值全部保留为默认值，并同步到config/templates/gimbal.yaml和chassis.yaml。新增参数的必要范围检查一并补齐。

## 验证范围

六组既有主机回归（IMU、云台、底盘、I6x、公共算法、功率控制）、Debug/Release编译、云台+底盘+功率+联锁完整链接，以及18快照/22模块冻结离线恢复通过。
冻结算法数值对照用于检查默认行为；未把独立测试期望值改成生产宏，避免同改同错。
组织模块与BSP CI结果另在对应Actions运行中核对；本次未烧录、未验证实际设备收发或机构闭环。

## 云端最终结果

16个修改模块的Module CI全部通过；[BSP检查及Debug/Release构建](https://github.com/HFUT-WDR/wdr_xrobot_dmmc02_framework/actions/runs/42349797785)全部通过。
对应提交、模块运行链接及本地验证范围见[验证记录](validation/constants_refactor_20261107.json)。本节为文档补充，未修改已验证源码。
