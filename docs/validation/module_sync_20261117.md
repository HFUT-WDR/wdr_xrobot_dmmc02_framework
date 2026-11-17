# 模块同步与软件验证（2026-11-17）

本工程与27_sentry_work合并公共模块修改，保持独立三摩擦配置，不新增底盘或云台实例。共享模块版本由xrobot.lock锁定，ModuleIndex及GitHub源使用HTTPS。

- CMD统一open_fric/is_fire，AUTO默认开轮且可配置，遥控离线由CMD清命令。
- 三摩擦Send迁移到MotorCmd.normalized_current；四电机、速度单位m/s、对称几何与共享PID配置保持原行为。
- ControlAlgorithms合并LibXR PID适配与实测dt优先策略，保留可传dt的公共滤波器。
- ESKF独立数值测试补齐真实LibXR头文件依赖，原CI工作流保持不变。

验证：真实LibXR主机测试4/4通过；真实Gimbal/HostData/DR16调用链编译通过；本工程Debug、Release均构建通过。setup解析已核对远端的WDR最新提交，frozen/offline复现通过。未烧录、未发送实体控制指令、未执行机构闭环。

LibXR保持原版5161373dbe6d9d9fb1904d90e0ebd4344a44047a，UART补丁已在27_sentry_work的docs/validation/patches/libxr_uart_unproven_20261117.patch归档，不自动应用。
