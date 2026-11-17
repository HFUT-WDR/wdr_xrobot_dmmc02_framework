# 对称三摩擦配置简化

三轮共用 fric_radius_m=0.020m（原直径0.040m）、一组 fric_pid 参数，三个控制器独立保留状态。firc_motor_ratio 为三轮共用的单一减速比（电机转速/轮速，直驱为1）；fric_target_speed 保留各轮轮缘速度，单位仍为 m/s，目标21不变。方向继续配置在各 RMMotor 的 reverse 中。

换算由 rpm × PI × diameter / (60 × ratio) 改为 rpm × TWO_PI × radius / (60 × ratio)，物理含义一致。速度状态字段同步改为 fric_speed/fric_target_speed，单位仍为 m/s；Topic 消费者需同步更新。拨弹减速比统一命名 trigger_motor_ratio，到速误差命名 fric_ready_speed_tolerance，制动释放阈值命名 fric_release_speed。

XRobot 实际生成、三项主机测试及 Debug/Release 构建均通过。参数结构与原三组参数格式不兼容，调用者需按新 README 迁移；当前工程与模块测试已同步。尚未提交、推送、更新依赖锁定或烧录。

后续归档说明：实现阶段补丁已移到忽略目录 build/publication_backups/pid_review_patches；最终模块源码以 pid_unification_20261117/versions.md 列出的提交为准。
