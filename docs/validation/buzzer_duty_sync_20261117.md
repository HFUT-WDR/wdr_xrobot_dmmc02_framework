# 统一蜂鸣器占空比同步

范围：同步 BuzzerAlarm 和 BMI088 的 dev 提交、全部蜂鸣器实例 duty 配置和 xrobot.lock。占空比统一为 0.5（50%），包含开机、校准、设备告警和致命报警，保留频率与时长配置。其他模块和 LibXR 不主动更新；不烧录、不执行机构动作。

验证：通过 XRobot 更新及离线锁定复现，构建原产品的 Debug/Release；构建失败则先定位兼容问题再决定必要修改。完成后提交并推送当前 dev 分支。

主框架原 BMI088 版本较旧，此次同时由工具补入其必需的 ControlAlgorithms 依赖，并迁移温控 PID 参数字段；原配置的比例、积分、微分均保持为零。其他模块提交与 LibXR 保持原锁定版本。

验证结果：xrobot setup --frozen --offline 通过，默认板级配置 Debug、Release 均构建通过。未烧录，未验证实板音量。
