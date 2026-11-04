# WDR MC02 开发约定

本工程采用 QDU-Robomaster 的 C++20 代码规范：
https://qdu-robomaster.github.io/电控组/code-standard

- 普通变量、全局变量、自由函数采用 lower_case。
- 类成员采用 lower_case_，类、结构体和枚举类型采用 CamelCase。
- 成员方法采用 CamelCase；常量、枚举值、宏采用 UPPER_CASE。
- 格式以项目 .clang-format 为准：Google 基础、90 列、Allman 大括号。
- 标准库、HAL/CMSIS/RTOS、LibXR、引入模块和生成器规定的名称保留原样；不批量重命名外部代码。
- CubeMX 文件只修改 USER CODE 区，硬件配置同步到 .ioc。LibXR 生成入口和 XRobot 生成头通过工具更新。
- 电机型号、总线、ID、方向、PID、几何参数放在机器人 YAML；模块实现可复用功能。
- 新增及维护的自研代码注释使用中文；第三方原有注释和工具生成内容保留。
- 注释说明单位、坐标系、缓冲生命周期、硬件限制及超时行为，不复述代码。
- 高频日志必须有频率限制；调试优先观察输入、状态和输出。
- 保留他人未提交改动；不进行无关格式化。命名/格式变更与控制行为变更分开。
- 构建使用 cmake --preset Debug/Release 和 cmake --build --preset Debug/Release -j4。
- 构建、烧录校验、设备收发和机构闭环是不同验证层次，记录范围及未验证项。
- 当前为板级基座阶段；新增电机模块前明确初始化是否发送、控制模式及失控行为。

## VS Code 构建与烧录同步

修改工程名、产物路径、CMake 预设、工具链、编译标准、调试器或烧录参数时，同步检查 `.vscode/tasks.json`、`launch.json`、`settings.json`、`c_cpp_properties.json` 和 README。编译任务先配置再构建，烧录任务先构建再校验、复位。普通烧录禁止附带修改选项字节。
