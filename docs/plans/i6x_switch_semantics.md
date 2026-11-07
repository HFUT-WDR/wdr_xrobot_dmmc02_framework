# I6x 拨杆语义对齐修改规划

状态：仅规划，尚未实施。交由 DeepSeek 完成。

工程：`/home/yunmo/WorkingSpace/STM32_Project/Xrobot/wdr_xrobot_dmmc02_framework`

参考：`/home/yunmo/WorkingSpace/STM32_Project/rm_26_sentry/26-sentry_-dm`

## 目标与边界

保留 I6x 当前解码值与实际方向的对应：上拨 +1（UP）、中位 0（MID）、下拨 -1（DOWN）。原工程沿用 DT7 的 switch_is_up/down 宏，I6x 分支宏值与物理方向相反；应按实际业务语义修正判断和事件绑定，不再次翻转解码或事件名称。

只修改遥控映射、对应测试和文档。不改 PID、总线、ID、电机方向、IMU 安装、CALI 算法或存储；不改 DT7/DR16，不修改参考工程，不增加旧导航/过点模式或上位机通信。

## 最终操作映射

| 实际操作 | 行为 |
| --- | --- |
| SW2 上位 | 云台 RELAX，通过现有许可停止底盘和发射 |
| SW2 中位 | CMD OP，云台绝对角遥控 |
| SW2 下位 | CMD AUTO，云台 AUTO |
| SW2 中位＋SW3 下位 | 底盘 NO_FOLLOW_YAW，使用遥控速度 |
| SW2 下位＋SW3 下位 | 底盘 NO_FOLLOW_YAW，使用现有 AUTO 命令通道 |
| 离开上述底盘组合 | 底盘 RELAX |
| SW1 切入下位 | 摩擦轮开启请求切换，使用边沿事件 |
| SW0 下位 | OP 开火电平 |

无有效 AUTO 目标时保留当前无力行为。

当前显式 CALI 快捷组合 SW1_DOWN＋SW3_UP 保留，不机械互换上下，不恢复原车摇杆手势。保留校准请求覆盖普通模式请求的顺序，以及完成后保持 RELAX、退出组合再恢复模式的行为。

## 修改位置

### I6x 模块

- `Modules/HFUT-WDR/I6x/I6x.hpp`：MakeCommand 的开火条件从 switches[0] == 1 改为 -1。摇杆交换、符号、量程、旋钮和失控阈值不变。
- `Modules/HFUT-WDR/I6x/i6x_events.hpp`：保留单拨杆事件含义及编号；追加 SW2_MID_SW3_DOWN_ENTER/EXIT、SW2_DOWN_SW3_DOWN_ENTER/EXIT 及对应 ChordHold 事件，使用未占用编号。
- 不把旧 UP 组合名称偷偷改为 DOWN 条件；保留旧事件原有含义，避免改变已有事件编号或发生碰撞。
- 增加组合后同步检查数组、编号计算与循环。保留组合先 EXIT 后 ENTER、随后发保持事件，以及离线恢复的现有边沿/保持语义。

### 主工程配置

同时修改：

- `User/RobotConfig/sentry_remote.yaml`
- `config/templates/chassis.yaml`
- `config/templates/gimbal.yaml`

AUTO 来源/模式改为 SW2_DOWN，RELAX 改为 SW2_UP，摩擦轮切换改为 SW1_DOWN。底盘使能与退出改用新增 DOWN 组合。保留现有底盘、发射许可联锁及 CALI 组合。

若发现其他 switch_is_up/down 遗留调用，逐处结合接收器类型和实际业务判断，不全局互换宏。

## 验证要求

更新 `tests/i6x/i6x_test.cpp`，保留原有解码对照，覆盖：

- 同一 SBUS 输入的通道和拨杆解码不变，+1/0/-1 对应 UP/MID/DOWN。
- SW2 上位无力、中位 OP、下位 AUTO。
- 两种新底盘组合使能、退出停止，旧 EXIT 不覆盖新组合请求。
- SW1 下位进入沿仅切换一次，保持不重复，恢复首帧不产生假进入沿。
- SW0 仅下位产生 OP 开火。
- 断连与恢复行为不变；CALI 组合、覆盖顺序及退出行为不变。
- 检查实际 YAML 绑定，不能只修改测试中的手写绑定。

运行现有回归：

```sh
python tests/i6x/run_tests.py
python tests/gimbal/run_tests.py
python tests/launcher/run_tests.py
```

模块版本同步后直接验证完整整车配置：

```sh
xrobot gen -c User/RobotConfig/sentry_remote.yaml
cmake --preset Debug
cmake --build --preset Debug -j4
cmake --preset Release
cmake --build --preset Release -j4
python tests/external_modules/run_tests.py --preset Debug
python tests/external_modules/run_tests.py --preset Release
python tools/check_module_snapshots.py
```

不烧录、不驱动设备，不将构建通过视为实车验证通过。

## 文档、版本与交付

更新 `docs/sentry_remote.md` 操作表及 I6x、Launcher 等受影响模块的说明，明确 UP/DOWN 表示物理方向。遵守 AGENTS.md，保留他人改动，不做无关重构，不删除 DualYaw 仓库。

模块为独立 Git 仓库：先提交模块修改，再按现有协作方式发布模块版本，通过 XRobot 工具更新主工程锁文件及对应离线 bundle，不手改锁文件。提交/推送前检查实际 diff。

交付修改清单、最终操作映射、回归与构建结果、模块和框架提交号。

BMI088 温控 PID 尚未对齐是另一项已知问题，不混入本轮；遥控映射修复不代表整车已完全对齐。
