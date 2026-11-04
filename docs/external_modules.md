# 基础设备与 i6x 模块

主工程使用 YunMo-yanyu 名下的私有组件仓库。CMD、Motor、RMMotor、DMMotor、DR16、Referee、
HostData、I6x 均通过 XRobot 管理；不再从 QDU 仓库解析构建依赖。LibXR、BMI088、BuzzerAlarm、
SharedTopic、SharedTopicClient 继续使用官方来源。

模块索引由 `ModuleIndex` Git 子模块提供，依赖请求在 `Modules/modules.yaml`，实际提交固定在
`xrobot.lock`。需要对主工程、索引及各组件都有读取权限的 GitHub 账号。

## i6x 的 YAML 用法

`User/xrobot_i6x.yaml` 是包含已验收板级功能、CMD 和 I6x 的独立配置。其外接输入为：

```yaml
- module: YunMo-yanyu/I6x
  id: i6x
  args:
  - uart: uart5
  - cmd: cmd
  - topic_name: "i6x_input"
```

前面必须先创建 CMD。默认选中仍为 `User/xrobot.yaml`，其中没有 CMD 或 I6x，不会启动外接接收。
确认接线后才显式执行 `xrobot gen -c User/xrobot_i6x.yaml` 并构建、烧录。
恢复板级配置使用 `xrobot gen -c User/xrobot.yaml`。本轮不烧录。

```sh
xrobot module show YunMo-yanyu/I6x
xrobot instance -c User/<配置>.yaml add YunMo-yanyu/I6x --id i6x
xrobot instance -c User/<配置>.yaml set i6x args.uart uart5
xrobot instance -c User/<配置>.yaml set i6x args.cmd cmd
```

无需再手写 I6x 构造或给 app_main 增加专门的 i6x 启动代码。CMD 的独立 i6x 输入槽与策略已经
提交到个人 CMD 仓库，不再由构建脚本打补丁。旧补丁仅在 `docs/archive` 留存迁移记录。

## 接口与语义

MC02 原理图第5页：P6第1脚信号经Q1反相进入PD2/UART5_RX，第2脚5V，第3脚GND。
使用专用P6接口时不再启用软件RX反相。原厂使用手册表格的PC02与原理图不符，采用原理图与
实机26_sentry一致的PD2。接收机必须输出SBUS，不能拿iBUS/PWM代替。

I6x模块复用LibXR UART的DMA字节队列，在任务内组25字节SBUS帧，配置100000、8E2。
HAL侧8数据位加偶校验应为9B字长；锁定版LibXR驱动会正确转换。

通道与拨杆沿用26_sentry：六通道映射到±660，交换左摇杆通道，四拨杆保持+1/0/-1。
+1对应实际下拨、-1对应实际上拨。

| 输入 | 意图 |
| --- | --- |
| s[2] | -1云台无力、0手动、+1上位机 |
| s[3] | -1底盘无力；+1时按s[2]选择手动／上位机 |
| s[0] | +1开火意图 |
| s[1]进入+1 | 摩擦轮切换序号增加一次 |
| s[1]=-1且s[3]=+1 | 上位机发射许可 |
| ch[4] | 小陀螺旋钮归一化到[0,1] |

完整输入和意图发布到`i6x_input`。CMD保留独立底盘／云台来源，无力档不回退到上位机。
上位机数据超过100ms时，新i6x路径归零；原HostData的零目标／有效位契约仍需机构接入时复核。
单个失控／丢帧标志帧清摇杆和开火许可，不刷新有效接收时间；连续3帧失控或严格超过30ms无
有效帧时离线。重新上线首帧不触发摩擦轮切换。

原26_sentry还有20ms上线稳定窗口，本模块尚未迁移，不能宣称完整实车行为等价。
CMD的零目标不等于机构无力，后续机构任务必须读取无力意图实施零输出。PID、到速／热量许可、
校准、PASSING_MODE等机构业务没有在本轮接入。

## 其他基础模块

`config/templates/external_devices.yaml.in` 提供待填写配置。电机型号、方向、ID和接线没有
确认的实例保留null，不加入默认启动。DR16和I6x通常只启用其中一个，接收UART不能共用。
裁判模板的`cmd: nullptr`表示不将图传键鼠混入遥控源。

- RMMotor构造只登记CAN反馈；同组所有已登记电机提交一轮命令后才发帧，Relax不代表立即发帧。
  MODE_CURRENT复用MotorCmd.velocity表示归一化电流，不是rad/s；没有单独的6020电压模式配置。
- DMMotor调用Enable才发使能帧；反馈ID为0x10+can_id。上游过温分支先失能后仍发控制帧的
  行为尚未改写，不可视为完整机构停机保护。
- Referee为源版本所述的2025协议，未来赛季协议需核对。其接收解析有CRC与长度检查。
- HostData只创建接收Topic并汇总数据；SharedTopic接收、SharedTopicClient发送。
  可绑定USB第二路`usb_otg_hs_cdc2`，日志继续使用第一路。协议为LibXR Topic包格式，非任意裸结构体。

## 生成、编译和测试

```sh
xrobot setup --frozen --offline
cmake --preset Debug
cmake --build --preset Debug -j 4
cmake --preset Release
cmake --build --preset Release -j 4
python3 tests/i6x/run_tests.py
python3 tests/external_modules/run_tests.py --preset Debug
python3 tests/external_modules/run_tests.py --preset Release
```

普通构建自动通过官方`xrobot check-module`编译9个具体模块／依赖的构造接口；还在临时工程
分别生成、编译基础设备组合配置和完整i6x YAML，不替换主工程的已选配置。
所有检查对象不链接进默认固件。主机测试对照本机26_sentry冻结解码源，并覆盖失控、超时和CMD
路由；这不替代真实接收机、电机或机构硬件验收。
