最新接口与验证见 [CMD迁移说明](cmd_migration_20261117.md)。

# 独立三摩擦发射产品

## 配置与源码

- 产品配置：`User/three_fric.yaml`。
- 机构模块源仓库：`WDR-Xrobot-org/ThreeFricLauncher`，内置1ms任务，同拍Update/Send。
- 下载目录：`Modules/WDR-Xrobot-org/ThreeFricLauncher*`。不要在这里改源码。
- 模块索引：ModuleIndex子模块；xrobot.lock由工具生成，锁定WDR远程仓库提交。

只装配发射产品，没有底盘、云台、裁判或上位机实例。板级外设与终端仍保留；板载IMU业务模块只在另一份xrobot.yaml配置内。

## 接线与操作

CAN1、1Mbps：拨弹ID1（反馈0x201）、左轮ID2、右轮ID3、上轮ID4。四台均为M2006/C610。
摩擦轮40mm、直驱，拨弹36:1；三轮初始目标21m/s为轮缘速度，不代表弹丸速度。方向目前沿用原工程三个轮和拨弹均不反向，实际机构方向尚未验证。

UART5接I6x/SBUS，运行时由I6x设置为100000、8E2。WDR当前开关标注如下：

1. 上电或遥控恢复后不因当前拨杆位置起转。
2. SW1拨到下位（raw=-1）的边沿切换摩擦轮开关。
3. 摩擦轮已开启、三轮到速后，SW1上位（raw=+1）且SW0下位（raw=-1）允许拨弹。
4. 松开拨弹请求，摩擦轮仍转，拨弹零速保持。
5. 再次SW1下拨关闭：四电机各自主动刹停，各自降至阈值后锁存零电流。停止后手拨不会再次制动。
6. 任一电机掉线撤销机构开启：掉线电机直接零电流，其余在线电机刹停。恢复须重新操作。
7. CMD只采用I6x的online判定，离线清除摩擦轮和拨弹请求。Launcher指令Topic超过30ms无更新则停机。AUTO开轮由CMD的auto_open_fric配置，默认true，遥控离线时AUTO同样停止。

旧框架UP/DOWN宏对raw值的称呼与WDR相反，以上使用WDR标注；实际取值对应保持不变。

## 恢复、生成与构建

本交付含两个有本地dev提交的独立源仓库。搬动目录时必须连同其`.git`目录保留，不能只复制Modules下载目录。

```sh
xrobot setup --frozen --offline
xrobot gen -c User/three_fric.yaml
cmake --preset Debug
cmake --build --preset Debug -j4
cmake --preset Release
cmake --build --preset Release -j4
```

换机器首次恢复已有远程依赖时使用`xrobot setup --frozen`，需要组织私有仓库权限。
修改对应远程模块仓库并提交推送后，运行xrobot setup --update WDR-Xrobot-org/ThreeFricLauncher，工具更新锁文件及下载目录，再重新生成/构建。不要手改锁文件或生成头。

## 软件验证

```sh
python3 tests/three_fric/run_tests.py
```

测试使用真实LibXR、CMD、I6x事件和RMMotor，替代CAN/时钟并拦截线程启动，显式单步真实调度器。覆盖三轮到速、到速锁存、各电机制动/释放、掉线过期反馈、掉线恢复、遥控超时/失控、两拍之间瞬时掉线、OP/AUTO门控、卡弹边界、四槽组帧、反向符号、与冻结源PID逐拍对比和IIR数值。它不验证RTOS实际调度抖动。

`launcher_status`为状态Topic，字段解释见机构模块README；跨任务使用订阅副本，不直接并发读取GetState内部引用。未添加绕过遥控许可的终端开火命令。

运行许可依赖任务持续推进，不宣称调度任务永久挂起时仍能撤销电流；硬件看门狗不在本次范围。构建、板上通信和实际发射是分层证据，当前未烧录或进行机构验证。

## 2026-11-16 烧录检查

已按后续授权烧录Debug并校验通过，启动与1ms任务推进正常；遥控和四电机反馈尚未观察到，调试器随后从USB断开。完整范围、旧固件备份和未验证项见[硬件记录](hardware_20261116/README.md)。

2026-11-17：拨弹反馈改用ControlAlgorithms的SecondOrderLowPass(1000Hz, 27.951037Hz)，零历史启动、停止时Reset。该截止频率匹配旧递推的-3dB带宽，但瞬态响应不同；本次软件验证不覆盖此前已烧录的旧版本。

2026-11-17 dt适配：配置增加trigger_filter_sample_hz（默认示例1000Hz）；滤波调用传入实际dt，公共库不传dt的入口使用构造配置。调度任务用微秒时钟测相邻Step周期，首拍按1ms名义调度周期。PID仍按原逐拍离散增益、卡弹仍按原拍数计时，不因滤波dt适配顺带改变。此次未烧录。
