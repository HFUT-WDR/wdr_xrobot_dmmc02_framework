# 输入链路在线与 PC 命令时效（2026-11-04）

## 实现

CMD::Data 只保留 input_online 作为遥控输入链路状态，删除 chassis_online、gimbal_online、controls_valid。
I6x、DR16、裁判图传输入同步迁移。遥控源选择、START/LOST、整体失联归零沿用原规则。
I6x 不新增单帧 failsafe 撤销：解码器仍清摇杆，开火沿用拨杆电平；连续3帧或超过30ms无有效帧离线。
这也意味着短暂单帧失控标志不会单独撤销 PC 目标，符合本轮确认的契约。

HostData 不再按数值是否全零判断在线，每个 Topic 回调直接调用 FeedAI(Channel, Data)。
云台完整传递三个角度及其一阶、二阶导数。旧 HostCMD 汇总入口与未使用的 GetAIGimbalStatus 删除。
CMD 分别保存三个通道的接收时间和是否收到过；未收到或达到100ms时对应输出全零，不回退遥控目标。
FeedRC/FeedAI 驱动时效检查，没有额外定时线程；I6x 状态提交至少每5ms执行一次。
机构模式、电机健康和失能输出仍由后续机构模块实现，本轮没有新增 valid 或机构状态。

## 验证

- 主机 ASan/UBSan：1568组冻结解码对照；真实 EventBinder、CMD、HostData 回调；RC失联、来源切换、单帧行为、PC计时回绕。
- 零角度保留导数；底盘未收到归零；云台持续收包不刷新底盘或开火时效；100ms边界归零；RC失联覆盖全部PC输出。
- Debug/Release 构建，10个构造接口与两个完整生成配置编译；默认ELF外接模块隔离通过。
- 独立目录选择 i6x YAML 完整链接通过。主工作区默认仍为板级配置。
- J-Link 烧录默认 Debug 固件并 Flash 校验通过，普通复位，没有修改选项字节。
- SWD读取：550000000 Hz、6任务、空闲堆/最低空闲堆65200字节；Tick从532021增至534028；CFSR/HFSR均为0；退出调试时恢复运行。

烧录镜像没有启用 CMD、HostData 或 I6x，因此板测仅证明板级启动；新输入链路尚无真实遥控或PC硬件收发证据。
当前只枚举到 J-Link，没有日志CDC口，未执行USB终端回归。构建或主机测试不等于机构闭环验收。

更新依赖时发现 I6x 的 THIRD_PARTY_NOTICES.md 有本地改动，已原样保留，未纳入组件发布。
详细提交与镜像散列见 validation/input_online_20261104.json。
