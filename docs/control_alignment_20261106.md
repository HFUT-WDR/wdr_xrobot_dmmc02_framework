# QDU组织与26_sentry控制对齐

本次按确认方案执行，替代此前文档中关于独立来源、Topic健康回调和最低损耗预留的描述。

## 命令与失控

CMD全局OP/AUTO，不再提供各机构来源选择。OP使用遥控；AUTO各通道AI消息在100ms内有效，失效输出零命令，保持AUTO，不回退遥控。AI开火不与遥控扳机相与，但遥控必须在线；遥控离线所有命令归零并触发LOST_CTRL。上下线均使底盘RELAX，恢复连接不自动启动，来源选择不隐式重置。

底盘命令、姿态、机械角仅通过常驻ASyncSubscriber读取；无新消息沿用旧值。停止使用模式事件，不再通过机械角消息传递STOP。移除命令与角度停更门控，保留算法数值检查。任一电机反馈超过10ms失效时整底盘无力，无三轮降级。RELAX调用各轮Relax，当前RMMotor实际为零电流，不是物理断电。

## 控制与模块

Chassis<Omni>由YAML组装，Omni对应26_sentry控制实现，尚未实现其他轮系。运动学、模式、原车参数保留；公共PID改进不回退旧C PID。每拍时间戳计算dt，首拍使用2ms；PID、平移滤波、斜坡、轮缘加速度共用实际dt。PID每拍增益仅按标称2ms转换一次。

PowerControl独立XRobot模块，构造不发送、无线程；底盘依次调用Update、Calculate、Send。模块不拥有电机输出权。超电适配也在该模块内；电机失控保护由底盘负责。

## 功率算法

直接按原车误差/功率混合权重分配总预算，删除最低损耗预留。非正功轮保持PID请求、不抵扣正功预算。无实根或重根取顶点，有实根沿用原车求根分支，只限制协议电流范围，不额外限制在零至原请求区间。输出改变后通过公共PID回算积分。

保留40/20百分比、60/50J、原能量PD、30W下限、85%/95%过渡和目标减5W。超电离线回退裁判并冻结RLS；裁判离线使用有效上限或配置保守上限；两路离线降额。RLS初值、协方差、遗忘因子、范围、15W门槛与更新公式参考26_sentry；按控制周期更新，包括同一仍新鲜超电快照，不再添加样本去重或停机冻结。超电离线冻结是明确保留差异。

## 模块来源与验证边界

CMD、Chassis及新PowerControl以主工程module_sources中的Git bundle交付本地提交快照，项目本地源优先于公共索引；未发布远端。原私有索引未修改，CMD/Chassis原始远端保留为upstream。xrobot.lock由XRobot工具解析同步。

冻结参考位于tests/power/reference，保留源SHA256；分配尾段及RLS更新函数原样提取，外层仅适配主机数据。主机测试不证明实际功率、控制方向、制动效果、RTOS截止时间或栈余量。本次不烧录、不运行机构，默认固件保持板级配置。

## 验证入口

- `python3 tests/chassis/run_tests.py`：轮系冻结对照、实际dt、真实任务/电机组帧、断连/恢复、缺轮及数值故障。
- `python3 tests/i6x/run_tests.py`：全局来源、独立AI开火、各通道时效、断连和恢复事件。
- `python3 tests/power/run_tests.py`：10000组原车分配对照、2000组RLS模型序列、能量回退及超电协议。
- `python3 tests/control_algorithms/run_tests.py`、`python3 tests/board_support/run_tests.py`。
- `python3 tools/check_module_snapshots.py`：临时恢复三个改造模块，19模块冻结离线解析。
- `python3 tests/chassis/run_firmware_link.py`：临时完整组合链接；日志位于build/validation/chassis_full_link.log。
- Debug/Release配置、构建，以及`tests/external_modules/run_tests.py --preset Debug/Release`的默认固件隔离检查。

最终验证记录见[验证清单](validation/control_alignment_20261106.json)。完整组合DTCM静态占用123120/131072字节，不能据此推断动态任务堆栈余量。

## 后续板级烧录测试

默认Debug固件通过DAPLink烧录、Flash校验及复位，未修改选项字节。550MHz、7个任务，空闲/历史最低堆56368字节。两次调试采样tick从34148增长到35154，姿态序号从32857增长到33863；队列丢帧、未配对计数、SPI/ADC错误及CPU故障寄存器均为零。结束时确认目标运行，关闭调试服务。

这仅验证板级与姿态采样持续运行，底盘/功率模块未在此固件中实例化。未验证实际机构闭环、姿态方向/动态精度或USB终端。见[烧录与运行记录](validation/control_alignment_20261106_board.json)。
