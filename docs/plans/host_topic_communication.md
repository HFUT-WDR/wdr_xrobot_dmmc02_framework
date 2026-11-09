# 上位机Topic通信迁移规划

日期：2026-11-08；2026-11-09按ROS2共享数据、双时间戳与上下行拆合需求修订。状态：核心通信已实施；真实USB、时间精度和机构联动未验证。

实现入口见`host/README.md`与HostData模块README；上位机使用C++20、LibXR和XRobot，
由HostLink与可选HostRos组成。本文后续章节保留设计依据；sp_vision适配仍为后续工作。

## 1. 目标与既定行为

线协议使用当前仓库LibXR Topic打包协议，控制行为继续沿用已迁移的26_sentry实现。
首版链路建议使用已经生成并注册的 `usb_otg_hs_cdc2`（Host接口）；
第一路CDC继续供终端使用。此处是本规划的默认传输选择，USB实机收发尚待验证。

- 遥控SW2中位为OP、下位为AUTO、上位为无力；上位机不切换全局控制来源，不触发CALI。
- 线上mode沿用26_sentry：STOP=0、SCAN=1、OPEN_FRIC=2（本项目自瞄入口）；
  仅0/1/2为有效战术模式；删除旧导航/过点模式3/4/5，不保留兼容入口。板内枚举由HostData显式转换。
- AIM/SCAN时均可接收导航平移。STOP或云台无力仍联锁停止底盘和发射。
- 上位机只控制底盘vx/vy，wz由遥控旋钮提供，保持整车 `ChassisWzSource::RC`。
- 开火为独立电平输入，继续经过AIM、到速、热量、电机在线及机构许可判断。
- 线上合并为一条控制消息，内部仍区分云台、导航和开火有效性；桥接端按各上游真实到达时刻判定100ms时效，板端另判定100ms链路超时；不额外添加大量数值边界检查。

## 2. 已有实现与待补内容

已有：

1. HostData：四个本地输入Topic，转换为CMD的云台、底盘和发射AI输入。
2. SharedTopic：接收字节流、Topic解帧并发布，工作在接收任务上下文。
3. SharedTopicClient：订阅Topic并打包发送，使用固定槽池和串行化发送服务。
4. 双USB CDC：Log供终端，Host已在BSP中创建并注册。
5. 全局OP/AUTO、三通道独立时效及整车联锁。

待补：实际整车YAML接入、固定通信数据布局、紧凑状态回传、上位机编解码与串口工具、
线程并发梳理、端到端通信验证。当前整车YAML尚未实例化HostData及Topic收发模块。

### 2.1 26_sentry实际通信内容对照（2026-11-09）

本节直接依据参考仓 `USB_DEVICE/App/usb_callback.h/.c`，而不是裁判协议protocol.h。
正常业务包为GimbalToVision与VisionToGimbal；两者均没有采样或目标时间戳。
Identify_Data/pitch_data中的tick属于辨识调试包，不能当作正常上位机协议已有的时间同步。

迁移保留业务字段的含义，字节流改用LibXR Topic固定头与CRC，不复制旧SP帧头、CRC16和C位域布局。

| 原GimbalToVision上行内容 | 新链路归属与处理 |
| --- | --- |
| q[4] | robot_aim_state，来源为外置DM姿态，顺序固定w/x/y/z，包头保留该来源时间 |
| yaw、yaw_vel、pitch、pitch_vel | robot_aim_state，保持惯性角/角速度，rad及rad/s |
| big_yaw_angle | 放入robot_nav_state，明确为大Yaw惯性绝对角；另有big_yaw_relative机械相对角，不混用 |
| bullet_speed、bullet_count | robot_aim_state，测得弹速m/s及剩余弹量；注明有效性/来源 |
| game_progress、color、stage_remain_time | robot_nav_state，阶段、阵营及剩余秒数 |
| current_HP、base_hp | robot_nav_state，机器人及己方基地血量 |
| fortress_point_status | robot_nav_state，独立uint8字段 |
| enemy_outpost_is_destroyed、our_outpost_is_destroyed | robot_nav_state，独立标志；保留裁判状态有效性 |
| sentry_state | 仅在本车实际提供对应姿态反馈时发布；未迁移消费者时不伪造有效值 |
| shoot_heat | robot_aim_state，改为uint16或float的明确协议字段，不保留旧uint8截到240的精度损失 |
| mode | 旧发送函数固定填1；新上行消息不发送mode，也不发送请求/实际模式或OP/AUTO来源 |
| head、crc16 | 由Topic线格式替代 |

| 原VisionToGimbal下行内容 | 新链路归属与处理 |
| --- | --- |
| yaw、yaw_vel、yaw_acc、pitch、pitch_vel、pitch_acc | host_command的云台字段；保留角度和导数语义 |
| mode | aim部分沿用26_sentry线上编号，由HostData映射到板内枚举 |
| fire_control | host_command的fire字段，开火电平，保留独立有效性与许可门控 |
| vx、vy | host_command的vx/vy字段；坐标系在协议中固定为大Yaw系，单位m/s |
| tracking | 删除；不占下行字段 |
| id、armors_num、distance、enemy_center_yaw、back_find | 放入host_command.aim；保留明确业务语义，不隐式恢复旧板端判击策略 |
| nav_state、current_navi_task、sentry_state | 放入host_command.nav，任务字段统一命名navi_task；接收字段与策略是否启用分别说明 |
| big_yaw_angle | 放入host_command.nav，单位rad、与上行同一大Yaw绝对角参考；字段接入不隐式改变本地双Yaw协调 |
| head、crc16与位域 | 由Topic帧、固定宽度字段替代 |

线上mode仅保留26_sentry的0/1/2三个编号及对应战术行为，板内枚举不作为线协议直接发送：

| 线上mode | 板端适配 |
| --- | --- |
| STOP_MODE=0 | STOP |
| SCAN_MODE=1 | SCAN |
| OPEN_FRIC_MODE=2 | AIM自瞄入口，发射仍受当前许可门控 |

目前板内CMD仍为STOP=0、AIM=1、SCAN=4，HostData必须显式转换0/1/2，
不能static_cast直接透传。除0/1/2以外的mode均为无效值，整包不提交控制快照、
不刷新控制时效；包括旧3/4/5，不将其降级映射成扫描或其他行为。
STOP停止，SCAN扫描，OPEN_FRIC进入自瞄；AIM/SCAN下导航平移仍独立生效。
OPEN_FRIC的旧整套战术策略不因编号相同就视为已迁移。
旧位域改为独立uint8字段，保留含义，不复制C位域内存布局。

## 3. 分工和调用链

```text
上位机算法 / ROS节点
  → 上位机Topic编码器
  → USB Host CDC
  → SharedTopic / Topic::Server
  → HostData
  → CMD：OP/AUTO、输入时效、wz来源
  → Gimbal / Chassis / Launcher

机构状态Topic
  → HostTelemetry：取最新快照、选字段、限频
  → SharedTopicClient
  → USB Host CDC
  → 上位机Topic解码器
```

HostData处理输入适配，机构模块继续持有控制行为。
HostTelemetry是本规划新增的状态适配类，建议放在HostData模块仓内，不另建模块仓。
通信类型集中放在HostData仓的协议头中；上位机SDK按同一份字段表编码。

Topic回调不进行电机控制、阻塞等待或同步Flash写入。
SharedTopic接收回调一次提交完整控制快照；状态上行只取快照，不把每个控制拍排队发送。

## 4. 冻结Topic线协议

依据本仓 `LibXR/src/middleware/message/topic.hpp`、`packet/packet.hpp`、`packet/packet.cpp`。

| 字段 | 字节数 | 约定 |
| --- | --- | --- |
| prefix | 1 | 0x5A |
| payload长度 | 3 | 小端24位 |
| Topic名CRC32 | 4 | 对Topic名称的UTF-8字节计算；使用LibXR算法 |
| timestamp_us | 6 | 小端48位微秒时间戳 |
| version | 1 | 当前Topic帧版本0x01 |
| 头CRC8 | 1 | 覆盖前15字节 |
| payload | N | 本规划约定的固定字段布局 |
| 尾CRC8 | 1 | 覆盖16字节头和完整payload |

固定开销17字节，帧长为N+17。Topic帧本身不包含Topic名称字符串或domain名称。
当前服务端按已注册的Topic名CRC32分发；首版Topic统一使用默认domain，避免同名歧义。

CRC8使用LibXR当前实现：初值0xFF、反射多项式0x8C。
Topic名CRC32：初值0xFFFFFFFF、反射多项式0xEDB88320，返回值不做最终异或。
不能直接假定Python `zlib.crc32(name)` 与Topic键相同，必须用C++生成的黄金字节帧逐项对照。

所有浮点为小端IEEE754 float32；标志用uint8_t或固定宽度位图。
协议头固定offset/sizeof静态断言，padding显式保留并清零，不直接发送含bool、指针、PID对象的内部State。
Topic帧版本与业务payload布局分别管理；布局发生不兼容变更时更换Topic名称或明确同步升级两端。

### 4.1 时钟对齐

首版把时间对齐作为正式通信链路的一部分。板端使用LibXR单调微秒时钟，上位机使用
单调时钟（如Linux CLOCK_MONOTONIC），不使用系统日历时间；相机时间戳另行确认其时钟域。
上位机维护 `t_mcu = a * t_host + b` 映射，b为偏移，a补偿长期频率偏差；不修改板端时钟。

增加两个同步Topic：

| Topic | payload约定 | 长度 |
| --- | --- | --- |
| host_time_sync | uint32 seq、uint32 reserved=0、uint64 t1_host_us | 16 B |
| robot_time_sync | uint32 seq、uint32 boot_id、uint64 t1_host_us、uint64 t2_mcu_us、uint64 t3_mcu_us | 32 B |

一次往返记录四个时刻：

1. t1：上位机发起请求前。
2. t2：板端完整有效请求解析完成时。
3. t3：板端响应打包前。
4. t4：上位机解析出完整响应时。

起步阶段按a≈1估计：

```text
RTT = (t4 - t1) - (t3 - t2)
b   = ((t2 - t1) + (t3 - t4)) / 2
```

这依赖双向延迟近似对称。USB队列、操作系统调度和解析延迟均计入测量，
t3不是USB物理发送完成时刻，不能把上述结果宣称为硬件级精确同步。
启动建议10Hz收集低RTT样本，进入稳定状态后约1Hz更新；在滑动窗口中选择低延迟样本，
使用两端中点时间对拟合a/b。持续报告RTT、拟合残差、同步状态和距上次成功同步的时间。
低RTT筛选只能减轻排队干扰，不能消除固定的单向延迟不对称。

boot_id在板端每次启动时生成，不为此写Flash。USB重连、boot_id变化、板端时间回退，
或上位机通信进程重启后，丢弃旧映射并重新同步。相机重启只影响其自身时间映射，不直接重置MCU同步。响应seq必须匹配仍在等待的请求。
同步丢失时不能沿用上一启动会话的时间映射。

### 4.2 采样、曝光、目标和到达时间

- 姿态时间：回传时保留来源姿态时间戳，不在限频发送时改成当前发送时间。
- 图像时间：优先使用曝光中点时间，明确相机SDK提供的是曝光开始、结束还是主机收帧时间。
- 目标时间：目标角及其导数必须标明对应的预测时刻，区别于原始图像曝光时间。
- 到达时间：板端按本地收到有效host_command的时刻判定100ms链路超时，并按valid_mask处理各路有效性，独立于同步结果。

板端状态包头使用MCU时间；控制包头使用上位机换算到MCU域的发送时间，目标时刻另放payload，相机曝光时间暂在上位机侧维护。
同步请求尚无映射时包头时间可为0，PC的t1放在请求payload中；同步响应包头使用板端t3。
接收适配层需读取Topic原始消息视图中的时间戳，不能沿用目前仅取payload而忽略包头时间的回调。

同步后，上位机把曝光时刻转换到MCU时钟域，在带时间戳的姿态历史中插值，
得到曝光时的云台姿态；yaw插值处理±π跨越。姿态历史窗口建议先覆盖0.5～1s并实际测量容量。
目标预测建议先由上位机统一负责，预测到预计执行时刻；板端沿用现有角度及导数前馈，
避免上下位机重复补偿同一段延迟。若后续增加板端按时间外推，应作为独立控制行为变更验收。

首版合并下行控制包包含 `uint64 target_time_mcu_us`；相机曝光时间先留在上位机数据链，
后续确认相机时钟域后再决定是否需要下发，不在首版强制增加capture_time字段。
保留到达时刻作在线判断。对明显过期或时间基准不成立的时序命令不更新AIM/开火许可；
允许的历史/未来窗口根据实测链路延迟和预测周期确定，不在此猜一个固定毫秒上限。
同步未就绪时，OP和纯通信监视可以工作；首版上位机工具不发送时序自瞄及开火命令。

当前板载IMU的time_us为采样时刻，外置DM IMU的time_us是软件消费完整帧时刻，
两者精度含义不同。软件时钟对齐不能消除DM串口传输与消费排队延迟；
需在遥测中保留并说明该来源，后续按设备时间戳、DMA接收时刻等实际能力改进。

### 4.3 精度验收及硬件同步

首版先用软件往返同步测量并报告偏移残差、RTT分布、漂移和重连收敛时间，
把毫秒级对齐作为待验证目标，不预先承诺误差。需要独立实验验证绝对误差，
仅用自身拟合残差不能证明双向延迟对称或曝光已对准。
若实测精度不能满足目标运动速度与弹道需求，且相机支持外触发，则增加MCU GPIO触发曝光、
记录触发时间并匹配frame_id的硬件同步；触发到曝光中点的固定延迟仍需标定。
相机型号、触发引脚和电平尚未确认，此项不直接修改IOC或接线。

### 4.4 ROS2共享与双时间戳发布（2026-11-09修订）

按引用对话明确的用户需求，首版先校准MCU与上位机单调时间，相机时钟域后续单独处理。
上位机只保留一个USB通信所有者，由ROS2 bridge集中解包并服务自瞄与导航：

```text
MCU Topic → USB Host → wdr_bridge
                        ├→ 带双时间戳的IMU/姿态 → sp_vision适配层
                        ├→ 标准ROS IMU/底盘状态 → 导航
                        └→ 时钟同步状态及诊断
自瞄/导航控制 → ROS2命令 → wdr_bridge → MCU Topic
```

保持前文a/b定义 `t_mcu = a * t_steady + b`，桥接节点先做反变换：

```text
t_steady_sample = (t_mcu_sample - b) / a
```

数值实现使用参考点形式，避免直接计算大的截距；不要每包直接覆盖a/b。
启动时a=1先估偏移，积累30～60s低RTT样本后估计走速，慢速修正并保证换算连续。
转成ROS时间时，再使用同一台主机上邻近读取的steady/ROS参考时刻：

```text
t_ros_sample = ros_reference + (t_steady_sample - steady_reference)
```

上述ROS转换只适用于真实运行中速率一致、连续的区间；不能把一个固定偏移用于会暂停、
变速或跳变的/clock回放。实车bridge默认use_sim_time=false；回放另行使用明确的时间语义。
同一条样本的ROS和steady时间应由同一份映射快照产生，映射更新不反复改写已经入队的样本；
小修正平滑生效，时钟会话重建时清理历史。
ROS时间跳变时重建steady→ROS锚点并通知相关消费者清理跨跳变缓存，
不因此修改MCU时钟或重置仍有效的MCU→steady估计。

推荐在同一条自定义消息中同时携带：

- 标准 `sensor_msgs/Imu imu`，其中 `imu.header.stamp` 是同一采样事件的ROS时间。
- `uint64 steady_time_ns`：同一采样事件在上位机单调时钟域的时间。
- `uint64 mcu_time_us`：原始板端来源时间。
- `uint32 boot_id` 和同步有效标志：区分板端重启与尚未建立映射的样本。

标准sensor_msgs/Imu不能随意添加字段，因此使用包装消息（例如TimedImu）；
同时从同一份样本发布标准IMU Topic给导航。不要发布两条互相独立的时间Topic再按收包顺序配对。
实际frame_id、IMU到机构的外参、四元数w/x/y/z顺序都由适配层明确转换。
同一主机各消费者必须使用相同的单调时钟定义，不把另一台机器的steady时间直接相减。

sp_vision适配依据本地源码：
`io/cboard.cpp::imu_at(steady_clock::time_point)`按时间做四元数SLERP；
原CBoard::callback用PC收到CAN帧时的steady_clock::now()打戳，
`src/sentry.cpp`按图像时间减1ms查询姿态。
迁移时保留imu_at接口与算法主体，改成从ROS包装消息中的steady_time_ns建立姿态缓存，
由板端来源时间换算得到样本时刻，取代原先CAN回调的接收时间。
原有固定-1ms补偿不能未经测量直接叠加在新链路上，应重新标定其所补偿的延迟。

为支持该路径，robot_aim_state包含外置DM四元数与角速度，并保留来源时间、
boot_id和sample_seq；桥接端从同一条自瞄消息派生TimedImu与标准ROS Imu，
不再在线上单独发送一条重复的IMU消息。加速度仅在来源确实提供时发送并标明有效性。
板载IMU与外置DM不能拼成一套无明确坐标系的姿态；导航所需姿态在导航消息中明确来源。
高频自瞄包的实际长度及200～500Hz带宽在字段冻结后预算，不沿用原48字节IMU包的估算。
导航使用底盘BODY系轮速估计；板载IMU若随Yaw机构转动，不能直接当作刚性安装于base_link的IMU，
需要明确运动补偿或使用独立底盘IMU。该安装关系在实施ROS适配前核实。

MCU线上仍只传自身单调时间，不传ROS日历时间，也不同时维护两套主机时钟。
Topic通信协议与ROS2消息类型是桥接节点两侧不同的数据封装。

## 5. 下行控制Topic：合并控制快照

上位机到下位机使用一条 `host_command`，建议100～200Hz；时间同步请求
`host_time_sync`独立。首版USB接收白名单只开放这两个Topic。

同一host_command载荷分为公共头、aim部分、nav部分；两部分不是两个线上Topic。
字段按下表顺序显式序列化，不依赖结构体隐式padding。

| 部分 | 字段 | 长度与语义 |
| --- | --- | --- |
| 公共 | target_time_mcu_us | uint64，云台目标参考时刻，MCU微秒域 |
| aim | yaw、yaw_vel、yaw_acc、pitch、pitch_vel、pitch_acc | 六个float，rad、rad/s、rad/s² |
| aim | distance、enemy_center_yaw | 两个float，m及rad |
| aim | mode、fire_control、back_find、id、armors_num | 五个uint8，mode沿用26_sentry编号；无tracking |
| nav | vx、vy、big_yaw_angle | 三个float，m/s、m/s、rad |
| nav | nav_state、navi_task、sentry_state | 三个uint8，取代旧C位域，保留业务含义 |
| 公共 | valid_mask、reserved[3] | 四个uint8；显式保留位为0 |

payload共64字节，含Topic帧开销共81字节。aim与nav固定offset、独立有效性，
valid_mask的bit0/1/2分别表示云台意图、导航平移、开火请求有效，其余位保留。
mode只在aim中下发，不在上行回传。无roll或wz控制字段。
SCAN轨迹由板端生成，开火仍为持续刷新的电平，不按包数生成拨弹队列。
back_find属于aim，nav_state/navi_task/sentry_state/big_yaw_angle属于nav。
nav中的big_yaw_angle与上行同名量采用同一大Yaw惯性绝对角参考，不能当作机械相对角。
首版已有控制仍是vx/vy与本地双Yaw协调；旧任务字段、back_find及大Yaw目标如何参与策略，
需按原26_sentry消费者逐项迁移并单独验证，不能仅增加通信字段就声明其控制行为已生效。

bridge是唯一USB控制写入者：汇总视觉和导航各自的ROS输入，组成一个完整快照。
每次发包前分别检查上游真实到达时间；超过100ms的输入必须清除对应有效位，
导航失效同时置vx/vy为0，开火失效置fire_control为0，视觉断流置STOP并清除云台/开火有效位。
视觉健康但无目标时可按既定策略进入SCAN，与视觉进程断流明确区分。
周期发送合包不能给缓存的旧上游输入续期。

板端一次接收并一次提交整份快照，内部仍适配CMD的云台、底盘和发射通道。
无效导航立即平移归零；无效开火立即停止拨弹；无效云台或整包超过100ms未更新，
按现有AUTO失效语义使云台无力并联锁底盘、发射。有效AIM/SCAN加导航失效时，
仍保留遥控旋钮的旋转控制。时间同步包不能刷新控制包时效。

旧target_euler、host_gimbal_command、host_chassis_data、host_fire_notify仅可作为本地兼容接口，
默认不注册到USB白名单，不能与host_command竞争写入。
云台yaw/pitch目标与反馈保持同一外置DM惯性姿态参考；该参考上电重建，不默认等于地图航向。
导航输出若为地图系或BODY系，桥接端须转换为约定的大Yaw系。

## 6. 上行状态Topic：按自瞄和导航拆分

下位机到上位机只保留两类主要业务消息，分别供自瞄和导航直接消费；
时间同步响应robot_time_sync独立，不混入业务遥测。

| Topic | 消费者与字段 | 建议频率 |
| --- | --- | --- |
| robot_aim_state | 自瞄：外置DM四元数、角速度；yaw/pitch及其角速度；小Yaw机械相对角、Pitch机械角；测得弹速、剩余弹量、热量及上限、发射阶段与许可；阵营、比赛阶段及有效性 | 200～500Hz |
| robot_nav_state | 导航：BODY系vx/vy/wz、底盘航向、大Yaw绝对角及机械相对角；在线/功率状态；阵营、机器人ID、比赛阶段、剩余时间、机器人/基地HP、堡垒/前哨站状态及有效性 | 50～100Hz |

必要的公共状态可以在两包中重复，例如阵营和比赛阶段，保持同一来源和语义，
避免自瞄必须等待导航包才能工作。ROS侧分别发布自瞄和导航消息；TimedImu从自瞄包派生，
sp_vision无需订阅导航遥测才能取得姿态与弹速。

robot_aim_state包头使用DM姿态来源时间，包含boot_id、sample_seq；
电机、发射和裁判等异步字段带各自来源时间及有效性，不把高频重复发送当作新采样。
robot_nav_state包头使用导航状态快照时间，同样带boot_id、序号与来源有效性；
若包含IMU或其他异步采样量，另保留其来源时间。两包独立限频，不能以收包时间替代采样时间。
同一MCU时钟映射服务同时处理两包，ROS/steady双时间戳由桥接端生成。

来源为DM/板载姿态Topic、gimbal_status、chassis_status、launcher_status、robot_game_ref。
缺少的弹量、堡垒/前哨站、HP等字段需补齐真实发布通路，不以默认0冒充有效状态。
区分测得弹速与摩擦轮设定速度；不把轮速估计声明为地图定位。
导航IMU必须满足base_link安装关系或有明确补偿，不能直接复用随云台转动的DM姿态。

两包的字段offset和长度在实施时冻结；最大单帧须适配Host CDC队列，按256字节容量和
16槽发送池预算与压测。状态只取最新快照，不把1kHz控制状态逐帧外发；
SharedTopicClient满槽或Write失败会丢包，不提供逐帧可靠交付。

## 7. 接入前必须处理的实现点

### 7.1 payload长度匹配

当前Topic::Server在读取payload时取注册类型的target_size发布，未严格要求
报文data_len等于该Topic的payload_size。短包可能夹带旧缓冲内容，长包也不应被当成合法命令。
在接入控制链前补齐精确长度匹配与丢弃恢复，覆盖短包、长包、坏CRC、半包和粘包。
这是通信结构契约检查，不扩展为大量控制数值边界检查。

### 7.2 CMD跨线程输入

接入后I6x与Host接收任务会同时调用CMD。现有FeedRC/FeedAI会更新共享输入并调用
ProcessAndPublish，不能仅凭两个线程都能收包就宣称并发正确。
实施时在CMD内统一串行化输入更新、模式事件及快照发布，可复用现有LibXR SerializedService，
不恢复额外CommandTask。host_command的三个控制分量必须一次更新后统一发布，不能用三次FeedAI形成中间混合状态；
所有发布基于完整快照，并避免持锁调用可能重入CMD的Topic回调。
用RC/AI交错、同拍OP/AUTO切换和接收中失联测试验证最终输出与顺序。

### 7.3 启动顺序

HostData先创建下行Topic，HostTelemetry先创建上行Topic；随后再构造
SharedTopic与SharedTopicClient。它们查找不存在的Topic会进入致命错误处理。
整车YAML和模板显式指定 `usb_otg_hs_cdc2`，生成入口由xrobot更新。
同一Host字节流只保留一个读者和一个发送适配器。

## 8. 行为与失效验收

| 场景 | 预期 |
| --- | --- |
| OP收到任意Host命令 | 保持OP控制来源，上位机不接管 |
| AUTO，AIM/SCAN有效 | 云台对应行为，导航vx/vy可同时生效，wz仍来自遥控 |
| AUTO，STOP有效 | 云台无力，底盘及发射联锁停止 |
| mode为3/4/5或其他非0/1/2值 | 整包不提交控制快照、不刷新控制时效，不执行旧战术行为 |
| 导航上游超过100ms未更新，合包仍正常 | PC平移归零；有效AIM/SCAN下遥控旋转仍保留 |
| 视觉上游超过100ms未更新，合包仍正常 | AUTO云台无力，整车许可联锁停止 |
| 开火请求失效或有效位清除 | 停止拨弹，摩擦轮按当前AIM模式处理 |
| 遥控断连 | 整车按现有失控语义停止，Host不能维持接管 |
| USB断开/上位机退出 | host_command超过本地100ms时效即整车联锁停止，其他Topic不续期 |
| USB重连/上位机重启 | 不重复发送缓存的旧开火包；有效新输入按当前拨杆与现有恢复语义处理 |

普通合法零目标且对应有效位为1时，必须作为有效输入处理；坏帧、错误长度、无效模式不刷新控制时效。
OP/AUTO互切发射复位继续使用当前Gimbal/Launcher行为。

## 9. 实施顺序与交付

1. 按第2.1节完成旧字段对照与消费者核查，修正旧模式编号误解；冻结协议头、同步和字段表，
   为控制、同步与状态Topic生成黄金字节帧。
2. 接入SharedTopic字节流长度契约，并完成HostData wire类型与CMD串行化。
3. 实现HostTelemetry，接入整车YAML及增量模板，更新生成入口。
4. 提供ROS2 wdr_bridge、双时间戳消息和sp_vision接收适配器；另附协议调试工具，支持
   USB身份识别、状态监视和命令周期发送。上位机运行时统一使用C++/LibXR/XRobot；Python仅可作为离线测试工具生成黄金帧，
   与真实LibXR双向互认；普通启动默认只监视并对时。
5. 运行主机协议/并发、四时间戳映射、时钟漂移/延迟不对称/重启重连和角度插值回归，
   完成Debug/Release整车构建；保存实际模块版本、锁文件及离线来源。
6. 实机先测USB收发和回传，不使能机构；再测输入超时和OP隔离；最后用同一整车固件验证
   AUTO STOP/SCAN/AIM、导航平移及空载发射联动。

规划验收产物：固定协议头、Topic字段说明、上下位机黄金帧、可运行SDK/监视工具、
整车配置、主机回归及明确区分烧录/收发/机构闭环的验证记录。

既有Flash保存前零输出及IMU断流门控问题独立保留；正式带动力联动前处理，
不把Topic通信迁移完成等同于整车机构闭环已验收。

## 10. sp_vision最小修改方案

以本地 `/home/yunmo/WorkingSpace/sp_vision25/sp_vision_25` 的哨兵入口为对象。
首版不重构为完整ROS2视觉节点集合，在io::CBoard内部替换设备收发即可：

```text
原：sentry主循环 → CBoard → SocketCAN
新：sentry主循环 → CBoard的ROS2适配 → wdr_bridge → USB Topic → MCU
```

### 10.1 保留的接口与算法

保留 `CBoard(config_path)`、`imu_at(steady_clock::time_point)`、`send(Command) const`，
以及bullet_speed、mode、shoot_mode等现有上层读取接口。
保留Solver、Tracker、Aimer、Shooter、Decider与相机采集主体。
主循环仍调用 `cboard.imu_at(t)` 与 `cboard.send(command)`，不让算法直接处理USB、
Topic报文CRC或MCU时钟换算。固定-1ms补偿改为配置项并实测，避免把旧接收路径的补偿硬搬过来。

### 10.2 接收适配

- CBoard订阅桥接后的TimedImu，将四元数和steady_time_ns写入原姿态缓存。
- 保留现有SLERP逻辑；插值前必须有包围目标时间的两条有效样本。
- 不能直接沿用无限阻塞的queue.pop作为断连处理：无后续IMU时需有界等待、退出或返回失败，
  停止发有效控制和开火；若需要显式失败接口，可新增TryImuAt而保留旧签名作兼容包装。
- 板端boot_id变化时清理旧缓存和插值端点，避免把两次上电的姿态或时间混用。
- 订阅桥接后的自瞄状态，适配弹速；工作模式由本地自瞄意图和运行许可维护，上行不提供mode；采用同一条消息中的steady时间，不用ROS接收时刻now()重打戳。
- 相机暂维持原steady_clock输入路径，单独核实其时间含义，首版不迁移全部相机节点。

### 10.3 发送适配及语义

CBoard::send只发布ROS2自瞄意图，由bridge完成Topic编码；导航单独发布平移意图到bridge，
两者不竞争USB设备，也不各自发送整套云台/底盘/发射快照。

| 原接口字段 | 最小接入处理 |
| --- | --- |
| command.yaw / pitch | 按现有弧度语义传给bridge，适配后填入host_command云台字段 |
| command.shoot | 发射请求电平；板端仍受AUTO及当前AIM许可门控，bridge检查同步和上游有效性 |
| command.control=true | 在AUTO允许时表示有效AIM目标 |
| command.control=false | 表示无有效瞄准控制，不直接当作主动STOP；在明确允许自动扫描时映射SCAN |
| 主动停止/无力/OP/接收失效 | 独立运行许可决定发送STOP，板端自行判断OP不接管，不能靠control=false区分全部含义 |
| command.horizon_distance | 当前哨兵不使用，不塞进其他协议字段 |

STOP/SCAN/自瞄在线上分别编码0/1/2；bridge内固定单一策略，不让两个Topic写入者交替覆盖模式。
bridge分别记录视觉与导航ROS消息的真实到达时刻，不能用周期重发给旧上游命令续命。
视觉进程断流应停止有效瞄准和开火；这与视觉正常运行但报告无目标、允许进入SCAN的情况分开处理。
首版按最小接口发布角度目标，速度/加速度字段置0，不从相邻帧盲目差分伪造前馈。
若需要恢复完整速度/加速度前馈，再扩展Command和算法输出，该项与通信替换分开验收。
Command目前也没有精确目标参考时间；首版send记录本地steady发送时刻作约定参考，
不能将它描述成精确曝光或算法求解时刻。后续严格的目标时序需要显式扩展对应字段。

当前CBoard的shoot_mode为原双发射机构语义；本车单发射机构需在配置中明确选择兼容分支，
并核对Aimer的left/right_yaw_offset，避免无意引入原车双枪偏置。弹速使用真实测量值或明确标记的配置值，
不能把摩擦轮设定速度直接作为测得弹速。

### 10.4 预计修改范围

| 位置 | 修改 |
| --- | --- |
| io/cboard.hpp、cboard.cpp | 内部ROS2收发、姿态缓存接入和生命周期；保留上层API |
| io/CMakeLists.txt | 接入共享消息包和ROS依赖 |
| configs/sentry.yaml | Topic名、单发射兼容配置、时间补偿参数 |
| src/sentry.cpp | 原固定-1ms改成配置；仅在需要显式处理IMU失效时增加最小分支 |
| 独立wdr_bridge与消息包 | USB Topic、同步a/b、ROS/steady双时间戳、导航与自瞄汇总 |

原io::ROS2已经初始化rclcpp并启动导航相关线程；CBoard适配器复用进程上下文，
不再次无条件init/shutdown，也不复用被阻塞的控制主循环来执行订阅回调。
回调、缓存和上层状态读取需要保持线程安全，退出时先停止回调再释放CBoard缓存。

### 10.5 验证边界

1. 用同一段四元数序列对照原imu_at与ROS接入后的SLERP结果，检查单位、坐标与时间域。
2. 假ROS输入下验证CBoard构造就绪、断流有界退出、重启清缓存和send字段映射。
3. 验证OP不被接管，AUTO无目标按已确认策略扫描，STOP明确生效，SCAN不拨弹。
4. 验证单发射机构弹道偏置、真实弹速输入；导数置0的首版不冒称完整前馈性能已等价。
5. 桥接端验证导航与自瞄并发输入，最后才接入真实机构联动。

## 11. 源码依据

- 26_sentry参考仓 `USB_DEVICE/App/usb_callback.h/.c`：GimbalToVision/VisionToGimbal及实际填充/消费。

- `Modules/HFUT-WDR/HostData/HostData.hpp`：现有Topic、字段和AI适配。
- `Modules/HFUT-WDR/CMD/CMD.hpp`：控制源、模式、三通道100ms时效、wz策略。
- `Modules/xrobot-org/SharedTopic/SharedTopic.hpp`：接收线程与Topic注册。
- `Modules/xrobot-org/SharedTopicClient/SharedTopicClient.hpp`：槽池、打包及异步发送。
- `Middlewares/Third_Party/LibXR/src/middleware/message/packet/packet.hpp`、`packet.cpp`：线格式。
- `Middlewares/Third_Party/LibXR/src/middleware/message/server/server.cpp`：解析及长度处理。
- `Middlewares/Third_Party/LibXR/src/utils/crc.hpp`、`crc_o3.cpp`：CRC具体算法。
- `User/app_main.cpp`、`User/libxr_config.yaml`：双CDC及注册，修改配置后通过生成器同步。
