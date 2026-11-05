# 底盘模块接入（2026-11-05）

## 本轮边界

已确认：沿用26_sentry四轮底盘、功能与参数；模块形式参考QDU工程。保留“板载IMU航向－大yaw机械相对角”算法，
先定义云台反馈接口；缺少或停更时底盘无力。本轮只做代码、生成、编译与离线测试，不烧录、不驱动实车。
功率控制仍按 IMU→底盘→云台→校准→功率控制 的既定顺序后续接入。

模块为个人私有仓库 `YunMo-yanyu/Chassis`，保留原QDU Chassis Git历史及Apache-2.0 LICENSE；
26_sentry的来源和文件作者记录见组件THIRD_PARTY_NOTICES.md。没有修改参考工程。

## 数据链路与状态职责

```text
I6x/PC → CMD选择输入来源 → chassis_cmd
I6x事件 → EventBinder → Chassis模式
BoardAttitude → board_attitude ──────────┐
未来云台 → gimbal_chassis_feedback ───────┼→ Chassis → 四颗RMMotor → CAN1 0x200
四颗RMMotor实际反馈 ────────────────────┘             └→ chassis_status
```

CMD只选择输入来源，Chassis维护机构模式和本机构保护。没有恢复chassis_online/gimbal_online命令门控，也没有添加valid指令位。
遥控链路离线由现有CMD判断，底盘读取原子在线状态；命令本身仍由CMD选择，底盘不再查询命令来源。
所有来源的CMD底盘x/y为m/s、z为rad/s。遥控输入端完成死区与量程映射，默认x/y量程2.64m/s（660×0.004）、z量程12rad/s；I6x旋钮映射为0..12rad/s。
PC x/y为m/s、z为rad/s，不二次缩放，也不在下位机恢复旧导航/搜索等策略。
CMD::ChassisCMD新增frame，默认GIMBAL，可显式BODY；X前、Y左、Z逆时针。跟随控制与平移坐标系分开，BODY不会被模式隐式旋转。
这是内部命令Topic布局更新；HostData外部HostChassisTarget三浮点布局未变。其他直接共享CMD结构的节点必须同步版本。

`Chassis::GimbalFeedback`是云台发布契约：

| 字段 | 含义 |
| --- | --- |
| yaw_relative_rad | 原车 `get_yaw_motor_point(INNER_YAW)->relative_angle`，即大yaw机械相对角，rad |
| yaw_valid | 此轴反馈可用；不是整个云台机构就绪 |
| stop_chassis | 校准、复活或其他独占状态要求底盘无力 |
| time_us、sequence | 与该反馈同源的采集时间和序号；不可周期伪造新时刻掩盖停更 |

Topic默认 `gimbal_chassis_feedback`，本轮不添加伪发布者。板载输入为 `wdr_imu::BoardState`，默认 `board_attitude`。
角度必须为归一化rad；控制器沿用逐次加减2π的闭区间[-π,π]规则，不把NaN/Inf改成零角。

## 模块组织与参数

Chassis构造注入右前、左前、左后、右后四颗RMMotor，要求同一CAN总线、M3508、反馈ID唯一且完整覆盖同一命令组（0x201..0x204或0x205..0x208），与逻辑轮序无关。
构造不发送；独立LibXR MEDIUM优先级、4096字节（栈及优先级可配置）线程启动后，按2ms周期整组提交四槽0x200。
当前LibXR MEDIUM映射为22，未假称与旧工程AboveNormal优先级数值相同。

主流程直接呈现：

```text
FeedbackUpdate → SetMode → ModeTransition → SetControl → ControlLoop → SendOutput → PublishStatus
```

输入由常驻异步订阅在2ms控制任务中读取，本拍正常计算共用快照。Topic回调只维护不加锁的原子健康标记，确保后到的STOP/非法输入不被异步缓冲忽略；发送前重新检查健康与RELAX事件。
CAN电机回调仍只复制，Update在任务中解码；新增真实收帧时效查询，不改变旧Update调用次数兼容返回值。
短帧、非标准帧、错误ID不刷新时效，从未收到合法反馈不能因默认值或Update返回OK而上线。

| 模式 | 行为 |
| --- | --- |
| RELAX | 零电流，清除控制历史 |
| NO_MOVE | 零速度闭环制动，与无力分开 |
| FOLLOW_GIMBAL_YAW | 相对yaw目标0，原角度环与跟随斜坡 |
| FOLLOW_CHASSIS_YAW | 进入时记录本拍航向；同模式调用不重复记录 |
| NO_FOLLOW_YAW | 平移按显式frame变换，z直接为目标角速度 |
| RAW | 原开环混合量，仍受反馈/失控保护及协议限幅；当前配置没有进入该模式的事件绑定 |

参数集中于User/xrobot_chassis.yaml，包括0.18951m轮心距、0.000299 rpm→轮缘速度系数、1/1.6平移补偿、
2m/s平移限幅、10m/s轮速同比限幅、16000协议电流上限、原轮速/跟随PID、平移滤波时间常数及跟随斜坡参数。
PID.c/h、LPF.c复用原车源码；LPF.h补齐缺失的extern C闭合；stdint替代历史数值类型重定义。
平移滤波复用user_lib.c的原函数，正常遥控/PC切换共用历史，所有来源精确零目标当拍清对应轴平移滤波历史；轮速闭环继续制动。PC固定0.04m/s死区移除，小目标可运动。
故障恢复完整清除PID导数滤波及平移/斜坡历史，避免旧输出在恢复时回流。

保留差异：原车轮速反馈矩阵的0.25系数及符号未改，状态vx/vy/wz是历史诊断定义，不能当已校准里程计。
NO_FOLLOW的坐标旋转在原x/y灵敏度相同条件下改写为等价sin/cos矩阵；不保留绕一圈的atan2/sqrt往返。
纯计算支持moving_posture输入，但本轮模块尚未接裁判姿态；旋转衰减按非移动姿态处理。该输入将在功率/裁判阶段接入，不能声称已复现全部比赛策略。

## 保护与输出

故障位：INPUT_LOST=1、COMMAND_STALE=2、YAW_LOST=4、MOTOR_LOST=8、GIMBAL_STOP=16、INVALID_TARGET=32、INVALID_FEEDBACK=64。
默认命令100ms、板载与云台航向20ms、任一轮实际收帧10ms超时；阈值在Param/YAML配置。
任一故障使实际模式RELAX并清零全组输出和控制历史，RAW和BODY同样需要yaw反馈。
发送前再次检查健康，防止本轮计算期间出现失联或停止要求而继续使用已计算的输出。
每轮都会提交完整四槽，复用RMMotor组帧，不自动发送其他电机组。

`chassis_status`发布请求/实际模式、故障位、航向、反馈/目标、四轮协议给定、时间和序号。
current为请求发送的协议整数，不能据此证明硬件已执行或CAN总线发送成功。
终端新增只读 `chassis`，等待一份新状态后打印，不切换模式、不下发运动目标；默认未启用时提示无新样本。

本阶段没有功率/RLS/超电限制。不能把本阶段编译结果当作完整实车可用结论。

## 配置与验证

默认仍为 `User/xrobot.yaml`，底盘不实例化。`User/xrobot_chassis.yaml`是可选完整配置，含板载姿态、CMD/I6x、四颗M3508与Chassis。
原I6x组合SW2_MID_SW3_UP、SW2_UP_SW3_UP分别选择遥控/PC来源并进入NO_FOLLOW_YAW；退出或CMD失联事件进入RELAX。
沿用现有I6x事件名称，不在本阶段改变物理拨杆映射。缺少真实云台反馈，该配置始终无力。
生成：`xrobot gen -c User/xrobot_chassis.yaml`；恢复默认：`xrobot gen -c User/xrobot.yaml`。
VS Code增加“测试底盘（主机）”；原构建/烧录任务使用当前选中配置，本轮未运行烧录任务。

`python3 tests/chassis/run_tests.py`覆盖原车冻结轮速混合、算法源码散列、模式边沿、滤波历史、跟随PID/斜坡、
安装角旋转、同比轮速限幅、非法输入、以及真实Chassis/CMD/RMMotor配合LibXR替身的收发与失控链路。
已有I6x、IMU、板级RAM数据库回归继续执行；Debug/Release检查13个构造接口和六份完整配置。
底盘可选配置另做完整Debug固件链接；默认ELF检查Chassis与纯控制器均未链接。
结果、锁定提交和日志散列见[验证记录](validation/chassis_module_20261105.json)。

主机替身与编译不证明真实CAN收发、车轮方向、RTOS时延、栈余量、底盘闭环、云台角度链或功率安全；上述均未板测。
