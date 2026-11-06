# 通用控制算法提取与选型

## 选择结论

生产代码采用ControlAlgorithms公共依赖，不再将PID、LPF归Chassis所有。模块没有线程、设备或YAML实例。

| 实现 | 关键特点 | 本轮判断 |
| --- | --- | --- |
| 原26_sentry C PID | 每拍矩形积分、误差差分；标准PID只限制Iout、不限制隐藏Integral；clear遗漏D滤波；死区保持旧输出 | 不继续作为公共基座；只保留冻结对照 |
| sp_middleware PID | 显式dt、微分先行、首拍导数抑制、导数低通、梯形积分、清完整状态 | 参考其微分和首拍处理；不复制另一套积分器 |
| QDU Chassis + LibXR PID | 实车模块调用Calculate并传dt；当前基座LibXR有积分状态限幅、输出饱和门控、Reset | 优先复用其积分、限幅和抗饱和实现 |

LibXR本身没有导数低通，内部求导首拍以零反馈为基准，非法输入默认保持上次输出；公共PID用组合补齐导数首拍抑制、滤波及非法输入清历史/零输出。没有重写其积分与抗饱和公式。参数检查显式拒绝会被LibXR PID_SIGMA阈值静默关闭的非零输出限幅及积分配置，零Ki或零积分上限仍表示显式关闭积分。

参考现场：QDU Chassis 32761d0，Omni.hpp中CalculateMotorCurrent及LostCtrl；当前LibXR 5161373d的utils/pid.hpp。
sp_middleware不是Git检出，采用源码SHA256锁定本次对照：
- tools/pid/pid.cpp：633b490fd7e5e1636e9ba9ee0240635eb3e0c439eb1ec54315fa8e0368ca1b7b
- tools/low_pass_filter/low_pass_filter.cpp：133cafb5543924531cc859019828c81362a6bc74a986ac6c77400ad35d6ba034

## 已复现的旧实现问题

1. 一阶LPF Init没有清Initialized。新对象首拍输入4输出4，重新Init后首拍输入4却输出1（alpha=0.25）。二阶Init同样遗漏标志。
2. 标准PID clear没有清D滤波。纯D、alpha=0.5，先计算误差2，clear后误差0仍输出1。
3. 标准位置PID仅限幅Iout，隐藏Integral仍持续累积。Ki=1、I限幅1，连续1000拍正误差后，反向误差仍输出正积分，需大量周期退饱和。
4. 旧标准PID死区内不更新输出，可能保留旧电流。新公共PID死区内清历史并返回零；当前底盘死区参数仍为0。
5. 旧扩展角度函数对Inf以及很大的有限值逐次减2π可能不终止；生产不再编译这些扩展入口。新公共PID先拒绝非有限输入，角度模式复用LibXR周期类型。

旧实现保存在tests/chassis/reference/control_algorithms，原algorithm_hashes.json不改校验值。原样提取和替换为成熟基座分别提交；底盘依赖迁移和PID接入也分别提交。

## 接口与算法归属

- ControlAlgorithms.hpp：公共C++入口。
- wdr_control::PID：显式dt、连续时间增益、积分输出限幅、抗饱和、微分先行、首拍D=0、低通、角度差及完整Reset。
- LowPassFilter：alpha为新采样权重；首拍取输入；Reset恢复首拍状态。
- SecondOrderLowPass：按采样频率/截止频率构造二阶Butterworth，零历史启动，截止频率严格低于Nyquist；不是沿用旧任意三系数递推的别名。
- algorithm/translation_filter.c/h：原时间常数滤波，保留逐字节公式和零历史启动，与alpha一阶滤波是不同参数化/启动语义。
- Chassis：保留底盘专用运动学、模式和控制流程，通过manifest声明依赖；不再含algorithm目录或转发头。

## 底盘参数与行为变化

YAML保留原每拍增益，适配只在底盘构造阶段：dt=0.002s，Kp不变、Ki_cont=Ki_step/dt、Kd_cont=Kd_step*dt。
默认轮速Ki从1000换算为500000，跟随Kd从4换算为0.008；积分限幅仍表示输出，不能当成LibXR的裸积分状态上限直接传入。

这不代表逐拍完全等价：
- 输出饱和时不再继续积累同方向积分，反向能正常退出饱和。
- 导数针对反馈，不针对目标阶跃；首拍及复位后D为零。
- 跟随角度采用最短周期角差。
- 保留原斜坡。例如0.01rad相对角的首拍给定由0.16变为0.12rad/s；下一拍0.03rad，PID原始给定0.40，被原120rad/s²斜坡限为0.36；复位后的0.03rad首拍为0.36。

未改变底盘周期、硬件ID、方向、原平移滤波、斜坡参数、轮系解算、STOP/失控判据。本轮不烧录；增益换算不能替代实车参数复核。

## 验证入口

- tests/control_algorithms/run_tests.py：直接运行冻结旧实现复现缺陷，5000组sp LPF、5000组sp PID微分处理对照；积分限幅与饱和、dt、目标阶跃、角差、Reset、NaN/Inf及二阶滤波频响。
- tests/chassis/run_tests.py：原冻结轮系1000组、未改平移滤波散列、实际模块组帧、无力/时效/STOP、模式边沿；仅按新PID明确语义更新首拍断言。
- Debug/Release及底盘配置完整链接；检查公共源只编译一次，默认板级ELF仍不含底盘和公共PID实例。

构建和主机测试不证明电机方向、实车闭环、RTOS截止时间及栈余量。未对硬件执行烧录、复位或控制。

## 本轮执行结果

上述公共算法、底盘、I6x、板级回归通过；Debug/Release及未启用模块检查通过，五个可选配置完整链接通过。18模块/6配置冻结离线检查通过。公共源在每个预设中仅编译一次，默认板级ELF未引入公共PID实例。记录见[验证清单](validation/control_algorithms_20261106.json)。未烧录，未运行电机。
