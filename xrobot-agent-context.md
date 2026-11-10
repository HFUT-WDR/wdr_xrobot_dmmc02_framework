# XRobot / LibXR 专用 Agent 启动提示词

本提示词用于 XRobot / LibXR 相关仓库。当前仓库可能是 STM32、CH32、ESP32、Linux、Webots、HPM、MSPM0 平台工程，也可能是 XRobot BSP、驱动、XRUSB、调试、CodeGen、示例或测试仓库。

开始分析前，先根据当前仓库的目录和关键文件判断它属于哪一类，再进入对应文档和代码入口。

## 这几个项目分别是什么
- LibXR：运行时框架，负责核心语义、驱动抽象、中间件和 XRUSB。
- XRobot：`xrobot` 命令，LibXR 的模块管理与主函数生成工具：拉取模块、用 `xrobot.lock` 把每个模块锁定到具体的提交，并根据 `User/` 下的配置生成主函数 `XRobotMain`。
- CodeGen：`libxr` 命令（pip 包 `libxr`），读取 STM32CubeMX 工程的 `.ioc`，生成外设对象、入口函数 `app_main`，并把 LibXR 接入工程的 CMake 构建。

## 使用原则
- 本提示词只用于 XRobot / LibXR 相关仓库。
- 查看目录之前，不预设仓库是 STM32、ESP32 或 Linux 工程，也不预设要先运行 XRobot 命令。
- 先判断仓库角色，再决定看哪份文档、读哪部分代码、执行哪类命令。

## 第一步先看什么
- 先检查仓库根目录与关键配置文件，确认它更像 XRobot BSP、平台工程、驱动仓库，还是工具仓库。
- XRobot BSP 常见痕迹：`Modules/modules.yaml`、`Modules/sources.yaml`、`xrobot.lock`、`User/*.yaml`、调用 `XROBOT_MAIN()` 的入口源文件。
- LibXR 平台工程常见痕迹：`CMakeLists.txt`、`CMakePresets.json`、`libxr_config.yaml`、平台目录、芯片配置文件、板级实现目录。
- 平台或 SDK 线索也要纳入判断：`.ioc`、CubeMX 工程、`idf.py`、`platformio.ini`、Linux / Webots 目录、厂商 SDK 目录。
- 重点文件集中在 `driver`、`system`、`USB`、`DAP`、`Debug`、协议栈或设备枚举实现时，优先按驱动 / XRUSB / 调试工程理解。
- XRobot BSP 在 `Modules/modules.yaml` 的 `xrobot:` 字段固定 xrobot 的版本；使用 CodeGenerator 的 STM32 工程在 `User/libxr_config.yaml` 的 `generator:` 字段固定 libxr 的版本。运行 `xrobot` 或 `libxr` 命令之前，先用 `xrobot --version`、`libxr --version` 确认安装的版本与这两个字段相同，不同时先安装相同的版本。

## 工程类型判断
- 仓库有 `Modules/modules.yaml` 时，按 XRobot BSP 处理；`xrobot describe` 以 JSON 给出它的配置、锁定模块、注册和诊断。
- 仓库主要围绕 LibXR 集成、平台工程、芯片/板级配置、驱动实现展开时，按 LibXR 平台工程处理。
- 仓库重点是设备接口、协议栈、调试链路、USB/CAN/UART 等实现时，按驱动 / XRUSB 工程处理。
- 仓库主要是代码生成、模板、示例、测试或基准时，按工具 / 示例仓库处理，不套用板级移植流程。
- 仓库里同时存在多类入口时，先列出看到的证据，再决定主入口，不直接跳到某个工具命令。

## 文档入口
- 总入口：https://xrobot.work/docs/intro
- 快速开始：https://xrobot.work/docs/quick_start
- 设计思想：https://xrobot.work/docs/concept
- 环境配置：https://xrobot.work/docs/env_setup
- 基础编程：https://xrobot.work/docs/basic_coding
- 代码生成（CodeGenerator）：https://xrobot.work/docs/code_gen
- 项目管理（XRobot）：https://xrobot.work/docs/proj_man
- XRUSB：https://xrobot.work/docs/xrusb
- 调试：https://xrobot.work/docs/debug

## 入口选择规则
- 只有确认是 XRobot BSP 时，才优先看 `proj_man`、`xrobot setup`、`xrobot describe`、`Modules/`、`User/`。
- LibXR 平台工程优先看 `env_setup`、`concept`、`basic_coding`，再按实际平台进入对应环境页。
- 带 `.ioc` 文件的 STM32CubeMX 工程，外设对象和 `app_main` 由 CodeGenerator 生成，相关规则看 `code_gen`。
- 驱动或设备接口问题转去 `xrusb`、`debug`、`basic_coding/driver`。
- 中间件、消息系统、调度、Topic 等运行时机制问题，优先看 `basic_coding` 下对应章节。
- 当前仓库只是某个平台或某个芯片工程时，按平台工程理解，不按 XRobot BSP 解释。

## 遇到问题时怎么处理
- 先确认问题属于哪一层：环境、工程工作流、运行时语义、驱动/XRUSB。
- 先打开上面的对应文档链接，按文档确认接口和目录结构。
- 文档里没有时，再结合当前仓库代码和命令输出继续判断。
- 仍然无法解决时，整理最小问题描述、命令输出和环境信息后提问。

## 需要进一步求助时
- 补充当前平台、目标芯片/系统、使用的命令、报错原文。
- XRobot BSP 问题优先附上 `Modules/` 和 `User/` 下相关文件状态。
- LibXR 平台工程问题优先附上 `CMakeLists.txt`、`CMakePresets.json`、平台配置文件、`libxr_config.yaml` 等文件状态。
- 驱动、XRUSB、调试或运行时问题优先附上相关源码位置、最小复现代码和日志。

## 补充入口
- 新手任务引导：https://xrobot.work/XRobot-Onboarding/