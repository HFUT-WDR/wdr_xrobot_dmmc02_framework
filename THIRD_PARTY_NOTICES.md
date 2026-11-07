# 第三方来源与许可声明

本工程由 https://github.com/QDU-Robomaster/bsp-dev-mc02 的 `8c9ee21ace9f7c454a30b81e4edcada857ebd7cd` 演进，保留Git历史和Apache-2.0 LICENSE。
当前独立维护仓库：https://github.com/HFUT-WDR/wdr_xrobot_dmmc02_framework 。
当前基础组件均由WDR独立维护。官方LibXR/XRobot模块保留原上游。
当前组织维护方式见 docs/wdr_organization_ci.md；原迁移记录保留在 docs/private_repositories_20261104.md 。

## EventBinder

私有组件 `HFUT-WDR/EventBinder` 基于 [原始 EventBinder](https://github.com/QDU-Robomaster/EventBinder)，
迁移基线为 `3d4adde3a7c10152a6a0b6c67738b88c972bd675`。原许可证与 Git 历史保留在组件仓库。

## IMU移植

新增 HFUT-WDR/ImuFusion、BoardAttitude、DmImu。原始行为参考26_sentry的板载与DM专用EKF、DM协议；
本次复用MC02_Framework_CPP修正后的移植，保留原作者及来源，未把旧HAL入口搬入XRobot模块。
代码基线、修复和文件散列见各组件THIRD_PARTY_NOTICES.md与tests/imu/reference。
引用的26_sentry快照未附工程根许可证，本工程不替其推定新的许可；组件LICENSE只覆盖WDR新增部分。
ImuFusion模块内保留与基座Drivers/CMSIS同版本的CMSIS源码和原许可证，以支持模块独立编译；算法未改写。

## 发射机构参考代码

HFUT-WDR/Launcher 的发射状态、虚拟热量、弹速与卡弹流程参考26_sentry。
主机回归使用的原始 shoot_task、heat_control 和 LPF 文件冻结在 tests/launcher/reference，
SHA-256 记录于同目录 hashes.json；保留原文件内容，不将其重新声明为WDR原创。
模块重组、设备接入和行为差异见 Launcher 仓库的 README 与来源说明。
上述26_sentry参考代码沿用已有来源边界，本工程不替原始代码推定新的许可。
