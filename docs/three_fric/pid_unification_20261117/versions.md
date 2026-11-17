# PID 与发射配置版本记录

当前工程的 xrobot.lock 由 xrobot setup 更新，固定以下模块版本；BSP 分支为 feat/three-fric-launcher。

| 模块 | 提交 |
| --- | --- |
| ControlAlgorithms | `35b9d87408256b0888b057c5f7933869922306ac` |
| RMMotor | `35e72b6281fb224091726cb6add12247ec4910c7` |
| BMI088 | `0de06e30c3a58d8760881112ab73ba31c1514c71` |
| Launcher | `ffdf85bfb224a649864cd319492847a89d63f68e` |
| ThreeFricLauncher | `38a9fb3b7dd5da09c8805679e1bf519a9efdb306` |

BMI088 合并上游 README 七节格式提交后保留新公共 PID 参数说明，未覆盖上游文档工作。

锁定后离线解析通过（13个模块、2个配置），主机测试4/4通过，Debug与Release构建通过。此前板级BMI088生成配置单独编译和云台兼容编译通过。本次未烧录、未进行机构或加热运行测试。

README.md 及同目录补丁为实现阶段留存，所述“未提交”是当时状态；本文件列出后续实际提交。远程CI状态以对应提交的GitHub Actions页面为准，不能由本地构建替代。

后续归档说明：实现阶段补丁已移到忽略目录 build/publication_backups/pid_review_patches；最终模块源码以 pid_unification_20261117/versions.md 列出的提交为准。

五个模块的上述提交均已成功推送至各自远程 dev 分支。
