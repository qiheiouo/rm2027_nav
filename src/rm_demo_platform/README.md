# RM2027 Demo 集成环境

本包属于 `integration/demo-navigation-validation`。首轮只提供 **无硬件软件 bench**，
用于组合现有 V2 codec/transport、直接相对 Yaw、RSP、LIO adapter；真实固件、机械外参、
完整底盘姿态输入和运动授权尚待确认。[完整审计与分阶段准入](../../docs/validation/demo_navigation_integration_20261111.md)。

## 运行

优先复用已有 `rm2027_navigation:humble` 镜像。从仓库根目录执行：

```bash
# 局部构建 + 既有数学/协议测试 + 集成检查；完成后退出
src/rm_demo_platform/scripts/run_bench_container.sh check

# 同样局部构建，持续运行供软件观察；Ctrl-C退出
src/rm_demo_platform/scripts/run_bench_container.sh run
```

脚本使用非 root、禁网、私有 IPC、无设备映射的容器；源码只读，构建与日志保存到
`build/demo_validation/`，不使用当前工作区的 Jazzy install。镜像缺失时停止，不自动拉取或改系统。
这不是 MiniPC 实物验收；宿主 Ubuntu 版本不替代容器内 ROS 版本，也不能免除网卡/PTP/USB 配置。

在已经构建并 source 的**独立无硬件 ROS 环境**中，也可执行：

```bash
ros2 launch rm_demo_platform demo_bench.launch.py
# 默认零节点；下式显式启动软件夹具
ros2 launch rm_demo_platform demo_bench.launch.py enable_test:=true
```

不与真机、仿真或其他 canonical TF owner 共用 ROS domain。脚本的 `check` 模式会注入非零
`/cmd_vel` 验证 V2 仍拒绝授权，因此检查器只允许在脚本提供的隔离环境中运行。

## 链路与已知范围

```text
V2 mock lower → mock bytes → 原 V2 transport(dry_run, motion_test_mode=false)
  → /gimbal/state → 原 gimbal_state_adapter(use_input=true)
  → /joint_states → RSP → base_link → gimbal_yaw_link → MID360 → IMU
原 fake_lio_odom_publisher → /odometry/fast_lio_raw
  + exact timestamp mechanical TF → 原 lio_adapter → /odometry/lio + odom→base_link
原 map_odom_stub → map→odom（仅夹具）
任意 /cmd_vel → V2仍只发送zero-disabled mock帧
```

仅有一颗逻辑 MID360。使用原有 `mid360_left_frame` 命名是复用单雷达接口，不代表第二颗雷达。
无 Livox driver、真实 FAST-LIO、Nav2、Mission 或真实串口；IMU frame adapter 等待输入，不伪造IMU。
`bench.urdf` 与 `bench.yaml` 是配对的合成测试资产：有非零安装RPY/杆臂、0.25rad相对Yaw，
底盘姿态roll=0.12、pitch=0.22、yaw=0.4rad。**不是 Demo 图纸、footprint、标定或动力学参数。**
250mm中心圆盘和突出的四个轮子待实测后另建模型；不从该圆盘直径推断导航安全包络。

检查器验证默认零节点、完整base位姿恢复、5条TF边及期望发布者集合、CRC与零/disabled命令，
并在mock角度停止后验证joint/canonical输出暂停、raw和串口仍运行，以及正常进程退出。
这只证明软件时间/TF边界，不能证明真实同步精度、动态deskew、坡道LIO质量或物理停车。
mock角度恒定，不将本检查称作“旋转云台验收”。

## 接入实物前

依次确认固件/profile/CRC/独立停车、完整chassis attitude扩展、源时钟、实测机械链/单MID360配置，
再按审计中的Phase1→2→3准入。复用既有GimbalState，不新增Demo串口协议。
物理采集优先迁入已有ACQ时间溯源与bag配置；固定云台LIO需处理main已知delayed-zero-start/基准问题。
独立Demo Nav2/MPPI参数与hardware launch在这些事实明确后添加，不能将本夹具切换为实车配置。
main已有反应式导航功能，不因缺标定而重新写导航算法。
