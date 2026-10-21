# FAST-LIO covariance publication contract：ROS 系统验证

日期：2026-10-08。结论：**raw publication boundary Go；整体 canonical covariance Modify**。
这只是单时刻 marginal 的发布、表达和条件几何传播验证。没有启动定位 Task 1 的新工作或 Task 2；没有 EKF 数学模型修改、实车标定、factor graph information、跨时间独立性或 GICP/map 与 LIO 独立性结论。

## 1. 固定版本、工作区与恢复

| 项目 | 固定版本 / 处理 |
|---|---|
| superproject base | `d5673e3a1cfe6937e4126e1df894ee6d66e63e07`，现有 simulation baseline |
| 本分支 | `experiment/fastlio-covariance-ros-validation`，直接从上述 base 建立；没有 merge covariance / NEU 分支 |
| worktree | `/home/qihei/rm2027_navigation/build/fastlio_covariance_ros_validation` |
| base FAST-LIO gitlink | `e7864a63a7e2a1ad62ec8fd75fcb9149db08e321` |
| candidate gitlink | `ea63e62b20c5b5cddcaf01df334ffa1da75b3f72`，old 的直接子提交；原 covariance 研究的候选，未再修改 |
| 独立 FAST-LIO branch | `experiment/fastlio-covariance-ros-validation`，指向 ea63e62；不是原子模块分支 |
| 独立 FAST-LIO Git dir | `.git/worktrees/fastlio_covariance_ros_validation/modules/src/fast_lio_multi` |
| ikd-Tree / Livox driver | `0438b0daa6ccfaaad9f65f45a3addc318b19ae7a` / `2a2029a6e62a2196b280be6ec00bb2418065b8e0`，本 worktree 独立 checkout |
| main-new-car 对照 | `origin/main-new-car@bd6cf689d3d5aca5c8edd430ff760ca5ceb27496`；本地没有同名 local branch；base 与该 ref 的 FAST-LIO、adapter 路径差异为空 |

开始前只读检查所有 worktree 与已初始化 submodule 的状态；primary 和 R4 有其他任务的已有改动，未触碰。并行任务后来推进了其自己的提交，不能把这些变化归为本任务。没有 checkout/reset/clean 其他 worktree，没有修改 main、main-old-car、NEU、动态预测、MPC 或其他研究分支。完整只读清单、源码/二进制 SHA 和子模块 common dir 见 [provenance](fastlio_covariance_ros_validation/evidence/provenance.json)。

恢复提交对象使用原研究保存的 bundle，随后切到本任务子模块分支；本分支提交 gitlink，子模块内部没有未提交改动。为避免依赖另一个 worktree，bundle 已复制到本分支的 [实验目录](../../experiments/fastlio_covariance_ros_validation/fast_lio_candidate.bundle)。它是增量 bundle，前提是已有 old commit；不是完整 FAST-LIO 仓库备份。

仅在**本任务新建且干净的 FAST-LIO checkout**中恢复 candidate：

```bash
git -C src/fast_lio_multi bundle verify ../../experiments/fastlio_covariance_ros_validation/fast_lio_candidate.bundle
git -C src/fast_lio_multi fetch ../../experiments/fastlio_covariance_ros_validation/fast_lio_candidate.bundle refs/heads/experiment/fastlio-covariance-contract
git -C src/fast_lio_multi switch -c experiment/fastlio-covariance-ros-validation ea63e62b20c5b5cddcaf01df334ffa1da75b3f72
```

回到旧发布实现时，在本任务自己的干净 checkout 中 `git -C src/fast_lio_multi switch --detach e7864a63a7e2a1ad62ec8fd75fcb9149db08e321`，然后**同时提交该 gitlink 回退**，不要把 superproject 留在 dirty 状态。完整恢复本轮前状态可在一个新 worktree 使用 base d5673e3a 并按其记录的 gitlink 初始化；不用 reset/clean 原工作区。本次没有实际执行回退、merge 或 push。

## 2. 环境与证据范围

复用已有 `rm2027_navigation:humble` 镜像，image SHA `81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3`。镜像内已有 Livox SDK、Humble、PCL、Eigen、colcon；没有安装到其他任务环境，也没有改原镜像。

专用容器 `rm2027_fastlio_covariance_ros_validation`：仅 bind 本 worktree、network=none、IPC=private、2 CPU/6 GiB、Domain 116、localhost-only、Fast DDS (`rmw_fastrtps_cpp`)。build/install/log/runtime 均在本 worktree 的 ignored `build/` 下；未启动共享 ROS daemon。实验目录有 COLCON_IGNORE，普通全仓构建不主动发现研究 probe；probe 用显式 `--paths` 构建。收尾移除本任务容器，保留本地构建与日志。

实际完整构建了 Livox driver 和 FAST-LIO package 的 `laserMapping_bundle`、`laserMapping_adaptive`、`laserMapping_async` 三个原 executable。首次 native Jazzy 尝试缺 Livox SDK，已弃用；临时接口替代只在 ignored build 中，最终 Humble 测试使用原 driver 的真实生成消息和 SDK。初次 colcon 扫到临时重复包、adapter 缺 rm_description、bridge 缺 rm_mid360_driver_bridge；随后补齐隔离依赖并构建通过，不能把这些未完成尝试记作通过。

ROS publication fixture 包含**完整、未改动的各 production translation unit**，只重命名原 main，并在滤波后边界赋值真实 23×23 ESKF 的 state/P。随后调用原 `publish_odometry()`，走真实 rclcpp publisher、executor、跨进程 DDS、rclpy subscriber；没有 fake publisher/get_P/message，测试没有复制 covariance 算法。bundle 类内部的 zero-start pose 准备块按字节原样提取，SHA 见 [source manifest](fastlio_covariance_ros_validation/evidence/source_manifest.json)。

这是 **post-filter publication boundary injection**，不是传感器仿真，也不是原传感器 callback→去畸变→EKF 的闭环。原三个 executable 另做真实启动/discovery smoke，无点云/IMU输入，预期没有 Odometry。fixture 给定 stamp，不能证明真实传感器路径里 stamp 与状态物理有效时刻的误差。adaptive 自动方法切换及 async 调度的实际传感器运行**未执行**。

现有 Gazebo regular GPU cloud 没有可信 MID360 点内时间、同步 IMU 和去畸变输入。本轮复用已有软件测试，没有重新开发传感器仿真或 LIO 算法，也没有重跑与本合同无关的导航场景。

## 3. 三种发布入口

| 入口 | 完整 package 构建 | 原 executable 启动 / executor discovery | 实际 ROS publication boundary | 未运行部分 |
|---|---|---|---|---|
| bundle | PASS | PASS；无传感器消息时零 Odometry | 11 条：首帧 diagonal、dense rotated、连续更新、重复/倒退/future/zero stamp；zero-start 延迟、激活、后续、关闭 | 真 MID360/IMU滤波、去畸变和物理统计表现 |
| adaptive | PASS | PASS；同上 | 7 条：上述共同 fixture；原 adaptive publish 函数 | 自动 bundle/async 模式切换和真实滤波调度 |
| async | PASS | PASS；同上 | 7 条：上述共同 fixture；原 async publish 函数 | 异步传感器更新/调度及物理有效时刻 |

adaptive/async 没有 bundle 的 zero-start 实现，不把不适用项算通过。源码注释里的“ONLY adaptive”并不代表 CMake 只有一个 executable；本轮实际构建并执行了三个。

native layout 为 `[dp_W, dphi_B]`（position first，rotation right/body tangent），使用真实 P 的前 6×6 marginal。ROS 顺序为 `[x,y,z, rotation-X,Y,Z]`，row-major，pose parent=`header.frame_id`，twist child=`child_frame_id`。消息定义的官方依据：[PoseWithCovariance](https://raw.githubusercontent.com/ros2/common_interfaces/humble/geometry_msgs/msg/PoseWithCovariance.msg)、[Odometry](https://raw.githubusercontent.com/ros2/common_interfaces/humble/nav_msgs/msg/Odometry.msg)。本合同的 orientation covariance 是当前姿态附近的 parent-fixed **局部 infinitesimal tangent**，不是任意大旋转的 Euler angle 协方差。

独立期望 `J=diag(A,R_FB)`：无 zero-start 时 A=I；激活后 A 为冻结的 W→chosen-parent 旋转。每条消息核对当前 pose、P、全部 36 项 cross terms、源 stamp、parent/child；同消息关联的实际 `/tf` pose/frame/stamp 也核对。candidate raw 最大 covariance 元素误差 `1.38344e-16`，pose matrix 误差 `2.38698e-15`。连续 P 比例变化没有上一帧 covariance；首帧没有遗留零矩阵。

zero-start 激活帧 pose=identity 但 covariance 不为零：这是把 native marginal 用**固定选定的数值基准**重表达。没有计算“当前状态相对于一个随机初始状态”的真实协方差，也没有 P(t,t0) 或初始状态误差传播。不能把非零 marginal 解释成已解决随机锚点问题。

## 4. DDS / ROS transport 与时间、epoch 边界

两轮 25 条 raw 消息各自均收到一次，应用接收次序与发送次序一致，36 个值及 frame/stamp 满足给定当前 fixture。candidate 的命令发送→C++执行→DDS→订阅接收 roundtrip：bundle 中位 1.193 ms / 最大 1.554 ms；adaptive 1.301 / 2.156 ms；async 1.412 / 1.930 ms。这包含两段 transport、调度和测试代码，**不是纯 DDS 单向延迟或负载性能门**。仅小样本单 writer、同容器不同进程；没有跨主机、多 writer、拥塞、长期 QoS/reliability 性能验证。

重复 source stamp、倒退 stamp、未来 stamp、零 stamp 被按给定值原样传输；这是字段保持的正例，也是 freshness 合同尚未实现的负例。DDS 没有替应用检测这些错误。本测试没有把源码输入重复说成 DDS 自发重复，也没有发现这组样本中额外丢失/重复/交付乱序。

原 passthrough adapter 和本轮候选均把 stale/duplicate/reverse/future/zero source stamp 转发；本轮没有扩大成通用时间 gate。finite_difference 保留首帧 warmup 和异常 dt 检查；heading boot/reset 变化实际 ROS 测试确认 latch invalid，不再输出。

bundle 的 zero-start 前后使用同一个 frame 字符串，数值 basis 已改变；没有 epoch ID、重基准事件或关联 transform 元数据。receiver 无法只靠 frame/stamp 判定 basis continuity。raw 每帧局部表达可正确，但跨切换累计/缓存/增量消费仍是 **Modify**。conditional adapter opt-in 要求调用方保证固定 parent basis，不提供自动 epoch 检测。

## 5. lio_adapter 反例与最小修复

原 baseline adapter 源码 SHA 和二进制 SHA 已保存；修复前走实际 raw FAST-LIO boundary→ROS→原 adapter→ROS，修复后用同样通道和独立参照。参照通过左右微扰 pose、对 sensor→base SE(3) 变换做有限差分得到 J，而不是重复调用候选 helper。

| 情况 | 原 adapter 实际行为 / 反例 | 本分支处理 |
|---|---|---|
| canonical parent，非零 lever + mount rotation | pose=`S E^-1`；直接复制 P；与正确矩阵 max 元素差 `0.0131336386572` | opt-in 完整条件传播，FD max error `7.35193e-12` |
| 动态云台、同时间 TF | 当前 E 改变但 P 不变；漏 lever/cross terms | 每帧使用对应 TF snapshot 传播；仅 conditional on joint/TF，未包含其不确定性 |
| sensor_initial parent rotation | pose=`E S E^-1`，P直接复制；FD差 `0.0140209003385` | all36 unavailable；不重写已有 pose 算法 |
| heading fusion | 直接保留 raw cross terms，只把 yaw diagonal=.01 | all36 unavailable，移除单项 yaw 替换 |
| default / parent 仅改名 / latest TF / placeholder | 复制 P，给错误的置信度外观 | all36 unavailable |
| 输入 asymmetric / indefinite / all-zero / NaN | 原路径未验证并继续发布 | all36 unavailable，最小 consumer 拒绝 |
| zero source stamp / zero-start | 原行为未表达 covariance 有效性边界 | 零 stamp 不传播；默认 zero-start 全36 unavailable；basis epoch gate仍缺 |

heading 反例：输入原 PSD 最小特征值 `0.00549369781275`；只改 yaw 后输出最小特征值 `-0.0172011904656`。输出的 roll/pitch mean 已被强制归平面，但它们的 raw covariance/cross terms 仍被复制。这不是合法融合置信度。仅修 yaw diagonal 不能补齐 heading/LIO/startup 的 joint covariance。

最小候选新增 `pose_covariance_mode`：

- `unavailable`（默认）：36 个 NaN。ROS PoseWithCovariance 无通用 unknown sentinel；这是本项目显式约定，不使用 IMU 的 -1 sentinel，也不以全零冒充确定性。covariance consumer 必须拒绝；pose-only consumer 可继续消费 pose。JSON evidence 中非有限值编码为 null，ROS 接收端先断言了全部为 NaN。
- `conditional_sensor_tf`（显式 opt-in）：仅 `sensor_tf + canonical_odom`、parent 名与输出一致、stamp>0、真实同时间 TF（或 direct base pose），允许传播。要求调用方保证相同且稳定的**数值**parent basis；相同名字本身不是充分证明。
- `l=p_base-p_sensor` 在共同 parent 中，`J=[I,-hat(l);0,I]`，`P_base=J P_raw J^T`。不把 right-fixed mount rotation 错乘到 parent-fixed orientation；全部 cross terms一起传播。仅容忍相对 roundoff 的对称/PSD误差，没有 covariance floor、regularization 或求逆；合法 singular marginal也可保留。
- sensor_initial 的当前 E 同时用在启动和当前两端、heading 初始化参考、heading测量噪声/相关性、动态joint噪声都未具备完整合同，仍 unavailable。旧 yaw_variance 参数保留可读但不再作为 fused covariance。

default NaN 是本实验分支的有效性约定变化，未经其他 covariance 消费链兼容性验收，不应直接合并主线。实测 pose-only bridge 接受有效 pose + NaN covariance，并按相同 pose 算出相同 map→odom。没有运行完整 Nav2/Gazebo 回归，旧 simulation Go 不能自动继承到这个候选的全导航验收。

## 6. /odometry/lio 字段与下游

| 字段 / 模式 | 本轮可信程度 |
|---|---|
| raw FAST-LIO 36 pose covariance | ROS布局、给定当前 state/P 的单时刻表达成立；不是已校准统计量 |
| canonical default、heading、sensor_initial 等 | 全36 unavailable，无可用 covariance diagonal 或 cross terms；yaw也不例外 |
| canonical opt-in、稳定共同parent、同时间已知TF | 全36 条件 marginal 的数值传播可信；不含joint/外参/初始锚点误差，未经实车校准 |
| finite_difference twist covariance | `[1,1,1,4,4,4]` 等配置名义 placeholder；未传播 cross-time/P(t,t-1)，不可作为已验证统计 covariance |
| FAST-LIO twist passthrough | backend留空/零，不构成有效速度统计置信度；没有在本轮修复 twist 数学 |
| frame/stamp | 字段保持可信；数值basis continuity/freshness须另行验证 |

实际 ROS 最小审计 receiver 对 canonical 条件结果检查 frame/layout/finite/symmetric/PSD；拒绝 unavailable/invalid，未求逆、未生成独立增量 factor。真实 production consumer `map_odom_from_global_pose` 只用 pose，改变/置NaN covariance不改变同pose的修正；没有把 LIO 与 global pose 信息矩阵相加。

真实 `global_pose_gate_node` 对合法 covariance 放行，对NaN拒绝，但对“正对角、整体非PSD”的独立 PoseWithCovarianceStamped 反例仍放行。它目前是有限/对角阈值 gate，不是完整矩阵有效性验证；本轮记录为待改，未顺便重构全局定位 gate。

源码审计还确认：GICP covariance 是 fitness 派生的 heuristic diagonal，不是统计标定；simulation 的 `sim_chassis_heading_lio_source.cpp` 在变换 synthetic sensor pose 时也复制输入 covariance，其 covariance 不在本轮 Go 内。上述替代source不是本次FAST-LIO fixture链，本轮未修改或借它宣称真实LIO性能。

检查的消费路径没有用 raw marginal 逆矩阵作为独立增量 information；这不代表所有未来/未运行consumer都正确。没有启用 factor graph，也没有验证地图/GICP和LIO观测独立；共享输入/地图相关性仍必须保留。

## 7. 软件测试、执行修正与证据

现有 adapter 15 个 canonical tests、6 个新增 covariance tests，以及 bridge 的既有 tests/pytest：**42 tests，0 errors / failures / skipped**。[结果](fastlio_covariance_ros_validation/evidence/software_test_results.log)。新增覆盖 identity origin、diagonal lever、dense covariance 独立FD、无效矩阵/非有限外参/overflow、合法singular，不测试EKF模型。

ROS结果：[baseline](fastlio_covariance_ros_validation/evidence/baseline_ros.json)、[candidate](fastlio_covariance_ros_validation/evidence/candidate_ros.json)。baseline 234 / candidate 233 个check全部通过；数目差源于baseline heading反例额外检查最小特征值。其中 check 数包含进程启动、discovery、收发、矩阵和metadata等断言，不应换算成实车试次。baseline证明错误存在，candidate证明条件传播与不可用边界；并非所有断言通过就意味着完整系统Go。

初次 heading fixture 没增加 sample_sequence，被真实节点按合同丢弃；修正夹具后取得实际 covariance 反例。下游尚未构建时启动失败，以及测试端最初误写 map_to_odom_topic 参数造成等待超时，都不是生产covariance失败或通过；修正后完成实际consumer检查。原尝试和构建失败保留在 ignored runtime logs，最终有效观测另存JSON。后来加入固定Git源码独立baseline executable，避免复验时checkout或覆盖candidate。

## 8. 最终判定与真实输入门

**Go**：三种候选原 publish 函数的真实ROS publication boundary，以及给定固定parent与TF时的完整条件sensor→base传播。

**Modify**：整体 covariance 系统。canonical heading/sensor_initial缺joint/startup统计合同；zero-start缺epoch；passthrough缺freshness；下游gate缺full-matrix有效性；twist仍placeholder。不能称为整体 System Simulation Go / canonical covariance Go，更不能解冻 Task 1或进入Task 2。

必须等待真实 MID360 / 新车输入验证：点内时间与扫描末时刻、IMU时钟/同步与noise/bias、去畸变与async/adaptive真实调度、IMU/body与云台/车体home/轴/lever外参、动态joint/heading噪声和相关性、基准切换/重启生命周期、实际stamp有效时刻、长期DDS负载与延迟、对独立真实GT的marginal一致性/标定。跨时间P、随机初始锚点和map/LIO相关性需要另外研究，不能由本轮transport测试推出。
