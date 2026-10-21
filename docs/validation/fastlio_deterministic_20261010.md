# FAST-LIO 首批确定性工程修复

开始于 2026-10-10，收尾于 2026-10-11（Asia/Shanghai）。本轮为隔离的 Integration 工程修复；范围仅 AUD-038 与默认 bundle 路径的 AUD-046。没有进入实车验收，没有实施 AUD-001/003、B1、Mission、Nav2、MPPI、TF 架构或研究恢复。

## 版本、来源与隔离

只读核对远端正式 HEAD：`main=fcd9a05b03c9539b4b5ea44de0f09d30c4468455`、`main-new-car=bd6cf689d3d5aca5c8edd430ff760ca5ceb27496`、`main-old-car=820179457b70789b0f46697758dbe3d947ec1d4a`。三者 FAST-LIO gitlink 都是 `e7864a63a7e2a1ad62ec8fd75fcb9149db08e321`。开始时及收尾前通过 `git ls-remote` 核对，没有假定旧基线仍最新。

修复工作区：`/home/qihei/rm2027_navigation/build/fastlio_deterministic_fix_20261010`；主仓和 FAST-LIO 分支均为 `codex/fastlio-deterministic-fixes-20261010`。从上述新车正式对象创建主仓 worktree，FAST-LIO、ikd-Tree 和 Livox 分别用 `--no-hardlinks` 本地克隆建立独立 checkout/Git dir，未复用其他 worktree 的可写子模块 checkout。正式分支未修改；未 push、merge、reset 其他工作区或冻结 ref。

原 FAST-LIO covariance 研究分支 `experiment/fastlio-covariance-contract` / `experiment/fastlio-covariance-ros-validation` 均指向 `ea63e62b20c5b5cddcaf01df334ffa1da75b3f72`。它只改三个发布入口的协方差表达/顺序；IMU header 未变、三个测量入口仍有 AUD-046。该未迁入修复继续冻结，本轮未导入。

用户指定的五份审计/处置文档实际位于先前审计任务的 `outputs/`，并未进入这次正式基线的 `docs/research/`。已只读复用其结论和测试资产；真实路径与 SHA256 见 [provenance](fastlio_deterministic_20261010/provenance.json)。保留必要的[历史断言与完整 ROS 栈](fastlio_deterministic_20261010/prior_audit/fastlio_ros_diagnostic/attempt2/abort_stack_resolved.txt)，没有复制整套全仓审计或重新开展全仓审计。

## 实际代码

- **AUD-038**：`src/IMU_Processing.hpp` 的 `Process()` 初始化分支在每个初始化测量后设置 `last_lidar_end_time_`。单雷达选当前 `lidar_num` 的结束时刻；双雷达选两个结束时刻的最大值，与后续去畸变结束后写入的切点一致。初始化仍假设静止；下一传播从最后一个初始化测量结束时刻接续，保留末尾 IMU 样本用于跨切点插值，避免重复积分已完成的区间。初始化完成当轮仍返回；后续传播公式未修改。构造和 `Reset()` 同时将初始世界系净加速度 `acc_s_last` 设为静止初态零，补齐审计指出的同组首姿态读取。时间没有设为零；Reset 后必须重新走初始化分支，不能沿用旧 epoch 的结束时刻。
- **AUD-046**：仅 `src/laserMapping_bundle.cpp::h_share_model()` 的两个非法索引赋值改为 `PointCloud::push_back()`。每次 clear 后，仅为接受的点和对应法向量成对建立对象；有效计数仍在同一分支递增，Jacobian 仍按该计数构造。原始 100000 点预分配容量继续保留，没有新增每帧 resize、大块初始化或无条件分配；阈值、搜索、平面拟合、残差、Jacobian 和 ESKF 更新表达式均不变。

默认 `fast_lio_multi_phase2a.launch.py` / `phase2a_lio_bringup.launch.py` / `navigation.launch.py` 选 `update_method=bundle`。async/adaptive 是显式可选入口，没有现用部署选择证据；其 AUD-046 同型根因只读确认，**仍未修复，不能使用本轮结论宣称它们已通过测量路径验证**。共享 IMU 修复会被这两个入口编译使用，但其异步调度/模式切换未运行。本轮不改未选路径，最终 profile 若改选它们必须另行处理。

## 修复前后证据与验证

固定环境：原有 `rm2027_navigation:humble` 镜像 `sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3`；ikd-Tree `0438b0daa6ccfaaad9f65f45a3addc318b19ae7a`、Livox `2a2029a6e62a2196b280be6ec00bb2418065b8e0`。专用非 root 容器仅挂载本任务 worktree，禁网、无设备、私有 IPC，2 CPU/6 GiB，ROS Domain 117。使用真实 Livox driver/SDK 构建及生成消息；未改镜像或共享 ROS 环境。

| 验证 | 修复前 / 对照 | 修复后 |
|---|---|---|
| 首初始化与首次传播 | 原 header 静态证据：首次读时间先于首次赋值；不伪造未定义 double 的特定失败 | 原完整 `ImuProcess` + 真实 ESKF/MTK/PCL/IMU 类型，单雷达、双雷达、lidar2 三路径，各含多初始化帧、首传播、5 连续帧及 Reset 后重新初始化；切点、点数、finite 状态/P 正确 |
| 连续传播数值 | 独立给定剪裁时间区间及输入，调用同一真实 ESKF 过程模型作对照；没有理想滤波器替代 | 最大状态差 `1.135e-15`，最大 P 差 `6.537e-13`；正常时序接续一致 |
| 有效容器边界 | 原始整个 bundle translation unit，正常有限 5 点/121 地图点匹配；`_GLIBCXX_ASSERTIONS` 在原索引写处 SIGABRT；无匹配 exit 0。另复用原完整 ROS backend 的 abort 栈 | 原完整 bundle translation unit 和真实库，36 次测量回调，输入数量 5/30/1/17/5、重搜索/缓存、多迭代及 extrinsic 开关；有效点、法向量 size 和 H/h 行数一致，无越界断言 |
| 数学与输出回归 | 从原 Git 方法生成明确的合法存储对照：仅两次 clear 改 resize(input size)，函数改名；选择/数学表达式原样保留。这是 test control，不冒充未定义旧代码的有效输出 | H、h、非零残差后的真实 ESKF 状态/P 与该对照差 **0**；独立平面解析 H 差 `8.428e-8`（float 平面拟合舍入） |
| 容器分配 | 原容量 100000 | 上述有效数量变化及迭代中容量、存储地址不变；未新增该容器的每帧分配 |
| 完整 backend 构建 | 原版本固定、已有审计构建与诊断记录 | Livox 与固定 FAST-LIO package 构建通过；bundle/adaptive/async 三 executable 均以 O0 + libstdc++ assertions 编译；另对选用 bundle 做默认 Release 构建 |
| 完整 ROS 连续路径 | 历史完整 bundle matched 断言 abort；unmatched 预测仍发布 | 复用 900 点平面/200 Hz 静止 IMU 软件夹具，先初始化/建图后连续 matched，以及移远点云的 unmatched；诊断构建与默认 Release 分别核对初始化、连续 finite raw odom、无匹配路径及正常退出。具体计数见最终结果 JSON |

最终证据见 [结果索引](fastlio_deterministic_20261010/results.json)。测试载体在 [tests/fastlio_deterministic](../../tests/fastlio_deterministic/README.md)，生产方法均完整包含；只有 IMU private 可见性和 backend main 名称作测试访问调整。没有跳过真实滤波更新、关闭功能或改变阈值。原始 ESKF 载体 valid/converge 按库调用合同显式设置。

完整 ROS 诊断和 Release 两种构建，各有 matched/unmatched 两个案例：每例初始化一次、连续发布 6 条 finite raw odom、正常退出 0；unmatched 各有 4 次真实特征拒绝。两种构建各案例的 6 个对齐源时刻、全部 pose 分量差均为 0；这是同一修复代码的构建模式对照，不是旧版未定义行为或实车精度对照。

准备/夹具失败均保留在本 worktree 的 ignored `build/fastlio_validation/`：最初 ROS setup 与 shell nounset 不兼容；夹具缺 deque/include 顺序及非 const 引用参数；首次测量载体 valid 未初始化；首次 ROS 观测未取得足够发布，后一次未先确认初始化/建图即移远点云，不能算 unmatched 正对照。完成版增加有界 DDS 连接/后台等待和建图前缀，明确要求到达真实 `No Effective Points!` 路径。原版本 matched 已 SIGABRT 但此 PCL ABI 没有打印预期断言文字，观察器因此失败；复用审计的 abort 栈观察器确认 `std::vector<PointXYZINormal>::operator[] → h_share_model`，随后委托原 libc abort，未抑制失败。失败尝试不计通过；没有因此改生产算法。

兼容性：没有改变话题、TF、消息、发布顺序、feature threshold 或 ESKF 数学。数值/存储合同对照范围内未发现回归。容器稳定容量证据支持无新增每帧分配；未测目标机峰值负载、长期 CPU/热/内存或实时时限，**不能宣称整车性能无回归**。单平面存在不可观测方向，有限 raw odom 不是定位精度或物理正确性证明。

仍需真实新车输入：MID360 点内时间、LiDAR/IMU 时间同步、IMU 初始化静止条件/噪声/偏置、真实云台与 IMU/LiDAR 外参、实际扫描间隙和调度、匹配退化、目标机负载及实测定位精度。没有获取可信新车录包，本轮合成消息明确仅为软件回归，未运行实体机器人。

## 下一批 AUD-001/003 的只读适用性

目前未取得完整新车实际部署 launch/overrides、地图、supervisor、MCU 清单，仓库也未闭合新车硬件比赛组合。以下是已有版本声明与文档化候选，不能写成现场已部署 graph。

| 候选 | 现有链及下一批范围 |
|---|---|
| 新车 LIO/heading 文档候选 | Phase2A → 默认 single/left bundle → raw `/odometry/fast_lio_raw` → adapter → canonical `/odometry/lio` / `odom→base_link`。heading fusion 的 alignment=false 与 derived 参数覆盖 AUD-060 仍在，本轮不启用或修复。Phase2A 自身不启动 Nav2、全局定位、Mission 或串口 |
| 新车串口 | 文档要求独立显式 `competition_v2` transport，heading 能力 mask 33；V2 无硬件测试入口为 dry-run。它不是完整硬件导航 profile。AUD-003 若正式采用 V2，范围在 V2 实际 cmd subscriber/编码出口；不能将 HPM 当新车已选协议 |
| generic 全局导航 | `navigation.launch.py` 可选 AMCL/GICP + external_pose bridge，canonical `map→odom` owner 仍是 bridge；默认 backend=none/stub，Nav2 默认关闭。generic bridge 的 upstream_valid 默认空，AUD-001 接线与 producer lease 仍适用；GICP registered-cloud AUD-061 保留 |
| generic 速度/障碍 | Humble controller `/cmd_vel_nav` → velocity_smoother → `/cmd_vel`；behavior 直接 `/cmd_vel`，因此存在 smoother 旁路及 behavior/smoother 同 topic 多 publisher（AUD-055）。generic 默认 YAML 是 `nav2_phase2f_deployment.yaml` 的 ObstacleLayer；Phase2G 显式 YAML 用 VoxelLayer，不能默认认定新车选 STVL。该入口还默认 chassis stub=true；实际串口组合及外部 publisher 清单未取得 |
| 旧车 competition（同样存在于 N 分支） | 选择 AMCL/GICP 时 bridge 接相应 backend_valid，controller **和 behavior** 经 scope remap `/cmd_vel_nav` → smoother → `/cmd_vel` → HPM（默认 `hpm_crc_v1`，不能选 V2）；默认 left STVL。没有上表 generic 的 behavior 旁路。AUD-001/003 仍适用；旧 TX AUD-004、STVL AUD-022 是独立条件项。本轮只读复用审计原生参数正对照 |
| Mission | 上述新车 Phase2A/V2/generic 入口没有 Mission owner；旧车 competition 中 use_mission=false、startup_enabled=false，启用需显式选择。未以 disabled 默认证明实际不会有人启动额外 owner |

源码检查可确认上述配置中的旁路，但本轮未启动完整 Nav2/串口 graph，不能排除现场额外节点、RViz/人工工具、bag 或外部进程的第二 publisher。AUD-001/003 下一批必须先固定实际新车组合，分别核对 controller、behavior、smoother、serial 最终 topic；本轮不设计或实施其方案。

## 提交与恢复

FAST-LIO 修复提交为 `ae5bef29c89ba7a1004d75657db482bb277e603a`；bundle 身份见 [provenance](fastlio_deterministic_20261010/provenance.json)。在本任务独立的原版本 checkout 中验证并实际 fetch bundle，恢复出的对象确为该提交，原版本 checkout 仍保持 `e7864a63` 和 clean。superproject 本报告所在提交同时更新 gitlink，完整提交身份可用 `git log -1 --format=%H -- docs/validation/fastlio_deterministic_20261010.md` 核对。bundle 是以原 `e7864a63` 为 prerequisite 的小型增量恢复资产，保留 GPL-2.0 源码历史和许可证；没有上传任何远端。

在**新的独立且干净**工作区初始化原子模块后，可验证并导入：

```bash
git -C src/fast_lio_multi bundle verify ../../docs/validation/fastlio_deterministic_20261010/fast_lio_fix.bundle
git -C src/fast_lio_multi fetch ../../docs/validation/fastlio_deterministic_20261010/fast_lio_fix.bundle refs/heads/codex/fastlio-deterministic-fixes-20261010
git -C src/fast_lio_multi switch --detach FETCH_HEAD
```

回退选定修复，只在新的回退分支/独立 checkout 中将子模块切回 `e7864a63` 并提交 gitlink；完整回到整改前用新 worktree 检出 `bd6cf689` 并初始化其原 gitlink。不 reset 正式分支或他人的工作区。本轮完成后暂停，等待集成决策与真实新车输入。
