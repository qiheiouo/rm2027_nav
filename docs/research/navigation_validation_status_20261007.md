# 导航验证状态审计：2026-10-07

本报告审计仓库中**已有的实现与证据**，不接线、不合并其他分支、不解冻实验。新车统一仿真的 S2 改动与结果另见 [S2 报告](new_car_simulation_baseline/s2_collision_passability.md)。没有开始 MPC 开发，也没有恢复东北大学 Task 2。

## 审计口径与仓库快照

- 当前实验起点：`experiment/new-car-simulation-baseline@89e2e9fab3fd0884bd3963294580392efcf33ee0`；继续原分支。
- 比较目标 N：`origin/main-new-car@bd6cf689d3d5aca5c8edd430ff760ca5ceb27496`。实验 worktree 中的 S1/S2 **尚未进入 N**。
- 老车 O：`origin/main-old-car@820179457b70789b0f46697758dbe3d947ec1d4a`；main：`fcd9a05b03c9539b4b5ea44de0f09d30c4468455`。
- 枚举 97 个 local / remote-tracking ref，逐项索引树中的导航包和报告；只读查询远端 73 个 heads，均与已有对应 tracking ref 一致。未 fetch、merge 或 push。[快照](navigation_validation_audit_20261007/repository_snapshot.json)
- 查看重要模块的源码位置、历史提交、结果型报告及冻结记录；`Expected`、验证计划、建议命令、旧报告中的下一步均不算验证完成，也不作为本轮执行指令。
- 以下历史实车/回放数值是**固定 commit 中报告记录的证据**。本轮没有重取外部原始 bag、没有重新执行历史实车实验，不声称重新独立验算其原始数据。保留失败、版本差异与不可比较条件。
- 实验分支不在 N 的祖先链，不能推出其每个模块都不在 N；共享实现需比较具体路径/版本。FAST-LIO gitlink 是一个明确例子：N 和老车高转速分支均为 `e7864a63a7e2a1ad62ec8fd75fcb9149db08e321`，变化主要在驱动时序、adapter、全局修正和 profile。

### 状态定义

| 状态 | 本报告的含义 |
|---|---|
| A | 有真实车辆或真实数据证据的**特定版本/修复**，尚未进入 N；可以是失败证据，不等于可合并 |
| B | 最强证据仍为源码/单测、离线/合成或 Gazebo；没有该版本的实车验证 |
| C | 已有实现，但缺少本轮新统一场景下的系统验证；可与 A/B/D 的有限证据并存 |
| D | 实现在 N，且对表中明确限定的能力已有充分证据；不等于新车硬件或整个模块已验收 |

这四类不能简单互斥：同一模块可能有 N 中的有界 D 基础、N 外的 A 修复及新车 C 缺口。表中按版本拆行，并明确最强证据类型。**本次没有找到足以宣称“新车完整导航链已实车验收”的 D 项。**

“当前仿真能否验证”指 S2 入口：Jazzy/Harmonic、四轮接触代理、实际关节与 full SE(3) truth、规则局部 fixture / 2025 mesh、regular GPU cloud、最小静态 Nav2。它目前没有真实 Livox pattern / 点内时间 / IMU、没有 competition_v2 的硬件闭环、没有新场地多层导航图，也未安装 STVL。仅源码存在不能当作节点已在这个入口启动。

## A：N 外的真实车辆 / 真实数据版本

| 模块 | branch/commit | 当前实现位置 | 最强证据 | 是否实车 | 是否已进 main-new-car | 当前仿真能否验证 | 建议下一步 |
|---|---|---|---|---|---|---|---|
| FAST-LIO 输入时序、LIO adapter input health；A+C | `origin/fix/old-car-fast-lio-high-spin@46c521b2` | 该 ref 的 `src/rm_mid360_driver_bridge/`、`src/rm_localization_adapters/`、`src/rm_lio_bringup/config/*old_car*` | E1：native CustomMsg 20 Hz 老车实车包 130.29 s；高速连续 52.751 s，峰值 7.273 rad/s；raw age P99 38.6 ms，1303 health 样本 healthy。保留 native50 backlog FAIL | **老车实车主动运行**；不是新车 | FAST-LIO 同 pin 已有；native/input-health/profile 修复版本未进入 | 可测 stale/future/reset/jump 的软件合同；不能复现真实 FAST-LIO backlog/去畸变质量 | 先独立 adapter 输入故障测试；将原生 CustomMsg 时序与车辆标定作为后续真实数据门，不整分支合并 |
| 高转速 global hold / correction gate / 停车恢复；A+C | 同上；`origin/fix/old-car-amcl-correction-gate@e15b6bda` | `src/rm_relocalization_bridge/`、old-car bringup/profile | E1：高速期间两个 validity 全 true，map→odom 最大间隔 61.8 ms；停车恢复最大单步 .159 m/.205 rad。E2：四次真实错误修正被隔离；更早 home 包出现 12.15 s TF 中断 | **老车实车**，有失败及有界成功 | N 有基础 gate/bridge；此修复版本未进入 | 可测可信 correction 保持、拒绝计数和 TF/状态时序；不能证明真实转速下位姿准确 | 验证状态→运动许可→唯一最终速度链；“最高速结束立即 home”仍未通过，不继承老车阈值 |
| AMCL high-spin 模型/strict SE(3) deskew 候选；A+C | `origin/fix/amcl-high-spin-candidate@2f5ae666`；E1/E2 的后续 ref | `src/rm_relocalization_bridge/config/*spin_robust*`、`src/rm_mid360_driver_bridge/src/pointcloud_deskew.cpp` | E3：真实 trial02 bag 的固定 seeds 回放 100/100 安全；后续实车再次失败，促成 E1 的 LIO/hold 修复 | **真实 bag/replay + 老车后续实车**；不能把 100 seeds 算 100 实车试次 | 显式旧 candidate 与 deskew 基础已继承；后续健康/hold 版本未进入 | 可测几何/时间拒绝；regular GPU cloud 无点内时间，不能证明 Livox deskew | 先 audit 实际输入字段/clock/coverage，再做有真实时间模型的回放；不把 alpha/阈值直接用于新车 |
| Nav2 / STVL 动态全局重规划与 BT；A+C | `origin/fix/old-car-global-dynamic-replanning@9b608b9d`；O | `src/rm_nav_config/config/nav2_old_car_2026_*stvl.yaml`、`behavior_trees/old_car_replan_on_follow_failure.xml` | E4：2026-08-06 老车受控动态阻挡时停车、障碍持续占用后改路通过；左 scan 写 global 动态层，FollowPath failure 触发 replan | **老车实车主动导航** | N 有 Nav2/STVL 和基础 BT；此重规划 profile/BT 版本未进入 | 当前最小图只有 static+inflation；加受控输入与已审计 profile 后才可复验；不能直接 claim STVL 已测试 | 优先验证终端阻挡→重规划→任务结果，同时记录真实 controller odom 与最终速度 owner |
| 无标注坡道 perception 共享 tracker；A+C | `origin/fix/old-car-ramp-live-tracking@1525bb5e` | `src/rm_mid360_driver_bridge/src/automatic_ramp_filter_node.cpp`、`ramp_plane_filter.cpp` | E5：真实 123.937 s / 84325 消息 bag，原故障复现；修复后同包两次回放，50 s 平地零误删，两路同 plane 删除帧 4295/2252。**修复后主动实车验收 0%** | **真实 bag/replay**；修复后不是主动实车通过 | N 有旧规则 synthetic filter；该自动共享检测修复未进入 | GPU cloud 可增加坡面/真实碰撞障碍保留证据；需要独立节点接入，不能套用旧场景固定平面 mask | P2 做 false-delete/坡面障碍保留；缺 /cmd_vel、/plan 的原 bag 不能验证 MPPI 坡道跟踪 |
| 动态障碍 tracker / short-horizon prediction；A+C | `origin/fix/hwsentry-tracker-field-gates@c04278f3`，代码修复 `cc254b4e` | `src/rm_dynamic_obstacle_tracking/`（N 不含此包） | E6：老车左 MID360、真实 map/scan 的 D01–D07 shadow；同 bag 修复回放改善 D01、资源。**总体 FAIL**：D05 ID fragmentation 27；D07 3.781 false tracks/min、P95 23.883 ms 未过门 | **老车 shadow 实车 + 真实回放**；未接控制 | 否 | 当前无受控动态场景/tracker 接线；可新增可观测静态残差/遮挡/交叉证据，不能替代真 scan 分布 | 保持 shadow，先固定 map/footprint/观测定义再做少数可判定场景；不能因“有实车”就进入 MPPI |

## B：尚无实车验证的研究候选

旧车 recovery 依赖已有可信correction/局部guess与停车；`origin/fix/old-car-lio-jump-recovery@51e3b233`的bounded-rebase也是这个有界路线。现有证据没有给出经过独立GT sweep的最大恢复位移/角度，**通用LOST、kidnapped、错误首次基线、大范围对称basin和真正LIO发散的恢复范围均UNKNOWN**。阈值1 m或容差.75 m不是可恢复范围。新仿真可以增加故障时序与错误接受证据，不能直接补上真实地点召回能力。

| 模块 | branch/commit | 当前实现位置 | 最强证据 | 是否实车 | 是否已进 main-new-car | 当前仿真能否验证 | 建议下一步 |
|---|---|---|---|---|---|---|---|
| FAST-LIO covariance contract 修复；B+C | `experiment/fastlio-covariance-contract@04364d50` | `experiments/fastlio_covariance_contract/`、该 ref 的 FAST-LIO pin `ea63e62`；N 仍 `e7864a63` | E7：实际源码 C++ probe 14 cases；修正当前 P 在 publish 前更新、pose frame contract。**没有 ROS executor/消息 transport、cloud、bag、整包构建或实车验证** | 否；源码/数学 probe | 否 | 可测 native→ROS→adapter 顺序/布局及退化输入；无法证明 EKF P 校准、统计一致性 | P1 优先打通可观测消息链并保留 baseline；不把 source-only Go 升成系统 Go |
| full-base heading adapter；B+C | `experiment/full-base-heading-adapter@5733f126` | `experiments/full_base_heading_adapter/adapter.py` | E8：31 候选测试、100 固定 seed full SE(3) 随机样例；现行 canonical 15 项；倾斜启动、时差、双根/奇异反例。现有 sensor_initial helper 在动态 joint 两端外参反例虚构 2.131 cm/34.377°运动 | 否；**离线/合成** | 否；N 只有现行 planar 候选 | **能**用真实仿真 joint 与 full pose 增加边界证据；truth 只能生成已声明输入/评估，不能偷偷选数学分支或假定正确重力标定 | P1 单一候选 shadow、倾斜启动/云台运动/时差；仍不证明 IMU origin、home、真实 clock |
| Seeded GICP / L1 discriminative audit；B+C | `experiment/neu-localization-baseline@bfc23546`，L1 `3d20656a` / L1-M `4f438f7a` | `experiments/neu_localization_l1/`；生产 seeded backend 在 N 的 `src/rm_gicp_relocalization/` | E9：生产 PCL seeded GICP 的 synthetic fixtures 34 configs；24 接受中 6 错误。12 m 重复场景错误 basin 仍 low fitness/full rank，通过质量门 | 否；**离线/合成** | GICP 基础有；L1 私有候选/分析不在 N | 可做 field mesh/cloud 的错误 seed / alias 反例；当前未启动 GICP，也不是新车实测 bag | P2 对生产 baseline 做少量跨 basin 负例；保持东北大学路线冻结，不推进 Task 2 |
| Scan Context / FPFH / Top-K + multi-seed GICP；B+C | 同 NEU ref，L2 `976b249e` | `experiments/neu_localization_l2/` | L2 报告：descriptor / multi-basin fixture、模拟 yaw/gimbal 归一化；没有目标新车真实 cloud/bag；路线冻结 | 否；离线/合成 | 否 | 能增加 synthetic recall/alias 证据，不能解决真实数据缺口 | **本轮不恢复**；未来必须先有真实新车输入，不列作立即实施待办 |
| factor graph / multi-frame recovery backend；B+C | 同 NEU ref，Task 1 报告 | `experiments/neu_factor_graph/` | E9：SE(3) batch cost、因果/source-order、toy 多时刻 covariance probe；Task 1 **Modify，等待真实输入**；无真实跨时间校准 | 否；离线/合成 | 否 | 能检查接口因果性；无法由 oracle 证明真实残差/独立性/协方差 | 保持 Task 1 结论与 Task 2–6 未启动边界，不借当前 sim 解冻 |
| GICP degeneracy / quality gate；B+C | `origin/feature/gicp-degeneracy-gate@6a38b745` | 该 ref `src/rm_gicp_relocalization/` | centered point-to-plane 近似 H、overlap、min eigen/condition；源码/测试；E9 有全 rank 错配反例 | 否；源码/单测及合成反例 | 质量增强版本未进入；seeded core 已有 | 可以测试稳定错误接受，但此 H 不是 GICP 原生加权 Hessian 或 calibrated covariance | 保留“几何可观测不等于正确 place”；先审计 wrong-basin gate 作用，不自动迁移阈值 |
| localization integrity shadow；B+C | `origin/feature/navigation-integrity@64870dc7`；后续 HWS ref | `src/rm_navigation_integrity/` | E10：第一版 Windows 13 tests + 1 skipped；后续 Linux 接口链；已有 field plan，未找到 integrity 自身正常/失败区分的实车接受结果 | 无该模块实车效果验收；不能借 tracker 的 D01–D07 替代 | 否 | timestamped scan/map/jump/freshness 故障注入适合；当前没有接此节点 | P2 shadow 观察 GOOD/SUSPECT/REJECT/UNKNOWN，先证明诊断有效，不能把它当 TF/停车 owner |
| T-DT A* / DP / Minimum-Snap / square corridor；B+C | `origin/experiment/tdt-planner-phase2@e374894a`，snapshot fix `a419654` | `experiments/tdt_planner/rm_tdt_planner`（COLCON_IGNORE） | E11：static A*/QP 各 5/5、recovery=0；动态首轮两种均 goal success，但 recovery 1/13、实体平面 overlap FAIL，停止重复。800 配对 grid 请求/3200 调用是离线，不是动态 bag 回放 | 否；**Gazebo + 离线** | 否 | 适合新 footprint 下静态窄通道/endpoint/corridor 验证；尚未在 S2 接入 | P3 仅比较当前静态 baseline；不自动继续冻结的动态扩展。类名 MinimumSnap 的当前 order6/maxdx3 更近最小 jerk；输出 Path 无完整 polynomial/time/corridor execution contract |
| DynamicObstacleCritic / V1 预测消费；B+C | `feature/dynamic-obstacle-critic@91d7eda0`；`origin/experiment/dynamic-prediction-v1@f85149e0` | `src/rm_dynamic_obstacle_critic/` 与 `experiments/dynamic_prediction_v1/` | E12：source/offline/Gazebo；V1 首轮成功后重复恢复次数 5/1，零 recovery 门未过，2026-09-25 Frozen。visible centroid/fixture SDF 整箱假设不能包络真实隐藏物体 | 无该消费版本实车验证 | 否 | 可测暴露面/净空/停止输入，但当前最小 Nav2 不加载 critic/tracker；不可把场景 GT 混入 runtime | 保留冻结、失败及 witness，不作为本轮开发方向 |
| Temporal MPC；B+C | `experiment/temporal-mpc-main-20261004@04291a41` | `experiments/temporal_mpc/`、其中 native ROS2 controller/interfaces | E13：Humble/Fortress，actual scans + tracker V2 的候选/native/fallback；工程 tests 多项通过，但最终 head-on 40 s 取消、仅 1 accepted cycle，future clearance −14.95 mm，received gap 95.5/82.7 ms 超 75 ms；**Frozen / NotAccepted** | 否；**Gazebo**，不是“真实数据” | 否 | 当前静态入口无法增加其主要动态失败证据；迁移版本本身需系统验证 | 只归档；不新增 MPC、不恢复冻结实验 |
| R4 HWS prediction consumption / owner fences；B+C | `experiment/r4-hws-prediction-consumption@bdb8b8d0` | `src/rm_r4_prediction_consumption/`、`rm_r4_nav2_controller/`、`rm_r4_interfaces/`、`experiments/r4_gazebo_comparison/` | E14 最新 **Stop/Frozen**：受限 recovery cohort，B0 5/5 contact，R4 0/5 contact、through-gate 5/5，但完整 goal 2/5，3 次自身 degenerate-path 错误。缺 simple-reactive ablation，不得只取“零接触” | 否；**Gazebo** | 否 | 新入口未接 owner fence/动态模块，不能代表 R4 被验证 | 保留 Stop，禁止将工程 guards 的通过写作任务收益或部署通过 |
| controller odom / native-cycle provenance；B+C | `experiment/controller-odom-contract@96799412`；`origin/experiment/native-cycle-evidence@a12688b2` 等 | `docs/dynamic_obstacle_critic/stage2_evidence/`、隔离 experiment profile/tools | E15：实际 Humble controller 配置/订阅读回；旧 /odom 349 native 输入全零，修复 /odometry/lio 后 253 运动周期全非零、350 完整输入/输出 bitwise 匹配；同次物理门仍 FAIL，机械投影接触 | 否；“measured”是 Gazebo 实际输入，非实车 | 否；N 的各 profile 需逐一核对 | **能**复核当前 Jazzy controller 的真实路由；S2 已显式参数+读回+订阅检查 | P0 每个后续 profile 保留实际 route / 消费证据；不能用 Nav2 成功掩盖错误速度来源 |
| 机械 footprint、visible-box、extent prior、static/uncertain stopping、soft clearance、wait/go/replan 等动态研究；B+C | `a4076027`、`5b5f6544`、`e50b5eeb`、`faabc5a5`、`aafd87f6`、`26223712`、`b92cbe0f`、`fac071b2`；archive `9e08122a` | 各 ref 的 `src/rm_dynamic_obstacle_critic/`、tracker、隔离 config/tools/evidence | E12/E15 及全历史文档：不同 trial 的 stopping、raw203、future geometry、机械 gap/任务门有失败；不能由单条 source/SDK 等价推出物理可行 | 否；离线/合成/Gazebo | 否 | 当前真实 collision proxy 可改善静态机械净空定义；主要预测/动态时序仍无当前系统证据 | 记录模型域/失败，不整组迁移；本轮 footprint 修复不等于这些冻结路线已通过 |

## C：N 中已有实现的新车系统缺口

| 模块 | branch/commit | 当前实现位置 | 最强证据 | 是否实车 | 是否已进 main-new-car | 当前仿真能否验证 | 建议下一步 |
|---|---|---|---|---|---|---|---|
| FAST-LIO + canonical LIO adapter 新车链；C | N；`be5b616f` / `68eb89e4` 的现行候选 | `src/fast_lio_multi`、`rm_lio_bringup/`、`rm_localization_adapters/src/{lio_adapter,canonical_odometry}.cpp` | Phase 2A/2B 接口、构建/单测、fake source；老车 FAST-LIO 的真实证据不能证明 rotating new-car profile。N covariance 仍待修 | 核心有老车证据；**新车链无** | 是 | 现有 GPU cloud 不是 FAST-LIO 所需 IMU/点时序；可测 adapter 外参/时间/有限差分软件合同 | 先使用显式 synthetic raw shadow；真正 LIO 需 IMU/CustomMsg 时间模型或真实包 |
| planar chassis heading / dynamic gimbal adapter；C | N；`68eb89e4`、`bf475f7a`、`5a3be2f1` | `src/rm_localization_adapters/`、`src/rm_lio_bringup/config/lio_adapter_chassis_heading_fusion.yaml` | 合成杆臂、yaw 差与转云台，现行 helper 有 E8 指出的 full-base 反例 | 否；合成/Gazebo | 是，默认关闭、模板拒绝未标定启动 | 实际 joint / sensor pose 可测平地与坡道边界，不能默认为全姿态解 | P1 对照 E8；保证 startup E0 与 Et、拒绝条件和 owner，而非仅修画面 |
| AMCL / seeded GICP + global bridge 新车系统；C | N | `src/rm_relocalization_bridge/`、`rm_gicp_relocalization/`、`rm_navigation_bringup/` | 老车 AMCL fresh03 有限成功；GICP synthetic smoke；bridge unit/runtime；没有当前新车场景 wrong-seed/recovery 闭环 | 老车 AMCL 有；新车无；GICP 无实车通过 | 是基础版本 | 可在关闭 truth TF 后接一个选定 owner，测试时序/错误接受；当前入口没接 AMCL/GICP | P2 构建一个隔离 baseline，先错误 seed 与 correction 污染；不伪称 GICP 是 place recognition |
| map→odom / odom→base owner、epoch、nearest-stamp 对齐；C | N；Phase 2C | `src/rm_relocalization_bridge/src/map_odom_from_global_pose.cpp`、`lio_adapter.cpp`；`docs/contracts/tf_contract_2027.md` | 既有数学/无硬件 runtime；map correction 用近邻源 odom，generic 窗口约 .05 s、非插值；新 S2 truth 路线独立 | 既有老车有限链证据；新车 owner 交接无 | 是；S2 oracle 仅在实验 | **能**；`publish_truth_tf:=false` 为显式隔离开关，仍提供 truth odometry供评估 | P0/P1 检查每条 edge 唯一 owner、stamp/epoch/reset 与旧 plan 生命周期；不得同时启用 oracle 与定位 TF |
| STVL / dual obstacle layers；C | N | `src/rm_nav_config/config/*stvl*.yaml`、`src/rm_navigation_bringup/` | 老车 STVL 障碍观测/有限导航；Humble 文档。当前 Jazzy `spatio_temporal_voxel_layer` **Package not found**；S2 只有 static+inflation | 老车有；当前新车/STVL 无 | 是配置基础 | **现在不能直接运行 STVL 验证**；先完成 Jazzy runtime 相容/独立接线；GPU cloud 可以提供可审计合成输入 | P2 首先验证 marking/clearing/decay、clock/reset、坡面障碍保留，再动态任务 |
| MID360 A/B、云台 geometry 与双雷达融合；C | N；`f2f1c74c`、`d735ee12` / S1 | `src/rm_mid360_driver_bridge/`、`rm_description/`；新 `rm_new_car_simulation/config/new_car_cad_baseline.yaml` | 旧车左定位+双障碍链有 runtime/有限实车；新 A/B CAD 原点是假设链，S1/S2 实际 joint TF 数值一致 | 老车基础有；**新 A/B 无标定/实车** | 旧链是；S1/S2 CAD 新入口否 | **能**测不同 lever/yaw/roll/pitch 的 geometry、扫描遮挡；不能验证内部 IMU、home/time/rebase | P1 A/B 候选输入合同与 stale/epoch；真实 extrinsic/timing 单独验收，不虚构 IMU TF |
| chassis / serial / competition_v2；C | N；`4e109513`、`4c21df00` | `src/rm_chassis_interface/`、`src/rm_serial_driver/`、`reference/competition_v2/` | C/C++ codec golden vectors、mock/dry-run；旧 HPM/legacy 链有老车现场证据。V2 firmware/packet/sign/watchdog/home 无真实接受 | **V2 无**；旧协议实车不能迁移结论 | 是，V2 默认关闭 | 可测上层状态/请求/watchdog contract；当前插件直接消费 twist，不是 serial→MCU→轮内环 | P3 单独 mock protocol→状态→任务链；硬件 codec/time/sign/限幅不得用仿真替代 |
| dog-hole / chassis posture / slope execution；C | N `3e7e0ae3`、`070b5f24`；S1/S2 | `src/rm_dog_hole/`、旧 `rm_simulation/` 与新 `rm_new_car_simulation/` | 旧 proxy .22 m 通过、pitch9.16°、净空4.27 mm；S2 当前 CAD代理通行和坡道接触另报告。无实车变形机构链 | 否；**Gazebo** | 旧执行器是；新 S2 否 | 可测 fixture 物理通行/堵塞；新最小 Nav2 尚不跑旧姿态请求/狗洞 BT | P2 接现有执行器的隔离 mock，不新建 FSM；区分 open-loop 过洞和自主任务；不能预测真实机械可过 |
| 2026 field / ramp perception / multi-level map；C | N `ab17167e`、`1e8f44a0`；S1/S2 | `src/rm_simulation/models/rmuc2026_navigation_field.sdf`、`field_geometry_simulation*.launch.py`、filter config | 旧 25 primitives 与 fixed-surface masking 的 Gazebo；中央 4×6 deck 不符合规则。S2 局部规则尺寸/地面横坡已补，2025 mesh 年份明确 | 否；Gazebo/几何 | 旧粗场地是；S2 否 | 局部几何可验证；保守 2D projection 把坡/deck/roof 标 occupied，无完整多层通行图 | P2 先定义可通行高度层及障碍保留；不能用固定已知 plane 过滤制造“坡可规划” |
| BT / decision / recovery / operator goal owner；C | N；`8c2f8b3d`、`d50a1e15` | `src/rm_competition_mission/`、`src/rm_dog_hole/`、`src/rm_nav_config/behavior_trees/` | action mock/unit、老车三点 spin/低 HP 返回现场 smoke；真 referee/低弹量/新姿态/定位 LOST stop/比赛时长链缺证据 | 部分老车 smoke；**新车任务无** | 是基础；A 中新重规划修复否 | 可用 simulation+mock 状态验证抢占、stale result、recovery与唯一路由；当前 S2 没启动 mission | P2 按一条 task flow 验证；不要把 fake action 成功或手工 mock referee 当真实比赛状态 |
| semantic annotated path / COMMITTED execution / dynamic clearance；B+C | `origin/feature/semantic-region-path-contract@f60f62ee`；`origin/feature/dynamic-clearance-shadow@902261de`；HWS/R4 | `src/rm_path_annotations/`、`rm_dynamic_clearance/`、`rm_navigation_execution_adapters/`（各候选 ref） | Linux 57 tests + ROS mock `BLOCKED/predicted_overlap`；revision/hash/ETA/UNKNOWN contracts；R4 有 fences。**原狗洞/mission 未消费该 shadow 报告** | 无实车执行准入通过 | 否 | 有选择的 path revision/reset/UNKNOWN 合同可测；当前缺候选包和预测源，不能直接宣称系统启用 | P3 合同 fault injection；不恢复 R4、不新建竞争 action owner |
| pursuit / auto-aim target、referee、authority、readiness；C | N；`f66c55e6`、`f7cb20e2`、`49f67023`、`baa7c5bd` | `src/rm_pursuit/`、`rm_referee_interface/`、`rm_chassis_interface/`、`rm_system_monitor/` | 纯算法/接口/mock、freshness/range checks；Phase3 多为验收步骤，不是结果；没有真实 target producer 和新车比赛输入 | 无完整真实链；部分旧车 mock mission smoke | 是，默认保护/关闭 | **能**增加缺输入、时序、抢占、目标/定位 invalid 的闭环证据；不能证明 auto-aim 或物理急停 | P2/P3 聚焦 readiness→mission/action→最终速度 的一致性；不扩大到决策新功能 |

## D：已进 N 的有界基础证据

| 模块 | branch/commit | 当前实现位置 | 最强证据 | 是否实车 | 是否已进 main-new-car | 当前仿真能否验证 | 建议下一步 |
|---|---|---|---|---|---|---|---|
| canonical pose/twist 与唯一 TF 基础；D+C | N；Phase2A/B/C 原始实现 | `src/rm_localization_adapters/src/canonical_odometry.cpp`、`src/rm_relocalization_bridge/` | SE(3) sensor→base、body-frame finite difference、源 stamp/异常输入 unit；Phase2无硬件 runtime；现行 15 canonical tests被 E8 再跑。只对明确输入合同充分 | 不是该新车完整链实车 | 是 | 能做 actual joint / full pose integration；S2 oracle 自身不能证明 localization | P1 补相对 E0/Et、full-base、时序和 covariance gap，不重写 owner |
| Nav2 / MPPI 基础静态导航；D+C | N；`281dfaa` 及 Phase1.5C | `src/rm_nav_config/config/nav2_phase1_5_mppi.yaml`、`src/rm_simulation/` | [Phase1.5C](../phase1_5c_mppi_validation.md)：Gazebo 10/10 goal、无接触/recovery/loop-miss；E16 老车一次 .15 m/s 目标 success、recovery0。S2 另有 current one-goal smoke | 有**有限老车主动导航**；10/10是Gazebo | 是基础 profile；S2 生成配置否 | **能**验证静态通行/路线/endpoint，已有 S2 一目标；未证多次、动态、狗洞、全场 | P0/P2 固定新 footprint 的小量系统 baseline；每个 selected profile 读回 odom 而非只看 YAML |
| 地图 bundle/deployment policy、hash/live metadata | N；`2c6265a9`、`11b34631`；E16 | `src/rm_map_tools/rm_map_tools/{map_bundle,validate_map_bundle,resolve_map_bundle,verify_map_server}.py` | fresh03 occupancy-only candidate live/PGM 151257 cells diff0、scan人工对齐、一次低速 goal；哈希/revision/approved-candidate-test gating unit/runtime | **老车实车 + 真实地图**，不是 approved GICP 地图 | 是 | 能测 S2 world/PCD/PGM provenance一致与拒绝错 revision；不证明地图真实完备 | 保留 candidate policy；S2 mesh-sampled PCD 不能冒充真实 scan/map |
| managed mapping / PCD/OctoMap export 基础；D+C | N；Phase2I / `0e3347a8` recorder 候选 | `src/rm_map_tools/`、`src/rm_mid360_driver_bridge/`、`src/rm_navigation_bringup/` | 真实地图/回放质量诊断，受控映射/exporter与 bundle；PCD accumulation和OctoMap occupancy是并行输入链，**不是 PCD 转 PGM 的真值链** | 有真实地图/数据；新场地采图无 | 是基础；sidecar候选版本按 ref 核对 | 可测导出原点/units/hash与来源；S2直接 mesh map 不能验证在线建图误差 | 保留原已拒绝 cleanup 结论，不扩建图研究；有需要只做 provenance gate |
| PointCloud obstacle filter / dual fusion 基础；D+C | N；`f2f1c74c`、`d735ee12` | `src/rm_mid360_driver_bridge/` | 双设备单 driver、left LIO + dual obstacle 分工，Linux runtime与旧车可视化/障碍链记录；右 extrinsic/time/性能 acceptance 仍缺 | **有限旧车 runtime/现场**；新 A/B/全链无 | 是 | 可测过滤范围、source stamp、TF及传感器遮挡；当前 S2仅单 selected GPU lidar | P2 独立 input→costmap链；不升级为 dual-LIO 或真实 right extrinsic 已校准 |
| serial legacy / HPM transport 与 chassis 边界；D+C | N；`f58f30ed` / field debug | `src/rm_serial_driver/`、`src/rm_chassis_interface/` | E17 外部 406 entries/80.9 MB 冻结现场证据；有限旧车导航、单 velocity owner、link/profile/诊断。不能扩展到 V2/reference firmware | **老车有限实车链** | 是共享基础 | 只能测 mock上层；四轮速度直接仿真没有真实 link/MCU闭环 | 保持与 V2 明确分开，按具体串口 profile审计watchdog/符号 |
| 旧狗洞任务 / 旋转云台数学入口的有限实现；D+C | N；`3e7e0ae3`、`5a3be2f1` | `src/rm_simulation/`、`src/rm_dog_hole/` | [旧3D几何报告](../validation/new_car_dog_hole_3d_geometry_20260804.md)、gimbal synthetic truth；旧车体 proxy/特定 fixture充分 | 否；Gazebo/合成 | 是 | 新 fixture 可复验，但不得继续沿用旧半径/车高/固定 plane结果 | 用当前 CAD代理重做 system验证，保留实物机械包络 unknown |
| map cleanup / ray-DDA / quality tools 的拒绝与诊断证据；D（诊断）/A或B（候选） | N质量工具；HWS selective ref / `0e3347a8` sidecar | `src/rm_map_tools/`；候选 off-line 3D-DDA tools | E18真实 map/bag：旧高 endpoint cleanup损坏静态结构，99%保护门失败而拒绝；HWS DDA/sidecar纯算法/回环不是清图实车效果通过 | **真实数据诊断**；清图候选无实车接受 | 质量/拒绝工具是；增强候选部分不在 N | 可测保持 fixture opening/静态结构与来源；不替代真实 ray证据，也不从最终 PCD反造射线 | 不恢复已拒绝/冻结清图，不因为有工具就改变正式地图 |

## 证据索引（固定 ref 的路径）

下列路径应在相应 ref 用 `git show <ref>:<path>` 查看；不能用当前 worktree 同名文档替代历史版本。快照记录了 cited report 的 Git blob identity。

| ID | 固定 ref | 报告路径 / 关键位置 |
|---|---|---|
| E1 | `origin/fix/old-car-fast-lio-high-spin@46c521b2` | `docs/validation/old_car_fast_lio_high_spin_validation_20260903.md` §1.1/1.2/9；顶部后续20 Hz结果优先于保留的H02 FAIL快照 |
| E2 | `origin/fix/old-car-amcl-correction-gate@e15b6bda` | `docs/validation/old_car_high_spin_localization_engineering_report_20260825.md`；后续纠错/恢复仍有适用限制 |
| E3 | E1 ref | `docs/validation/amcl_high_spin_root_cause_and_candidate_20260720.md`；固定bag/seeds不是新实车重复 |
| E4 | `origin/main-old-car@82017945` | `docs/validation/old_car_global_dynamic_replanning_20260806.md` |
| E5 | `origin/fix/old-car-ramp-live-tracking@1525bb5e` | `docs/validation/old_car_ramp_live_tracking_report_20260830.md` §8主动验收仍0% |
| E6 | `origin/fix/hwsentry-tracker-field-gates@c04278f3` | `docs/hwsentry_migration/hwsentrynav26_phase1_linux_validation_report_20260824.md` §11修复/真实包A/B，不能只读首轮快照 |
| E7 | `experiment/fastlio-covariance-contract@04364d50` | `docs/research/fastlio_covariance_contract_20261007.md` |
| E8 | `experiment/full-base-heading-adapter@5733f126` | `docs/research/full_base_heading_adapter_20261007.md` §5–7 |
| E9 | `experiment/neu-localization-baseline@bfc23546` | `docs/research/neu_localization_l1_baseline_20261007.md`、`neu_factor_graph_task1_20261007.md`、`neu_serial_research_route_20261007.md`；L2私有实验文件 |
| E10 | `origin/feature/navigation-integrity@64870dc7` / HWS | `docs/navigation_integrity/implementation_report.md`；`docs/hwsentry_migration/dynamic_clearance_shadow_validation.md` |
| E11 | `origin/experiment/tdt-planner-phase2@e374894a` | `docs/tdt_migration/p2b_work_status.md`、`validation_20260912.md`、`validation_20260913.md` |
| E12 | `origin/experiment/dynamic-prediction-v1@f85149e0`；archive `9e08122a` | `docs/dynamic_navigation/v1_prediction_scope.md`；`docs/dynamic_navigation/{dynamic_obstacle_prediction_full_history_20261004,all_dynamic_avoidance_documents_20261004}.md`；原 feature validation |
| E13 | `experiment/temporal-mpc-main-20261004@04291a41` | `docs/dynamic_navigation/temporal_mpc_freeze_20261004.md`、`temporal_mpc_gazebo_results_20261004.md` |
| E14 | `experiment/r4-hws-prediction-consumption@bdb8b8d0` | `docs/dynamic_navigation/r4_final_corridor_research.md`；以最终 Stop/cohort 为准，不能只取早期 Modify |
| E15 | `experiment/controller-odom-contract@96799412` | `docs/dynamic_obstacle_critic/controller_odom_contract_experiment.md`、913文件manifest；失败结果与bitwise通过共存 |
| E16 | N | `docs/validation/old_car_fresh03_field_validation_20260720.md`、`docs/phase1_5c_mppi_validation.md` |
| E17 | N | `docs/validation/old_car_field_debug_20260717_evidence.md`、`old_car_three_point_spin_field_test_20260722.md` |
| E18 | N | `docs/validation/pcd_pgm_map_quality_implementation_20260717.md`、`pcd_pgm_dirty_map_end_to_end_report_20260720.md`；HWS版本DDA/sidecar状态 |

## 仿真验证待办优先级

此表是审计建议，不是解冻指令。优先新增当前场景能提供的证据；每项先固定 selected profile、输入、版本和失败门，再少量 decisive test。不能以大量重复替代缺少的观测。

| 优先级 | 项目 / 能新增的证据 | 当前依赖与判定边界 |
|---|---|---|
| P0 | **S2 geometry/dynamics/odom routing**：四轮/姿态包络、上下坡、洞口通行与负例、物理阻挡、actual MPPI odom subscription与一目标 | 本轮执行，详见 S2 evidence。限定代理域；不需要 merge 历史研究。多层自主坡/洞尚未证明 |
| P1 | **现行 planar 与 full-base shadow 输入合同**：实际 joint/full pose 下 stationary-gimbal、A/B、上下坡、倾斜启动、skew/reset/missing参考 | 关闭 oracle TF 才允许候选单一 TF owner；shadow可保留oracle供评估。不得拿 truth 选唯一根或假定真实标定；无需恢复 NEU |
| P1 | **covariance transport**：native publish→ROS→adapter 的顺序、frame、layout、freshness与unknown输出 | 需要隔离构建已有候选/原版本；sim只能验证传输/几何，不校准 EKF uncertainty，不代表必须合并候选 |
| P2 | **选定 AMCL / seeded-GICP baseline + bridge / integrity**：错误seed、重复结构、stale/epoch、拒绝后 active correction/plan 状态 | 先独立 scan输入、唯一owner；用field geometry真实仿真扫描，GT只评分。关注错误接受，不开始 ScanContext/FG 新研究 |
| P2 | **STVL/坡面感知**：marking/clearing/decay、roll/pitch下障碍保留、近坡false-delete与输入延迟 | 当前 STVL 未安装，先 runtime相容/隔离接线；不得套旧revision固定plane mask。规则局部fixture可以产生新增证据，full2026 topology不能 |
| P2 | **狗洞/decision/recovery/运动许可**：existing执行器mock构型→受限通行、blocked/timeout、localization invalid→实际撤速度、旧action结果隔离 | 当前直接wheel plugin无真实机制，只能验证上层。多层2D map需先定义；不新增FSM/最终cmd owner |
| P3 | **T-DT静态候选的限定对照**：A*/DP/QP的path是否落在实际新代理可通行空间，endpoint/corridor约束、低速静态任务 | 本轮只审计；不恢复冻结动态T-DT实验。Path-only输出不证明timed polynomial或实车minimum-snap执行 |
| P3 | **serial/authority/readiness/pursuit的已有mock闭环**：stale state、抢占、watchdog、sign合同与任务取消 | 可测软件，不证明V2 MCU/真实referee/autoaim、内环性能或急停；硬件门单列 |
| 延后 | tracker真实数据失效分布、新车LIO/内部IMU标定、比赛场地完整拓扑、多层地图、实车轮/质量/惯量/roller参数 | 当前proxy不能填补这些证据，需新资料/真实数据；不能用当前仿真扩大结论 |
| 保持冻结 | NEU Task2–6、Temporal MPC、R4、V1/其他停止的动态预测/清图开发 | 不属于本轮下一步，不因新仿真可跑而恢复。已有真实/仿真失败是保留它们当前状态的证据 |

实车证据外的候选并不天然更优；N 外的实车版本也不天然值得合并。集成决定必须围绕新车的具体缺口、适用条件、可回退的最小变化与新系统证据，另行评审。
