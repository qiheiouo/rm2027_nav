# 新车定位 L1 / L1-M 能力基线

2026-10-07，Research；依据 [L0 审计](neu_localization_controller_second_audit_20261007.md)、公共 contracts、runtime profiles 及 main 的长期研究 / Git 规范。

**L1-M 结论：Modify。12 m 合成反例确认是 perceptual aliasing：局部对应完整、质量矩阵满秩，但地点错误。独有特征和不同部分观测也未让固定门识别错误地点；不通过堆阈值修单例。** CAD 位置已复用为可信暂定基线，A 为本轮研究主雷达、B 保留，实车选择未定。完整 sensor / IMU 外参、点云与 canonical odom 的共同基准、真实地图与留出 query 仍未闭合。真实 attraction basin 与 recovery 最大范围 UNKNOWN。详见 §10；本轮不进入 L2+L3。

## 1. Baseline identity

后续阶段：用户在 L1-M 后明确授权离线全局候选研究，见 [L2 prototype 报告](neu_localization_l2_global_candidate_prototype_20261007.md)。上文“不进入 L2+L3”是 L1-M 当时的阶段边界；输入缺口与 Modify 结论仍保留，离线软件推进不代表已取得新车真实效果。

当前按串行路线停在 **Task 1 / Modify / waiting for real vehicle evidence**；新车待补输入及重跑入口统一见 [Task 1 §8](neu_factor_graph_task1_20261007.md#8-实车到位检查表与恢复入口)。本报告保留 L1 / L1-M 历史结果，不追加 toy 调参。

| 项目 | 本轮固定身份 / 边界 |
| --- | --- |
| 目标车辆 | 新车。用户先选老车，后要求按迁移成本决定；新车航向融合、云台姿态重建、杆臂 / home、MCU 时间与 reset 假设不同，不能只换参数复用老车标定，因此直接采用新车 |
| 算法源码 | `origin/main-new-car` 本地及只读远端均为 `bd6cf689d3d5aca5c8edd430ff760ca5ceb27496`；研究 branch 为 `experiment/neu-localization-baseline`；整个 `src/` 与固定 pin 无差异 |
| 历史对照 | T `e374894a7a713ea84b82de9e92479a37041f39ff`；只读取质量 / recovery / 实车报告，未 cherry-pick 算法进新车 |
| 规则文档 | main `dfd63887` 的 AGENTS、开发流程和长期规范作为文档 overlay；工作区治理后 main 为 `fcd9a05b`，未因此改变本轮源码、输入或参数 |
| runtime profile | **离线 GICP 核心 + 实际纯函数位姿门**；代表核对仅启动原 GICP 节点。未启动完整 navigation 或车辆 profile；实际部署 commit / profile 为 **UNKNOWN / NOT RUNTIME VERIFIED** |
| 参数 | `src/rm_gicp_relocalization/config/gicp_3d.yaml`：map/source voxel 0.25/0.20 m，配准 2 Hz，correspondence 1 m，epsilon 各 0.001，30 iterations，source/target≥100，fitness≤0.50，guess-relative jump≤2 m/1 rad，TF timeout 0.10 s |
| 基础位姿门 | 同一 YAML：age≤0.5 s、future≤0.1 s、xy variance≤1、yaw variance≤0.5、z≤0.5 m、roll/pitch≤0.2 rad。启发式 covariance 由 fitness clamp 0.01–0.50；不是校准后验 |
| 地图 | `src/rm_map_tools/maps/phase2e_test/phase2j_3d_test.bundle.yaml`，`phase2j_3d_synthetic/test-v1`、`test_only`、map frame。522 点 PCD，hash `30ad9cf2decae2ad45f1ed3698ed123c7954ea1792dfd8e8a2b47e271622e3d1` |
| occupancy | `phase2e_test.yaml/.pgm`，resolution 0.05 m，origin [-0.1,-0.1,0]，4×4；仅 toy。原 validator 通过 schema / hash，要求 approved 时正确拒绝；不等于几何对齐已实测 |
| 生成压力输入 | 对 fixture 平移 12 m 后合并；50 点子采样；31×31 平面网格。独立研究 PCD，**未绑定 occupancy bundle**，只供几何压力测试，不进入正式地图消费 |
| 传感器 / 外参 | 数学配准不使用真实 MID360、IMU 或云台。L1-M 复用用户认可的 CAD 轮组、footprint、yaw 轴 XY、MID360 A/B 机械位置；A 为任意研究基线。完整 sensor / IMU 外参、home、heading 符号及 MCU→ROS 时钟仍待核验，详见 §10.3 |
| 新车融合候选 | `lio_adapter_chassis_heading_fusion.yaml` 的 heading match 0.03 s、rate propagation、home / reset 边界；`initial_alignment_confirmed:false`，不得将占位外参当有效标定 |
| canonical owner | 设计合同：lio_adapter 独占 odom→base_link 与 /odometry/lio；external_pose 模式 map_odom_from_global_pose 独占 map→odom。现场 owner / 参数回读 / graph 未核验 |
| operational global source | generic launch 支持 AMCL / GICP 互斥，默认 backend=none；本车当前实际 global source **UNKNOWN**。本轮选择 GICP 作离线基线，不代表已部署 |
| 最终控制链 | 未参与；没有 active TF、cmd_vel、运动授权、串口或 Nav2 实验 |
| 环境 | 本机 ROS Jazzy / PCL 1.14.0 / Eigen 3.4.0 / Python 3.12.3；不同于原 Humble 实车环境，不外推性能或数值一致性 |

主线关系另见 main 的 `docs/repository_workspaces.md`：main 是默认 / 公共入口，old-car 与 new-car 是分叉的车型实现线。不能将不同 ref 的功能算作单一当前系统。

## 2. Existing capability matrix

| 能力 | 新车 V 当前基点 | 历史 T / O / G | 本轮证据状态 |
| --- | --- | --- | --- |
| seeded PCL GICP、有限点数 / 收敛 / fitness / jump 门 | 已有 | 已有 | 合成扫描和原节点代表核对 |
| overlap、法向点面近似 H、eigen / condition 门 | **未集成** | 已有 | 固定 T 代码仅被动旁路评价；不参与 V accept |
| map / 时间 / 平面 / covariance 位姿门 | 已有 | 已有 | 实际 V 纯函数正反输入验证；不是全局唯一性门 |
| 时间匹配 global pose→odom 桥、非单调时间失效 | 已有 | 有后续增强 | 源码复核；本轮不启动桥、不提交 correction |
| 3 次 innovation reject、high angular hold、停车 settle / reseed / bounded rebase | 无 T 的这套状态机 | T 老车 profile 有 | 复用指定老车源码 / 历史报告，不迁移、不重跑 |
| 9 月原生点时序与 LIO age / backlog 保护 | 不可算作 V 已具备 | T 老车有 | 老车有界实车报告；新车仍未验证 |
| navigation_integrity | 未作为本轮 authority 集成 | I / O / T 有 shadow | 诊断不能替代提交 / 运动 owner |
| 无 seed place / Top-K / global yaw candidate generator | 所查选定链没有 | L0 对应 refs 未形成完整链 | 无 seed 原节点没有 score 或 raw pose；不实现新算法 |

## 3. B1：有限 seeded basin

[原始 CSV](../../experiments/neu_localization_l1/evidence/results.csv) 共 **34 个配置，非 34 个独立 episode**：D1 26、D2 4、D3 4。D1 使用同一完整 fixture 作 source/target，非真实视域 scan；地图 / source voxel 不同。数学 GT 为恒等变换，评分容差在实验前固定 xy≤0.10 m、yaw≤0.10 rad、z≤0.10 m。

GT 不传给配准程序或门。B1 的真值附近人工 seed 是 upper-bound 特许；B2 seed 是固定人为压力条件，无 seed 路径不读取 seed / GT 生成候选。无配准输出时不把初始 guess 当最终估计。

| 扩张方向 | 正确且最终 ACCEPT 的已测点 | 已见边界 / 失败 |
| --- | --- | --- |
| x 单轴 | ±0.5、±1 m | -2/-3 m 错误 ACCEPT，最终约 2.666 m；+2/+3 m 结果被非平面姿态门拒绝 |
| y 单轴 | ±0.5、±1 m | +2 m 错误 ACCEPT，最终约 2.678 m；-2 m 非平面拒绝 |
| yaw 单轴 | ±0.25、±0.5、-1 rad | +1 rad 已配准到正确解，但 correction≈1.000171 rad 超门，false reject；±π 均错误 ACCEPT，最终 yaw 误差≈3.138 rad |
| xy+yaw | (1,1,0.5)、(-1,1,-0.5) | (2,2,1) 拒绝，结果约 1.994 m / 1.570 rad，不是正确解 |

D1 的 26 个点中：16 个正确 ACCEPT、5 个错误 ACCEPT、5 个 REJECT。这里是测试配置计数，不能计算赛场成功率。正确附近解约 xy 0.00076 m、yaw 0.00017 rad、z 0.0273 m；仅描述此 voxel / fixture 数学结果，**不构成新车绝对定位精度**。

这些有限点没有填满 xy/yaw 连续空间，不证明完整 ±1 m 立方区域，也没有验证真实 scan 视域 / gimbal 条件。**实车 basin UNKNOWN；2 m / 1 rad 只是相对 guess 的提交门，既可拒绝正确解，也可放过局部错解。**

PCL align 本轮约 2.93–19.72 ms，中位数约 4.73 ms；仅 32 次有配准的 toy 输入、单机取样，没有独立性能试次。总 probe 时间还包括加载 / T 旁路质量计算，不等于 production 周期或端到端延迟。

## 4. B2：wrong / no useful seed 与错误 ACCEPT

| 条件 | V 基础 / 位姿门行为 | 正确性与历史 T 旁路指标 |
| --- | --- | --- |
| 原地图，seed x=12 m | 不收敛 / fitness 极大，拒绝；原节点 0 raw pose | 没有自动从失败 guess 回到正确地点 |
| 无有效 seed | 原节点收到 3 份合成云，0 score、0 raw pose | early return；不是自行估计恒等位姿 |
| 原地图，seed yaw=-π | 接受；原节点复现 | xy≈1.005 m、yaw≈3.138 rad 错误；overlap≈0.868、min eigen≈0.01716、condition≈126.49，T 旁路门也通过 |
| 12 m 重复结构，正确 seed | 接受正确附近解 | 正对照；不能单独证明全局定位能力 |
| 12 m 重复结构，错误 seed x=12 m | **错误 ACCEPT**；原节点连续 3 份新时间戳、相同几何输入均接受 | 最终 xy≈11.99990 m；fitness≈0.008310、overlap=1、min eigen≈0.01090、condition≈178.47；T 旁路门通过 |

重复错解的相对 guess jump 只有约 0.02863 m / 0.00013 rad，位姿 / 时间 / covariance 也合法。六个 H eigen 约 `[0.01090,0.11382,0.22193,0.22797,0.51988,1.94519]`；正定且条件良好没有提供全局唯一性。新车 V 不运行这些几何质量门；即使只补上 T 默认门，此反例仍不能拒绝。

T 质量 H 为平均 `JᵀJ`，`J=[n,(p-centroid)×n]`，normal K=12、最近邻阈值 1 m、一侧 overlap；阈值 0.30 / 0.001 / 1e5。它不是 PCL GICP 完整 covariance-weighted Hessian，米 / 弧度尺度也没有统一统计标定。

全部 34 配置中 24 个最终 ACCEPT，其中 6 个错误 ACCEPT；6 个的 T 旁路指标均通过。其余 10 个拒绝。**这些计数是有限机制反例，不是实车 false-accept 概率。** 同一合成重复 query 连续三次错解只是说明反馈 / 重复稳定不能证明正确性，不冒充独立真实观测一致性。

V bridge 在时间匹配且消息合法时直接计算 `T_map_odom=T_map_base·T_odom_base^-1`，没有 T 的 innovation / recovery 门。**若正式接入上述新鲜合法错解，可能污染 correction**；这是源码推断，本轮未启动桥、未广播 map→odom。首次 baseline 错误也没有可信 old correction 帮助拒绝。

## 5. 当前各门能拒绝什么

| 门 / 指标 | 已有作用与本轮证据 | 不能推出的结论 |
| --- | --- | --- |
| 点数 / convergence / fitness | 50 点输入拒绝；远 seed 无对应拒绝 | fitness≈0.00831 的 12 m 错解仍通过 |
| jump | 联合大修改及 +1 rad 正确解被拒绝 | 相对错误 guess 的小修改不是正确全局 pose |
| frame / finite / 平面 / covariance | V 实际纯函数存在；+2/+3 x 和 -2 y 的非平面结果被拒绝 | 合法的错误 pose 不被拒绝；covariance 是 fitness 启发式 |
| 时间 | age=0.6 s、future=0.2 s 两个同几何正解分别拒绝 | 不修复点内时间、deskew 或 LIO backlog；不能给旧输入改 now 绕门 |
| T overlap | 无对应时低；重复错解为 1 | overlap 不是地点唯一性 |
| T H / eigen / condition | 平面例 overlap=1，但 eigen=[0,0,0,1,3.2,3.2]，旁路门拒绝；V 自身仍接受恒等解 | 满秩错 basin 仍通过；平面正解被 V 接受不证明其可观测 |
| temporal consistency | V GICP 接受后更新 guess，不做独立全局候选多帧确认；T 有局部恢复计数 | 三份重复错几何可持续接受，不能靠计数证明正确 |

## 6. B3：historical operational recovery 边界

仅复用 T `src/rm_relocalization_bridge/config/map_odom_from_global_pose_old_car_2026.yaml`、桥源码及 `docs/validation/old_car_fast_lio_high_spin_validation_20260903.md`。**这些是老车既有证据，不是新车当前功能或本轮实车复验。** 报告指向的原 bag 在本轮所查位置未获得，数值以已存历史报告为证据，不进行重新评分。

| 项目 | 已有有界能力 | 限制 |
| --- | --- | --- |
| 短暂 global 异常 | 0.35 m / 0.35 rad innovation，连续 3 次异常才失效；前两次保留旧可信 correction | 必须已有正确可信基线；首次错基线、持续对称错解未解决 |
| high spin | healthy LIO 且 angular≥3 rad/s 时保持最后可信 correction | 没有重新发现未知全局位置；LIO 不健康时不应继续 carry |
| 停车恢复 | v≤0.08 m/s、w≤0.15 rad/s，hold 0.6 s、settle 0.5 s、cooldown 5 s、5 次一致、最多 6 reseed | 局部可信 guess 周围重试，不提供跨场地召回 |
| bounded rebase | 位移偏离≥1 m 时触发；保留 last trusted XY、用 settled yaw；容差 0.75 m / 0.75 rad，10 次一致 | 1 m 是触发，0.75 是验证容差，均**不是最大可恢复误差**；前端发散不因换 correction 自动修复 |
| LIO timing | 原生 CustomMsg 点时序、age / backlog 与输入健康保护 | V 未直接继承该老车组合；无法代替新车 deskew / clock 验证 |

历史演进：7 月 AMCL catastrophic failure；9 月 R03 又出现约 3.495 s backlog、约 10 m 错误平移，说明不能全部归因“缺 place recognition”。原生点时序 / 20 Hz、三次拒绝、high-spin hold 逐步改善。

9 月 3 日 hold 报告：≥3 rad/s 持续 52.751 s，峰值 7.273 rad/s，LIO/global validity 全 true；raw odom age P99 38.6 ms、max 86.7 ms，map→odom 全包最大发布间隔 61.8 ms；停车更新 0.159 m / 0.205 rad，约 0.7 s 回 healthy。**仍未通过“最高速结束立即 home”的专门闭环，不能描述成通用 LOST / kidnapped 已解决。**

没有经过 GT sweep 的 recovery displacement / yaw 最大范围，结论 UNKNOWN。完全 kidnapped、大范围未知 pose、对称错误 basin、LIO epoch 改变、前端真正发散、没有可信 correction 或初始基线就错误均未得到通用恢复证明。V 现有非单调时间失效只是缓存边界，不是独立 epoch 恢复闭环。

## 7. Failure taxonomy 与 D1/D2/D3 readiness

| 失败类别 | 本轮可区分的信号 / 证据 | 尚需的独立新车数据 |
| --- | --- | --- |
| bad seed / wrong basin | 错 xy / yaw 及重复结构反例；合法低 fitness 错解 | 对称两地点分别采集、有地点标签和留出 query |
| observation degeneracy | 低点数拒绝；平面 H 三个零 eigen | 遮挡、低回波、动态机器人遮挡，保留真实有效点 / 覆盖变化 |
| timing / stale | 纯位姿门拒绝 age / future；历史 backlog | 原始点 offset、source/receipt time、MCU 同步误差、LIO age / queue |
| map mismatch | bundle schema / hash 通过不等于几何对应 | 同 revision PCD / occupancy、实测 origin / yaw / frame；场地变化记录 |
| LIO healthy + global LOST | T 短暂 global 异常与 hold 历史；V 无 seed无全局输出 | 健康 odom 下 wrong/no seed 的真实 query 和评分标签 |
| LIO divergence | T R03 时序 / 10 m 错误位移历史 | 前端失效 / reset 前后日志与 epoch；不能仅用 global jump 判断 |
| external prior missing | 没有核验可用 ego UWB / 雷达站 producer | 有真实 producer 时才记录 ID / 坐标 / 时钟 / 可用率，当前保持 missing |

数据清单见 [dataset_inventory.json](../../experiments/neu_localization_l1/evidence/dataset_inventory.json)。范围为本机 artifacts/maps、rm2027_navigation_logs、competition_materials/evidence_recovery 及下载 / 文档 / 桌面。找到的 83 个 bag metadata 未含所查 registered-cloud / native-lidar topic，4 个 PCD 均为 test/tampered 资产；artifacts/maps 没有实图。本轮**核验到的真实新车独立 D1/D2/D3 episode 均为 0**，不是断言其他机器没有数据。历史 AMCL 报告 / 配置可用于归因，但不替代新车完整 query / GT。

最低采集计划：D1 在固定地图下采不同位置、朝向和云台角的正常 query；D2 分别采对称 / 重复地点与明显错朝向，必须有可区分地点的外部标签；D3 独立记录遮挡 / 低点 / 高动态 / timing / reset 条件。每个 episode 标记 robot revision、源码 / profile、map revision、外参、raw/canonical 点云、IMU / heading / gimbal 源时刻、LIO 健康、GT / 人工标签来源；数据库 episode 与留出 query 分开。没有绝对 GT 时仅报告标签和内部一致性，不宣布绝对精度 / 零错误接受 / 通用成功率。

## 8. L2+L3 决策及九个问题

**Modify：目前证据足以建立机制实验，但不足以启动并得出有代表性结论的无 seed retrieval + GICP / gates 离线完整核心。** 阻止有效结论的输入 / baseline 问题只有：

1. 没有真实新车 PCD / occupancy 一致地图及独立 D1/D2/D3 query；toy 全地图自配准不能评价 MID360 实际召回 / basin。
2. 没有 keyframe poses、local submaps、建库 / 留出划分和地点标签 / GT；不能把整幅合并 PCD 的一个描述子当地点数据库。descriptor database 尚未建立，本轮不实现。
3. 新车云台补偿、sensor-frame normalization、source cloud→canonical odom 的几何基准、heading / IMU / 点时间、积累 query window 未核验；不能直接使用老车 transformed cloud 约定。
4. 尚未用真实数据确认固定 V gate 的误拒 / 错误接受与 Humble 环境差异，无法以这次 toy 耗时代替目标机预算或把历史 T gate 当已集成 baseline。

| 用户要求的问题 | 明确答案 |
| --- | --- |
| 1. 真实 attraction basin 多大？ | 实车 UNKNOWN；有限 toy 点结果见 §3，正确点到单轴 ±1 m、yaw -1/+0.5 rad 及两组联合点，不代表连续区域 |
| 2. quality gate 拒绝哪些？ | 基础点数 / 收敛 / fitness / jump / pose time / 平面各有正反例；T 的 overlap / H / eigen / condition 仅旁路，拒绝平面退化但未拒绝几何重复错 basin |
| 3. 哪些错误仍通过？ | 本轮 6 个错误配置，包含 12 m 重复错解及约 π 错 yaw；全部 T 旁路也通过 |
| 4. recovery 最大错误范围？ | UNKNOWN；局部策略参数不是经验最大范围，T 不能解决一般 kidnapped / 错初始基线 |
| 5. 高转速解决程度？ | T 老车有时序、短簇隔离、52.751 s hold 有界证据；立即 home 和新车未验证，未证明 global LOST 解决 |
| 6. 何时必须人工 seed？ | V GICP 初次启动 / reset 后以及无可信 guess 时，需要人工或另有可信 producer 的 seed；当前没有无 seed 替代。错误但未被门发现的 guess 不会自动得到正确 seed |
| 7. 真正无 seed generator？ | 选定 V 源码链没有；无 seed 原节点 0 配准输出。AMCL 初始化 / 老车 rebase 不构成已证明通用 3D candidate retrieval |
| 8. map bundle 支持 L2/L3？ | toy validator 通过但 test_only；实车 frame/origin/yaw/revision / preprocessing 未核验，当前不足 |
| 9. 缺哪些真实数据？ | keyframe pose / submap、独立 query、标签 / GT、同基准建库数据、传感器归一化 / 云台补偿、真实积累窗 / 点时序；描述子库待下一阶段生成 |

## 9. 工具核验、修正与保留

[工具入口](../../experiments/neu_localization_l1/README.md) 与 [manifest](../../experiments/neu_localization_l1/evidence/manifest.json) 固定源码 / map / 参数。V 真实 math、pose gate 链接进入 probe；T quality 从固定 Git 对象进入 ignored build，绝不导入 production src。保留 CSV、六 eigen、摘要、地图 validator、数据清单与 [native 核对](../../experiments/neu_localization_l1/evidence/native_crosscheck.json)。

原 V 节点的 5 个代表核对完成：identity、重复错误 seed（三份输入）、no seed（三份输入）、远 seed、wrong yaw。实际首帧 raw pose / fitness 与 probe 容差内一致。节点测试使用 domain 198、私有话题、odom=base 的合成单位变换；**不覆盖真实 TF 插值、odometry age、云台 / LIO 或正式 bridge**。identity 与 no/far seed 同时作为正反控制。

最小工具 / 环境处理记录：初次原节点编译因临时磁盘空间不足失败，`-pipe` 后未改源码重建通过；日志改存隔离 build，DDS 在私有 domain 经授权运行。一次 best-effort 首云尚未处理就超时，日志只有 seed 接受而无配准事件，不能算 GICP 失败；增加建连等待后五例通过。未更改算法阈值、输入几何或 source time。测试初始化时显式使用 T YAML 阈值，避免把 quality 结构的零默认值当 profile；满秩正例通过、平面负例拒绝。

早期 `results.json` 保持检查点字节，尚未包含六 eigen，且无输出场景的旧评分字段不能当估计。当前权威结果为 CSV / manifest / 小摘要；早期文件原样保留，不因工作区治理清理 L1。构建与生成 PCD 留 ignored build 可重建。

Git：Git 规范单独在 main `dfd63887`；工作区治理维护单独在 main `fcd9a05b`；L0 / L1 仅本地研究分支。无 push、无历史重写、未改正式配置或 active TF / cmd owner、未解冻 F/R，也未进入 SC、place recognition、因子图、外部 ego prior 或 MPC。原 L1 的 34 配置与原节点证据保持原样；本轮 L1-M 增量如下。

## 10. L1-M：错误 basin 归因与新车输入复核

### 10.1 12 m 错误 ACCEPT：对应、质量与各门

最小假设：低残差、高 overlap、满秩局部信息矩阵能否证明全局地点唯一？保持 V 源码 / 参数与 T 被动评价不变。原 fixture 平移 `[12,0,0]` 后合并作为 target，source 为原 fixture，真值 `T_map_odom=I`。这不是实车发生过的 12 m 事故。

[17 配置 CSV](../../experiments/neu_localization_l1/evidence/l1m/results.csv)、[输入协议](../../experiments/neu_localization_l1/evidence/l1m/manifest.json) 与 [12 m 完整诊断](../../experiments/neu_localization_l1/evidence/l1m/diagnostics/repeat_wrong12.json) 保留完整精度；GT 仅用于事后评分。

| 量 | 12 m 重复结构错解 |
| --- | --- |
| seed | x=12 m、y=z=0、RPY=0；单位 synthetic odom→base |
| 最终平移 | `[11.9998970032, 0.0007240546, 0.0286182091]` m |
| 最终 RPY | `[0.0003714809, 0.0001376931, -0.0001313757]` rad |
| 相对 seed jump | 0.0286275523 m / 0.0001313757 rad |
| PCL fitness | **0.008309846791 m²**，对应点的均方最近邻距离；等效 RMS≈0.09116 m |
| source / target | raw 522 / 1044；voxel 后 **477 / 630** |
| post-fit 有效对应 | fitness 分母 **477**；严格距离<1 m 的几何对应 **477**；距离≤1 m 且有效 normal 的 T 对应 **477** |
| 去重 target 对应 | **315**；不是 477 个独立几何约束，多 source 可对应同一 target |
| target 对应范围 | x∈[10.10000038,14]、y∈[-2,2]、z∈[0.10000000,1.60000002]，全部在 +12 m 错误副本 |
| T overlap | **477/477=1** |
| T 六 eigen | `[0.0108992911,0.1138215725,0.2219266827,0.2279665158,0.5198806703,1.9451892770]` |
| T min eigen / condition | **0.0108992911 / 178.4693388** |
| V 最终 accept / T 旁路 gate | **true / true**；T 不参与 V 实际提交 |

这些是最终变换下的后验对应数，**未插桩 GICP 每轮 optimizer 内部对应数**，不能称为最后一轮求解器日志。PCL 1.14 源码中 GICP 用平方距离 `<max_correspondence_distance²`，`getFitnessScore(max_range)` 用平方距离 `≤max_range`。本轮参数为 1，截止值一致；推广到其他距离参数时需区分这两个 API。

完整 T 质量矩阵如下（顺序 tx,ty,tz,rx,ry,rz；显示 6 位，完整精度见 JSON）：

```text
[ 0.489732 -0.029913  0.002381  0.003158  0.002987 -0.422241 ]
[-0.029913  0.490186  0.002807  0.005369  0.004806 -0.408074 ]
[ 0.002381  0.002807  0.020081  0.029134  0.031717 -0.007964 ]
[ 0.003158  0.005369  0.029134  0.160514  0.049584 -0.014306 ]
[ 0.002987  0.004806  0.031717  0.049584  0.166480 -0.011742 ]
[-0.422241 -0.408074 -0.007964 -0.014306 -0.011742  1.712690 ]
```

这是 fixed T 的 centered point-to-plane `average(JᵀJ)` 信息 / Hessian 近似，**不是 PCL covariance-weighted GICP 真正 optimizer Hessian，也不是校准后的 pose covariance**。导出矩阵的 eigen 与原 T 函数一致。中心化使整体平移从 lever arm 中消去；重复副本具有相同局部法向与几何分布，因此两个地点都可以满秩、condition 良好。米 / 弧度混合尺度、采样和 voxel 也影响谱。

| gate | 固定限值与为什么通过 |
| --- | --- |
| source / target 点数 | 477 / 630 均≥100 |
| convergence / finite / fitness | hasConverged=true、矩阵有限、0.00831≤0.50；收敛是局部停止，不保证全局最优 |
| guess-relative jump | 0.02863≤2 m、0.000131≤1 rad；seed 本身错 12 m，门只看附近修改 |
| frame / timestamp | 合法 map frame、age=0；原节点的三份新时间戳同几何反例见 §4 / §9 |
| covariance | clamp 后 xy / yaw variance=0.01，z / roll / pitch=0.02；来自 fitness，未携带地点不确定性 |
| z / 平面 | z=0.02862≤0.5 m、roll / pitch 绝对值≤0.2 rad，错误地点也可以合法 |
| T overlap / eigen / condition（仅旁路） | 1≥0.30、0.010899≥0.001、178.47≤1e5，只评价当前对应集合的支持与局部可观测性 |
| temporal / 独立地点唯一性 | V 没有全局唯一性门；首次错误 baseline 没有独立旧可信 correction 作参照 |

**归因：人工 wrong seed 进入地图重复几何的另一 basin，局部配准很好，现有质量指标无法判断全局唯一性。** 原地图同样 seed x=12 无对应 / 不收敛，是负控制。此实验从未使用 CAD 外参、点时序或动态云台，不能把该合成错解归因于它们；真实车辆的输入 / timing 故障仍需另外排除。

![两个全局地点的局部配准与质量谱均可很好](../../experiments/neu_localization_l1/evidence/l1m/aliasing.png)

### 10.2 小规模 baseline / false-match 数据集

保留 **4 个合成几何模板、17 个固定配置**：原 fixture、12 m 重复图、只在正确地点增加独有高处特征的重复图、平面；稀疏和部分云为输入变体，不计独立实车 episode。[labels.json](../../experiments/neu_localization_l1/evidence/l1m/labels.json) 与输入分开，恒等真值 / xy≤0.10 m、yaw≤0.10 rad、z≤0.10 m 容差在求解前固定。手工压力 seed 不由 GT 算法生成，后端不读取标签。PCD 在隔离 build；脚本、Git fixture、输入 hash 可重建全部合成数据。

| 分类 / 配置 | 结果与意义 |
| --- | --- |
| 正确 basin：identity、重复图 seed0 | V 接受，xy≈0.000757 m；正控制 |
| wrong seed：原图 far12 / yaw=-π / no seed | far12 不收敛；no seed 不配准；wrong yaw 错误接受，xy≈1.005 m、yaw≈3.138 rad，T 也通过 |
| 重复结构 seed12 | 12 m 错误接受、477/477 对应、T 也通过 |
| 三份部分云 × seed0 / seed12 | 人工高度裁剪 / 固定点 mask，filtered source=371 / 328 / 289；三份 seed12 均约 12 m 错误接受、T 全通过；不是 MID360 beam / occlusion 模型 |
| 独有特征 seed0 / seed12 | source=525、target=648。正解 fitness≈0.007904、overlap=1；错解 fitness≈0.008300、477/525 对应、overlap≈**0.90857**、min eigen≈0.010899、condition≈178.47，仍全通过 |
| 退化平面 | source=961 / target=625，fitness≈0.007742、overlap=1；T eigen `[0,0,0,1,3.2,3.2]`，非正定被拒；V 接受。恰好等于 GT 不代表姿态可观测 |
| 低点 / stale | 50 点在配准前拒绝；正确配准 age=0.6 s 被真实 V 位姿门拒绝 |

17 配置中 V 链接受 13，其中 7 个不满足预定合成真值容差；4 个拒绝。**不是实车成功率或 false-accept 概率。** `partial0_seed0` 是保留的非预期结果：xy 很小，但最终 z=0.122711 m 超过评分容差，仍被 V / T 接受。部分高度观测下沿重复垂直采样的偏移说明正确 seed 也不保证完整 pose 正确；未为它调整几何、评分或阈值。

独有特征例的 48 个 filtered query 点在错误地点超出对应距离，被排除在 fitness 分母外。剩余 477 点残差仍低，一侧 overlap 只下降到约 0.91。**地图有独有特征，不代表当前质量计算实际利用了独有证据。** 增加一个 overlap 数值只能修这个比例案例，无法解决完全重复的观测。

[temporal.json](../../experiments/neu_localization_l1/evidence/l1m/temporal.json) 比较三份不同部分云上的固定 seed0 / seed12 假设，并以理想 canonical odom `[0,0]→[0.2,0.05]→[0.4,0.1]` 作事后运动诊断。错误组相邻完整 map→odom 平移变化最大 0.161861 m、yaw 0.0000715 rad；正确 seed 组为 0.163661 m / 0.0000984 rad。变化包含 Z，两组相近，而错误组平均全局 XY 误差≈12.000115 m。这里未运行新的 temporal gate、状态机或 feedback profile。

固定全局变换 A 下，`(A·P_i)⁻¹(A·P_j)=P_i⁻¹P_j`。因此相邻相对运动 / odom innovation 会消去稳定地点偏移。Temporal consistency 可拒绝跳变、不相容运动和短簇噪声，**稳定一致的错误地点依旧可以通过**；已有独立正确 correction 可拒绝跳入 +12 m，但首次错误 baseline 没有该参照。

### 10.3 新车输入基础：已知位置与待闭合接口

优先复用已有记录：仓库历史 `7bfb0bbf` 的新车狗洞几何改动、现有 URDF / mesh / 契约，及历史聊天 **“提取导航仿真参数”**（`6ab52130-c330-83ec-a480-8f17f9ae6dce`）。聊天的原提取值与本次附件一致；没有重新猜 CAD。本轮未获得原 SLDASM / STEP，只复用既有提取与用户“基本可信，暂按此使用”的认可，不伪称重新验证了原始装配。

[CAD 研究基线](../../experiments/neu_localization_l1/new_car_cad_baseline.yaml)、[数值核对](../../experiments/neu_localization_l1/evidence/l1m/geometry_audit.json)、[输入 / 源码审计身份](../../experiments/neu_localization_l1/evidence/l1m/input_audit.json) 保留出处及参数。

| 输入项 | 已复用 / 核对 | 冲突、限制与处理 |
| --- | --- | --- |
| wheel / footprint | radius=0.06649 m、wheelbase=0.38923 m、track=0.38754 m；8 点 polygon 跨度 0.5665×0.5670 m，计算外接半径 0.295871 m | CAD 各轮心 Z 为悬挂装配姿态；对称轮心是建模建议。未改正式导航 / 控制几何 |
| base 原点 | CAD 数值以轮轴平面中心为原点；TF 契约 Phase1.5 仿真 base_link 在地面投影旋转中心 | 理想平地差 +0.06649 m Z。研究用 `cad_wheel_center` 区分，不静默改正式 base；契约明确实车原点未最终决定，未来若改变需 URDF / SDF / 外参 / 下位机速度语义共同更新 |
| yaw 轴 | XY=[-0.00194,-0.00002] m | 近中心不等于雷达无杆臂；机械 joint Z 未给出，不根据相对坐标自行补成关节高度 |
| A 机械原点 | 轮轴系 `[0.13776,-0.09151,0.14112]`；地面基准 Z=**0.20761** m | 按用户新指示，任意选 A 作本轮研究主雷达；实车选择未定，也未将 CAD A 绑定 driver left / IP |
| B 机械原点 | 轮轴系 `[-0.17344,0.16021,0.14621]`；地面基准 Z=**0.21270** m | 保留第二组参数假设，共用算法 / 数据协议，避免重复一套研究 |
| 杆臂 | 原点相减得到 A XY=[0.13970,-0.09149]、B=[-0.17150,0.16023] m；半径 0.166993 / 0.234704 m | 附件给出的相对 Y 各差 0.00001 m，保留原值并记录为舍入；不能解释 12 m 反例 |
| 当前 URDF | gimbal height=0.115；left/right=[0,±0.12,0]、内部 IMU=[0,0.12,0]、sim lidar=[0.12,0,0.065] 相对云台 | 文件明确占位，与 CAD A/B 不同。不得当硬件标定，也不在 Research 中直接替换正式 TF |
| mount→sensor→内部 IMU | CAD 模型原点位置已知，模型轴 / 原点与官方 LiDAR / 内部 IMU 系关系未知 | 不将 CAD RPY 或机械 XYZ 冒充 FAST-LIO 的 IMU-to-lidar 外参；现 single / dual YAML 的 zero / identity 及双雷达相对外参仍占位 |
| home / heading / gimbal | heading fusion 用 home、yaw 轴、匹配 heading / rate propagation，源码补偿旋转杆臂 | initial_alignment_confirmed=false，home 全零占位；车头、home、yaw 符号、heading 源 boot/reset/time mapping 未核验 |
| 点内时间 | 固定 driver：PointCloud2 timestamp=FLOAT64 absolute ns；CustomMsg timebase=header stamp、offset_time=absolute-timebase。FAST-LIO 将 offset ns 转 ms，deskew 再转秒 | PointCloud2→CustomMsg double 舍入约数百 ns，不能解释秒级 backlog。packet time 并非接收 now；新车设备 / 内部 IMU / MCU / ROS 时钟同步未实测 |
| 输出云时间 / 积累窗 | bundle 用 lidar_end_time（双雷达取较晚者）给 registered cloud / odometry 打 stamp；云为 deskew 后当前 full / downsampled 输入 | scan_publish_en=false / publish_tf_results=false 默认未提供 GICP 云；实际启用参数、query window / source receipt age、目标机预算仍 UNKNOWN |
| map bundle / PCD | toy 的 map frame / test_only / hash / origin 声明已核对；重复 PCD 未冒充 occupancy bundle | CAD 是车体数据，不能推导场地 map origin / yaw / revision。新车真实同基准地图、独立 query、keyframe / submap、地点标签仍未获得 |
| 固定源码与部署 | V 不变；FAST-LIO gitlink e7864a63、Livox driver 2a2029a6，从冷归档已初始化 submodule 只读核对，均 clean / 匹配 V gitlink | generic、heading candidate、老车 competition 是不同 profile；新车实际 deployed commit / profile / graph / 参数回读未核验，不能合并能力 |

杆臂 sanity check：理想纯 yaw 中，底盘固定、云台转 180°，若忽略 home→当前杆臂变化，CAD 模型原点误差 A≈**0.333985 m**、B≈**0.469408 m**。30 ms / 3 rad/s 的角度错配，仅杆臂项约 0.0150 / 0.0211 m（不含远处点朝向误差）。这是数学量级检查，**不是实车误差或内部 IMU 外参**；yaw 轴近中心不能替代传感器杆臂。

**点云基准待闭合问题（源码推导，非现场故障认定）：** `laserMapping_bundle.cpp` 的 raw registered cloud / IMU pose 在其 zero-start / sensor-initial 基准 R，header 可写成 odom。新车 heading fusion 用 `T_B0_R=T_B0_Sstart·inverse(T_R_Sstart)` 重建 base-initial canonical odom B0。GICP 默认 `/lio/cloud_registered`，header=odom 时直接使用点。若按这些默认拼接、缺少额外已核验转换，点云与 canonical odom 的数值基准不一致。

必须先做到 `p_B0=T_B0_R·p_R`，或提供独立 raw frame 与一致时刻的已核验变换；只换 frame 名不够。旧车 transformed cloud 使用另一 basis transform，不能直接认为适合新车。还需确认 bundle 的 **40 scan zero-start** 与 fusion 首次 home / heading 初始化属于同一 epoch；heading boot/reset 检查不能证明 backend 的数值基准未发生变化。

geometry audit 保留独立数学正反例：给定**任意测试** home yaw=0.4 rad 与 A 机械位置，直接混合 R correction 与 B0 轨迹，sensor translation 相差 0.221474 m、yaw 0.4 rad；显式换基的 roundtrip error=0。这个任意旋转只用于证伪“同 frame 名即同坐标”，**未写入真实外参或配置**。

A↔B 切换共用算法，但须成组切换 IP / serial / topic、选定内部 IMU、mount / sensor / IMU 完整 6D 外参、home / 杆臂、时钟同步和云基准。CAD A/B 与仓库 left/right 暂无实车映射，不能只换 IP。双雷达时还要切换公共 IMU 参考并重算 / 求逆相对变换。地图若保持固定物理坐标且 source 正确归一化，可复用；否则应重建或保留显式已验证坐标变换。

### 10.4 指标边界与最终决策

| 指标 / 方法 | 能拒绝 / 诊断 | 不能拒绝 / 证明 |
| --- | --- | --- |
| fitness | 无对应、局部残差大、部分标定 / 畸变导致的失配 | 错误副本可低分，独有点可退出分母；不是全局唯一性、不是 GICP covariance-weighted objective |
| overlap | 当前 query 大比例点无对应、局部支持不足；应同时报告对应数及分母 | 重复副本完全覆盖；一侧支持不等于双向覆盖、独立约束数或地点正确 |
| H / eigen / condition | 当前对应下弱约束方向、平面退化、病态几何 | 满秩仅描述该 basin 局部约束，尺度依赖，两个错误 / 正确地点可有几乎相同谱 |
| temporal / innovation | 非单调时间、孤立跳变、不相容运动；已有正确 baseline 时拒绝跳到错地点 | 固定全局偏移可在相对运动中消去；重复计数、反馈自洽不能证明地点正确 |
| 输入 / 地图核验 | 排除 frame、外参、home、点时间、积累窗、revision 不一致 | 输入全对仍可能 perceptual aliasing；quality 通过也不证明输入已正确 |

**Modify，保持在 L1-M；不 Go 到 L2+L3。** 实际新车输入基准尚未闭合，也没有代表性独立 map / query / 地点标签。这些是得到有意义 Research 结论的最小输入，不要求提前完成 Deployment 全套验收。机制方向仍有价值，所以不 Stop。

解除 Modify 的最小条件：

1. 用 A 的实际 role 或可追溯录制数据核验完整 frame / IMU / home / point time，并证明 registered cloud 与 canonical odom 共享基准；B 保留参数即可，无需重复开发算法。
2. 取得同 revision map bundle 与少量独立正常、重复地点、退化 query；地图 / database 与留出 query 分开，保留可信地点标签，不能继续用全地图自配准代替。
3. 固定 baseline，分别统计 input mismatch、degeneracy、stable wrong basin 的拒绝 / 错受。后续全局判别需独立多地点证据，当前新增阈值、重复计数或已有 recovery 未满足该需求。

本轮只给 probe 增加可选诊断输出，没有改门或参数。关键正例 / 12 m 反例与原 CSV 的 fitness、pose、eigen、condition 数值一致；完整 H 谱与 T 函数、residual 均值与 PCL fitness 一致；CAD 地面高度 / 相对坐标舍入 / 换基正反例通过。原 34 配置 / 原节点结果未改，本轮未启动 ROS 或真机链。整个 src、正式参数、active owner 均无改动；报告、研究参数、可重建数据和小规模证据只保留在既有本地研究分支，不 push，不进入 Scan Context、因子图、UWB 在线融合或控制器。
