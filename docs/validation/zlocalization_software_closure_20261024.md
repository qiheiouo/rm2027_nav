# Z-Localization 剩余三项软件闭合记录

Integration；继续 `codex/zlocalization-upstream-parity-20261024@474572e`。
严格顺序为 shutdown → canonical session → prior/Nav2 coordinates。
不重新审计功能范围；不 merge/push main；不接设备；没有 Hardware Go。
本页保存本轮最小可复核证据；原 parity 报告和矩阵在三阶段结束后统一更新。

## Phase 1 — shutdown `-11`

**Not reproduced；根因未确定；S03 blocker 不能关闭。**
对本轮记录的 workload 未发生崩溃有高置信度；对原始单次 SIGSEGV 的根因判断置信度低。
不把未复现或 sanitizer 没有报告推导为“已修复”。没有生产 lifecycle 修改。

### 复现环境与方法

- 原现场：诊断节点因 Python byte 字段问题先退出，随后整组 SIGINT，backend 曾 `-11`。
  原 core/堆栈未保存，跨重启后原临时构建/日志丢失；当前无法做原现场 gdb 回溯。
  host core_pattern 转交 apport；没有该 backend core；没有读取不相关崩溃文件。
- 固定 upstream `76e1263`，只应用基线已有 scan-end pose patch，没有先改 shutdown。
  ROS Humble / Ubuntu 22.04 / GCC 11.4；现有 `rm2027_navigation:humble`。
  GTSAM 4.2 `4f66a491ffc83cf092d0d818b11dc35135521612` 私有前缀，system Eigen、TBB OFF。
  Docker `--network none --user 1000:1000 --cap-drop ALL --security-opt no-new-privileges`，无设备。
- 构建：`tests/zlocalization/build_shutdown_probe.sh`；夹具：
  `tests/zlocalization/shutdown_stress.py`，README 有复核命令。
  每次直接启动实际 `backend_node`，观察就绪；非 idle 必须观察真实 projection。
  混合场景轮转 idle/scan/heavy world-cloud+visualization/loop+maintenance；启用高频 prediction。
  peer 场景则禁用 prediction，订阅节点先停止，再立即 SIGINT backend。
  输入源为 synthetic room，只检验软件生命周期，不检验定位精度。
  变动输入工作量以覆盖不同信号到达点；没有 shutdown sleep、退出码忽略、catch-all 或 retry-to-pass。
- ASan+UBSan：`-O1 -g1 -DNDEBUG -fsanitize=address,undefined -fno-sanitize-recover=all
  -fno-omit-frame-pointer -fno-pie`，link `-no-pie`；ASAN `abort_on_error=1:detect_leaks=1`，
  UBSAN `halt_on_error=1:print_stacktrace=1`。无 suppression。
  `EIGEN_MALLOC_ALREADY_ALIGNED=1` 与未插桩 GTSAM/PCL ABI 一致：编译器验证 release 为 1，
  GCC ASan 默认为 0；显式一致避免跨库 Eigen allocator ABI 错误。
  backend/core 已插桩；ROS/PCL/GTSAM/TBB 内部没有插桩，不能排除第三方内部 race。

有效结果：**250/250** backend exit 0，无 timeout。基线 mixed 100、基线 peer 30、
ASan+UBSan mixed 100、ASan+UBSan peer 20；ASan/UBSan/LSan 无报告。
混合各含 25 idle/stream/heavy/loop；最大退出时间（秒）：0.160693, 0.183183, 0.407456, 0.224488。
夹具 share 路径错误等启动前失败不计入有效轮次，最终 tracked 夹具已复跑 20 轮。

### 源码生命周期检查

| 边界 | 检查结果 | 能否归因为原故障 |
|---|---|---|
| executor / node | executor 在 node 后声明；spin 返回时工作线程已汇合，remove_node 后 finish，再销毁 executor/node | 未发现明确悬空 callback 证据 |
| callback groups | input/default、maintenance、high-rate、map 分组；reset reply/input 重建受 epoch 与组约束 | 无可证明 race；TSan 未能运行 |
| object lifetime | node 默认析构；subscriber/timer 在 controller 前释放；frontend/controller、publisher/model raw 引用在 node 域内 | 未发现确定 UAF/double free |
| async reset | pending request 保存 ID/时刻并加锁；client 及其 callback 登记随 node 释放；没有 detached app thread | 未发现确定非法 lifetime |
| parallel work | backend TBB parallelFor 等待整批完成再传播异常；非 detached | 未建立退出后仍访问模型的证据 |
| logger | 原子 raw Logger*；node/executor 析构后恢复默认 logger，RosLogger 仍存活 | 未发现确定 UAF |
| SIGINT context | rclcpp signal thread 异步 shutdown；main 中 `if(ok()) shutdown()` 在已 shutdown 时跳过显式卸载 signal handlers | 仅假设；没有因果证据，未做猜测性修改 |
| third-party | DDS/ROS/TBB/GTSAM/PCL 共享库参与退出 | 无原栈，无法肯定或排除第三方问题 |

### 观测与工具限制

结果由同目录 `zlocalization_shutdown_evidence_20261024.json` 保存聚合计数，原逐轮记录在
worktree `build/closure/phase1/*/summary.json`（ignored，不作为长期日志资产）。

- TSan backend 编译成功，但首轮进入 main 前失败：exit 66，
  `FATAL: ThreadSanitizer: unexpected memory mapping`。
  空 `int main(){return 0;}` 的同编译器 TSan 对照一次 `-11`，另三次均 exit 66 + 同错误。
  这是检测运行环境不可用，**没有获得 race-free 结论，也不是 backend `-11` 复现**。
- 初版 peer 使用 rclpy 默认异步 SIGINT，出现 duplicate shutdown / pybind take_message
  RuntimeError，backend 均 0。无效 peer 试验不计作通过或 backend 复现。
  最终 peer 以 signal flag 停止 spin，再同步销毁 node/context；没有任意异常吞掉逻辑。
- 无 SIGSEGV/UAF/double-free/race 根因证据可提交最小生产修复；本阶段只保留复现工具与证据。
  关闭 S03 仍需原故障可复现 + 根因修复/回归，或独立可复核的原故障归因排除。

## Phase 2 — canonical session / reset

先在 [接口合同](../zlocalization_interface_contract_20261023.md#2026-10-24-canonical-会话状态机本轮先定合同再实现)
确定九类事件、状态机及 consumer 行为，再修改代码。连续性选择是：
**保留同会话 odom basis；暴露真实 map correction 跳变；换 basis 锁定并整组重启。**

明确根因（高置信度）：基线 adapter 在 LOST 时清 pose，但没有撤销 source 水位；同 stamp
的延迟 pose + 恢复状态会重新组成可信输出。对 `474572e` 原方法的 deterministic 复核得到
`baseline_old_pose_resurrected=true, stale_pose_x=99.0`。这不是物理新观测。

修复：新增小型 `SessionWindow`（水位/版本/时钟准入，不做 rebase）；旧 generation/revision
状态丢弃；loss/steady expiry 记录已见 source 水位并清 pose；恢复必须严格新 source。
ROS rollback、非零初始 frontend epoch 锁定；用 steady timer 保持 paused/latch 下 invalid 心跳。
源 stamp、covariance 杆臂传播、唯一 TF owner、默认 feature gate 均保持原合同。

自动跨 reset 继续输出仍不能在现有 plain Odometry 和固定 startup reference 下安全完成。
**F14 仍 Partial**；本轮关闭的是旧证据复活/时钟准入缺陷及语义歧义，不将安全锁定冒充
reset continuity。完成该能力还需 versioned canonical input 和完整 owner reset 事务，
属于明确软件缺口；新车 bag 本身不能弥补。没有新增 public reset wrapper 或 owner。

验证结果：候选 pytest **38/38**（含 13 项新 deterministic session cases）；
3 个直接相关 package 构建通过；canonical CTest 1/1（15 gtests），bridge CTest 3/3
（5 math + 7 global-gate gtests + 4 profile cases）。
实际 adapter DDS/TF clock smoke 通过 pause expiry、invalid heartbeat、fresh resume、
backwards latch 与同刻 covariance/pose；无 estimator/真实地图效果声明。
实际 backend → candidate → lio_adapter → bridge 的 native epoch 和 legacy publisher-restart
smoke 均通过；动态 TF correction 最大矩阵误差 `1.1102230246251565e-16`；
源失联撤销有效性并停止刷新 correction TF；新 epoch/换 publisher 后保持锁定；子进程 clean exit。
测试 share symlink 配置错误的首轮在 launch 前失败，修正后两条真实链路重跑通过，未改被测逻辑。

## Phase 3 — prior / Nav2 坐标

先更新 [地图坐标合同](../contracts/map_bundle.md#z-localization-prior--nav2-坐标合同parity-候选2026-10-24)，
再实现 `rm_map_tools check_zlocalization_coordinates`。现有 runtime 要求整个 prior 已在 Nav2
map 下；不增加 runtime TF 或修改已有 owner。严格区分 PCD frame、mapping gauge、raster
origin/yaw 和像素中心/行翻转，记录 CloudCompare shift/scale 与完整 bundle 变换链。

Checker 接受 prior PCD（ASCII/常见 PCL binary）、map.yaml、可显式指定的 PGM、transform
metadata，验证文件绑定、SE(3)/frame/单位、预期 origin/orientation/resolution、旋转栅格下
bounds overlap、height-band points、独立非共线 landmark（map/pixel/PCD 支持/可选 mapping 链）
及 deterministic occupied sample correspondence。输出每项原因和 JSON，成功 0 / 失败 2。
未烘焙非 identity prior→map 即使几何可对齐也失败。未支持压缩 PCD/其他图像模式明确报错。
没有自动 map alignment、产物写回或批准开关。随包提供不可直接通过的 metadata 模板。

验证：checker 新增 **25/25** synthetic tests；rm_map_tools 全部局部回归 **63 passed / 1 skipped**。
唯一 skip 是仓库未打包的真实 field baseline，不计作已验证真实地图。
宿主完整包测试因未安装 ROS sensor_msgs 无法收集，改在现有 Humble 环境重跑得到以上结果；
无新依赖。package 安装通过，实际 `ros2 run` entry 对非零 origin + yaw + 非对称图像 synthetic
fixture 输出 consistency=true、real_map_accepted=false、hardware_go=false。
测试包含坐标/单位/scale/reflection、变换方向/未烘焙、错行像素、地标缺失/共线/错位/PCD 无支持、
错 mapping 链、unknown/free cells、negate/等阈值、文件/图像绑定、binary extra/vector 字段与截断。

**M11 软件合同/checker 已就绪；真实地图接受仍 Partial。** 新车 acquisition/acceptance 流程
在坐标合同中维护：独立 mapping/query bag、完整 raw/final bundle、PGM/YAML 与变换来源、
至少三处分布良好的独立实测地标和匹配 hashes → 结构/几何 checker → 人工空间复核 →
query bag 定位/恢复/动态机械/同刻 TF 评价。synthetic 结果不填作真实地标或真实 prior 验收。

安装命令正/负例均已复核：有效 synthetic fixture exit 0，未填写模板 exit 2；直接调用
installed console executable 时 stdout 是完整可解析 JSON。首个负例调用器测试用 `ros2 run`
包装器，包装器的 failure footer 令整段 JSON 解析报 Extra data；改为直接测试真实 executable
后正/负例均通过。未改 checker 判据，也没有吞掉失败码。

## 三阶段结束决策

| 原项 | 本轮最终状态 | 关闭条件 |
|---|---|---|
| S03 | Partial / Not reproduced；root unknown，confidence low；blocker 未关闭 | 归因原退出故障并做最小修复/回归，或独立可复核地排除原故障；可用的 race 工具/原 core 是下一项有效证据 |
| F14 | Partial；正式语义/隔离缺陷已解决；自动跨 reset canonical 接续未实现 | versioned canonical 输入 + owner reset 事务及回归；或明确仅接受锁定/完整重启支持范围，不把它标成连续接续已实现 |
| M11 | Partial；软件 checker/合同就绪；真实地图接受未完成 | 实际 prior/map/变换来源/独立地标及 query bag 的真实接受流程 |

**Software Modify；不能进入 `FROZEN_WAIT_NEW_CAR_DATA`；不具备合入 main 条件。**
保留全部 feature gate，不开启默认路径，不 merge/push main。没有 Hardware Go。

本轮独立实现/证据 commit：`52e4e5a`、`7deded0`、`fd50098`；最后原 parity 报告/矩阵
同步作为单独文档提交。完整 69 行迁移清单保持原分类，无重复 upstream audit。
主线合入仍需 S03 关闭、拟支持范围明确、实际 M11 与 G0–G8 证据、受影响回归和项目要求的
目标机/受控实车接受。真实 bag 不能解决缺失的软件 reset 事务；软件 fixture 也不能解决
新车时间/机械/地图正确性、实际精度/恢复效果或硬件授权。

## F14 后续事务闭合（基线 cebb150；仅 F14）

**Resolved（native opt-in 软件事务）；整体 Software Modify 不变。**
此前本页 Phase 2 的 Partial 是 cebb150 当时状态；本节取代其中“owner 协议未实现”的结论。
S03 / M11 未开展新调查或功能工作，原 blocker/真实接受要求保持。本轮不 merge/push main。

### 实现与状态

先补九事件合同，再实现 `canonical_session_transactions=false`（默认）gate，true 仅支持
native ZLIO + managed 既有 lio_adapter。backend 的上游幂等 ResetFrontend 仍是唯一 reset owner。
parent 补丁 `zlocalization_session_frame.patch` 在原 scan-end patch 后应用，不改估计器/reset 数学，
上游 pin 不变。新增 ordered atomic LocalizationFrame；状态通知和 scan 观测使用同一发布锁与
全进程 sequence；旧 projection 仍在发布前复核。candidate 仅逐字段作 schema 转换，避免正常
RM canonical/bridge 对可选 upstream package 产生构建依赖。

既有 lio_adapter 原子切换 PREPARED→PRIMED→ACTIVE：清 source reference、pending TF、twist
history；发 invalid barrier；新 epoch 新 source-time TF 锚定；后续样本才生成 finite-difference
速度。不拼接旧末端 pose。LOST/expiry/revision-only notice 撤销旧 source；新 source 才可恢复。
旧 epoch、retired UUID、失序 sequence/version、stale ticket、过期 emission time 均拒绝。
新 backend process 由显式 select-source pin UUID/minimum_epoch；服务 ACK 只表示 PREPARED，
COMMITTED 由新 canonical epoch 的 valid status 表示；新 frontend instance 则由 backend
原协议确认并映射 epoch，自动接续不需重启 owner。

Bridge 只消费单一 canonical frame（odom+map→base pose/covariance+version+validity），不按
裸 timestamp 跨话题配对。新 status 暴露 session_id/session_epoch；status producer/sequence
在同一进程内持续递增。map→odom TF 使用接受 source stamp，每个 source 最多发布一次，
publication 与撤销共锁；不以心跳伪造新测量。full 6×6 covariance 用同刻 TF 的杆臂 Jacobian
传播；canonical odom covariance 仍明确 missing，twist variance 使用既有配置。

Clock backwards 取消旧 ticket，须新 backend epoch 且 clock/source 超过旧高水位才恢复；
catch-up/旧心跳本身不能解锁。普通 reset/session 切换不依赖整组重启。所有 public TF owner
保持原有两位；消费者按新 session 清缓存，不能仅凭 TF cache 或 Bool 使用旧会话。
新 interface 字段要求所有相关 consumer 使用同一次重建的 overlay，禁止混用旧生成二进制。

### 决定性回归与限制

- **7 新 C++ cases PASS**：10,000 epoch+late work stress；1,000 rapid/idempotent prepare；
  epoch/UUID 隔离、LOST/expiry/clock、协方差 cross terms。原 canonical 15 gtests 通过。
- Bridge CTest **3/3 targets PASS**（5 math + 7 gate gtests + 4 profile cases）；
  candidate **38/38 pytest PASS**；两套 interfaces、backend、三个直接 consumer 均构建通过。
- 上游只选 F14 相关 reset/session/epoch **17 cases** 与 reset/warm/cold/continuity **18 cases**，
  共 **35 PASS**；没有运行 S03 shutdown stress 或声称该问题关闭。
- 实际 ROS translation→canonical→bridge fault smoke：**100 次快速 epoch barrier 后恢复**，
  pending TF 中 reset、旧 epoch/producer/reordered callback、LOST 后旧 pose、revision-only
  notice、pause/resume、过期 emission callback、source retirement、time rollback+新 epoch+
  high-water recovery 均通过。moving body + dynamic mount 的同刻 pose/covariance、公共
  map→odom→base 组合一致；两位 owner 的 UUID 不变。
- 实际 backend RPC smoke：**10 次 reset + 1 次 frontend instance replacement 后恢复**；
  **22 RPC calls**，每次先 IN_PROGRESS 后同一幂等键 COMPLETED；无 owner 重启。
  逐源原子输出的 pose/covariance 传递误差见 [最小证据](zlocalization_f14_evidence_20261024.json)，
  量级约 2.22e-16 / 1e-20；旧 native frame/session 注入不污染新会话。
- `gate=false` 原 native epoch-latch full-chain smoke PASS，原 38 pytest/默认配置保护保留。
  两个总 gate 均仍 false；shadow 新输出仍映射到 private prefix，没有新增公共 owner。

发现并消除的实现/夹具问题：首次 header 安装布局不匹配 ROSIDL include root，修正后重建；
对称房间 prior 曾接受 180° 匹配，加入不对称结构后最小复核通过，未改估计器/阈值；
新 stale-packet probe 曾发现“拒绝旧 sample 却取消当前 work”，最小反例 exit 1，修正后 exit 0
并纳入 regression；扩展测试自身意外回退 1ns，被 time barrier 正确拒绝，固定夹具时钟。
最终真实 wall-clock 链还捕获一次“先保存 now，后以稍晚 clock check，再准入旧 now”造成
几微秒假回退，修正读取顺序后真实/受控两条时钟链均重跑通过，未放宽时间门槛。
这些失败均不计入通过次数；保留结论所需最小证据，原始日志在 ignored build/f14。

本轮仅为 **Integration 软件事务闭合**。synthetic frontend/prior/机械 TF 不代替真实 ZLIO
精度、地图接受或新车机械时间合同；没有 Hardware Go。F14 从矩阵 Partial 改为 gated
Already migrated；S03/M11 结论保持，整体依然 Software Modify，不因本轮合入 main。
