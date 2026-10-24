# Z-Localization / ZLIO upstream parity audit — 2026-10-24

**Integration 软件审计与候选补齐；DO NOT MERGE；没有 Hardware Go。**
本报告回答功能是否有可调用实现及其边界，不把编译、fixture 或上游宣传当成新车效果。

## 对照对象与完整性

| 对象 | 固定状态 | 本次含义 |
|---|---|---|
| 用户当前 checkout | `experiment/dynamic-surface-members@b440138`，开始时 clean | 没有 Z 子模块/候选包；既有 FAST-LIO、canonical、AMCL/GICP 不是 Z 后端等价迁移 |
| RM main | `d37f8d6` | 没有 Z 集成；本次没有修改、合并或推送 main |
| 已冻结基础迁移 | `ddabf8b`，分支文档 checkpoint `7999717` | 原 FAST-LIO→Z backend 候选、4 行 legacy 同步 pose patch、地图工具 workflow；原 laptop 证据是历史记录 |
| 本次隔离分支 | `codex/zlocalization-upstream-parity-20261024`，从 `7999717` 分出 | 保留冻结代码，不混入当前动态障碍研究分支；本次增量单独可审阅 |
| 原始上游 | [`mozihe/z-localization@76e1263`](https://github.com/mozihe/z-localization/tree/76e126328b670adc92981d9b4900cc059a6f2343) | 2026-10-24 经 GitHub `commits/main` 核实，当前 main 与冻结 gitlink 相同，无版本漂移；GPL-2.0-only |

[完整矩阵 TSV](zlocalization_upstream_parity_matrix.tsv)是逐项分类的机器可读来源，
[上游文件覆盖清单](zlocalization_upstream_file_inventory.tsv)将 **376 个 tracked files**
（含测试、文档及 bundled driver/第三方文件）关联到矩阵条目。
覆盖清单证明检查范围可枚举，不声称逐行形式化证明。
检查了 upstream README、design、前后端参数文档、所有 launch/config、消息/服务、
前端处理/会话发布、后端输入/图/配准/恢复/描述子/地图/输出及对应测试入口。
没有将 upstream 实验 SVG、召回率描述或公开数据结果归为 RM 的实测证据。

上游 `config/backend.yaml` 为 **203 个叶参数、17 个顶层组**；
`config/lio.yaml` 为 **72 个叶参数、6 个顶层组**。
代码另有默认值而示例 YAML 未列的项目（如 G-PROBE、legacy 同步参数、PointCloud2 timing）
也按源码/参数文档审计。算法参数仍交给原解析器；RM 只限制 input/frame/owner、
关闭不兼容输出、添加显式开关，不复制或重写估计器。

| 配置组 | 矩阵条目/处置 |
|---|---|
| backend common/frame/body/descriptor | B01–B04、B13–B14、B18；private basis、sensor identity、显式 descriptor |
| prior_map / trajectory / storage | B09、M01、M10；保留原生加载、关键帧、缓存与目录保护 |
| registration / tracking / innovation / graph | B05–B08；保留原生 coarse/fine、退化验收、NIS、固定时窗/鲁棒核 |
| recovery / reset / startup_hint / pure_odometry | B10–B12、F13–F15、B19；shadow reset 可用，canonical 换代锁定 |
| loop / validation / PCM | B15；仅 `mode=mapping + enable_loop=true` |
| uwb / bootstrap / 在线时空标定 | B16–B17；无 RM UWB producer/frame/clock/天线合同，强制关闭 |
| visualization / output / services | D01–D06、L06；显示/世界云显式开启，旧 TDT 输出关闭，服务私有化 |
| frontend sensor / initialization / uncertainty | F02–F03、F07–F09；必须显式选择 config 和 IMU 单位 |
| frontend filter / map / sampling / window | F01、F04–F06；原生实现，未为新车调参 |
| frontend ROS timing/capacity/lifecycle | F07–F13、L01；原生输入校验，RM 暴露格式与点时间参数 |

## 分类口径与结果

- **Already migrated**：RM 有受控在线入口或明确可调用的离线流程；不等于默认开启、
  功能效果合格或 Hardware Go。仅 submodule 中存在源码不算迁移。
- **Partially migrated**：已有可用子集，但关键语义/证据仍缺；矩阵写明剩余条件。
- **Missing**：RM 没有对应入口或流程。
- **Intentionally excluded**：因本车契约/owner/传感器范围明确排除，写明原因；不伪装成已实现。
- **Not applicable**：上游本身没有该能力，或是非运行实验资产。

当前 69 项分类（F14 后续闭合后）：**58 Already migrated、2 Partially migrated、0 Missing、
7 Intentionally excluded、2 Not applicable**。这里的 0 Missing 只针对本表定义的上游
主要功能和安全软件入口；两个 Partial 与七个排除项继续构成明确边界。
当前 checkout 的 `Missing` 列特指没有 **Z 集成**，不是说 RM 原有定位/驱动能力不存在。

cebb150 的三项历史复核如下；本轮仅继续 F14，并已闭合原生 opt-in 事务，见下节。
S03 / M11 没有重新处理，既有结论保持。以下按当时条件保留：

1. **S03：shutdown 原 `-11` 仍未归因，blocker 不关闭。** Phase 1 先做复现，
   250/250 有效启停 exit 0：baseline mixed 100 + peer-first 30，ASan/UBSan/LSan mixed 100
   + peer-first 20。无 timeout、sanitizer 报告。TSan backend/空程序均在 main 前遭遇环境错误，
   不能给 race-free 结论。无原 core；UAF/race/double free/shutdown ordering/第三方问题均没有
   足够因果证据确认或排除。Not reproduced，原根因 confidence low；未改生产 lifecycle。
2. **F14：canonical 自动跨 reset 接续仍未实现，保持 Partial。** 已先形成九类事件的
   [正式会话合同/state machine](zlocalization_interface_contract_20261023.md#2026-10-24-canonical-会话状态机本轮先定合同再实现)，
   明确真实 pose discontinuity、process identity/sequence、clock、loss recovery 与旧消息隔离。
   复现并修复 LOST 后延迟旧 pose 被重新标 valid 的缺陷；38 pytest、实际 DDS paused/backwards
   clock、native epoch/legacy publisher restart 完整链 smoke 通过。reset/session/time rollback
   仍锁定，必须整组重启；现 plain Odometry 缺 epoch，owner 缺协调 reset 事务。
   不将安全锁定重命名成自动 continuity；该软件协议缺口不能靠新 bag 关闭。
3. **M11：坐标合同/checker 已就绪，真实地图接受仍未完成。** 新增
   [正式坐标合同及真实 acquisition/acceptance](contracts/map_bundle.md#z-localization-prior--nav2-坐标合同parity-候选2026-10-24)
   和手动 `rm_map_tools check_zlocalization_coordinates`，覆盖文件/SE3/frame、origin/yaw、
   bounds、landmark/PCD/pixel、height-band sample；25 checker tests，map tools 63 pass/1 skip。
   未烘焙 prior→map 变换会明确失败。synthetic 仅检验 checker，不作为新车 prior/map 证据。

本轮细节、根因置信度及工具限制见
[三阶段软件闭合记录](validation/zlocalization_software_closure_20261024.md)；
[最小 shutdown 证据](validation/zlocalization_shutdown_evidence_20261024.json)。
cebb150 时分类为 57/3/0/7/2；F14 本轮闭合后为 **58/2/0/7/2**，不因此关闭 S03/M11。

**当前决策：Software Modify；不能进入 `FROZEN_WAIT_NEW_CAR_DATA`；不具备合入 main 条件。**
F14 原生事务已闭合，S03 仍是既有软件 blocker；M11 仍等待实际地图接受。
受限锁定/重启子集可继续离线候选验证。没有 Hardware Go。

七个排除项：bundled driver 启动（用 RM driver）、legacy 假 reset、UWB producer、UWB
融合/在线标定、固定 sensor→body 应用、TDT legacy 输出、直接上游 canonical TF。
这些边界不能被 overlay 打开。HAP/多 MID360/有限 FOV 示例及网络参数不作为本车配置。
body 标定 executable 仍可离线使用；固定标定结果不能覆盖转动云台的动态 TF。
上游没有任意 roll/pitch 网格生成器，也不提供 RM 的 Nav2/motion authority。


## F14 后续闭合（从 cebb150 继续，只处理 F14）

**Resolved：原生 opt-in canonical reset/session 软件事务。**
新增 `canonical_session_transactions=false` gate；true 仅允许 native frontend + managed
既有 lio_adapter。backend 同一锁内的 ordered LocalizationFrame 将 epoch、sequence、
状态版本、raw odom、global pose/covariance、validity 原子交付；candidate 只作 schema 转换。
canonical owner 换代发 invalid barrier、清 reference/TF pending/twist，再由新 source-time TF
和新观测重建 basis；bridge 消费原子 canonical frame，公共 TF owner 不变。
reset 不依赖重启两位 owner；允许 odom 原点和真实 map correction 跳变，不伪造连续。

LOST/expiry 水位、旧 producer 退役、旧 epoch/sequence/version/source 拒绝、stale ticket
提交检查、正常 revision-only notice 撤销旧证据和 clock high-water barrier 均已实现。
`LocalizationStatus` 新增 session_id/session_epoch；status process ID/sequence 不因 reset 归零。
map→odom TF 使用接受观测的 source stamp，心跳不刷新 TF 或 source 租期。

验证：7 新 C++ cases，含 10,000 epoch 和 1,000 幂等 prepare stress；实际 ROS 100 次快速
reset 后恢复，覆盖 pending TF、旧消息/失序、loss/recovery、revision notice、source 退役和
clock jump；真实 backend 10 次 reset + frontend instance replacement，22 次 RPC（每次先
IN_PROGRESS 后同键重试 COMPLETED），canonical/bridge 进程 ID 保持不变。
真实 atomic pose / covariance 传递误差分别约 2.22e-16 / 5.42e-20；另有公共 TF 组合校验。
gate=false 旧 native latch smoke 通过；原 38 pytest、canonical/bridge 和 35 个上游定向
reset/session/continuity tests 通过。没有开展 S03 调查或扩展 M11。

[正式九事件合同扩展](zlocalization_interface_contract_20261023.md#f14-versioned-canonical-transaction2026-10-24显式-opt-in)、
[验证条件与限制](validation/zlocalization_software_closure_20261024.md)、
[最小证据](validation/zlocalization_f14_evidence_20261024.json)。
F14 的软件闭合不改变整体 **Software Modify**，也不授予 Hardware Go。

## 已实现的软件增量

1. 两个入口默认 `enabled=false`，且关闭时无需安装 upstream backend 即可返回空动作。
   原 FAST-LIO 接线仍可显式启动，延迟 zero-start 仍禁止。
2. 新增 `zlio_shadow.launch.py`：实际原生前后端、原始 Livox/PointCloud2、会话和 warm/cold
   reset 入口；消息、服务、预测、世界云和 TF 全部私有化，不包含 canonical owner。
3. canonical 候选新增 `frontend=zlio`：消费原子 Frame/Session，保留源时刻、私有 frame，
   用既有 lio_adapter 处理动态 sensor→base 和 finite-difference chassis twist。
   观测换代/失序/失败锁定；状态/pose/legacy cloud publisher 更换也锁定。
4. 同步 pose patch 由原 legacy 4 行改为两种前端均适用的 **3 行**，估计器数学不改。
   canonical 禁用独立 prediction；shadow 保留 native 高频 map→sensor odometry。
5. `descriptor=scan_context|gprobe`、`enable_loop`、`visualization`、`world_cloud`、`rviz`
   显式选择。禁止 loop 与 immutable-prior 同时成为校正来源；RViz 复用上游文件并 remap。
6. 新增只读 DiagnosticArray：保留五态、源 stamp、epoch/generation/revision、failure
   count/reason，拒绝畸形状态，源证据独立过期；steady-time 可报告暂停时钟下的断流。
   它不替代既有有租期的 LocalizationStatus。
7. 模型 covariance 在原始 native/shadow 输出保留。交给现有 canonical owner 的 raw pose
   covariance 显式 missing，避免把未转换的 sensor covariance 改标签后继续使用。
   全局 map→base covariance 沿用已测代数的杆臂传播，未包含机械/时钟不确定度。
8. 所有既有 map tools 直接复用，补齐 G-PROBE/both、四点/target-map 变换等能力说明与
   相关 upstream 软件验证；不为已存在工具新增生产 wrapper。

## 完整 migration matrix

完整 upstream 路径、三个 RM 状态列、feature gate、软件证据与 bag gate 见
[TSV](zlocalization_upstream_parity_matrix.tsv)。下表由同一数据生成，便于直接审阅。

| ID | 功能 | 冻结基础迁移 | 本分支 | gate / 剩余条件 |
|---|---|---|---|---|
| S01 | 来源、GPL 与固定版本 | Already migrated | Already migrated | external/COLCON_IGNORE |
| S02 | 显式构建与重复 Livox 隔离 | Partially migrated | Already migrated | 显式 base-paths；不发现 external |
| S03 | 运行后停机与资源释放可靠性 | Already migrated | Partially migrated | 250 次未复现；根因未知，blocker 保留 |
| F01 | ZLIO 核心 Eq/状态滤波 | Missing | Already migrated | frontend:=zlio;enabled:=true |
| F02 | 多帧惯性/几何初始化与协方差 | Missing | Already migrated | 显式 frontend_config |
| F03 | 连续时间轨迹、预积分、逐点 deskew | Missing | Already migrated | 原生入口 |
| F04 | 概率平面地图与相关点面残差 | Missing | Already migrated | 原生入口 |
| F05 | 时间窗口自适应 | Missing | Already migrated | frontend_config window.* |
| F06 | 空间采样自适应/容量限制 | Missing | Already migrated | frontend_config sampling.* |
| F07 | Livox CustomMsg 解码/时间/tag | Missing | Already migrated | input_type:=livox;livox_use_timebase |
| F08 | PointCloud2 逐点时刻/布局 | Missing | Already migrated | input_type:=pointcloud2;point_time_* |
| F09 | IMU 加速度单位和固定内部外参 | Missing | Already migrated | 必填 frontend_config 和 imu_acceleration_scale |
| F10 | scan-end 原子 Frame 与高频预测 | Partially migrated | Already migrated | native；高频仅 shadow |
| F11 | instance/epoch/sequence/失败通知 | Partially migrated | Already migrated | native canonical 锁定边界 |
| F12 | start/stop 输入排空服务 | Missing | Already migrated | /zlocalization[/shadow]/{start,stop} |
| F13 | 原生幂等 warm/cold reset 协议 | Missing | Already migrated | shadow /upstream_reset_request |
| F14 | native canonical 跨 reset 连续接续 | Missing | Already migrated | native opt-in 原子事务；同进程 reanchor；默认关闭 |
| F15 | legacy FAST-LIO reset 等价协议 | Intentionally excluded | Intentionally excluded | legacy 不提供 ResetFrontend |
| L01 | 原生 lio.launch 参数入口 | Missing | Already migrated | RM native 参数直接调用 zlio_node |
| L02 | frontend.launch 显示点云/轨迹 | Missing | Already migrated | shadow visualization:=true |
| L03 | bringup.launch 前后端组合 | Missing | Already migrated | zlio_shadow.launch;enabled:=true |
| L04 | backend.launch 独立 legacy 入口 | Already migrated | Already migrated | candidate enabled:=true |
| L05 | body_calibration.launch 工具 | Partially migrated | Already migrated | 手动 upstream 工具；隔离域 |
| L06 | RViz 配置/地图、路径、配准调试 | Partially migrated | Already migrated | rviz:=false 默认；visualization:=false |
| L07 | mapping/localization/prior 参数校验 | Already migrated | Already migrated | mode;绝对 prior_map |
| L08 | 配置 extends/merge/循环校验 | Already migrated | Already migrated | config_file |
| L09 | driver 统一启动和设备网络配置 | Intentionally excluded | Intentionally excluded | 由 RM 原有 driver 单独授权 |
| B01 | legacy 同步 odom/world cloud 输入 | Already migrated | Already migrated | frontend:=legacy |
| B02 | ZLIO 原生 frame/session 输入 | Missing | Already migrated | frontend:=zlio |
| B03 | 高频 map→sensor odometry | Missing | Already migrated | shadow 原生 prediction；私有 sensor_odometry |
| B04 | 同步 projection 位姿/状态 | Partially migrated | Already migrated | 顶层 scan-end patch |
| B05 | 增量图/固定时窗/边缘协方差 | Already migrated | Already migrated | graph.* 保留 |
| B06 | 里程计增量 covariance/noise fallback | Already migrated | Already migrated | graph.odom_noise.* |
| B07 | 鲁棒核、白化、创新 NIS | Already migrated | Already migrated | registration/innovation/graph.robust.* |
| B08 | tracking 子图、源累积、coarse/fine | Already migrated | Already migrated | tracking/registration.* |
| B09 | 不可变 prior 与别名/描述子一致性 | Already migrated | Already migrated | localization mode |
| B10 | startup_hint 辅助初始定位 | Already migrated | Already migrated | startup_hint.enabled=false 默认 |
| B11 | SC 无 hint 全局候选/多 yaw 验证 | Partially migrated | Already migrated | descriptor:=scan_context；recovery.* |
| B12 | SUSPECT/LOST 恢复与手动 relocalize | Partially migrated | Already migrated | recovery.automatic_when_lost;/relocalize |
| B13 | Scan Context 整平/增强/存储 | Already migrated | Already migrated | descriptor:=scan_context |
| B14 | G-PROBE 分支/FOV/验证/存储 | Intentionally excluded | Already migrated | descriptor:=gprobe 显式 |
| B15 | mapping loop 检索/几何/PCM 一致性 | Intentionally excluded | Already migrated | mode:=mapping;enable_loop:=true |
| B16 | UWB 原始 producer/时钟/frame/天线 | Intentionally excluded | Intentionally excluded | uwb.enabled 强制 false |
| B17 | UWB 对齐/xy xyz/在线时空标定/融合 | Intentionally excluded | Intentionally excluded | uwb.enabled 强制 false |
| B18 | 固定 body transform 与 body 标定应用 | Intentionally excluded | Intentionally excluded | IMU identity；动态 TF 在 RM 边界 |
| B19 | 纯里程计 gauge/origin | Already migrated | Already migrated | mode:=mapping;无 prior/loop/UWB |
| M01 | runtime 关键帧、局部云、地图保存 | Already migrated | Already migrated | /save_map;显式 trajectory.save_directory |
| M02 | 多帧可见性动态残影 clean | Already migrated | Already migrated | clean_map_bundle 手动 |
| M03 | CloudCompare 文件导出/重导入过滤 | Already migrated | Already migrated | filter_map_bundle --export-reference-map/--reference-map |
| M04 | 精确点匹配/距离过滤/原始帧保留 | Already migrated | Already migrated | --exact/--keep-distance |
| M05 | 显式 SE3 地图变换 | Already migrated | Already migrated | transform_map_bundle --translation/--quaternion |
| M06 | 四点定轴和 target-map 注册对齐 | Partially migrated | Already migrated | transform_map_bundle 对应 CLI |
| M07 | 从 PCD 网格生成 bundle/FOV/遮挡 | Already migrated | Already migrated | build_pcd_map_bundle 手动 |
| M08 | SC/G-PROBE/both 离线描述子生成 | Partially migrated | Already migrated | --descriptor-type scan_context/ gprobe/ both |
| M09 | SC 原地 rebuild/参数元数据 | Already migrated | Already migrated | rebuild_scan_context_descriptors |
| M10 | 原子写出、非空目录/别名/损坏保护 | Already migrated | Already migrated | 上游工具原生约束 |
| M11 | 3D prior 与 Nav2 occupancy 原点一致性 | Partially migrated | Partially migrated | 坐标 checker 就绪；真实 prior/map/landmark 接受待完成 |
| M12 | 任意 roll/pitch 网格采样生成器 | Not applicable | Not applicable | 上游没有此能力 |
| D01 | 五态/可信/冻结/故障原因 | Already migrated | Already migrated | 私有 /state |
| D02 | epoch/generation/revision/failure 诊断 | Partially migrated | Already migrated | diagnostics=true；私有 DiagnosticArray |
| D03 | source age/进程失联/clock pause | Partially migrated | Already migrated | diagnostic 0.5s；steady-clock timer |
| D04 | 世界云、prior/runtime 地图、轴、路径、loop/debug | Partially migrated | Already migrated | visualization/world_cloud 独立 false |
| D05 | TDT legacy pose/twist/path/danger | Intentionally excluded | Intentionally excluded | output.legacy.enabled 强制 false |
| D06 | 直接上游 canonical map/odom TF | Intentionally excluded | Intentionally excluded | TF quarantine;frontend.publish_tf=false |
| D07 | RM source-time sensor→base/covariance | Already migrated | Already migrated | canonical candidate only |
| D08 | RM LocalizationStatus 有效租期 | Already migrated | Already migrated | scan stamp + 0.5s；不改原 owner |
| T01 | 上游 unit/integration/ROS/tools 测试入口 | Partially migrated | Already migrated | 显式 BUILD_TESTING=ON 与选定测试 |
| T02 | 公开大数据 cleanup/evaluation 指标 | Not applicable | Not applicable | LOCALIZATION_TEST_DATA_DIR 显式 |
| T03 | 上游 launch/config 参数组全覆盖清单 | Partially migrated | Already migrated | 工具原生 CLI + 两个受控入口 |

## 本轮剩余项软件验证（基于 474572e）

本轮没有扩展 upstream 功能清单；按 Phase 1→2→3 完成，生产退出逻辑没有猜测性修改。
独立提交：`52e4e5a` shutdown 复现工具/证据；`7deded0` canonical 会话隔离修复；
`fd50098` prior/Nav2 坐标合同/checker。三阶段结束后才更新本报告和原矩阵。

| 检查 | 本轮真实结果 | 限制 |
|---|---|---|
| baseline shutdown stress | 130/130 exit 0；含 30 peer-first | Not reproduced，不是根因排除 |
| ASan/UBSan/LSan stress | 120/120 exit 0，无报告/timeout；最大退出 0.408s | app/core 插桩；ROS/PCL/GTSAM/TBB 库内部未插桩 |
| TSan | backend 编译成功，运行 main 前 exit 66 unexpected memory mapping；空程序对照复核同故障 | 不可用，未证明无 race；空程序一次 -11 也不算 backend 复现 |
| session 确定性 harness | 38/38 pytest；旧版 LOST→旧 pose 复活可复现，修复后拒绝；含单次/多次/100 次快速 reset、延迟/重排、旧 session、loss/recovery、clock | 自动 rebase 仍锁定 |
| 直接依赖构建/回归 | candidate/adapter/bridge 3 package 构建；canonical CTest 1/1（15 gtests），bridge 3/3（5 math + 7 gate + 4 profile cases） | 没有重跑无关模块或声称硬件效果 |
| 实际 ROS clock | paused expiry、invalid heartbeat、fresh resume、backwards latch、source-time TF/covariance PASS | adapter DDS fixture；无 estimator claim |
| native/legacy 实际全链 | 两条 smoke PASS；动态 correction 最大误差 1.11e-16；断流 TF 停刷；epoch / publisher restart 锁定；clean exits | 原子消息/room fixture，无新车定位精度证据 |
| 坐标 checker | 25/25 tests；map tools 63 passed / 1 skipped；安装命令 synthetic pass=0、未填模板 reject=2、JSON | skip 为未打包真实 field baseline；从不批准真实地图 |

复现命令、事件合同、失败夹具/工具限制、真实数据接受步骤见
[闭合记录](validation/zlocalization_software_closure_20261024.md)和
[tests README](../tests/zlocalization/README.md)。以下为 `474572e` 及更早验证，保留原条件。

## 基础迁移与原 parity 软件验证（历史证据）

以下历史结果按两次运行区分。中断前完整运行的结果来自本次对话保存的真实工具输出；
临时目录在中断期间清理后，源码按原执行顺序恢复，恢复后的复核单独列出。

| 检查 | 结果与适用边界 |
|---|---|
| 中断前构建 | 私有 GTSAM 4.2、RM pinned Livox 消息/driver 包（未启动）、localization_interfaces、frontend、bringup、backend、RM 5 个直接依赖/候选包均通过 |
| 前端 CTest | **8/8 目标 PASS**：config/state/input/estimation/initialization/map/adapter/sensor_pipeline；最后一项实际处理 Livox/PointCloud2 并检查恢复，不只是 mock 消息 |
| 后端 CTest | **14/14 目标 PASS**：measurement/lifecycle/graph/innovation/relocalization/localization/descriptor/calibration/storage/cleanup/tools/launch_config/calibration_launch/map_service |
| 上游 bringup | reset_protocol PASS；composition 首次 4/5 PASS，唯一失败为 `driver=true` 找不到 bundled driver launch；与 L09 明确排除一致。最小复核保留支持范围后 **4 PASS / 1 deselected**，不把排除用例说成通过 |
| RM 单元/几何 | 中断前 **25 pytest PASS + 15 canonical gtest PASS**；包含 native instance/epoch/failed/reset_required/重复帧及 covariance missing 边界 |
| 默认 feature gate | 两个真实 installed launch 不传 enabled 时均正常退出、无节点；另有 disabled-entry 不解析 optional backend 的单元检查 |
| legacy 完整边界 smoke | startup 不可信、动态源时刻几何闭合最大元素误差 `1.11e-16`、唯一 TF owner、断流失效/停止刷新 correction、publisher restart 锁定 PASS |
| native canonical 消息边界 smoke | 原子 Frame/Session→既有 canonical/bridge、误差 `1.11e-16`、源 lease、epoch 锁定 PASS；此项使用原子消息 fixture，不冒充真实 ZLIO 传感器处理 |
| 独立 prior smoke | 真实 scan-to-map/图接受、fixture correction 最大元素误差 `0.002032`（筛查限 `0.03`）、invalidate 锁定 PASS；有 startup hint，不据此声称全局召回 |
| G-PROBE + loop mapping smoke | 启动/数据链、lease、几何误差 `2.22e-16`、失效锁定 PASS；真实回环检索/验证/图提交和伪边拒绝另由 backend_localization_test 覆盖 |
| 原生 shadow 原始传感器 smoke | 实际 ZLIO 生成 **20 frame / 176 map-sensor odometry**，start/stop/协调 reset 到 epoch 1 PASS；公共 TF/canonical odom/status/cmd publisher 均为 0；显示/world cloud 入口实际启用 |
| 地图/标定 | 原生 tools 覆盖 save/load/clean/filter/build/both descriptors/rebuild/显式 SE3 与 target-map 对齐及退化拒绝，全部通过；四点定轴只有源码/CLI 审计，CloudCompare/RViz GUI 未执行 |
| 停机 | 首次失败 smoke 出现一次 backend `-11`；随后六组停机 clean。**未定位根因，S03 保持 Partial、禁止据此合入** |
| 恢复后构建与测试 | 在持久 worktree 重建两套 interfaces 与候选包 **3/3 PASS**；**25/25 pytest PASS**，未将完整 backend/frontend 重新构建伪装成恢复后运行 |
| 恢复后暂停时钟诊断 | 实际 ROS clock 固定在 1 秒，重复同 stamp 心跳后变 STALE；畸形 state 变 ERROR；**10 条诊断，断言与干净退出 PASS**。已修正原测试向 ros2 run 包装器发 SIGINT 不转发的问题，改为直接运行 installed executable |
| 审计覆盖/源码 | 376 文件 inventory、69 项分类/全部配置组、Python 解析与 diff 空白检查通过；子模块保持 clean upstream pin，补丁单独保存 |

本次实际选定的 CTest 命令为：

```bash
ctest --test-dir "$BUILD_BASE/frontend" --output-on-failure \
  -R '^frontend_(config|state|input|estimation|initialization|map|adapter|sensor_pipeline)_test$'
ctest --test-dir "$BUILD_BASE/backend" --output-on-failure \
  -R '^backend_(measurement|lifecycle|graph|innovation|relocalization|localization|descriptor|calibration|storage|cleanup|tools|launch_config|calibration_launch|map_service)_test$'
ctest --test-dir "$BUILD_BASE/bringup" --output-on-failure -R '^bringup_reset_protocol_test$'
```

修正过的软件/测试问题还包括：Humble `DiagnosticStatus.level` 是 bytes，显式转换后复核通过；
G-PROBE 模式必须去掉 SC 示例参数，因为 upstream 严格 reader 只声明选中 descriptor 的 schema，
对应配置回归和真实启动均通过。这些修正不改变后端估计器数学。

验证环境是已有 `rm2027_navigation:humble`，无网络、无设备映射、非 root，私有 ROS 域。
GTSAM 4.2 `4f66a491` 仅构建到本 worktree 的 ignored 私有前缀；不安装系统依赖。
复用 RM 已固定 Livox driver 消息，没有构建/运行 upstream bundled driver。
初次 frontend 构建被 GCC 11 `std::optional` 的 `-Werror=maybe-uninitialized` 拒绝；
检查 `current_cost` 赋值/has-value 分支后，仅添加 `-Wno-error=maybe-uninitialized`
保留警告复核。没有改初始化器数学；此编译条件必须在复现与 CI 中明确。

长期复现入口在 [tests/zlocalization/README.md](../tests/zlocalization/README.md)，
源/配置覆盖检查为 `python3 tests/zlocalization/audit_inventory.py --check`。
原 worktree `/tmp/rm2027_zlocalization_parity_20261024` 的源码/日志在中断期间被清理；
分支和 Git 元数据保留。已从本次对话执行记录按原顺序恢复源码到持久目录
`build/zlocalization_upstream_parity_20261024`，覆盖清单和差异复核通过。
中断前完整构建产物/原始日志现不可用；其结论在上表明确标为中断前实测。
恢复后的日志位于该 worktree ignored `build/parity/recovered_checks.log` 与
`build/parity/paused_clock_recheck/`；不将 build/cache 作为长期 Git 资产。

## 新车 bag 到来后的合入门槛

**先确定拟合入的配置及功能集合。** 下列 G0–G8 针对拟保留入口；未验收的 loop、G-PROBE、
native canonical 能力必须维持关闭，不能借主路径通过顺带启用。bag 能提供软件/回放证据，
不能单独给出 Hardware Go；本项目要求的受控实车/目标机检查仍独立执行。

| Gate | 必需输入/场景 | 允许进入下一步的可观察条件 |
|---|---|---|
| G0 版本/输入资格 | 固定 A/B commit、参数、同一原始 bag、独立建图与 query 分段、同一 prior 及 map 接受记录 | 明确车型/安装/场地/时间；不回放 canonical odom/TF 与 live owner 竞争；默认 gate 仍关闭；先记录评价阈值，不能看结果后放宽 |
| G1 原始传感器/时间 | Livox timebase/header+offset 或 PC2 字段/单位、IMU g/SI/轴、clock、丢包统计；低速/高速/断流 | 点时刻和 IMU 源时刻单调且同时间域；原生两输入格式至少验证实际使用者；legacy cloud/odom 同基准并满足 1ms 同步；无 scan40 静默 rebase；单位和固定内部外参经确认 |
| G2 动态几何/canonical | 静止转云台、纯车体 yaw、高速自转、pitch/坡道/横坡；同步真实 gimbal/heading/home/boot 数据 | 车体静止时不把云台转动当车体运动；sensor→base 只施加一次；源时刻 TF 缺失不使用 latest/固定角替代；唯一 map→odom、odom→base owner；full-base/reference 对齐资格明确 |
| G3 ZLIO vs FAST-LIO | 同 bag、相同校准/单位/时间和独立 session；静止初始化、运动、遮挡/弱几何、狗洞等 | 初始化成功/失败可解释；deskew、速度符号/量纲正确；ATE/RPE/姿态/漂移/失效率与稳定 baseline 同口径比较，预先确定绝对误差预算且无不可接受退化；没有参考真值时不得只以轨迹闭合作为精度证明 |
| G4 地图流程 | 实测 raw 多帧 bundle→clean→人工复核→filter/transform/build/rebuild；静态 landmark 与动态物体标签/可见性 | 不删除关键静态结构；地图完整可重载、原始不可变；poses/clouds/map/descriptors 同一坐标，SC/GP 参数元数据一致；网格水平/FOV/高度假设适用；3D prior 与 Nav2 occupancy 原点/尺度/朝向经独立 landmark 确认；先跑结构 validator 和本轮 coordinate checker，保留实际 metadata/报告，synthetic pass 不算接受 |
| G5 正常先验定位/统计 | 独立 query；正常巡航、云台运动、自转、坡道、窄结构；原定位 A/B | 有 hint 和无 hint 分开评价；误差、correction 跳变、有效覆盖率、收敛时间满足预先预算与 baseline 对照；检查 rank/退化、NIS/创新相关性、covariance coverage；不使用 missing canonical covariance 做置信判断 |
| G6 丢失/恢复/会话 | 遮挡、错误/相似场景、输入暂停/恢复、时间回退、节点重启、instance/epoch/MCU boot 变化、manual relocalize/invalidate | 无输入/不匹配/故障不 trusted；源 stamp+0.5s 内失效，旧心跳/冻结 TF 不续租；错误 basin 接受数必须为 0；恢复使用新有效输入并经完整几何验收；gate=false 换代锁定；native gate=true 按正式原子事务 invalid→prepare→fresh commit，旧 session/回调不能复活；loss/expiry 要求严格新 source，暂停/回退按时钟高水位合同验证；任何默认启用仍待真实接受 |
| G7 descriptor/loop 可选能力 | SC/GP 分别使用匹配 prior；不同 yaw/位置/倾斜/重复结构；mapping 真回环和伪回环 | 分别记录 recall@K、错误接受、恢复成功/延迟；最终错误接受为 0，召回/成功率满足预先固定预算；GP 不得沿用有限 FOV 示例冒充本车调参；loop 只 mapping，真实边改正漂移、伪边被拒、无重复提交或地图破坏 |
| G8 runtime/诊断/资源 | 回放长段、bag 暂停/恢复、backend/adapter 杀进程；拟部署 NUC 负载、温度/CPU/内存/延迟 | state/version/reason 与失效时刻一致；steady-time 断流可见，无 stale valid；无双 owner/命令副作用；源到输出延迟持续落在 0.5s lease 内并满足控制时延预算；内存/图/云缓存有界，无持续排队或热降频导致失效 |
| G9 Hardware 与运动授权（独立） | 确认过的 MCU/安装/固件/时钟/ESTOP/停稳；若将来接 UWB，另行契约与标定 | bag 通过不替代现场检查。完成项目已有目标机、受控低速实车及 AUD-003 最终授权验收后再讨论 Hardware Go；定位 trusted 从不直接授予速度 |

每个 bag gate 留最小可复核指标/失败片段/配置。绝对精度、recall 和延迟预算须依据场地、
车体安全净空与控制频率在实验前确定；本次没有新车数据，不编造统计阈值或成功率。
任何核心效果失败，先排除输入/外参/时钟/实现问题并做一次最小复核，再给 Modify/Stop。

**允许提出合入的必要条件**：拟合入入口的 G0–G8 有证据，S03 停机问题必须关闭；
F14 的 gate=false 可保留锁定策略；gate=true 按已实现事务及真实数据资格选择，M11 在先验定位入口合入前必须完成地图审查。
源码/contract/默认 gate 一致；同步最新 main 后重跑受影响构建与核心回归，
按项目要求补足受控实车/目标机证据后单独 PR。UWB/未经真实接受的 canonical 配置/硬件默认启用
不能包含在这次软件审计的通过范围。本任务不执行 merge 或 push。

F14 后续闭合后仍不允许冻结为“只等新车数据”：S03 的原 blocker 保留。F14 native
opt-in 已支持同进程 session/reset 事务；gate=false 保留旧锁定行为，不为其他 owner/profile
自动解锁。新车真实输入/动态几何/效果与 M11 地图接受、目标机/实车授权仍按原门槛执行。
本轮未重新调查 S03、未扩展 M11、不 merge/push main。
