# RM2027 本地迁移候选来源记录

## 状态与边界

- 候选状态：**Dependency Migration Candidate（持久化本地候选）**。
- 阶段：Integration 依赖独立化；不是正式开发入口，也不是 Deployment Go。
- 原仓库：`https://gitee.com/qiheiovo/rm2027_navigation.git`。
- 本地只读来源：`/home/qihei/rm2027_navigation`。
- 正式冻结来源：`3a2c03e9dd3b6b487c8fc7d0bb0863d5cb02fa95`。
- 已验证源码父提交：`cc073ab17d85e9f29de87533c9aa4a06f59bd728`。
- `3a2c03e9` 相对 `cc073ab1` 仅增加最终冻结报告；本候选以完整 `3a2c03e9` 树为正式快照。
- 旧仓的 branches、tags、hooks、reflogs、worktree 管理数据和对象历史均未导入。

## 新旧 Git 身份关系

| 新仓提交 | 含义 | 与旧仓关系 |
|---|---|---|
| `1a592949dcebddaeaaba09cfee9186c8ea94a628` | 全新历史的根提交，完整正式冻结快照 | tree `1ddb66884319a2ad6f9f549ab0701051421bb297` 与旧仓 `3a2c03e9^{tree}` 完全一致；提交 SHA 不延续旧历史 |
| `8799b669ad182a288a9eb92421dab03473b7c3bd` | 精选研究结论与冻结代码快照 | 仅新增文件，不修改根提交中的正式冻结文件 |
| 包含本文的提交 | provenance、逐文件来源清单和迁移边界 | 使用 `git rev-parse HEAD` 获取；提交不能在自身内容中自引用其 SHA |

新仓提交 SHA 只描述新的三段本地历史，不应被表述为旧仓提交的后继或替代。

## 正式快照与子模块

主仓保留两个 mode `160000` gitlink，没有把子模块内容摊平成普通文件；本阶段只把 Livox URL 从旧 Gitee 来源改为项目控制的 GitHub 维护仓库：

| 依赖 | 固定 SHA | 恢复边界 |
|---|---|---|
| FAST-LIO | `9aa8c79ef064eafe0fae1308888248be7efa1d2e` | 主仓 gitlink；当前 URL 为团队 GitHub fork；旧审计已只读确认远端归档 ref 可达 |
| Livox Driver | `ffb5c721c3356817a84f1c6eeaf2f66c680be24e` | 主仓 gitlink；`https://github.com/qiheiouo/livox_ros_driver2_humble.git` 的 `rm2027/aud020-aud050-20261020` 精确指向该提交，保留 `2a2029a6 -> 9bdedb48 -> ffb5c721` 父链 |
| ikd-Tree | `0438b0daa6ccfaaad9f65f45a3addc318b19ae7a` | FAST-LIO 内嵌 gitlink；固定 FAST-LIO 提交声明 `https://github.com/engcang/ikd-Tree`，其 `lio` ref 可达 |

依赖迁移与验收合同见 [dependency_and_acceptance_contract_20261021.md](dependency_and_acceptance_contract_20261021.md)。Docker 基础镜像已固定到冻结构建使用的 digest，Livox-SDK2 已从浮动 `master` 固定到经镜像时戳、官方提交图和已安装头文件指纹共同确认的 `f5d9375f84efe2b15bc0a052d3e18482ed13adf4`（v1.3.1）。

## 跨分支资产来源

最终逐文件路径、Git blob 与 SHA256 见 [migration_asset_manifest.tsv](migration_asset_manifest.tsv)。下表说明固定来源和迁移范围；未列出的原分支文件没有迁入。

| 资产 | 固定来源 SHA | 原路径 | 新路径 | 状态/必要性 |
|---|---|---|---|---|
| 导航阶段冻结、动态研究总索引、外部证据 manifest | `d91d0e93f880b414c86c94eb7e4f82468db3493b` | `docs/navigation_stage_freeze_20261015.md`、`docs/dynamic_navigation_freeze_20261015.md`、`docs/research_asset_manifest_20261015.json` | 原路径 | Frozen；保存总裁决、恢复入口和证据指纹，不包含 manifest 指向的 raw 数据 |
| 旧车高速自转、坡道、真实定位/运行边界 | `46c521b2d2f8691f3cfd159b99fad8e7dc764ce7` | `docs/validation/old_car_{connected_dog_hole_ramp,fast_lio_high_spin,high_spin_localization_engineering,map_odom_correction_gate,ramp_live_tracking,ramp_selective,runtime_robustness}*` | 原路径 | Frozen；保留现场成功、失败条件和适用边界，不迁旧车完整运行源码 |
| 新车统一仿真总结 | `d5673e3a1cfe6937e4126e1df894ee6d66e63e07` | `docs/research/navigation_validation_status_20261007.md`、`docs/research/new_car_simulation_baseline/`（排除 `evidence/`） | 原路径 | Research/Frozen；保留结论与两张小型解释图 |
| FAST-LIO 确定性修复报告 | `3911af9c0ae6af551fea9900d20d4bb34f16a655` | `docs/validation/fastlio_deterministic_20261010.md` | 原路径 | Frozen；同时关闭正式快照中 `docs/external/fast_lio_multi_ros2.md` 的已知缺失链接；证据 bundle/日志未迁入 |
| Full-base heading adapter | `5733f126b71b52e7f4ddd2340b226c1589b4419b` | `docs/research/full_base_heading_adapter_20261007.md`、`experiments/full_base_heading_adapter/`（排除 `evidence/`） | 文档原路径；代码到 `research_snapshots/full_base_heading_adapter/` | Research/Frozen；保留输入合同与可复用最小实现 |
| Full-base 输入合同 | `1b722e93efd4d4f747578ef4d9497ce70f95d063` | `docs/research/full_base_input_contract_freeze_20261008.md`、`docs/research/full_base_input_contract_freeze/validation_matrix.md` | 原路径 | Research/Frozen；保留边界和停止条件 |
| Full-base 系统验证 | `4d7f40cbe4645267c002e4dd9cab2f9f252b1257` | `docs/research/full_base_simulation_validation*`、`experiments/full_base_simulation_validation/`（排除 `evidence/`） | 文档原路径；代码到 `research_snapshots/full_base_simulation_validation/` | Research/Frozen；不是正式定位代码 |
| FAST-LIO covariance 合同 | `04364d50a3db1f3a6dd79575e5532bb2f4130ab0` | `docs/research/fastlio_covariance_contract_20261007.md`、`experiments/fastlio_covariance_contract/` | 文档原路径；代码到 `research_snapshots/fastlio_covariance_contract/` | Research/Frozen；canonical covariance 仍为 Modify |
| FAST-LIO covariance ROS 边界 | `42deff2a803ecfddf0c6264007a6e7a2ba9303ea` | `docs/research/fastlio_covariance_ros_validation_20261008.md`、`experiments/fastlio_covariance_ros_validation/`（排除 `evidence/` 与 bundle） | 文档原路径；代码到 `research_snapshots/fastlio_covariance_ros_validation/` | Research/Frozen；保留 publication-boundary probe |
| NEU Task 1 | `bfc235463eb621ac377c2226c0b988c4d901679f` | `docs/research/neu_*`、`experiments/neu_factor_graph/`、`experiments/neu_localization_l1/`、`experiments/neu_localization_l2/`（均排除 `evidence/`） | 文档原路径；代码到对应 `research_snapshots/neu_*` | Research/Frozen/Modify；保留负面 basin 与权重结论，不启动 Task 2–6 |
| T-DT | `e374894a7a713ea84b82de9e92479a37041f39ff` | `docs/external/tdt_nav_kit.md`、`docs/tdt_migration/`（排除 `evidence/`）、`experiments/tdt_planner/`（排除 `evidence/`） | 文档原路径；代码到 `research_snapshots/tdt_planner/` | Research/Frozen；保留第三方许可证、上游记录和 local changes，不并入正式 planner |
| R1/R2 动态障碍完整结论 | `9e08122a6df758b7f0131a5e2de5813a17f0e97c` | `docs/dynamic_navigation/dynamic_obstacle_prediction_full_history_20261004.md`、`docs/dynamic_obstacle_critic/{validation,observation_diameter_consumption_experiment}.md` | 原路径 | Research/Frozen；保留失败路线和消费边界，不迁当前 dirty WIP |
| R1/V1 冻结 | `f85149e08cb116a53159b0482ae1db5993ae3a06` | `docs/dynamic_navigation/{v1_prediction_scope,v1_research_freeze_20260925}.md` | 原路径 | Research/Frozen；保存旧 V1 的能力边界 |
| R3 Temporal MPC | `04291a410f193c009e043af88e014cf420e1f68b` | `docs/dynamic_navigation/temporal_mpc_*`（排除 `evidence/`） | 原路径 | Research/Frozen/Stop；保留负例、取消/时序失败和停止裁决，不迁 controller 实现 |
| R4 | `bdb8b8d00308be4384ea94e8b060d60fd2173d30` | 五份 `docs/dynamic_navigation/r4_*` 最终/对照/Pareto/进展报告 | 原路径 | Research/Frozen/Stop；不迁全 controller、owner 或 lease 链 |
| Prediction Value Test | `4d59854fb8a2937ab9cf884f684cdc286842932c` | `docs/research_history/prediction_value_20261013.md`、`experiments/prediction_value/`（排除 `evidence/`） | 文档原路径；代码到 `research_snapshots/prediction_value/` | Research/Frozen；保留“更复杂预测未证明更值”的负结论 |
| STVL 最终时序与生命周期 | `40bdc50144ed6b91e0770d60ac930396432d3119` | `docs/research_history/` | 原路径 | Research/Frozen/Modify；保存最终消费、实时、C83、生命周期和 fresh-coasting 结论，不迁 raw runs |

## 研究快照隔离与许可证

- `research_snapshots/COLCON_IGNORE` 阻止整个快照树被 colcon 自动发现；各原快照内已有的 `COLCON_IGNORE` 也被保留。
- 快照不得作为正式运行依赖，不得自动进入 launch、安装或部署链。
- T-DT 快照保留 `LICENSE`、vendored `LICENCE`、`UPSTREAM.json` 与 `LOCAL_CHANGES.patch`。
- 其他研究快照的来源树没有独立许可证文件；本候选记为 `NOASSERTION`，不替其选择许可证，也不把它们视为已获公开再分发授权。
- 正式快照保留各 ROS package 和第三方目录原有声明；旧审计发现根目录无统一 LICENSE，若干一方 package 仍为 `TODO` 或 `Proprietary`，这是正式发布前阻塞。

## 冻结报告的历史链接

冻结报告保持来源字节不变，因此没有把指向旧分支 `experiments/`、`evidence/` 或历史报告的链接改写成不存在的新仓路径。完整扫描结果见 [migration_unresolved_links.tsv](migration_unresolved_links.tsv)：共 1141 条引用，其中 290 条为已排除 evidence，86 条为迁位或未纳入的研究代码，764 条为旧历史文档引用，1 条依赖尚未初始化的子模块。计数按引用出现次数统计，不代表 1141 个唯一资产。

这些引用是选择性迁移的已知边界，应通过本表固定 SHA、旧冷归档和外部证据 manifest 恢复；不能据此宣称新仓独立复现了历史实验。正式快照原有的 FAST-LIO 修复报告链接已通过迁入 `docs/validation/fastlio_deterministic_20261010.md` 解决，未出现在缺口清单中。

## 未迁入内容

- 旧仓全部历史 objects、历史分支/引用、tags、reflogs、hooks 和 worktree 元数据。
- 任一 `build/`、`install/`、`log/`、缓存、ignored/untracked 文件。
- rosbag、大型 PCD、传感器原始流、批量 JSON/CSV、视频、压缩包、bundle、调试日志和实验中间产物。
- 各研究目录的 `evidence/` 原始数据；本候选只保留结论文档、少量解释图和 `docs/research_asset_manifest_20261015.json`。
- 子模块工作树和子模块历史；候选仅保存 gitlink。
- 当前动态障碍 9 项 dirty WIP 和 R4 的 1 项 dirty 修改。

## 本地唯一资产风险（未关闭）

- 旧仓主工作区仍有 3 个已修改文件和 6 个未跟踪研究文件，属于 dynamic surface-member WIP；没有迁入或备份到本候选。
- 登记 worktree `build/r4_hws_prediction_consumption` 仍有 `experiments/r4_rotation_value/analyze.py` 本地修改；没有迁入或备份到本候选。
- `docs/research_asset_manifest_20261015.json` 记录当时 5669 个现存 raw 文件、约 1.55 GiB；它只记录指纹，不是 raw 数据副本，且可能晚于该日期继续变化。
- 本机旧仓、同机 build/worktree 和本地 backup 目录不构成经验证的异机备份。任何历史清理或旧仓退役前，必须把 manifest 对应数据和更新后的增量转移到获准的独立存储并逐项校验 SHA256。

## 下一阶段前的阻塞

1. 解决根许可证、一方 package 的 `TODO`/`Proprietary` 声明，以及研究快照 `NOASSERTION` 的保留/发布边界。
2. 正式主仓公开前确认第三方 fork 的修改声明、发布范围和长期维护责任；本阶段仅建立依赖维护分支。
3. 明确运行时外部资源：真实 `/data/rm27_maps`、车型标定、footprint、底盘模式和物理停止数据不在此 Git 快照内。
4. 按 [dependency_and_acceptance_contract_20261021.md](dependency_and_acceptance_contract_20261021.md) 在固定环境重跑构建、110 项回归、AUD-003、新车八步工具与关键无硬件 smoke；本阶段不宣称回归已重跑。

## 已完成的静态验证

- 根提交 tree 与旧冻结提交 tree 完全相同。
- 第二提交只新增精选研究资产，没有修改正式冻结源码、配置、测试或文档。
- 两个主仓 gitlink 与 FAST-LIO 内嵌 ikd-Tree pin 已核对。
- 已知 FAST-LIO 文档链接通过迁入其固定报告得到处置。
- 跨分支冻结报告的未解析历史链接已完整列入 `docs/migration_unresolved_links.tsv`，没有通过搬入 raw evidence 来掩盖缺口。
- 新仓未配置 remote、alternates、旧 refs 或旧对象替代库。
- 持久化候选本身没有把子模块工作树纳入提交；另在全新临时目录完成了 GitHub 递归恢复，三个固定 SHA、clean 状态、全仓 `fsck` 和无 alternates 均通过，详见依赖与验收合同。
- 本阶段未构建、未运行 110 项回归或 ROS/Gazebo、未执行硬件运动，也不作 Deployment Go 声明。
