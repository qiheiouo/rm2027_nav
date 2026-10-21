# L2 private global-candidate prototype

Research only；报告见 [L2 决策与证据](../../docs/research/neu_localization_l2_global_candidate_prototype_20261007.md)。没有 ROS 入口或正式导航输出，所有结果 `active_accept=false`。`COLCON_IGNORE` 防止日常 colcon 构建引入实验。

原写 SC-style 高度描述子：20 rings / 60 sectors；小库穷举 circular cosine distance，保留 Top-K 边界近似同分与多个 yaw 峰。不是作者原版 Scan Context / SC++，没有复制第三方代码。可选粗配准只调用已有 PCL 的 FPFH + SACPrerejective API，不引入 Quatro。

## 复现

在本研究 worktree 根目录，需已有 ROS Jazzy 构建依赖、PCL 1.14、Eigen、Python numpy / PyYAML：

```bash
source /opt/ros/jazzy/setup.bash
python3 experiments/neu_localization_l2/build_probes.py
python3 experiments/neu_localization_l2/run_experiment.py --output build/l2_v2
```

首次生成 6 个局部 keyframe 与 8 个独立采样 query，运行两种粗配准 × 三种模式，共 48 个配置。`build/l2_inputs` 是可重建的合成输入，`build/l2_work` 是私有诊断，`build/l2_v2` 是当前 schema v2 结果；`build/l2_full` 保留本轮 v1 baseline。精选证据在 `evidence/`。`--reuse-results` 仅重新评分已有输出，不重新调用 GICP；不是新的性能测量。

继续最小 prior / K / perturbation / 坐标对照：`python3 experiments/neu_localization_l2/run_followup.py --baseline-output build/l2_v2 --output build/l2_followup/followup_v2.json`。依赖上面的完整结果；`--reuse-perturbations` 复用保存的 8 个扰动配置，只更新诊断，其他小对照仍执行。不能把同一 holdout 的四个变换当独立扫描。

分层歧义 / floor 来源审计：`python3 experiments/neu_localization_l2/audit_uncertainty.py --baseline-output build/l2_v2 --output build/l2_uncertainty/audit.json`。也可对已有 v1 `build/l2_full` 执行；保留旧策略结果，不改配准 / 门。

```bash
python3 experiments/neu_localization_l2/prototype.py build-db \
  --manifest build/l2_inputs/manifest.json --output build/l2_inputs/database
python3 experiments/neu_localization_l2/prototype.py query \
  --database build/l2_inputs/database/database.json --query build/l2_inputs/q_repeat_A.json \
  --mode global --work build/l2_one --output build/l2_one.json
```

同一 query 改用 `--mode manual --manual-prior build/l2_inputs/manual_prior.json` 可提供离线人工 coarse pose。`--mode configured` 从 `config.yaml` 读取 `prior.expected_start_pose`、XY 半径、yaw 范围或 region IDs；yaw 可为 null。`--mode global` 完全关闭 prior。人工位姿没有发布 `/initialpose`；只作为检索区域和额外 seed，经过原 GICP / 位姿门及固定 T 研究质量门。

## 输入与结果语义

- 每个数据库条目包含 keyframe / place / source-frame identity、timestamp、`T_map_keyframe`、局部 cloud、descriptor、map / database / numerical-basis revision。数据库 pose 为建图几何；query GT 仅在独立评分文件中。
- 当前文件读入只支持 **ASCII XYZ PCD**；真实二进制 PCD / bag 适配尚未实现。输入必须提供 `T_odom_input`、`T_odom_base` 与同一 epoch 的时刻。两者必须处于同一 **数值 odom 基准**；名称相同不够。
- 归一化到 query 时刻、重力对齐的 base 原点：`T_Q_I = inverse(T_O_Q) * T_O_I`；`T_O_Q` 使用 base 位置与 yaw。O 必须已验证重力对齐，点云须已正确 deskew。此处不执行 deskew / MCU 同步 / home 标定。
- Q 是重力对齐的 query frame；车身倾斜时 Q 与 `base_link` 的旋转不同。结果保留 `T_query_base`，未来若需要公开 map/base pose，须计算 `T_map_base = T_map_query * T_query_base` 并重新验证。当前 probe 的位姿门作用于 keyframe-local query pose；不是正式 map/base 接受测试。source age 使用记录的 receipt-source，不包含离线求解耗时；在线提交时须重新判断结果年龄。
- 硬件输入 `normalization_status=verified_recording` 需 calibration evidence 与 point-time / deskew / home / clock / gravity verified 声明。这些是**上游证据契约**，程序不能凭声明证明硬件已标定。未知基准、revision 不符、时间错配、过期或建库 frame / cloud 泄漏会拒绝。
- schema v2 分别保留 `registration_basins`（局部门通过）和 `basins`（再通过 SC 策略）；`policy_status` 保留原策略结果，顶层 `status` 对多个局部解释优先 `ambiguous`。`single_hypothesis` / `prior_conditioned_single` 均不证明全局唯一；同分组截断为 `incomplete_search`，无合格候选为 `LOST`。所有状态无 active pose 权限。
- 结果保留所有 candidate ID、地点 / keyframe、SC score / yaw、粗配准、fine map pose、原 GICP / T 旁路指标与对应计数、候选与不同 basin 间差距、query 身份时刻和 map / DB revision。FPFH 分支增加 seed，保留 SC 分支。
- XY / yaw prior 为 bounded 搜索；附近无合格候选会扩大全局。若错误地点仍拟合良好，单凭局部门无法自动发现 prior 错误，结果会明确保留 prior 条件。

## 稳定身份与来源

新车源码 `bd6cf689d3d5aca5c8edd430ff760ca5ceb27496`；T 质量代码 `e374894a7a713ea84b82de9e92479a37041f39ff`。新加 full SE(3) seed CLI 仅作用于 L1 私有探针；旧 CLI / 17-case L1-M 回归保持一致。PCL / 文献来源、许可证和引入范围见 [source_intake.json](evidence/source_intake.json)。
