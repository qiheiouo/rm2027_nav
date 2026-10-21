# Task 1 离线 SE(3) 后端

Research only；[报告](../../docs/research/neu_factor_graph_task1_20261007.md)、[串行路线](../../docs/research/neu_serial_research_route_20261007.md)。不含 ROS 依赖或发布者，`COLCON_IGNORE` 隔离构建。既有 FAST-LIO、canonical adapter、GICP、map/odom 桥均不改。

**当前：Modify / waiting for real vehicle evidence。** 从[报告 §8：到位检查表 / 数据接口 / 固定 A/B](../../docs/research/neu_factor_graph_task1_20261007.md#8-实车到位检查表与恢复入口)恢复。下面保留历史合成复现，不作为等待期继续跑 toy 的任务。真实输入用既有 JSON / 数学 kernels；`run_experiment.py`、两个 `audit_*.py` 都不是 bag reader。A/B helpers 当前是 source-order prefix，实车 causal 评测需按真实 arrival sidecar 控制 `solve` 的已到达前缀；单独 `backend.py` CLI 计算 full batch，仅作参考。先转换输入与独立评分，不重新设计算法。

## 复现

需已有 Python NumPy 1.26.4、SciPy 1.11.4、matplotlib；PCD 连通对照另需原 L1 probe（已有 PCL / ROS 构建环境，不是新的定位节点）。来源见 [source_intake.json](evidence/source_intake.json)。不安装 GTSAM，不复制东北大学或 GTSAM 后端代码。

在本研究 worktree 根目录：

```bash
# 若 L1 probe 尚未存在，先复用既有构建入口。
source /opt/ros/jazzy/setup.bash
python3 experiments/neu_localization_l2/build_probes.py
OPENBLAS_NUM_THREADS=1 python3 experiments/neu_factor_graph/run_experiment.py
python3 experiments/neu_factor_graph/plot_evidence.py --output build/neu_factor_graph/comparison.png
python3 experiments/neu_factor_graph/backend.py build/neu_factor_graph/drift_noise_input.json build/neu_factor_graph/one_result.json
```

这里调用 L2 **构建脚本**仅重建 L1 probe；不运行 SC / retrieval，不重新给 L2 toy 调参。`run_experiment.py` 生成全部输入及 scorer labels；`backend.py` 只读测量 JSON，不读 GT。合成 GT 仅用于构造仿真观测和在 runner 中评分。`build/neu_factor_graph` 可重建且忽略；长期只留代码、总结、一组代表测量与小量结果。

## 私有输入 / 输出

`T_A_B` 将 B 点转换到 A；状态 `X_i=T_map_base(t_i)`，里程计 `O_i=T_odom_base(t_i)`，边测量 `Z_ij=inverse(O_i)*O_j`，绝对地图测量 `S_i=T_map_base(t_i)`。完整 SE(3)，不是只优化 x/y/yaw。

统一右扰动 `T*Exp([rho_xyz,phi_xyz])`；米 / 弧度，旋转向量不是 Euler 角；`L` 为 sqrt information，`Lambda=L.T*L`。顺序与 GTSAM Pose3 的 rotation-first、ROS covariance 的 fixed-axis xyz/rpy 不同，禁止只复制 6×6 数组。直接因子残差：

```text
odom: L_odom * Log(inverse(Z_ij) * inverse(X_i) * X_j)
map:  L_map  * Log(inverse(S_i) * X_i)
```

- context 必须包含 session / odom epoch / numerical basis / map / calibration revision 及 canonical frames；`input_kind` 只允许 synthetic 或上游明确标注 verified_recording。这是上游证据声明，不会靠标签证明真实标定。
- 每个 scan 因子保留 query identity、source timestamp、node、单 basin **假设**与权重来源。map/odom 时间必须已精确关联；prototype 不插值，也不把旧测量改成 now。重复 query、context / epoch 不同或 ambiguous 输入直接报错，**不负责判断现实中的盆地是否正确**。
- odom nominal incremental model 必须显式提供；绝对 pose covariance 不自动相减为增量 covariance。map `L` 可以秩亏，本最小版本要求至少一个 full-pose 测量建立全局基准；没有 map 因子仅输出 relative-only，禁止输出全局关系。
- 也支持 `odom_factors` 逐段提供 `id/from_id/to_id/from_timestamp_ns/to_timestamp_ns/context/sqrt_information/noise_model_origin`，按连续 sample 顺序，一段一份。与通用 `odom_sqrt_information` 二选一；源位姿决定 Z，不重复接受可能矛盾的另一份 Z。不同间隔 / 运动条件的噪声由上游有依据地提供，程序不自动按 dt 猜权重。
- 输出所有节点位姿及时间相关 `C_i=X_i*inverse(O_i)`、源时刻、factor IDs、solver diagnostics；`active_accept=false`，global uniqueness 未证明，posterior covariance 为 null。求解成功不能解释成定位有效。
- `reserved_factors` 可留空槽 `uwb_position`、`loop_between`、`global_candidates`；本轮任何非空槽都拒绝，避免静默忽略。未来 UWB 需 frame / lever arm / clock / covariance 与 map-UWB alignment，loop 需双 node ID / relative pose / tangent information；SC 候选需 place / basin / query / revision 集合，由 Task 2 决定能否形成单一地图因子。这里不虚构生产消息合同。

## 固定实验模型

数学对照：31 poses、0.2 s 间隔、每 3 poses 一次地图观测；rng 20261007。odom 每步噪声 sigma `[.025,.025,.015,.003,.003,.004]`，另有 body-x `.015 m` / yaw `.001 rad` 偏置；固定求解 nominal sigma `[.04,.04,.03,.008,.008,.008]`。地图模型 sigma `[.12,.12,.08,.025,.025,.03]`；这不是 FAST-LIO 或赛场 GICP 的校准协方差。weak case 首个明确 anchor、随后纵向权重精确为零，未观测的 x 值仅随初始 seed + odom 传播。

PCD 连通：16 poses、6 次既有 GICP；同一解析房间，地图 rng 100、query rng 200+i，独立点采样，point noise `.015 m`；预先声明的合成 coarse prior，无检索 / 无 seed 成功声明。V probe 参数固定 `.25/.20 m` voxel、corr 1 m、30 iterations、fitness .5、jump 2 m / 1 rad；权重固定 xyz `.08 m`、旋转 `.03 rad`，不从 fitness / T Hessian 估计。当前 graph 精度劣于单次修正，保留失败结果，不为改善表格调权重。

SciPy sparse finite-difference `least_squares(trf)`、linear loss、最多 100 nfev；每次地图到达以已到达的 prefix 离线重解；另报使用未来观测的全 batch。不是 iSAM2 / fixed-lag / 实时求解器性能证明；完整图规模会增长，Integration 前另需有依据地选择增量 / 窗口 / 边缘化。

## Task 1 协方差续审

```bash
OPENBLAS_NUM_THREADS=1 python3 experiments/neu_factor_graph/audit_noise_model.py
```

默认读取原始冷归档中的固定 FAST-LIO：`/home/qihei/rm2027_navigation/src/fast_lio_multi`；其他机器以 `--fast-lio-source PATH` 提供同一 pin 的只读源码。需要已有 g++ / Eigen / Boost / OpenMP。生成的 C++ 复核源码和程序只留在忽略的 `build/neu_factor_graph/noise_audit`；它编译真实 IKFoM header，抽取真实 publication fragment 配复制消息的 stub，不运行 ROS 或传感器。原生源码不修改；精选结果见 [noise_audit.json](evidence/noise_audit.json)。

并行任务运行时，不盲用上述历史共享默认路径；显式指定本任务独立、clean 且固定 e7864a63 的 `--fast-lio-source`。当前 worktree 两个 submodule 均未初始化，本次文档收口不初始化、不编译、不运行 ROS。输出 / 临时文件留本 worktree 的 build；以后需要 ROS 时先另选未占用 Domain。

`noise_model.py` 是原写的切空间传播，不是实车标定器：

- native mixed `[dp_world,dphi_sensor(right)]` → right sensor covariance；不能直接传 legacy ROS 数组，不能靠简单交换字段完成所有 frame 处理。
- 已知、时间匹配的 `T_base_sensor` 条件下，用 Adjoint 传播 sensor → base，包括 lever arm；外参 / gimbal 不确定性未知时不默认置零。
- 相邻右误差的 joint covariance → increment covariance。未知跨时刻相关项报错；显式 `independent=True` 只是调用者声明的独立近似，不表示滤波状态真实独立。
- 逐边信息只描述各段的噪声；即使已知一对状态的 joint covariance，仍不能据此宣称不同 odom edges 或 odom/map factors 独立。本图没有 cross-factor whitening，结果 `cross_factor_covariance=null`，不能解释成零相关。
- 正定 covariance → sqrt information；没有方差 floor / pseudo inverse，不将零 covariance 或 planar 投影制造成 full-state 权重。RP 被投影掉不等于机器人实际 RP 精确为零。

复核限定于源码、有限差分和闭式数学特例。相同 marginals 对应100倍不同 increment variance，说明真实权重来源仍未闭合；三节点不同 edge variance 的调整分配只验证逐边白化，没有定位成功率或权重搜索。原5组轨迹对照及 GICP 负面结果保留。

## Task 1 完整底盘状态续审

```bash
OPENBLAS_NUM_THREADS=1 python3 experiments/neu_factor_graph/audit_base_state.py
```

需要已有 g++ / ROS Jazzy headers；编译不改动的 V `canonical_odometry.cpp` 作实际平面 baseline，数学函数文件探针不启动 ROS。默认精选结果生成到忽略的 `build/neu_factor_graph/base_state_audit.json`；探针 / 二进制同样只在 build。长期结果见 [base_state_audit.json](evidence/base_state_audit.json)，模型及判断见报告 §7。

`base_state.py` 为原写 full-base 候选：`from_heading` 要求完整 raw sensor pose、已知 home 外参 / base-z 单轴 center、同时刻 chassis heading，及 heading 世界中**完整启动底盘旋转**。heading 取 base +x 在该世界的水平 azimuth，只有 h0 不足以确定倾斜起步 RP；未知参考报错，无默认水平假设。先做 `E*inverse(raw_start)*raw_current` 换父数值基准，再解析 joint 分解 / 动态 lever arm，保留 full SE(3)。`from_measured_joint` 只作独立已知关节角条件下的几何控制，不表示新车已有这个输入。

`solved_conditioned` 不代表真实定位有效，`active_accept=false`、covariance=null；退化分解输出 ambiguous / pose=null。近奇异敏感性没有经过实测 gate 标定。关节角从 raw/heading 推导，不能在 covariance 传播中当作另一份独立观测；有限差分 J 仅对固定 startup / calibration 的条件模型成立。当前未接 graph 输入、未发布公共 topic / TF、未改 profile、未调图权重。

固定6个刚体链控制复用原 adapter 测试的几何 fixture，非 A/B 标定；full candidate 对正常情形数值恢复真值，倾斜 baseline 丢 RP 并偏移位置。另保留错误水平起步、连续多解、两点 heading 敏感性和未补偿时差反例。没有实车 / 地图精度验证，总体仍 Modify。
