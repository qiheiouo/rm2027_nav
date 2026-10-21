# Full-base heading / gimbal pose adapter

独立 research-only 实验。基线 `origin/main-new-car@bd6cf689`；设计结论和实际合同审计见
[研究报告](../../docs/research/full_base_heading_adapter_20261007.md)，精选证据见
[summary.json](evidence/summary.json)。不含 ROS 节点、TF/topic 发布、factor graph 或 recovery 状态机。
`COLCON_IGNORE` 隔离正式构建。所有 fixture 为合成输入，不能作为 A/B 新车标定。

本 worktree 为 `/home/qihei/rm2027_navigation/build/full_base_heading_adapter`，
branch 为 `experiment/full-base-heading-adapter`。只在此目录修改和构建；其他任务目录仅用于
读取固定 Git blob，不 checkout / reset / clean / initialize submodule。本地两个 submodule
保持未初始化；FAST-LIO pin 为 `e7864a63a7e2a1ad62ec8fd75fcb9149db08e321`。

## 复现

已有依赖：Python NumPy 1.26.4、SciPy 1.11.4、g++、ROS Jazzy headers、系统 gtest。
无需启动 ROS；不占用 ROS Domain。编译的数学探针和日志只写入本 worktree 的
`build/full_base_heading_adapter/`，原 C++ 与测试保持不变。

在本 worktree 根目录：

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1 \
  python3 experiments/full_base_heading_adapter/run_validation.py \
  --fast-lio-source /home/qihei/rm2027_navigation/src/fast_lio_multi
```

`--fast-lio-source` 是只读源码仓库，必须包含固定 commit。脚本用 `git show <pin>:<path>`
读取 bundle 源码，不依赖该 checkout 的可变工作文件。它记录源码仓库的 branch、commit、dirty
情况，但不改动它。别的机器可传同 pin 的仓库路径。默认结果写入忽略的 build 目录；
本轮长期证据以 `--output experiments/full_base_heading_adapter/evidence/summary.json` 生成。

只运行候选测试（其中 planar parity 会编译实际 canonical 探针）：

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1 \
  python3 -m unittest discover -s experiments/full_base_heading_adapter -p 'test_*.py' -v
```

## 接口

`adapter.py` 的 `solve(raw, heading, calibration, startup, joint=None, policy=Policy())`
是无状态函数。`fixtures.py` 展示显式输入的构造；不能将其数值用于实车。

- `PoseSample`：raw FAST-LIO full **IMU-origin** SE(3)、原始时间、clock、raw epoch、父数值基准和 sensor frame。
- `HeadingSample`：同一重力参考中 base +x 的水平 azimuth（rad / CCW）、原始时间、clock、heading epoch、有效性。
- `Calibration`：完整固定 `T_base_joint` 和 home 时 `T_joint_sensor_home`、机械 home angle、单转动轴模型、frame 与 revision。
- `Startup`：时间匹配的 raw0 / heading0、完整 `R_reference_base0`、归一化 gravity-down in B0、时间匹配且确认的 home 和 calibration revision。
- `JointSample` 可选：**独立测量**并同步的机械角，符号绕 joint +z；不能传本 adapter 自己推导的角当作独立证据。
- `Policy`：研究演示的时间 / 条件数边界，非实测新车 gate。默认最大 skew 为 1 ms；上游应完成同步。

`Result` 包含状态、原因、输出基准 / source time / calibration ID、诊断和可选 full pose。
只有 `SOLVED` 的顶层 pose / angle 非空；`AMBIGUOUS`、`ILL_CONDITIONED`、`INVALID_TIME`、
`INVALID_CALIBRATION`、`INVALID_INPUT` 不输出可用顶层 pose。diagnostic candidates 仅用于研究多解。
`SOLVED` 只表示给定条件下数学成立；所有结果 `active_accept=False`、`covariance=None`。

`T_odom_base` 的 odom 轴与 heading/gravity reference 相同，原点为 startup base 原点。
`T_base0_base` 保留传统 B0 数值基准，便于和 planar baseline 比较。
倾斜启动时前者的启动姿态为完整 `R_reference_base0`，后者为 identity。
`startup_attitude_from_gravity` 可以显式构造该完整姿态；它不从动态加速度估计 gravity，
也不证明外部输入正确，`solve` 不会自动调用它或补默认水平姿态。

## 当前结论

完整单轴链 + 已知 startup 参考 + 正确 heading 语义 + 同步时，有条件成立的 full-base
adapter 存在。未知关节角时仍有连续多解、倾斜轴双根和近奇异姿态；独立关节角可移除
heading 分解的多解，但真实 heading/reference 仍须验证。

本轮 31 项候选测试（含 100 个固定 seed 的完整姿态控制）和 15 项未改动 canonical
测试通过。实车 episodes 为 0。硬件仍 No-Go，先在本分支验证，再决定是否将最小实现迁入
`main-new-car`；正式 YAML、owner、launch 和 graph 均未变。
