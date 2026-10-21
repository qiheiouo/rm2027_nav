# FAST-LIO 单时刻 covariance publication contract

本实验独立于东北大学定位研究、Task 1 / Task 2、动态预测和 MPC。
研究结论、数学定义及恢复步骤见
[研究报告](../../docs/research/fastlio_covariance_contract_20261007.md)。

从本任务 worktree 根目录运行：

```bash
python3 experiments/fastlio_covariance_contract/run_probe.py --baseline-only
python3 experiments/fastlio_covariance_contract/run_probe.py
```

需要系统已有的 g++、Eigen、Boost、OpenMP 和 ROS Jazzy C++ headers；
`--ros-prefix` 可指定 ROS 安装目录。无需启动 ROS、DDS、仿真或传感器。
不安装依赖，不占用 ROS Domain。实验有 `COLCON_IGNORE`。

runner 从旧 commit `e7864a63a7e2a1ad62ec8fd75fcb9149db08e321`
读取三个真实 `publish_odometry` / `set_posestamp` 函数体，
从当前子模块读取候选函数体，并编译真实 MTK state、ESKF、ROS message 和 tf2 headers。
bundle 的 zero-start 代码块同样直接提取；不以代数 mirror 代替其执行。
publisher / TF broadcaster 是保存消息副本的 test doubles，日志和时间转换也是 test doubles。
没有运行 ESKF 点云更新求解或完整节点，所以结论只覆盖源码及单测层。

生成的函数体、编译日志和二进制位于本 worktree 内的
`build/fastlio_covariance_contract_probe/`；JSON 证据位于 `evidence/`。
完整 probe 使用 C++17 编译 ROS Jazzy 对照 headers；生产 helper 另做 C++14 编译检查。

证据：

- [旧源码反例](evidence/baseline.json)：状态布局、首帧、连续帧、索引交换，以及排除延迟和交换后的 zero-start 基准错误。
- [候选测试](evidence/contract.json)：14 组测试、有限差分误差及 C++14 检查。
- [固定身份](evidence/revisions.json)：测试源码 SHA256 与已提交候选的一致性。
- [子模块离线 bundle](evidence/fast_lio_candidate.bundle)：未 push 的实验 commit；需要旧 commit 作为 prerequisite。

重新运行会更新 JSON 证据；生成物留在本任务目录。提交前检查本分支状态。
