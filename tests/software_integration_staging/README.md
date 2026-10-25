# RM2027 软件组合回归

Integration，无设备；此目录显式运行，由 COLCON_IGNORE 隔离自动包发现。
完整状态、提交集合及依赖见 `docs/validation/software_integration_staging_20261024.md`。

先构建/source 同一 staging 的 RM 接口和 consumer、固定 FAST-LIO/Livox、两个补丁的
Nav2 1.1.20 overlay（包含全部重编 BT action plugins）。按
`tests/confirmed_software_fixes/README.md` 构建并安装 fixture，再 source 它的 local_setup。

在独占 localhost ROS domain 的 Humble 容器中，从仓库根运行：

```bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=70
python3 -m pytest -q tests/software_integration_staging/test_combination_runtime.py
```

本检查点预期仍有 **3 failed / 3 passed**，不得把失败改为 xfail/skip：

- session epoch/source 换代的两个用例：新会话必须撤销旧 NavigateToPose；以 invalid status
  能取消同一动作作正对照。当前 Mission 没有消费 session 字段，故失败。
- atomic bridge covariance 用例：PSD 正对照可用，带负特征值的有限矩阵必须拒绝；当前失败。
- controller→smoother：实际修复 controller/合成 currentness 插件，取消后归零且地图恢复后
  旧请求不再计算/输出；不是立即物理停车 SLA。
- 两个 Z 入口：各重复 3 次默认关闭启动，零子进程。

完整 A/B/C 的既有用例仍按各 package CTest 和 Z README 执行；本目录只覆盖组合新增交互。
所有失败保留用于后续明确授权的兼容修复。本轮没有更改生产状态机、放宽检查或开展新功能开发。
