# RM2027 软件组合回归

Integration，无设备；此目录显式运行，由 COLCON_IGNORE 隔离自动包发现。
完整状态、提交集合及依赖见 `docs/validation/software_integration_staging_20261024.md`。

先构建/source 同一 staging 的 RM 接口和 consumer、固定 FAST-LIO/Livox、两个补丁的
Nav2 1.1.20 overlay（包含全部重编 BT action plugins）。按
`tests/confirmed_software_fixes/README.md` 构建并安装 fixture，再 source 它的 local_setup。

在独占 localhost ROS domain 的 Humble 容器中，从仓库根运行：

```bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=70
python3 -m pytest -q tests/software_integration_staging
```

当前两个组合修复后为 **16 passed**，没有 xfail/skip。`965b97e` 的初轮证据
`3 failed / 3 passed` 保留在 integration evidence JSON 的原字段，最新结果见 repair_20261025。

- `test_combination_runtime.py`：原六项检查现在全通过；两个 session 边界和非法 covariance
  已从失败转为通过。controller→smoother 取消/归零及两个默认关闭入口保持通过。
- `test_session_handoff_runtime.py`：事件控制 pending acceptance 和 terminal result；覆盖
  active epoch/source、LOST/recovery 及间隙缓存、100 次追加快速 epoch、迟到成功、退役身份、
  Spin 换代。cancel ACK 后旧终态未到时不得重发；旧成功不得推进 patrol。
  事件 deadline 只判定测试失败，不参与生产取消/解锁；不以 sleep 排序制造通过。
- `test_covariance_parity_runtime.py`：atomic bridge、canonical→bridge、direct legacy pose、
  Z legacy embedded status 四条入口；每条 7 类合法兼容输入和 7 类非法输入，覆盖
  finite/symmetry/PSD、近 PSD/tolerance、NaN/Inf、rank-deficient、missing、尺度差异。
  reject 立即 invalid/清 pose；即使新 emission heartbeat 也不能用同 source 恢复，严格新观测才恢复。

完整 A/B/C 既有用例按各 package CTest 和 Z README 执行；本目录只覆盖组合交互。
没有改变比赛优先级、取消终态屏障、默认 feature gate 或任何物理运动权限。
