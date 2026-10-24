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
