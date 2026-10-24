# Posture-Aware Motion Constraints（2026-10-24）

阶段：Integration；正式基线 `main@d37f8d6e5ebe53533b7fc62aca8bb84392bee31b`。
独立工作树 `/tmp/rm2027_posture_motion_constraints_20261024`。
不依赖上一轮仲裁提交 `7be5dc62`，不修改 main、定位研究、协议或硬件配置。

## 1. Decision

```yaml
Decision: GO
Scope: six-state upper motion profile software infrastructure
Branch: feature/posture-aware-motion-constraints-2027
Commit: 本报告所在提交（git rev-parse HEAD）
Dependency on arbitration feature: false
Main changed: false
Hardware used: false
Physical profiles calibrated: false
Merged: false
Pushed: false
```

仅限软件基础设施成立；不是 Hardware Go，六组真实 motion profile 仍未标定。

## 2. Posture source audit

| State | Source can represent? | Effective/requested? | Freshness available? |
| --- | --- | --- | --- |
| MOBILE | 是：MOVE=1 | `posture_actual`，与 target 独立 | 主机接收 stamp、valid/online、transport/consumer lease |
| ATTACK | 是：ATTACK=0 | actual，非 ACK/requested | 同上 |
| DEFENSE | 是：DEFENSE=2 | actual，非 ACK/requested | 同上 |
| ENHANCED_MOBILE | 是：ENHANCED_MOVE=5 | 独立 actual enum | 同上 |
| ENHANCED_ATTACK | 是：ENHANCED_ATTACK=3 | 独立 actual enum | 同上 |
| ENHANCED_DEFENSE | 是：ENHANCED_DEFENSE=4 | 独立 actual enum | 同上 |

审计基于 `RobotPosture.msg`、`PostureState.msg`、V2 codec 与
`CompetitionV2TransportNode::handlePostureState()`，没有根据历史文档推测枚举。
`PostureRequest` 只表示 requested；`posture_target` 和 ACK 是期望/确认，`completed` 还依赖
本上位机当前请求。profile 消费 actual，要求 valid/online、无 fault/transitioning，
不要求 ACK/completed。上游明确上报 enhanced → ordinary actual 时即可恢复普通槽位，
即使 target 还未改变也适用。导航不计算 5 s、3 min、15 s 或自主退出强化。

`RefereeState` 没有六态姿态字段，Mission 也不是姿态来源。wire 有 frame sequence，当前
PostureState ROS 消息没有独立姿态 sequence、MCU sample time 或 boot，header 是 host RX。
本轮没有伪称已验证机械测量 freshness/源采样防重放。真实固件持续报告 measured actual
仍属硬件接口验收。协议已能表达六态，**不需要修改 MCU wire protocol**。

## 3. Existing motion constraint chain

所选 baseline 为现有 competition 默认 `nav2_old_car_2026_left_stvl.yaml`。
数值是既有保守软件配置，不是本轮测出的新车性能。

| Layer | Constraints | Runtime mutable | Final role |
| --- | --- | --- | --- |
| Mission/BT | 不提供数值限幅 | 本轮不改 | 任务/安全条件，与 profile 正交 |
| MPPI Omni | vx [-0.30,0.50]，vy ±0.50，wz ±1.20；无本版本独立 accel/decel 参数 | 四速度参数可动态修改 | 预测/控制速度边界 |
| velocity_smoother | max [0.50,0.50,1.20]；min [-0.30,-0.50,-1.20]；accel [0.80,0.80,2.00]；decel [-0.80,-0.80,-2.00] | 四组数组可动态修改 | upper command envelope |
| Nav2 behavior | 既有 Spin max 1.20、rotational_acc_lim 2.00 | 本轮不动态修改 | 输出仍经过 smoother |
| managed ingress | finite/source GID 过滤 | 不涉及本轮 | 非授权 owner |
| V2 serial adapter | 默认 ±[0.80,0.50,6.00]，无 accel clamp | 启动读取 | 原有限幅及最终 host 编码授权 |
| legacy/HPM adapter | 默认 ±[0.15,0.15,0.30]，可启动覆盖 | 启动读取 | 互斥旧车路径，保持原状 |
| chassis stub | 默认 ±[1.0,1.0,2.0] | 启动读取 | 仿真/stub |
| MCU | 实际 posture 下最终 vx/vy/wz hard clamp；物理值待验 | 下位 owner | 最终物理命令限制 |

TinyMPC 不在当前 `src/` 运行链，本轮只审计其未接入，未恢复冻结研究。
[完整审计合同](../contracts/posture_motion_constraints_2027.md) 链接对应 Nav2 1.1.20 官方实现。
仅更新 smoother 会保留 MPPI 的旧速度预测范围，因此选择最小两层同步；未动态改五层参数。

## 4. Final architecture

```text
effective posture (actual)
  -> six-state selector / lease / debounce
  -> six independent profiles + fallback
  -> smoother four arrays + MPPI four velocity parameters
  -> existing cmd_vel chain / host authorization
  -> MCU hard clamp
```

新节点仅有 PostureState 订阅、参数服务客户端和 DiagnosticArray 发布者。
启动读回 active 后端参数，baseline 不匹配则零写入。选中 profile 与已确认 applied profile
分开报告；同值跳过参数写入，但不合并姿态、槽位、diagnostic 或 generation。
顺序为 smoother 后 MPPI；两节点不是分布式原子事务。RPC 未确认/拒绝会报告 ERROR、
applied=unconfirmed，停止继续写入，不宣称 fallback 已应用；修复后需按已知 baseline 重启。
这不是新的运动授权或安全控制器。速度 cap 下调经过已有 decel，允许短暂平滑收敛。

## 5. Production profiles

**Physical values = HARDWARE_TBD；六组真实参数继续 HARDWARE_BLOCKED。**

`src/rm_posture_motion/config/profiles.yaml` 中 mobile、attack、defense、enhanced_mobile、
enhanced_attack、enhanced_defense 和 fallback 全部独立列出，分别继承所选既有 baseline。
未加入功率倍率、速度倍率、加速度倍率或伪造标定值。fallback 不超过 baseline。
production launch 默认关闭，固定真实 V2 topic 与 production manifest，不包含 mock。
本轮没有自动把 selector 接入旧车硬件启动链。

## 6. Test-only profiles

[测试配置](../../src/rm_posture_motion/test/fixtures/profiles_test_only.yaml)：
**TEST ONLY / NOT PHYSICAL / NOT COMPETITION CALIBRATION**，不安装进 production config。

| Test slot | max_velocity [vx,vy,wz] | min_velocity | max_accel | max_decel |
| --- | --- | --- | --- | --- |
| mobile | [0.100,0.100,0.200] | 对应负值 | [0.6,0.6,1.0] | [-0.6,-0.6,-1.0] |
| attack | [0.125,0.125,0.250] | 对应负值 | 同上 | 同上 |
| defense | [0.150,0.150,0.300] | 对应负值 | 同上 | 同上 |
| enhanced_mobile | [0.175,0.175,0.350] | 对应负值 | 同上 | 同上 |
| enhanced_attack | [0.200,0.200,0.400] | 对应负值 | 同上 | 同上 |
| enhanced_defense | [0.225,0.225,0.450] | 对应负值 | 同上 | 同上 |
| fallback | [0.050,0.050,0.100] | 对应负值 | [0.4,0.4,0.8] | [-0.4,-0.4,-0.8] |

这些数字只用于区分软件状态，不来自物理或功率关系。六态在 native/逻辑表示、profile
选择、测试、配置和诊断五个层面均独立；production 数值相同时仍保留六个不同 key/generation。

## 7. Code changes

- 新增 `rm_posture_motion`：六态映射、配置校验、freshness/debounce、小型参数同步器与诊断。
- 新增默认关闭的 production launch、baseline 继承 manifest 与独立 test-only fixture。
- 新增纯配置/enum/lease 测试和小型运行集成测试；未新增 ROS message 或改变已有姿态 enum。
- 同步 package README、topic/runtime/V2 消费合同、架构与能力状态。没有改 Mission、BT、
  controller/smoother 实现、serial codec/协议、authority、TF 或 localization。

## 8. Validation

环境：沿用固定 `rm2027_navigation:humble` 镜像，源代码只读、无网络、私有 IPC、非 root、
无设备映射；没有 Gazebo。结果和构建位于 `/tmp/rm2027_posture_motion_validation`，不入 Git。
只构建新包及 `rm_competition_interfaces`、`rm_nav_config`、测试用 `rm_serial_driver`。

运行测试使用真实 Humble smoother、真实 selector、真实 V2 transport/codec dry-run；
MPPI 参数服务与持续 active 的 NavigateToPose server 是 fixture。production 段由合成
V2 字节经真实 decoder 发布 actual 六态，test topic 不能影响 production consumer。
四组 smoother 参数逐次读回核对，实际输出也到达六套不同测试上限。

最终结果：4/4 局部包构建通过；**16 tests passed，0 errors/failures/skipped**（15 个
配置/enum/lease case + 1 个受控运行 case，运行 case 内覆盖全部 S1–S12）。40 Hz 重复态
产生 0 次额外参数写入；同一 Nav2 goal 全程 active、0 次 cancel；在姿态切换期间观察的
**683 帧**真实 V2 ChassisCommand 全为零速度/disabled。production 六态经真实 V2 codec
解码后选择各自 profile，数值继承 baseline 时写入计数保持 **0**。baseline mismatch 负例
按预期产生 ERROR、applied=unconfirmed，写入次数 **0**；这是故障注入预期结果。

| 场景 | Result | 核查 |
| --- | --- | --- |
| S1 startup/no posture → fallback | PASS | baseline 核对后应用 fallback |
| S2 fallback → mobile | PASS | MOBILE 实际态 |
| S3 mobile → enhanced_mobile → mobile | PASS | target/ACK 故意不匹配，actual 正常回切 |
| S4 mobile → attack | PASS | 独立 attack slot |
| S5 attack → enhanced_attack → attack | PASS | 两态独立 |
| S6 attack → defense | PASS | 独立 defense slot |
| S7 defense → enhanced_defense → defense | PASS | 两态独立 |
| S8 valid → stale/source lost → fallback | PASS | 不保留强化参数，输出按 decel 收敛 |
| S9 40 Hz identical posture | PASS | generation 与参数写入计数不变 |
| S10 active goal + posture changes | PASS | 同一 goal active、零次 cancel |
| S11 authority denied + posture changes | PASS | 真实 V2 最终 ChassisCommand 全部零/disabled |
| S12 six distinct test profiles | PASS | 六个参数 payload + smoother 输出 + diagnostics 独立 |

还覆盖：六态 native enum/parser、production 全量 baseline 继承、TEST config 在 competition
模式下拒绝、非有限值预校验、debounce、同时间戳重放、unknown/invalid/offline/transition/fault、
production 仅实际 V2 来源、同值六态零参数写入、runtime baseline mismatch 时拒绝写入。

复现（挂载本提交为 `/workspace:ro`、结果目录为 `/results`，使用上述隔离容器选项）：

```bash
source /opt/ros/humble/setup.bash
cd /workspace
colcon --log-base /results/log build \
  --base-paths src/rm_competition_interfaces src/rm_nav_config \
  src/rm_serial_driver src/rm_posture_motion \
  --build-base /results/build --install-base /results/install \
  --executor sequential --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
source /results/install/setup.bash
export ROS_LOG_DIR=/results/ros_log
python3 -m pytest -s -q src/rm_posture_motion/test \
  --junitxml=/results/results.xml -o cache_dir=/results/pytest_cache
```

实现期发现并修复了 ROS Node 只读属性重名导致的启动错误；补充退出处理时还修正了
Humble 不导出 RCLError 名称的兼容问题。未通过放宽 freshness、枚举或 authority 取得通过。
没有验证真实 MPPI 轨迹质量或物理性能，不能把假的参数服务/goal server 当成实车闭环。

## 9. Runtime behavior delta

- 原 Mission、BT 和优先级：未改；本轮从 main 独立开发，没有依赖仲裁 feature。
- Nav2 goal：姿态切换不取消、不改变目标；MPPI 参数更新重置内部优化序列，不是 goal 重发。
- motion authority：仍高于 posture；selector 不生成 owner/session 或授权，只能配置 upper 限制。
- MCU：仍是最终 hard clamp owner；本轮既未改 MCU 合同，也未证明其固件物理执行。
- timestamp/TF/localization：不改消息时间戳、不发 TF、不改定位；只消费既有 posture host-RX stamp。
- 默认关闭；启用 production 也只保留 baseline 数值；物理参数待硬件。

## 10. Remaining Hardware Blockers

- 真实四全向轮运动学矩阵；
- 轮径、轮位、滚轮方向；
- 电机、减速比和传动；
- 真实底盘质量、质心、摩擦与载荷；
- MCU 速度闭环、最终 clamp、模式/ESTOP/watchdog 实现及控制合同；
- 功率限制实现；
- supercap 对瞬态能力的影响；
- 六姿态真实 vx/vy/wz；
- 六姿态真实 accel/decel；
- power-to-motion relationship 未知，本轮未研究或拟合；
- 实际姿态 measured actual 的可信上报、周期、增强结束/普通恢复、故障与采样时序。

## 11. Next Gate

```text
Posture-aware motion constraint infrastructure complete.
Physical six-profile calibration waits for real hardware.
```

Task 1 / Z-Localization 保持 FROZEN_WAIT_HARDWARE_DATA；硬件状态仍 HARDWARE_BLOCKED。
本轮完成报告后停止，不自动进入其他功能或物理标定。
