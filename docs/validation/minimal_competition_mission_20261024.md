# Minimal Competition Mission Layer（2026-10-24）

阶段：Integration，仅比赛导航任务语义与受控软件验收，不是比赛策略或硬件验收。
工作树：`/tmp/rm2027_competition_mission_layer_20261024`。

## 1. Base

```yaml
Base branch: feature/navigation-intent-arbitration-2027
Base SHA: 7be5dc62bbada643c9472ec93d9880c0256cd177
Depends on arbitration feature: true
Official main: d37f8d6e5ebe53533b7fc62aca8bb84392bee31b
Posture feature included: false
```

依赖审计：main 已有 Mission/BT，但没有仲裁候选已经验证的 action 终态交接、原子 pursuit
候选有效期和 final guard 消费。临时任务要直接复用这些 infrastructure，避免重新实现抢占。
因此按授权从仲裁冻结提交建独立分支；没有 merge 两个 feature，也没有复制姿态功能。

## 2. Existing Mission audit

核代码：`competition_mission_node.cpp`、两个官方 XML、`SetMissionMode.srv`、Mission 配置、
referee/pursuit 输入及仲裁合同，优先于历史能力描述。

已有而不重复开发：

- 五种 mode：hold、home、patrol、pursuit、auto，通过 `/mission/set_mode` 与启动 YAML 选择。
- home_pose、patrol_waypoints、patrol_index；成功到点前不推进被抢占的巡逻索引。
- `ShouldReturnHome`：显式 home 或新鲜有效 referee HP `<= retreat_hp_threshold`。
- pursuit：allow_pursuit + auto/pursuit mode + 合法有效候选，competition radar-only。
- 唯一 Mission NavigateToPose/Spin client、安全否决、取消等待终态、重试/backoff。
- pursuit 失效后 auto 自然恢复 patrol；home 到点保持 completed goal，不重复发送。

真实缺口：

- 没有 supply/recovery/special 临时任务入口，没有 mission ID、deadline、cancel 或通用临时 waypoint。
- 没有临时任务完成/取消/过期后的恢复语义。
- 低 HP home 原是瞬时条件，HP 恢复就可能提前退回 auto/pursuit，需要明确持续性。
- 既有 `SetMissionMode` 只有 enable/mode，不能承载临时目标与有效期；因此新增一个小服务，保留原接口。

## 3. Final Mission contract

[权威合同](../contracts/minimal_competition_mission_2027.md)：

| Mission | Persistent/Temporary | Priority | Completion | Resume behavior |
| --- | --- | --- | --- | --- |
| HOLD/disable/safety | 持续 mode 或外部否决条件 | 0 | 显式解除/证据恢复 | 不发替代运动目标；重算有效任务 |
| RETURN_HOME | Persistent/latch | 1 | Nav2 SUCCEEDED 后保持 | 显式 SetMissionMode 释放；不自动回旧 pursuit/temp |
| SUPPLY | Temporary | 2 | 配置 supply_pose 的 SUCCEEDED | 清槽，重算长期 mode；patrol 原索引 |
| RECOVERY | Temporary | 2 | 配置 recovery_pose 的 SUCCEEDED | 同上；不确认 HP/增益 |
| SPECIAL_MISSION | Temporary | 2 | 命令 map pose 的 SUCCEEDED | 同上；不确认地形/占点奖励 |
| RADAR_PURSUIT | Lease-bound overlay | 3 | 保留现有 goal/lease 语义 | stale/lost → auto patrol 或 pursuit-only hold |
| PATROL | Persistent | 4 | 到点推进/循环 | 抢占保留索引 |
| Idle | 无运动目标 | 5 | 等待任务 | hold |

三个临时语义没有内部固定优先级：最新合法的不同任务替换唯一槽位。没有 task stack。
默认 `maximum_temporary_mission_sec=300` 只是可配置命令剩余时长上限，不是补给/回血或比赛规则计时器。
实际有效期由命令 valid_until 决定，附同样剩余时间的 steady deadline，重复不续租。

接受临时任务要求已 enable、非显式 hold/home/低 HP home。暂时失去 safety authority 时可存放
合法有期限意图，恢复前不发 goal。过期清槽；完成/取消不恢复旧 pose，而是重算当前长期 mode。
`SetMissionMode` 作为明确控制切换会清临时槽并释放 home latch；低 HP 仍成立则再 latch。

## 4. Final arbitration relationship

```text
persistent mode + patrol index
  + one temporary explicit task (supply/recovery/special)
  + fresh radar pursuit candidate
  -> existing Mission BT priority
  -> one desired GoalRequest
  -> existing pending/active action reconciliation
  -> terminal-result barrier
  -> Nav2
```

`/mission/set_temporary` 只更新槽位。所有任务仍由原 Mission client 发送，无独立 SupplyNode、
RecoveryNode 或 Special Nav2 owner。home 抢占清槽；safety loss 只取消、不替代；恢复重新判定期限。
完成判据是导航到达，不读取弹量增加、回血、金币或 gain confirmation。

命令身份/去重：非零 sequence + mission_id + immutable payload。旧 sequence 拒绝；同一命令
重传仅 ACK、不更新 deadline/goal/失败计数。即使 transport sequence 增加，同一最新逻辑任务
也不能改变目标/期限或重新执行已结束任务。不同内容使用新 ID，新任务使用递增 sequence。
取消只匹配当前 ID；过时 cancel 不能撤掉后来的任务。
该服务是受信任本地控制面，不能桥接为非雷达队友 tactical bypass。

## 5. Code changes

仅列相对 base `7be5dc62` 的本轮修改：

- 新增 `SetTemporaryMission.srv`；原 SetMissionMode/pose/action 消息保持兼容。
- Mission 增加一个临时槽、supply/recovery 配置、双时钟 deadline、幂等命令、取消/完成/替换、
  home latch 与最新状态 detail；复用原 GoalRequest 身份字段和动作交接，不重写 action client。
- 两个官方 BT 各增加 home 后的一段临时任务条件/选择节点。
- safe YAML 不填实地 supply/recovery 坐标；launch 支持省略空可选数组。
- 新增 S1–S14 受控运行测试并注册；原仲裁回归补显式释放 home，原 Spin 树断言同步新增节点。
- 同步 owning README、topic/arbitration/mission/runtime 合同及能力状态。

未修改姿态、定位、TF、planner/controller、serial/MCU、地形/地图或自动策略。

## 6. Validation

沿用固定 Humble 镜像 `rm2027_navigation:humble`，源只读、无网络、私有 IPC、非 root、
无设备映射。只构建 interfaces、Mission、pursuit、bringup 四个直接相关包，不做 workspace rebuild。
结果位于 `/tmp/rm2027_mission_layer_validation`，不将 build/log/重复 evidence 入 Git。

真实生产导航组合运行 Mission + pursuit；Nav2、radar、referee、localization、chassis/guard
由隔离 fixture 提供。旧 Nav2 取消终态延迟 0.15 s，用于发现 cancel ACK 后提前重发。
先完成 patrol 点 0，使当前 index=1，再验证临时任务恢复到点 1。

| Scenario | Result | Evidence |
| --- | --- | --- |
| S1 idle → patrol | PASS | 到点 0 后当前 index=1 |
| S2 patrol → supply → complete → patrol | PASS | 到配置 supply 点，回原巡逻点 |
| S3 patrol → recovery → complete → patrol | PASS | 到配置 recovery 点，回原巡逻点 |
| S4 patrol → special → complete → patrol | PASS | 到命令 pose，回原巡逻点 |
| S5 radar pursuit → explicit mission | PASS | 三种临时任务均确定性抢占 pursuit |
| S6 supply → home | PASS | 低 HP 抢占，到家后保持，显式释放再巡逻 |
| S7 recovery → home | PASS | 显式 home 抢占，旧任务不回来 |
| S8 special → home | PASS | 低 HP 抢占，到家保持 |
| S9 cancel temporary → long-term | PASS | 取消后回仍有效的 patrol |
| S10 temporary expires | PASS | 过期后回 patrol；重发旧命令不重启 |
| S11 authority denied | PASS | 活动任务取消，拒绝期间零替代目标 |
| S12 authority restored | PASS | 运行当前有效 recovery，不恢复已过期 supply |
| S13 missing supply/recovery pose | PASS | 请求拒绝，无原点/home 等替代坐标；非法 frame/deadline 同样拒绝 |
| S14 repeated command at 40 Hz | PASS | 目标/取消数不变；同 ID 改目标拒绝 |

补充覆盖同级不同任务替换、完成命令重传、到 home 后拒绝新临时任务、单 owner 交接。
新场景记录 **39 个 goal、29 次取消，最大同时活动目标数 1**；每个后续请求均晚于旧 action 终态。
继承的仲裁 S1–S10、pending 接受延迟、planner 退出、源 lease、generic research、三点巡逻/Spin、
low-HP home 与 pursuit 数学回归全部通过。总计 **12 pytest cases passed，0 failures/errors/skipped**。
新增运行 case 的正式 CTest 注册入口也已通过（1/1）；不把重复运行计为新测试数量。
注册用例收尾的 SIGINT 曾打断刚启动 planner 的日志初始化，出现在请求清理之后，未影响
功能断言；本轮没有扩展修改既有 pursuit 退出逻辑。测试容器退出后无残留运行节点。

复现（在无设备/无网络容器内，将本提交挂到 `/workspace:ro`，结果目录挂到 `/results`）：

```bash
source /opt/ros/humble/setup.bash
cd /workspace
colcon --log-base /results/log build \
  --base-paths src/rm_competition_interfaces src/rm_competition_mission \
  src/rm_pursuit src/rm_navigation_bringup \
  --build-base /results/build --install-base /results/install \
  --executor sequential --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
source /results/install/setup.bash
export ROS_LOG_DIR=/results/ros_log
unset RM_COMPETITION_MISSION_SOURCE_DIR
ctest --test-dir /results/build/rm_competition_mission --verbose
python3 -m pytest -q src/rm_pursuit/test/test_pursuit_math.py -o cache_dir=/results/pytest_cache
```

未运行真实 Nav2 planner/controller、硬件、Gazebo 或比赛策略。guard 是受控状态输入，不能把
本报告当作实际 Mission-to-V2 正准入会话、MCU 取消/停车或场地坐标验收。

## 7. Runtime behavior delta

- Patrol：点位/点序与成功推进不改，临时任务结束后恢复原索引。
- Pursuit：本轮不改目标计算、来源或更新阈值；临时任务现在优先，结束后只允许新的有效观测进入。
- Return-home：首次选中即 latch，途中 HP 恢复也继续回家，到家保持；显式 mode 才释放。
  这是本轮有意变化，防止自动复活旧追踪或临时任务。
- Nav2 owner：仍只有一个任务级 owner，复用终态交接；没有新的 motion publisher。
- Authority：安全/显式 Hold 仍最高，命令不能 enable 或创建最终授权。恢复只重算当前有效任务。
- SetMissionMode：原参数/模式名不改；明确切 mode 时清临时槽和 home latch，避免隐藏恢复。
- 缺配置/非法请求：不替换现有合法任务，不填造目标。

## 8. Deferred strategy

本轮故意不做：

- automatic supply trigger / ammo threshold；
- automatic recovery trigger / 复活、虚弱状态机；
- economy strategy / coin management；
- posture strategy 或姿态约束集成；
- gain confirmation / 地形奖励、狗洞或跨越计时；
- semantic map / geofence / rule engine。

后续配置真实地图点、裁判/雷达与 final host owner-session 接入、实际 Nav2 与物理停止验收仍待安排。
自定义 BT 需加入对应临时节点；多个命令 issuer 需先约定共享 sequence owner，本轮没有 scheduler。

## 9. Decision

```yaml
Decision: GO
Scope: minimal competition Mission / BT software contract only
Branch: feature/competition-mission-layer-2027
Commit: 本报告所在提交（git rev-parse HEAD）
Main changed: false
Hardware used: false
Merged: false
Pushed: false
```

不是比赛策略完成、Hardware Go 或 Deployment Go。两个原候选 feature 和定位冻结状态保持原样。

## 10. Next Gate

```text
Minimal 2027 competition mission infrastructure complete.
Tactical trigger policies and physical validation remain deferred.
```

完成报告后停止，不继续添加规则功能。
