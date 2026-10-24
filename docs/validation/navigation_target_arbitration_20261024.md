# Radar / Mission Target Arbitration and Preemption（2026-10-24）

阶段：Integration。基线：正式 `main@d37f8d6e5ebe53533b7fc62aca8bb84392bee31b`。
工作树：`/tmp/rm2027_navigation_intent_arbitration_20261024`。
本轮不修改定位代码，不解冻 Task 1 / Z-Localization，不运行硬件或 Gazebo，不做全工作区构建。

## 1. Existing target sources

代码审计确认：基线有 **3 类实际导航坐标来源：home、patrol、pursuit**。
`hold/home/patrol/pursuit/auto` 是 5 种模式，不是 5 个独立坐标 producer。
referee、chassis 和 operator enable 是条件/权限，不产生坐标。下表的优先级为本轮最终合同，
“Current entry”同时指出基线入口及必要的新边界。

| Source | Current entry | Priority | Can preempt | Can be preempted | Competition enabled |
| --- | --- | --- | --- | --- | --- |
| Safety / disable / explicit hold | `SafetyReady`、`SetMissionMode`；新增 V2 guard 只读否决 | 0 | 所有导航/Spin | 恢复后重算 | 是，2027 强制 guard |
| Low-health home | `/referee/state` + `_valid`，`ShouldReturnHome`，YAML `home_pose` | 1 | pursuit、patrol | Safety/hold | 是，须配置 home |
| Explicit home | `/mission/set_mode(mode=home)` 或 YAML default_mode；复用同一 home | 1 | pursuit、patrol | Safety/hold | 仅受信任配置/调试，不是队友 tactical |
| Radar pursuit | 基线没有雷达专用消息入口；本轮 `/radar/target_track` (`TargetTrack`) → planner → 原子候选 | 2 | auto 模式下 patrol | home、Safety、失效/替换 | 仅允许此类外部 tactical；默认仍关闭 |
| Generic research pursuit | 基线 `/perception/target_track` → planner → `/mission/pursuit_goal` + Bool；本轮用原子候选 | 2，限研究 opt-in | 研究 auto 下 patrol | home、Safety、失效/替换 | 否；研究接口保留 |
| Patrol | YAML `patrol_waypoints` + `patrol_index_`，`auto|patrol` 模式 | 3 | idle | pursuit(auto)、home、Safety | 是，须配置点位 |
| Idle / hold fallback | 默认树 `SelectHold` | 4，无运动目标 | 无 | 任一有效且允许的任务 | 是 |
| Raw operator coordinates | serial/HPM/V2 → `/operator/navigation_target_raw` (`OperatorNavigationTarget`) | 无 | 无 consumer，不能发 goal | 不适用 | 不进入仲裁 |
| Arbitrary teammate / multi-robot | 当前 `src/` 无专用 tactical consumer；generic research topic 原先不区分 publisher 来源 | 无比赛权限 | 比赛中不能 | 不适用 | 禁用 |

基线最终调用者：

- `competition_mission_node::sendGoal()` 调用 `NavigateToPose`，`sendSpin()` 调用 `Spin`。
- pursuit/referee/serial 都不调用 Nav2。`FollowPath` 是 Nav2 自身 BT/controller 的下游 action，
  正式任务层没有第二个直接客户端。
- `dog_hole_manager` 及 simulation smoke 工具有自己的 NavigateToPose 客户端，但属于互斥仿真入口，
  当前 competition 组合未同时启动它们；没有发现“Mission 与 pursuit 两个独立 Nav2 owner”。
- 基线默认 BT 已是 home → pursuit → patrol。`auto` 中 pursuit 覆盖 patrol；显式 `patrol`
  不允许 pursuit。low-HP 条件原本也已排在 pursuit 前。
- 原有 disable、安全失效和 hold fallback 会调用 cancel；chassis 模式/ESTOP 经 gate 影响 Mission，
  V2 最终编码 guard 独立禁行，但基线 Mission 不消费它。

## 2. Existing conflicts

只列实际代码缺口：

1. Mission 消费锁存 pursuit Bool 和无身份的 Pose；planner 退出后最后一个 true/pose 可无限生效。
   原 planner 虽校验 TargetTrack 0.3 s freshness，但输出 stamp 改为 now，身份和源期限未传入 Mission。
2. `sendGoal()` 异步 cancel 后立刻发新目标；旧 action 尚未终止时存在重叠窗口。Spin → home 同样如此。
   旧 pending handle 迟到虽会补 cancel，也无法阻止此前已发出的替代请求。
3. 原来的全局 `minimum_goal_update_sec` 同时延迟 home 等分支抢占；目标身份变化无法被识别。
4. `SelectHome` 缺坐标返回 FAILURE，使高优先级 return 条件成立后仍可能落到 pursuit/patrol。
5. `hold` 原只是模式字符串，低 HP 仍能选 home；显式停止语义不明确。
6. Mission 对 referee/chassis 缺独立接收 lease，上游 gate 退出可能留下缓存健康状态；未消费 V2 guard。
7. 原 competition pursuit 组合沿用 generic perception topic，不能表达 radar-only 来源政策。
   未发现已经接通的非雷达队友专用 tactical→Nav2 接口；raw operator 坐标也尚未有 consumer。

## 3. Final arbitration contract

[权威合同](../contracts/navigation_target_arbitration_2027.md)：

```text
Safety / Mission disable / explicit hold
  > low-health or explicit home
  > valid radar pursuit (auto|pursuit)
  > patrol (auto|patrol)
  > idle hold
```

沿用现有树顺序，不新增 SPECIAL_MISSION/task framework。显式 home 已是已有 override。
缺少 home 时保持停止；不添加新的 return/recovery 策略。

```text
ACTIVE/PENDING old goal
  -> cancel requested (once; pending handle may arrive later)
  -> wait terminal result
  -> re-evaluate latest desired goal and safety
  -> send at most one replacement, or remain hold
```

patrol 被抢占时不重置索引。安全恢复不重用 hold/撤权前的源观测。同 ID 小幅更新不重发，
大幅更新受 0.5 s 最短间隔限制；新 ID 确定性替换，即使坐标相同。原始目标过期/丢失回到
当前仍允许的 mission，auto 恢复被打断的巡逻点。取消等待期合并输入，不积压新 action。

## 4. Code changes

- `rm_competition_interfaces`：新增极小 `PursuitCandidate`，携带原始时间、deadline、ID、来源、pose 和 valid。
- `rm_pursuit`：复用原验证、TF、standoff 和 0.3 s freshness；新增 radar-only 输入选择、原子候选与稳态 lease。
  原 pose/Bool 输出保留供诊断，未开发 radar wire decoder 或新 tracker。
- `rm_competition_mission`：单 owner 终态交接、pending 取消、同 ID 限频、缺 home/显式 hold 停止、独立安全 lease、
  V2 guard 否决及恢复后的目标重验证。BT XML、定位、controller、serial codec 均未改。
- `rm_navigation_bringup`：新增默认关闭的 2027 纯导航组合，强制 radar-only + guard；旧 competition 固定 radar-only；
  显式无硬件研究组合保留 generic input。
- 增加受控运行测试；原 Spin fixture 补源时间戳和取消终态延迟/互斥检查；同步 owning README、topic/runtime/mission 文档。

## 5. Validation

环境：现有 `rm2027_navigation:humble` 镜像
`sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3`。
容器源码只读、`--network none --ipc private --cap-drop ALL --security-opt no-new-privileges`，
非 root；无设备映射。结果位于 `/tmp/rm2027_arbitration_validation`，不将构建/重复日志加入 Git。

测试运行真实 Mission、真实 pursuit 和真实 2027 launch 策略，Nav2/safety/radar 由 fixture 提供。
默认 20 Hz Mission tick、40 Hz radar 输入、旧 goal 取消终态延迟 0.15 s、pending 接受延迟 0.35 s；
另对原 Spin fixture 加 0.2 s 取消终态延迟。YAML 刻意试图关闭 radar/guard，组合策略仍覆盖为 true。

| 场景 | 结果 | 判据 |
| --- | --- | --- |
| S1 idle → patrol | PASS | 先完成点 0，再活动于点 1 |
| S2 patrol → radar pursuit | PASS | 旧 patrol 终态早于 pursuit 请求 |
| S3 stale/lost → resume patrol | PASS | stale 与显式 lost 均回点 1 |
| S4 pursuit → low-health home | PASS | 取消 pursuit 后到 home |
| S5 home 收到 radar | PASS | 继续 home，无替代 goal |
| S6 authority denied | PASS | 取消活动目标，拒绝替代 |
| S7 authority restored | PASS | 恢复当前 patrol，不复活旧 tactical |
| S8 continuous target updates | PASS | 40 Hz 小幅抖动 0 cancel；运动更新受限频 |
| S9 different ID same pose | PASS | 确定性替换一次 |
| S10 non-radar input | PASS | generic/research/legacy pose+Bool 均不触发比赛追踪 |

补充检查：重复源采样不能续租；planner 被直接终止后缓存 true 无效；pending handle 迟到正确取消；
ESTOP、guard/chassis 失联均停止；home 缺坐标时不落入 pursuit；显式研究模式仍可 generic pursuit → patrol；
成功后的同 ID 更新仍限频；原三点巡逻/Spin/low-HP home 和 pursuit 数学回归。
最终结果：4/4 定向包构建通过；Mission 注册的 3 个 CTest 任务全部通过，包含 8 个 pytest
case（仲裁运行 1、原配置 6、Spin 运行 1）；另有 pursuit 数学 3 个 case，合计 11 个。
S1–S10 全部 PASS。最终 trace 的同时 active 数最大为 **1**，共 29 个 goal、26 次取消；
每个后续请求均晚于旧 action 终态。40 Hz 小抖动产生 **0** 次取消，移动更新在 2.25 s 观察窗
只发送 **3** 次。成功后的同 ID 大幅更新间隔不少于受控断言的 0.45 s（配置 0.5 s，考虑采样边界）。

可复现的容器内定向命令（挂载本提交到 `/workspace:ro`，空结果目录到 `/results`，关闭网络及设备）：

```bash
source /opt/ros/humble/setup.bash
cd /workspace
colcon --log-base /results/log build \
  --base-paths src/rm_competition_interfaces src/rm_pursuit \
  src/rm_competition_mission src/rm_navigation_bringup \
  --build-base /results/build --install-base /results/install \
  --executor sequential --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
source /results/install/setup.bash
export ROS_LOG_DIR=/results/ros_log
unset RM_COMPETITION_MISSION_SOURCE_DIR
ctest --test-dir /results/build/rm_competition_mission --verbose
python3 -m pytest -q src/rm_pursuit/test/test_pursuit_math.py \
  -o cache_dir=/results/pytest_cache
```

补充失效 fixture 首次因 ROS launch 子进程包含 Python 解释器前缀而识别失败；修正为按父子关系与完整路径
识别本次测试的子进程。注册测试首次还遇到手工预设环境变量与 CMake APPEND_ENV 叠加，
清除手工变量后使用测试自带路径；以上均属测试编排修正，未放宽任何业务 gate。

## 6. Runtime behavior delta

- 显式 hold 现在也压过低血量 home；没有 home 坐标时停止，不再偷偷继续低优先级任务。
- 分支/新目标抢占立即发起，替代 action 要等待终态；中途可能出现有意的短暂停顿。
- Mission 不再消费旧 pursuit Pose/Bool。正式组合只接受 radar candidate，研究用途需显式关闭 radar-only。
- 持续更新同一目标采用阈值与限频；planner 消失、目标 stale/lost 不再无限追最后坐标。
- 安全证据不再无限缓存；撤权后 tactical 需新观测。patrol/home 仍由当前条件重新计算。
- 保留既有模式选择、home 阈值、standoff 数学、patrol 点序与索引、失败 backoff/次数上限及 Spin 基本流程。

## 7. Remaining gaps

- 真实 radar→TargetTrack 解码、时间/ID/坐标映射与可信 producer 仍未实现；测试仅证明软件入口政策。
  topic/source 标签不等于加密来源认证，部署须保证可信 ROS graph，不能把队友桥接到 radar topic。
- 真实 Mission 尚未成为 AUD-003 正准入 owner/session 租约生产者，本轮只接入最终 guard 的**只读否决**。
  2027 导航组合缺少外部授权时保持停止，不构成可直接上车的完整 profile；没有改变 V2 BLOCKED 重启恢复要求。
- 未启动真实 Nav2 planner/controller、真实 radar/serial 或机器人；取消执行取决于 action server 合同。
  服务端不提供终态时，Mission 保持等待，不并发重发；无物理停车或比赛部署结论。
- 互斥 owner 合同限受测组合；不能与 dog-hole、手动目标工具或另一个 Mission 同域共同发 Nav2 目标。

## 8. Decision

```yaml
Decision: GO
Scope: navigation target arbitration software contract only
Branch: feature/navigation-intent-arbitration-2027
Commit: 本报告所在提交（git rev-parse HEAD）
Main changed: false
Hardware used: false
Merged: false
Pushed: false
```

只表示上述受控软件合同满足本轮验收；不是 Hardware Go，也未关闭真实雷达链路与
AUD-003 正准入 owner/session 集成缺口。定位冻结状态保持原样。

## 9. Next recommendation

下一项候选为 `Posture-aware motion constraints`。本轮不执行该方向，报告完成后停止。
