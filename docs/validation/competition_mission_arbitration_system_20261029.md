# Competition Mission / Navigation Arbitration 系统验证

2026-10-29，Integration。基于 `b105183`，分支
`codex/competition-mission-arbitration-validation`；不合并 main，不访问硬件。
定位恢复开发继续保持 `FROZEN_WAIT_HARDWARE_RECOVERY_DATA`。
本文保留 `5f52d8e` 阶段结论；后续生命授权闭合见
[Mission life authority](mission_life_authority_20261029.md)，不将本轮 D 组缺失记录改写成通过。

## 架构与合同

以本轮源码、默认 Mission BT、event BT 和实际 launch 为依据，正式 priority table、
Mission/执行两层状态图及六项合同集中在
[Navigation Target Arbitration Contract](../contracts/navigation_target_arbitration_2027.md)。

- **Priority**：Safety/hold/disable > 显式或低 HP HOME > 有效 PURSUIT > PATROL > HOLD。
  AUTO 是 pursuit 优先、否则 patrol 的请求模式；显式 patrol 不消费追踪目标。
- **Ownership**：唯一任务级客户端为 Mission；pursuit planner 只产原子候选，Nav2 BT
  只在当前 commitment 内调用 ComputePath/FollowPath/Wait，不另发 NavigateToPose。
- **Cancellation**：保留 pending/active handle，cancel ACK 不是 terminal；旧终态
  结果只能释放旧 owner，互斥新 action 必须在其后发出。
- **Session**：新定位 session 撤销旧执行，rebase 额外等待子 action drain 与缓存 prime。
  旧 session 结果不能推进巡逻；普通小 correction 不更换 session。
- **Resume**：保留当前请求和未完成 patrol 索引；重新仲裁、重新建 action/path。pursuit
  要求撤权后的新源观测；显式 set_mode/新 intent/新 session 按合同重置失败预算。
- **Safety**：所需 localization/referee/chassis 证据及已观察 guard 的 lease 均可否决。
  这是软件任务否决，不是已完成最终串口正授权或机械停车证明。

当前没有通用 temporary-goal topic；temporary target 对应带 track_id、原始采样时间与
deadline 的 PursuitCandidate。雷达输入使用受控 TargetTrack 夹具和真实 pursuit planner，
不新增真实 radar producer。物理夹具直接提供候选，不伪称已验证雷达定位精度。

## 先复现、后修复

先建立一次正常 radar→planner→Mission 追踪，再注入异常，排除启动订阅未连接造成的
假通过。未修复版本的 6 个时间戳测试中 4 failed、2 passed：

| 问题 | 实测反例 | 最小修复 |
| --- | --- | --- |
| 未来采样污染排序水位 | radar 与 candidate 各注入 now+3600 s；随后的正常新目标在 5 s 内始终无法进入 | 在写入最新样本/排序水位之前检查年龄与未来容差 |
| 负时间戳导致退出 | radar 使 Python planner 抛 ValueError（退出 1）；candidate 使 C++ Mission 抛 runtime_error（退出 −6） | 转换 ROS Time 前校验 sec/nanosec；candidate deadline 同样校验 |

修复保留既有 TTL、更新阈值、优先级和模式，不改变追踪策略。Mission 通用 freshness
检查也先验证时间结构，避免安全输入在该共同入口抛异常。没有新增姿态约束、算法、
reseed 或恢复编排。

另一个测试配置问题：最初 planner→HOME 物理夹具给普通场景也施加了 0.8 m 偏差，
HOME 无法到达并耗尽重试；该次不计功能通过。修正为仅 rebase 场景带初始偏差，并等待
Nav2 lifecycle active 后开始，同一抢占场景通过。没有据此修改导航算法或放宽验收阈值。

最终合并运行还暴露一个测试时序问题（13 passed、1 failed）：set_mode 返回时目标可能
已被接收，测试随后才取“原目标数”会误等第二个目标。实际轨迹只有一个正常 patrol
action，进程均正常退出。夹具改为在触发之前取计数，并在 guard 仍关闭时设定恢复意图，
消除服务回复与 action 到达先后的依赖；不修改生产状态机。

## M / T / P / L / R / D 结果

| 场景 | 结果 / 证据 |
| --- | --- |
| M1–M5 | HOLD 无 action；PATROL 成功推进下一点；HOME 正常；PURSUIT 有效目标进入；AUTO 追踪优先、目标过期回巡逻。真实 Mission + planner 软件测试通过。 |
| M6–M9 | PATROL→HOME、PATROL→PURSUIT、PURSUIT→HOME、HOME→PATROL/AUTO 均先终止旧 action，无双 owner。物理 planner_home/controller_home 复核通过。 |
| T1–T4 | 合法目标、TTL 失效、不同 ID 替换、乱序拒绝通过；同 ID 抖动/更新速率、重复源时间不续租由既有仲裁回归覆盖。 |
| T5–T6 | 未来、负数、越界 nanosecond，以及非法 candidate deadline 修复后恢复正常追踪，不再永久阻塞或进程退出。 |
| T7–T9 | lost/cancel 在 auto 回未完成巡逻点；pursuit 专用模式回 hold；HOME 和 Safety 均压过追踪。 |
| P1–P4 | HOME 抢占、Safety 撤权、快速连续抢占通过；取消期间只保留最新意图，不发并行承诺。 |
| P5–P7 | 软件 cancel terminal 延迟 0.25–0.45 s，迟到 SUCCEEDED/ABORTED 均只释放旧 owner，不推进旧点、不消耗新目标预算。 |
| P8–P10 | 真实延迟 Navfn 期间 HOME 抢占、延迟 controller cancel、event Wait 期间 HOME/Safety/新目标抢占通过。 |
| L1 | 巡逻中 raw-LIO 静默使定位 invalid、Mission hold，旧 action terminal 后无重算；恢复后保留 patrol 意图建立新 action，再切 HOME 完成。3 个 Navigate/ComputePath/FollowPath，最大 owner 1。 |
| L2–L3 | pursuit/Wait 中 raw LIO 静默后 hold、无 ComputePath storm；恢复健康并取得新定位会话后建立新 action，旧 action 不复活。 |
| L4–L5 | 物理 patrol_rebase 与既有 pursuit repeated rebase 验证，旧上层/子 action terminal 在 commit 前；新会话重规划。 |
| L6–L7 | 延迟 planner 中 rebase/session 改变；既有仲裁回归覆盖 pending/active 旧会话迟到 success，旧结果拒绝计数增加、patrol 索引保持。 |
| R1 | patrol_blocked：真实障碍加受控 progress failure 触发 Wait/重算，仍为同一 patrol commitment；不把受控故障宣称为自然碰撞检测性能。 |
| R2 | 原有 blocked 场景：单个 Navigate、2 次 ComputePath、1 次 Wait；绕行后到达误差 0.088914 m。 |
| R3–R6 | event Wait 内 HOME、guard veto、LIO invalid、更新 pursuit ID 均通过；撤权后停止重算，恢复后按当前意图新建 action。 |
| R7 | 移除真实障碍后重规划到达，误差 0.129806 m；单个 Navigate，2 次 ComputePath。 |
| R8 | 原有 no_route 通过：1 个 Navigate、4 次 ComputePath、3 次 Wait；耗尽后 5 s 无新增请求，goal_retry_limit 保持。 |
| D1–D5 | **NOT INTEGRATED**：当前 RobotPosture 是 ATTACK/MOVE/DEFENSE 及三个增强态，没有 DEAD/REVIVED；Mission 不订阅 posture state。HP=0 实测仍选择 HOME。不得用 guard fixture 模拟恢复就声称 REVIVED 已接入。 |

D 组的相关基础机制已单独验证：现有 guard 撤权能取消 HOME；guard 恢复而定位无效
时仍 hold。该结果不等于 DEAD 生产映射或 REVIVED 策略。按本轮禁止新增 posture
constraints 的要求保留此缺口，完整比赛生命周期不能据此授予 Go。

## 故障与长序列

- planner failure：真实无路可达；controller failure：受控 progress failure 与真实堵路；
  两者进入当前 Nav2 BT 内的 Wait/重算，最多初次加 3 次 recovery。
- costmap unavailable：deactivate planner 及其拥有的 global costmap，观测 costmap
  inactive；Nav2 lifecycle manager 随后撤销关联服务，导航请求被拒绝。Mission 达到
  goal_retry_limit，2 s 观察窗无新增请求/路径。不是仅断传感器的测试。
- action accept delay、cancel delay、旧会话/旧目标 success/failure、定位 invalid、TTL
  expiry、Safety veto 均由真实 Mission 客户端执行，不由假 Mission 状态机替代。
- duplicate result：重发真实软件 action server 的旧 GetResult 响应，含新 HOME 已启动
  后的重放；真实 rclcpp_action 客户端未重复推进 patrol，也未污染 HOME。测试使用
  Jazzy 的私有发送钩子做 wire fault injection；不声称应用 result callback 被调用两次。
- 固定种子 **7、19、41**，每个 3 轮，均执行 patrol→pursuit→HOME cancel pending→
  定位无效→迟到旧结果→新 session→HOME→guard veto→patrol，末尾再测 pursuit 撤权。
  每种子 **13 个请求、13 个 terminal**；最大同时 active Mission action **1**。
  请求发送时刻也逐项检查晚于前一 terminal，而非只统计被接受的 action。
- 两次物理 long 均为 **6 个 Navigate、6 次 ComputePath、1 次 Wait**，无死锁/泄漏；
  包含真实 LIO/bridge 新会话、旧结果拒绝和 guard 撤权。没有伪造 DEAD/REVIVED。

临时 trace 记录 action 请求序号/UUID、分支、目标 ID、定位 session、取消与终态。
软件请求序号对应唯一 sendGoal 递增的 commitment generation；不是额外生产 generation
owner。当前目标代次由 track_id/原始样本时间表达，不虚构不存在的整数消息字段。

## 回归与 S12

- 统一 build：10 packages 成功；22 个 source packages、10 个 install prefixes 检查通过。
- 相关单元实际用例：bridge 27、localization adapters 29、nav config 1，均 0 failure/error/skip。
- 既有 Mission/Arbitration/Spin/profile/pursuit pytest：11 passed（35.50 s）。
- 新系统测试：时间戳、deadline、模式、长序列、旧响应重放和姿态边界，最终
  **14 passed（58.11 s）**，14 组 launch 均退出 0、无子进程异常退出。
- 既有 input-health、correction C1–C12、rebase ordering runtime 全通过；退出码分别为
  [0,0]、[0,0,0]、[0,0,0,0]，包含 PRIME 二次大修正拒绝。
- 物理 14 次交互/退出矩阵全部通过；额外 planner_home 修正复核通过。
- 原有 event blocked/removed/no_route 与 repeated rebase 均通过且正常退出；后者验证
  两次 rebase 的子 action drain、commit 顺序、缓存 barrier 和最终到达，无重规划风暴。
- 统一 baseline 通过：点云、静态场景、canonical TF owner、controller odometry、命令
  路由、Navigate/Spin 均通过，到达误差 0.114952 m；退出 0、无异常进程和残留。
- 收尾的 patrol_lio、normal、normal 共 3 次功能与退出均通过。合计 **24 次有效物理
  场景运行，功能 24/24，正常退出 23/24**；分组为 14 次交互矩阵、1 次 planner_home
  复核、1 次 system_final、3 次原有 event、1 次 repeated rebase、1 次统一 baseline、
  3 次收尾复核。最初配置错误的 planner_home 不计入有效场景，前述失败仍保留记录。

**S12 已复现，不能标记 Root Cause Closed。** 在后续普通 `system_final` 场景中，
Mission 功能通过，结束前 planner/controller/navigator/global costmap 均 active。
SIGINT 后 6 s 仅 launch 与 Gazebo（ruby 包装进程）存活，全部我方节点/Nav2/ROS bridge
均正常结束；Gazebo 的 44 个线程中 39 个停在 futex wait，另有 epoll/poll wait。
SIGTERM 后仍不退出，launch 最终 SIGKILL，退出码 −9；清理后无残留进程。退出前
Gazebo topic inventory 可返回 26 个 topics。

当前归因只到 **Gazebo 退出阶段未完成**：没有证据表明 Mission/node shutdown 卡住；
wait channel 不能区分 Gazebo/Harmonic 内部、render/transport 或 launch 交互根因。
之后启用有界线程栈诊断的 3 次复核均正常退出，没有进入抓栈条件，因此未取得失败栈。
保留 **Reproduced / root cause unknown**；不得用成功复跑覆盖此失败，也不为退出问题
大改 Mission。上述 23/24 是本轮受控样本结果，不是退出可靠性概率估计。

### S12 独立基础设施 issue

**OPEN_SIM_INFRASTRUCTURE — Gazebo shutdown timeout**。功能阶段正常，多轮已复现；
强制终止后无残留，同配置也有成功退出，root cause unknown。已有证据只定位到 Gazebo
进程退出未完成，不能进一步声称 transport shutdown 已确诊，也无我方节点 lifecycle
直接致因证据。后续生命授权闭合仅复用该 issue，不将其混入 Mission 语义修复。
受控复跑需确认无残留、下一轮能独立启动且没有观察到数据/状态损坏；该条件满足时，
它保留独立 infrastructure Modify，不否决通过的 Mission 软件合同。

## 裁决与硬件边界

**Software Modify / Simulation Modify / Hardware Blocked。** 已实现 Mission/Arbitration 合同的核心软件与
物理交互通过，I1–I5 在声明的单 Mission、受控 ROS 图、当前 Nav2 profile 下成立；
没有需要推倒重设计的双 owner 冲突。完整比赛系统仍缺 DEAD/REVIVED 生产映射，
且 S12 已复现，退出根因未关闭。

仍需真实雷达源/坐标/TTL、裁判与底盘状态、DEAD/REVIVED 到安全门的定义和接入、
V2 guard 正授权/撤权/重授权、实际停车与制动、真实定位时间同步、rebase 前后传感器
和 costmap 连续性。软件终态不证明车辆静止，fixture PERMITTED 不授予真实运动权限。
本轮不推进 autonomous reseed，不改恢复线冻结状态，不自动增加下一开发阶段。

## 复现与资产

```bash
tools/build_simulation.sh
source install/setup.bash
RM_COMPETITION_MISSION_SOURCE_DIR="$PWD/src/rm_competition_mission" \
  python3 -m pytest -q src/rm_competition_mission/test/test_mission_system_runtime.py
python3 src/rm_new_car_simulation/scripts/validate_mission_system.py \
  --scenario long --output /tmp/mission-system-long
```

物理脚本 `--help` 列出场景；每次独立 ROS domain、Gazebo partition 和进程组。
`--shutdown-backtrace` 只在本次仿真退出超过 6 s 时抓取该进程组的模拟器栈，不读 locals。
完整日志/trace 仅留临时目录；Git 保存修复、测试、本报告和既有权威合同的更新。
main 保持 `d37f8d6e5ebe53533b7fc62aca8bb84392bee31b`；最终提交号以同分支 Git 为准。
