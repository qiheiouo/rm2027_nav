# Mission DEAD / REVIVED 最小闭合

Integration，基于 `5f52d8e06310e4932863d11f306b6647d2e0d05a`，分支
`codex/mission-dead-revived-authority`。本轮只改生命授权、直接验证和合同；无硬件访问。

## 当前真实输入与修复

| 层 | 当前事实及边界 |
| --- | --- |
| RobotPosture | ATTACK/MOVE/DEFENSE 与三个增强态；没有 DEAD/REVIVED 枚举，Mission 不消费 posture state。未增加第二套枚举。 |
| HPM feedback | 已解码 HP 和 byte 35 `life`，但 ROS RefereeState 只带 HP，`life` 没有已确认的生命语义映射。帧内没有源序号/源时间，由 host RX 打戳。 |
| competition_v2 | Referee payload 带 valid/HP/diagnostic；没有独立 DEAD event。heartbeat 的 ready/capability 只参与链路有效性。frame 有 sequence，但当前 referee 接收未去重，且重新打 host RX stamp。 |
| referee_state_gate | 负责 valid、范围、源年龄与 timeout；保留 ROS header stamp，但没有生命 generation 或严格排序水位。 |
| Mission（修复前） | 每包覆盖 HP/接收时间；只判断 HP ≤ retreat threshold，故 HP=0 进入 HOME。重复、乱序消息也可覆盖缓存。 |
| Mission（本轮） | 消费有效、时间结构/年龄合格、源 stamp 严格更新的规范 RefereeState；HP=0 锁存 DEAD，更新的 HP>0 是复活输入。重复不换代、不续租，乱序与非法未来包不改水位。 |

没有比 HP 更权威的现成 DEAD event 可供接入。不添加猜测协议的适配器或 debounce：
确认死亡立即撤权，所有更高层 Safety 门保持有效。HP 恢复只移除生命否决，绝不是
MCU 运动重新授权。真实线上重传被重新打戳后的新旧识别仍须在硬件输入验收闭合；
D11/D12 的通过范围明确为保留原始 ROS stamp 的规范消息边界。

正式语义和正交生命状态图集中在
[Life-State Authority Contract](../contracts/navigation_target_arbitration_2027.md#life-state-authority-contract)。
不重写已验证的任务策略树。主要变化：

- 输入回调立即取消 Navigate/Spin，清除旧执行/候选记忆，但保留 pending/active handle
  到 terminal；因此 DEAD 的软件 authority=0 与仍存在待取消 owner 可以同时成立。
- life generation 在初次有效输入和生命转换时增加；新 Navigate 再增加既有 goal generation。
  `(life, goal, localization session)` 隔离执行，旧 life 结果只释放 handle。
- 复活时定位无效，需要比当时更新的有效 session；同 epoch valid 心跳不放行。
- 请求模式、patrol 索引保留；恢复时重新仲裁，旧 pursuit 候选即使 TTL 未过也不能复活。
- MissionState.detail 输出实际 life/goal generation、session 和软件 authority，测试 trace
  同时保存 owner、UUID、请求、取消与 terminal；不增加新的命令接口。

## D1–D12

| 场景 | 结果与实际证据 |
| --- | --- |
| D1 DEAD navigating | PATROL/PURSUIT/HOME 三组均立即 authority=0，取消并闭合 terminal；DEAD 中没有 HOME 或新请求。 |
| D2 DEAD cancel pending | 旧 patrol 正在被 HOME 抢占时死亡，0.6 s 延迟 terminal 前保持唯一 owner。 |
| D3 REVIVED | 终态前复活仍等旧 terminal，life generation 增加，新 action generation 增加；旧 SUCCEEDED/ABORTED 均被计入 stale rejection。 |
| D4 valid localization | 按当前 HOME 或 AUTO→PATROL 策略重建，旧 patrol success 不推进索引；不恢复旧 action。 |
| D5 invalid localization | 复活后 HOLD；同 session 重新 valid 仍不动，新 session 才重建 patrol。 |
| D6 event replan | 真实 Nav2 Wait 中 DEAD，Navigate/子 action 均 terminal；之后 2 s 无新增 Compute/Follow/Wait/Navigate。 |
| D7 explicit rebase | 延迟 planner 下 DRAIN_OLD_NAV 中 DEAD；旧 planner 终态 9.3413 s，commit 9.4792 s，prime 后仍 DEAD；复活后的新 Navigate 10.5336 s。 |
| D8 target during DEAD | 软件持续雷达输入与物理持续候选均无导航授权或新任务。 |
| D9 stale target | 死亡期间目标不跨复活；额外定向用例在目标原 deadline 之前观察到新 patrol，排除“仅 TTL 到期”的假通过。 |
| D10 rapid transitions | DEAD→REVIVED→DEAD 在旧 cancel terminal 前完成，未产生中间新任务或 action leak。 |
| D11 repeated packet | 相同 stamp 重复/冲突 HP 不换生命代次；持续重放 ALIVE 不续 lease，随后 HOLD。 |
| D12 out of order | 新 DEAD 后旧 ALIVE、新 ALIVE 后旧 DEAD 均拒绝；非法未来 ALIVE 不污染后续正常复活。 |

最终 **9 项生命授权 + 原 14 项系统测试 = 23 passed（83.21 s）**；全部 launch 退出 0、
无子进程异常，所有观察到的 DEAD authority 均为 0，最大同时 owner=1。
D6/D7 两次物理交叉均功能通过、正常退出、无残留。
D7 的旧上层 Navigate 已取消时，下层延迟 Compute 仍可返回旧 success，但不能越过
drain/commit 屏障，也不能在 DEAD 中恢复导航。未以软件取消宣称车辆已经停稳。

## 长序列与回归

新确定序列：PATROL→PURSUIT→HOME→DEAD→REVIVED→DEAD→旧 success→新追踪输入
→REVIVED/HOME→定位 invalid→新 session/PATROL→Safety veto。
实际 **5 次请求、5 个 terminal，max active owner=1；所有 DEAD 状态 authority=0**。
同一个 trace 包含真实 Mission life/goal generation、localization session、owner 和 UUID。
原固定种子 7/19/41 的三轮长序列仍在原 14 项系统测试中保留。

- build：10 packages 通过，22 source packages / 10 install prefixes 检查通过。
- 原 14 项系统测试（将“HP0 仍 HOME”的旧缺口断言改为 DEAD）及仲裁/Spin/profile/pursuit：
  合并 **25 passed，121.25 s**。补齐共享夹具就绪屏障后的 9+14 项最终复核亦全部通过。
- 定位/nav config 单元：bridge 27、adapters 29、nav config 1；0 failure/error/skip。
- input-health、correction、rebase ordering runtime 全通过，进程退出分别 [0,0]、
  [0,0,0]、[0,0,0,0]。
- event blocked 通过，到达误差 0.081317 m；repeated rebase 两次事务通过，3 次规划，
  到达误差 0.105915 m。
- 统一 baseline 首次导航/Spin 通过，但启动点云的 TF 查询 0/10；原配置复核为 10/10，
  所有 gate 通过，到达误差 0.120182 m。保留首次启动采样失败，不改运行参数。
- 新增短目标用例首次功能通过，但退出时 planner 仍在 Python import，SIGINT 导致 −2。
  这是夹具只等 Mission、未等 planner ready 的启动屏障遗漏；补等真实候选 publisher，
  不修改生产节点退出逻辑。修正后的结果在最终软件汇总记录。

### 未通过的既有 no_route 回归

**OPEN_SIM_VALIDATION — no_route bounded-exhaustion timeout**。
原配置首跑和一次定向复核均未在既有 70 s exhaustion 窗口内到达 `goal_retry_limit`。
两次均只有 **1 Navigate / 4 Compute / 4 Follow / 3 Wait**，没有无限重发或第二 owner；
但第四个 FollowPath 到停止测试时仍 active，因此不能把“重试计数有限”当成“终态已闭合”。

复核的四个 Follow 开始时刻为 7.3613、21.3208、33.0812、49.9417 s；前三个分别于
19.6627、31.4224、48.2819 s 失败，末次到 78.756 s 仍在运动。规划均返回 success；
物理围挡与局部 progress checker 并不提供每次 Follow 的绝对完成时限。这是当前场景
对耗尽截止时间的未满足要求，尚不能区分控制器绕障进度与验收时限选择的完整根因。

该场景没有 referee publisher，life generation 始终 0；未观察到本次生命转换触发。
但没有旧二进制配对试验证明因果，因此不宣称已排除所有回归。按最小验证原则，一次
原配置复核仍失败后停止扩展；没有调大时限、强制 controller failure、调参或修改 planner
来换取通过。该问题独立于 S12，也不能被归档成 Gazebo shutdown。

## S12 与裁决

S12 统一保留在既有报告的
[独立基础设施 issue](competition_mission_arbitration_system_20261029.md#s12-独立基础设施-issue)：
**OPEN_SIM_INFRASTRUCTURE — Gazebo shutdown timeout**。本轮 8 次物理运行（含两次
失败场景及原配置复核）均正常退出、无残留；后续独立启动正常，未观察到跨轮状态污染。
此前多次 timeout 的证据不被覆盖，root cause unknown；本轮不改 Mission/launch/Gazebo
架构来追查它。唯一 S12 不足以否决软件闭合，但本轮还存在上述 no_route 验收失败。

**DEAD/REVIVED 定向软件合同通过；整条系统线仍为
Software Modify / Simulation Modify / Hardware Blocked。**
不能给出“Modify 原因只剩 infrastructure shutdown”的结论。必须保留 no_route 为未关闭
的功能验收项；不存在证据要求推倒 Mission ownership 模型重设计。

**未正式冻结系统可靠性工作线，未正式切换到经典规划/启发式搜索研究。** 用户规定的
“D1–D12 和回归全部通过”前提未满足。下一项计划仍为 Independent Classical Planning /
Heuristic Search Research，本轮不擅自启动，也不顺手扩展动态导航、姿态或新 Mission 功能。
定位恢复线继续 `FROZEN_WAIT_HARDWARE_RECOVERY_DATA`。

Hardware Blocked 包括真实 referee/MCU 输入排序与延迟、DEAD/REVIVED 物理含义、
最终 V2 guard 重授权、MCU braking/motor disable/停车距离、真实定位时间同步与 session。
软件 HP 策略、Nav2 cancel、零 Mission authority 均不是实车停车或可参赛证明。

## 复现与 Git

```bash
tools/build_simulation.sh
source install/setup.bash
RM_COMPETITION_MISSION_SOURCE_DIR="$PWD/src/rm_competition_mission" \
  python3 -m pytest -q src/rm_competition_mission/test/test_life_authority_runtime.py
python3 src/rm_new_car_simulation/scripts/validate_mission_system.py \
  --scenario life_rebase --output /tmp/mission-life-rebase
python3 src/rm_new_car_simulation/scripts/validate_event_replanning.py \
  --scenario no_route --output /tmp/mission-no-route
```

原始日志/trace 留临时目录，Git 只保存最小代码、夹具和可判定摘要。未合并或推送；
main 保持 `d37f8d6e5ebe53533b7fc62aca8bb84392bee31b`。最终提交号以本分支 Git 为准。
