# 2027 Navigation Target Arbitration Contract

状态：Integration 软件候选；不授予硬件运动权限。适用于自动哨兵。

## Ownership Contract

`competition_mission_node` 复用现有 BT，每 tick 只产生一个 `desired_goal_`，并拥有
唯一任务级 `NavigateToPose` 客户端。pursuit 只产生候选，referee/serial 不发导航目标。
Nav2 自己的 BT 调用 controller 的 `FollowPath`；本仓任务层不直接调用它。

## Priority Contract

默认树的正式 priority table（从高到低）：

| 优先级 | 选择条件 | 抢占与后续行为 |
| --- | --- | --- |
| 1 SAFETY_STOP | DEAD、复活尚未满足定位条件、disable、显式 hold、必需安全证据失效、底盘撤权/ESTOP，或已观察 V2 guard 非 PERMITTED/过期 | 否决 HOME/PURSUIT/PATROL，取消 Navigate/Spin，不发替代目标 |
| 2 RETURN_HOME | 新鲜且 validity gate 有效的 referee 0 < HP ≤ retreat_hp_threshold，或显式 home | 高于 pursuit/patrol；无 home 坐标则 hold_missing_home，不降级 |
| 3 PURSUIT | allow_pursuit，模式 auto/pursuit，候选有效 | 可抢占 auto 巡逻；不能抢占 HOME/Safety |
| 4 PATROL | 模式 auto/patrol，已配置 waypoint | 保留 patrol_index，只有当前未撤销的成功才推进 |
| 5 IDLE/HOLD | 没有合格目标 | 不发导航 action |

没有现成的 SPECIAL_MISSION 坐标入口，本轮不新增占位 task planner。显式 home 已包含在
RETURN_HOME；`SetMissionMode` 和 YAML 属于启动/调试配置，不是队友 tactical 输入。
显式 `patrol` 继续禁止 pursuit；需要“平时巡逻、雷达出现就追踪”时使用 `auto`。

## 两层状态与转换责任

以下图恢复当前代码语义，不新增 enum/state machine。Mission 图中的 AUTO 是请求模式，
实际分支仍为 home/pursuit/patrol/hold。`SetMissionMode` 改请求；输入租约、HP 和候选
改变每个 tick 的选择；执行层独立保留旧 action 所有权直到终态。

```mermaid
stateDiagram-v2
    [*] --> HOLD
    HOLD --> SELECT: enable + 非 hold 请求 + 安全证据新鲜
    SELECT --> HOME: 显式 home 或低 HP
    SELECT --> PURSUIT: auto/pursuit + 合格目标
    SELECT --> PATROL: auto/patrol + 已配置路线
    SELECT --> HOLD: 无可选目标 / 缺 home
    HOME --> SELECT: set_mode / HP 条件变化
    PURSUIT --> SELECT: target lost/TTL / replacement / set_mode
    PATROL --> SELECT: 到达 / 新候选 / set_mode
    HOME --> HOLD: Safety veto / disable / hold
    PURSUIT --> HOLD: Safety veto / disable / hold
    PATROL --> HOLD: Safety veto / disable / hold
```

```mermaid
stateDiagram-v2
    [*] --> IDLE
    IDLE --> PENDING: 合格且未完成的 intent + SafetyReady + 预算
    PENDING --> PLANNING: Nav2 接受 / ComputePath
    PENDING --> RETRY_LIMIT: 请求拒绝且外层预算耗尽
    PLANNING --> FOLLOWING: 新路径成功
    PLANNING --> REPLANNING: FAILURE + 剩余 BT 预算
    FOLLOWING --> REPLANNING: FAILURE + 剩余 BT 预算
    REPLANNING --> PLANNING: Wait 结束
    FOLLOWING --> IDLE: SUCCEEDED / 当前目标完成
    PLANNING --> RETRY_LIMIT: FAILURE 且预算耗尽
    FOLLOWING --> RETRY_LIMIT: FAILURE 且预算耗尽
    REPLANNING --> RETRY_LIMIT: BT/外层预算耗尽
    RETRY_LIMIT --> IDLE: 新 intent / 显式 set_mode / 新定位 session
    PENDING --> CANCELLING: 抢占 / 撤权 / session 变化
    PLANNING --> CANCELLING: 抢占 / 撤权 / session 变化
    FOLLOWING --> CANCELLING: 抢占 / 撤权 / session 变化
    REPLANNING --> CANCELLING: 抢占 / 撤权 / session 变化
    CANCELLING --> LOCALIZATION_HOLD: terminal 且定位仍无效
    CANCELLING --> IDLE: terminal 且定位有效
    LOCALIZATION_HOLD --> IDLE: 新鲜有效定位 / rebase prime 完成
```

PLANNING/FOLLOWING/REPLANNING 是 Nav2 action/BT 观测，非 Mission 新增内部状态。
取消未被接受或未 terminal 时停留 CANCELLING；没有超时后并发发目标的 fallback。
非定位 Safety 关闭时同样停止发目标，恢复只重新选择当前 intent。patrol 保留未完成
索引，pursuit 需要撤权之后的新观测，旧 action 从不直接 resume。

## Cancellation Contract

- 同一时刻最多有一个 pending/active 的 NavigateToPose 或 Spin。
- 分支或目标身份变化：立即请求取消；等待旧 action **终态 result**，再按最新有效决策发送。
  Cancel ACK 只是 CANCELING，不能据此交出 owner。
- pending 请求尚未返回 handle 时撤权：记录取消意图；迟到的 accepted handle 一到就取消。
- 等待取消期间的成功 result 不推进 patrol 索引，也不标记旧战术目标完成。
- 取消被拒绝或服务端不结束：保持等待，不并发发送替代目标。服务端执行取消及实际停车
  仍是下游合同；本层无法用重复发 goal 强迫服务端停车。
- Spin 同样必须结束后才允许 home。

## Resume Contract

patrol 被抢占后保留索引，只有未被撤销的成功到达才推进。恢复安全证据或新定位会话后，
Mission 重新仲裁当前请求并建立新 action，不直接恢复旧 action/path。
pursuit 要求原始采样时间晚于最后一个 hold/撤权 tick，不复活撤权前缓存的 tactical target。
显式 set_mode、新 intent 或新定位 session 可重置对应失败预算；相同失败目标不会无限重发。

## 原子 pursuit 候选

`/mission/pursuit_candidate` 使用 `PursuitCandidate`，由现有 planner 唯一生产：

| 字段 | 语义 |
| --- | --- |
| `header.stamp` | 原始 TargetTrack 采样时间，重复发布不得改为当前时间 |
| `header.frame_id`, `pose` | planner 转换后的 map standoff pose |
| `valid_until` | 原始采样时间 + 既有 `max_track_age_sec`（默认 0.3 s） |
| `track_id` | 稳定目标身份；非空；更换身份表示 replacement |
| `source` | planner 按输入策略填写 RADAR 或 RESEARCH，不由 TargetTrack 自报 |
| `valid` | 同一条消息内的质量/TF/观测有效性；false 表示 lost/不可用 |

Mission 独立检查 frame、有限值、单位四元数、来源、采样时间、deadline 和稳态时钟 lease；
`input_timeout_sec=0.5 s` 是上限，不能延长 planner 的 0.3 s 源期限。
同一采样时间重复出现不能刷新 Mission 或 planner 的稳态 lease；旧时间样本被忽略。
原始 track/candidate 的负数或越界 nanosecond、过期或超出当前未来容差的时间戳，
必须在更新排序水位之前拒绝；candidate deadline 也须结构合法。否则一条未来消息
可能永久排斥正常输入。非法消息不终止进程，也不延长旧目标的原始 TTL。

- **valid**：上述检查全部通过；才可参与仲裁。
- **stale**：采样年龄、valid_until 或稳态 lease 过期，包括 planner 退出。
- **lost**：观测 `valid=false`、质量不合格或 TF/计算失败。取消 pursuit，auto 回到同一 patrol 点；
  pursuit 专用模式回到 hold。
- **replaced**：新的 `track_id` 替代旧身份，即使坐标相同也取消旧 action 后重发一次。
- **same target update**：相同 ID；距离/yaw 变化不超过 0.35 m/0.35 rad 不重发；超过阈值后
  最短间隔 0.5 s，取消期间合并为最新候选。高优先级抢占不受此间隔限制。

原有 `/mission/pursuit_goal` 和 `_valid` 继续发布供研究工具和诊断使用，Mission 不再消费
这对无法携带身份/源期限的分离消息。研究自定义 producer 需要迁移至原子候选。

## 比赛与研究输入边界

- `pursuit_goal_planner radar_only=true` 只订阅固定 `/radar/target_track` (`TargetTrack`)，
  忽略通用 `input_topic`，候选标记 RADAR。没有 generic teammate 订阅或 operator adapter。
- `competition_2027_navigation.launch.py` 固定 planner/Mission 的 `radar_only=true`，
  强制 `require_motion_guard=true`、chassis gate、无 mock、Mission 启动 disabled。
  YAML 的 `radar_only=false` 或 `require_motion_guard=false` 不能覆盖组合策略。
- 通用 `pursuit.launch.py` 保留 `radar_only=false` 的感知/研究输入。研究 Mission 必须显式
  选择 `radar_only=false`。本次未迁移 legacy 组合入口的策略改动，使用它们前须独立核对参数。
- `/operator/navigation_target_raw` 仍是无 consumer 的原始诊断数据，不进入比赛仲裁。
- `/mission/set_mode` 由受信任的本地配置/调试工具调用。禁止桥接为队友战术命令。

来源标签和 topic 隔离是受信任 ROS 图内的策略边界，不是 DDS 身份认证。实际雷达链路
解码/坐标适配尚未实现；不得把任意队友数据桥接到 radar topic，部署时须限制 publisher。

## Safety Contract：证据与运行边界

Mission 保留既有 LocalizationStatus 合同；对 referee/chassis/V2 guard 增加独立 0.5 s
采样及接收 lease，防止上游 gate 退出后缓存状态无限存活。看到任何 V2 guard 后，即使
旧调试配置未要求它，也不能忽略它的拒绝或失联。

`V2MotionGuardState` 是最终编码边界的只读观测；消费 PERMITTED 不创建 owner/session，
也不替代 AUD-003 的租约或最终逐帧检查。现有 guard 在 BLOCKED 后仍需重启并重新授权。
本轮没有连接真实 Mission 到该正准入租约，测试中的 guard 发布者只是受控 fixture。
2027 导航组合只启动 Mission/planner，不启动 Nav2、serial、定位、referee 或物理设备；缺少
外部安全证据时保持停止。不得与 dog-hole manager、手动目标工具或另一 Mission 同域共同控 Nav2。

## Life-State Authority Contract

生命状态独立于 Mission 请求模式和六态 `RobotPosture`。不增加姿态枚举、速度限制、gain
或新 mode。当前没有可消费的独立 referee DEAD/REVIVED event；使用唯一规范输入
`/referee/state` 的有效 HP：**HP=0 为 DEAD；DEAD 后更新且有效的 HP>0 为 REVIVED**。
这是本候选的明确软件策略，不是对未确认 MCU `life` 字节或 heartbeat 的解释。

输入必须 `valid=true`、时间结构合法、非零、年龄在既有 −0.05～0.5 s 窗口内，且源
时间戳严格大于已接受水位（上述为默认年龄窗口）。同 stamp 的重复或冲突包、乱序包不能改变生命状态或续租；
非法未来包不能推进水位。正常比赛组合还要求 `/referee/state_valid` 与其他 Safety
证据。不加 debounce：经 gate 确认的死亡立即生效；重复相同生命状态不重复换代。
失联只使现有证据过期，不构造死亡或复活事件；DEAD 保持锁存直到有效的新 HP>0。

- **DEAD**：在输入回调中撤销 Mission 导航授权，取消 Navigate/Spin，清除旧执行完成/
  失败记忆与 pursuit 候选有效性；保留请求模式和未完成巡逻索引。显式 HOME、低 HP、
  pursuit、patrol、event replan 均不能绕过。取消中的 action 仍占 owner 直到 terminal，
  但已无 Mission authority；不把“尚未终态”错误解释成继续授权。
- **REVIVED**：增加 life generation，重新仲裁当前请求；每个新 Navigate 再取得新的
  goal generation。执行身份是 `(life_generation, goal_generation, localization session)`。
  不提前改旧 goal generation 丢失终态屏障；旧生命代次的成功/失败只能释放旧 handle，
  不能推进 patrol、完成新目标或消耗新预算。Spin 同样检查 life generation。
- **定位条件**：复活时定位有效，仍须通过所有 Safety 门并等旧 terminal 后重建。
  复活时定位无效，保持 HOLD，直到严格更新的定位 session 且状态新鲜有效；同一 session
  的 valid 心跳不足以放行。rebase 可以在 DEAD 中完成 drain/commit/prime，但不自动复活。
- **目标条件**：复活前缓存的 pursuit/temporary commitment 不恢复；须有撤权后新的源
  观测。新鲜目标可以在 DEAD 时作为输入到达，但不能获得导航授权。
- **Safety 关系**：ALIVE 只移除生命否决，不创建最终串口正授权，也不覆盖定位、guard、
  chassis、ESTOP 或操作员 hold。HP>0 仍可能因低血量选择 HOME。

```mermaid
stateDiagram-v2
    [*] --> ALIVE: 启动待有效安全证据
    ALIVE --> DEAD: 接受有效更新 HP=0 / 撤权并取消
    DEAD --> DEAD: 重复死亡 / 失联 / 旧包 / 目标到达
    DEAD --> ALIVE: 接受更新 HP>0 / life generation 增加
    ALIVE --> ALIVE: HP 更新不换生命代次
```

该图与 HOLD/HOME/PATROL/PURSUIT/AUTO 请求模式正交；ALIVE **不等于已授权**。
复活等待新定位会话是一个条件，不引入第二套 REVIVING enum。
`MissionState.detail` 追加 `life_state`、`life_generation`、`mission_generation`（既有
Navigate 请求序号）、`localization_session`、`navigation_authority`，用于观察而非命令。
最后一项仅表示当前 Mission 执行承诺仍获软件授权，不表示 MCU 或车辆实际运动状态。

**输入边界**：RefereeState 没有 producer/session/sequence 字段。gate 保留 ROS 源 stamp，
上述排序保障只适用于同一受信任来源和时钟域。HPM 帧无源序号/源时间；V2 虽有 frame
sequence，当前 referee 接收路径未用它去重且会重新打 host RX stamp。旧线上包若被上游
重新打成新 stamp，本层无法识别。真实传输去重、MCU 重启、referee 延迟与生命字节语义
必须在硬件输入验收中确认；不能用这份规范声称已闭合 wire-level replay 或物理停车。
本轮没有修改这些协议，也没有新增自动切源/时钟复位恢复。

生命语义验证和系统线冻结决定见
[DEAD/REVIVED 最小闭合](../validation/mission_life_authority_20261029.md)。

[受控验证与审计](../validation/event_replanning_port_20261027.md) 给出证据和适用限制。

## Session Contract：定位会话与事件重规划

接受更大的 `LocalizationStatus.started_at` 表示新 session；同一 producer 的 sequence
可在新 epoch 重新从 1 计数。Mission 立即请求取消 NavigateToPose/Spin，清除旧完成与
失败记忆，并等待终态才允许新 session 的目标。旧 session 的结果仅释放旧 action，
即使结果为 SUCCEEDED 也不得推进巡逻或完成目标；计数记录在 MissionState.detail 的
`stale_results_rejected`。旧 epoch 状态仍按现有规则拒绝。

`navigation_policy:=event_driven` 仅在统一仿真中启用。Nav2 内部
`event_driven_replanning.xml` 每个目标先 ComputePathToPose，再 FollowPath；任一步 FAILURE
才进入 1.5 s Wait，然后重算，最多 3 次 recovery（初次 + 3 次尝试）。不清图、不
Spin/BackUp、不轮询 IsPathValid。任务层目标变化由上述取消/终态/替换流程处理，BT 不发
NavigateToPose。仿真 Mission 配置将外层目标尝试限制为 1，耗尽后保持 goal_retry_limit，
直到新的意图或受信任的显式 set_mode 重置预算。

Nav2 自带的 `goal_pose` 订阅具有内部 self-client，不是第二个任务 owner；此 profile
把该订阅重映射至 `/simulation/unused_direct_goal_pose` 并移除 RViz GoalTool。
不允许测试工具、RViz 或 dog-hole manager 同时发目标。仿真探针只提供 Mission 输入。

显式 rebase 还必须等待配置的 Nav2 子 action 全部终止，然后由 bridge 提交 map→odom、
选择性失效 map 动态缓存并 prime，才能恢复有效性；见
[rebase 合同](../../src/rm_relocalization_bridge/README.md#explicit-map-only-rebase-transaction)。
它不重置 Mission 意图，不恢复旧 path/action，也不创建另一套 motion authority。

系统验证范围、两层状态图对应证据及未接入项见
[2026-10-29 系统验证](../validation/competition_mission_arbitration_system_20261029.md)。
