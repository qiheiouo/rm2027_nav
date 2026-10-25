# 2027 Navigation Target Arbitration Contract

状态：Integration 软件候选；不授予硬件运动权限。适用于自动哨兵。

## Owner 与优先级

`competition_mission_node` 复用现有 BT，每 tick 只产生一个 `desired_goal_`，并拥有
唯一任务级 `NavigateToPose` 客户端。pursuit 只产生候选，referee/serial 不发导航目标。
Nav2 自己的 BT 调用 controller 的 `FollowPath`；本仓任务层不直接调用它。

默认树的最终顺序：

1. **SAFETY_STOP**：Mission disable、显式 `hold`、任一必需安全证据失效、底盘撤权/ESTOP，
   或已接入的 V2 guard 非 PERMITTED/过期。取消导航和 Spin，不产生替代目标。
2. **RETURN_HOME**：新鲜有效 referee HP `<= retreat_hp_threshold` 或显式 `home`。
   未配置 home 时 `hold_missing_home`，不得降级为 pursuit/patrol。
3. **EXPLICIT_MISSION**：未过期的 supply/recovery/special，三者同级，最新合法任务替换。
4. **PURSUIT**：`allow_pursuit=true`，模式为 `auto|pursuit`，且候选有效。
5. **PATROL**：模式为 `auto|patrol`，且有 waypoint。沿用 `patrol_index_`。
6. **IDLE/HOLD**：没有可选目标。

最小比赛 Mission 扩展新增了一个显式临时槽和受信任 `/mission/set_temporary` 入口，
详见[任务合同](minimal_competition_mission_2027.md)。RETURN_HOME 首次选择后持续到显式
SetMissionMode 释放，到达后保持；不会自动回旧 pursuit 或临时任务。
`SetMissionMode`、显式任务命令和 YAML 都不是任意队友 tactical 输入。
显式 `patrol` 继续禁止 pursuit；需要“平时巡逻、雷达出现就追踪”时使用 `auto`。

## Action 交接

- 同一时刻最多有一个 pending/active 的 NavigateToPose 或 Spin。
- 分支或目标身份变化：立即请求取消；等待旧 action **终态 result**，再按最新有效决策发送。
  Cancel ACK 只是 CANCELING，不能据此交出 owner。
- pending 请求尚未返回 handle 时撤权：记录取消意图；迟到的 accepted handle 一到就取消。
- 等待取消期间的成功 result 不推进 patrol 索引，也不标记旧战术目标完成。
- 取消被拒绝或服务端不结束：保持等待，不并发发送替代目标。服务端执行取消及实际停车
  仍是下游合同；本层无法用重复发 goal 强迫服务端停车。
- patrol 被抢占后保留索引，只有未被撤销的成功到达才推进。Spin 同样必须结束后才允许 home。
- 安全恢复后重新计算当前 mission；pursuit 要求原始采样时间晚于最后一个 hold/撤权 tick，
  不复活撤权前缓存的 tactical target。

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
时间字段先检查格式与采样年龄，再更新 watermark；非法或远未来输入不能终止 owner，
也不能阻塞随后合法样本。被忽略的输入不续租已有候选。

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
- `old_car_2026_competition.launch.py` 也固定 radar-only；它仍属于 legacy/HPM 调试组合，
  不冒充接入 V2 final guard 的 2027 正式硬件 profile。
- 通用 `pursuit.launch.py` 保留 `radar_only=false` 的感知/研究输入。研究 Mission 必须显式
  选择 `radar_only=false`；`competition_no_hardware_test` 已作此选择，保留 mock 仿真用途。
- `/operator/navigation_target_raw` 仍是无 consumer 的原始诊断数据，不进入比赛仲裁。
- `/mission/set_mode` 由受信任的本地配置/调试工具调用。禁止桥接为队友战术命令。

来源标签和 topic 隔离是受信任 ROS 图内的策略边界，不是 DDS 身份认证。实际雷达链路
解码/坐标适配尚未实现；不得把任意队友数据桥接到 radar topic，部署时须限制 publisher。

## 安全证据与运行边界

Mission 保留既有 LocalizationStatus 合同；对 referee/chassis/V2 guard 增加独立 0.5 s
采样及接收 lease，防止上游 gate 退出后缓存状态无限存活。看到任何 V2 guard 后，即使
旧调试配置未要求它，也不能忽略它的拒绝或失联。

`V2MotionGuardState` 是最终编码边界的只读观测；消费 PERMITTED 不创建 owner/session，
也不替代 AUD-003 的租约或最终逐帧检查。现有 guard 在 BLOCKED 后仍需重启并重新授权。
本轮没有连接真实 Mission 到该正准入租约，测试中的 guard 发布者只是受控 fixture。
2027 导航组合只启动 Mission/planner，不启动 Nav2、serial、定位、referee 或物理设备；缺少
外部安全证据时保持停止。不得与 dog-hole manager、手动目标工具或另一 Mission 同域共同控 Nav2。

[受控验证与审计](../validation/navigation_target_arbitration_20261024.md) 给出证据和适用限制。

## Localization session handoff（staging 修正，2026-10-25）

每个 pending/active NavigateToPose 和 Spin 都属于发送时已接纳的定位上下文：
`(LocalizationStatus.producer_id, session_id, session_epoch)`，并捕获 Mission 内单调递增的
localization generation。`started_at/sequence/emission/deadline` 的原有准入仍先执行；
legacy `session_id="", session_epoch=0` 继续有效。它不是物理运动 owner/session 授权。

- 接纳新的 status producer、session_id 或更大的 session_epoch 时，在该 status callback 内
  立即撤销旧导航/Spin，清 desired/completed/failed、pending patrol spin 和缓存 pursuit；
  不等待下一 tick，不依赖 invalid 心跳是否被 depth-1 consumer 看见。
- 同一 producer/session 的 epoch 回退、退役 session_id 和 versioned→legacy 降级不重新准入。
  新 status producer 仍受原 started_at 身份门约束；会话身份不能以 UUID 字典顺序比较。
- 撤销保留旧 pending/active handle 和现有 action generation；不因换代直接丢掉请求或
  增加第二 owner。迟到 acceptance 必须 cancel；cancel ACK/拒绝都不能释放屏障，
  只能由 rejection/terminal result 结束原请求。
- result callback 同时检查 action generation 与发送时 localization generation。旧会话
  result 即使 SUCCEEDED 也只清旧 handle/取消状态，不推进 patrol index、不触发 Spin、
  不标记临时任务/home 完成，不写回新会话 completed/failed 状态。
- localization LOST、invalid 或 lease expiry 同样撤销当前 action；恢复只使用重新接纳的
  有效 status 和现有安全门。重复 invalid 不产生新授权。快速连续换代只合并最新上下文，
  仍等旧终态后再按原有优先级重新计算。
- 既有 map-frame home/patrol/显式任务是持久意图，换代不改变策略/坐标配置；旧 action 和
  执行结果被退休后，才可在最新定位会话重新发出。pursuit 必须是撤销及恢复 COMMITTED 水位之后的新观测；invalid 期间的缓存不能跨恢复边界。
  同 epoch 普通 relocalization correction 仍按现有定位合同处理，不新增战术行为。

这些规则只闭合软件导航消费端，不授予 AUD-003/MCU 权限，不保证物理停车 SLA。
