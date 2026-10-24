# Minimal Competition Mission Layer（2027）

Integration 软件候选。Base：`feature/navigation-intent-arbitration-2027@7be5dc62bbada643c9472ec93d9880c0256cd177`。
真实依赖其单 owner、cancel→终态交接、pursuit lease 与 guard 否决。没有合入姿态 feature。

## 最小任务合同

长期 mode 继续使用既有 `hold/home/patrol/pursuit/auto`，不重命名 API。
只有一个显式临时任务槽，没有队列、恢复栈、独立 Supply/Recovery Nav2 owner 或规则状态机。

| Mission | Persistent/Temporary | Priority | Completion | Resume behavior |
| --- | --- | --- | --- | --- |
| HOLD / disable / safety | 显式 hold/disable 持续；安全否决取决于证据 | 0 | 显式 mode/enable 或安全恢复 | 取消导航；恢复后重算当前仍合法的任务 |
| RETURN_HOME | 持续，首次选择后 latch | 1 | Nav2 SUCCEEDED 后在 home 保持 | 不自动回旧 pursuit/temp；显式 SetMissionMode 才释放，低 HP 仍会重新 latch |
| SUPPLY | Temporary | 2 | 配置 supply_pose 的 Nav2 SUCCEEDED | 清槽，重算长期任务；patrol 保留索引 |
| RECOVERY | Temporary | 2 | 配置 recovery_pose 的 Nav2 SUCCEEDED | 同上，不确认回血/增益 |
| SPECIAL_MISSION | Temporary | 2 | 命令 map pose 的 Nav2 SUCCEEDED | 同上，不确认区域/地形奖励 |
| RADAR_PURSUIT | 有效期约束的目标 overlay | 3 | 沿用原 pursuit goal/lease 规则 | stale/lost 时 auto 回 patrol；pursuit 专用模式回 hold |
| PATROL | Persistent | 4 | 每点 SUCCEEDED 后前进/循环 | 抢占时保存索引 |
| Idle | 无运动目标 | 5 | 等待有效任务 | hold |

Supply/recovery/special 没有固定内部高低，最新合法的**不同任务**替换当前临时任务；旧任务不排队，
被替换或 home 抢占的任务不会隐式回来。临时任务完成/取消/过期后重算长期 mode，不存储旧 pursuit pose。
清槽时要求后续 pursuit 使用新的源观测；auto 仍可接受此后的新鲜雷达目标。

低 HP 仍是既有 `current_hp <= retreat_hp_threshold` 条件，输入必须新鲜；没有加入任何弹量/经济规则。
RETURN_HOME 新增持续性：即使途中 HP 恢复也继续返航，到家后保持。显式 `/mission/set_mode`
释放 latch，同时撤销当前临时任务；若低 HP 条件仍成立则下个 tick 再返航。显式 Hold/disable 始终更高。

## 命令边界

新增 `/mission/set_temporary`，类型 `rm_competition_interfaces/srv/SetTemporaryMission`：

| Field | Meaning |
| --- | --- |
| sequence | 非零、单一受信任 issuer 在当前 Mission 进程内单调递增；旧序列拒绝 |
| mission_id | 逻辑任务身份，建议每个新任务使用唯一 ID；重复传输保持相同 ID/内容 |
| kind | supply / recovery / special |
| target | 仅 special 使用；必须有限、平面、单位四元数、frame=map |
| valid_until | ROS clock 绝对期限；必须在未来，剩余时间不超过可配置上限（默认 300 s） |
| cancel | 取消匹配当前 ID 的临时任务；不要求 target/deadline |

Supply/recovery 使用启动配置的 `[x,y,yaw]`，不使用命令中的原始 target 替换配置。
未配置就拒绝该请求，保留现有合法任务，绝不生成原点/home 等替代坐标。
可选 YAML 数组为空时按未配置处理；非空但格式/数值非法时拒绝启动。

服务只改变 Mission 候选，不发 Nav2 action、不设置 enable，也不绕过安全门。
必须已启用且长期 mode 非 hold/home；home latch/低 HP 期间拒绝新的临时任务。
安全暂时 denied 时可接收有期限的新意图，但不发送 motion goal；恢复时仍须未过期。

相同 sequence 的完全相同 payload 是幂等 ACK；不同 payload 拒绝。相同最新逻辑 ID、内容和
原始 deadline 即使以更大 sequence 重传，也不续租、不重发、不重新执行已完成/取消的任务。
修改目标/类型/期限须使用新任务 ID。已接受命令建立 sequence watermark；无效命令不改变当前任务。
进程重启会清空命令/幂等缓存，不恢复持久队列；调用者需要明确重发当前仍有效的任务。

这是受信任本地配置/测试/未来上层策略入口，不是任意队友下发战术目标的入口。正式外部 tactical
数据仍只允许雷达链路；禁止把 teammate/operator raw topic 无门控桥接到此服务。ROS 服务本身不提供身份认证。

## 有效期、抢占与完成

- 入队（唯一槽）同时建立 ROS deadline 和同样剩余时长的 steady deadline。
- 期限到、ROS 时间倒退：清槽；重复消息不能延长原始期限。
- safety loss：取消当前 action，不发替代 goal；槽内任务可在期限内等待。
- 恢复：重算当前槽/长期 mode；已过期任务无条件丢弃。
- home：丢弃临时槽，清旧 pursuit 有效期屏障；不保存恢复栈。
- complete：只有当前匹配 ID/source、未被取消的 Nav2 SUCCEEDED 才完成临时任务。
  到达导航 pose 就算完成，不等弹量、HP、金币或增益变化。
- 失败：沿用已有 backoff/重试上限；到上限保持不发新 goal，等待取消/替换/过期，不伪称完成。
- 所有目标切换复用仲裁 owner 的 pending/active handle 与 terminal-result barrier。
  服务端拒绝取消/不结束时继续等待，不能以并发发目标强迫替换。

两个随包 BT 都在 home 后、pursuit/patrol 前增加 `HasTemporaryMission/SelectTemporaryMission`。
自定义 BT 必须按同一优先级接入这对节点，否则不在本轮验收范围。没有重写 action architecture。
`MissionState.detail` 复用现有字符串报告 kind/id/accepted、completed、canceled、expired、replaced、
preempted_home/mode_changed 等最新状态，以及 home latch；无需新 diagnostics framework。

## 范围边界

没有自动 supply/recovery trigger、经济策略、姿态策略、语义地图/geofence、地形奖励确认或规则引擎。
补给/恢复点需要以后在实际地图中审核；本轮 safe config 保持空。定位、TF、posture、planner、
controller、serial 与 MCU 授权都未改。实际雷达入口、正准入 owner/session 和物理停车仍属于后续实车集成。
