# RM2027 STM ↔ Host 协议统一参考（源码审计版）

状态：**Host contract frozen / STM implementation pending / Hardware Go = NO**

适用分支：`integration/demo-navigation-validation`

基线起点：`de05e0858de078f5e035757f0cc8d48b63f34c52`

本文是本分支 STM ↔ 上位机协议的统一入口。它区分三种事实：

1. `Current source fact`：当前仓库实际编解码、transport、测试所做的事情；
2. `Host-frozen extension`：主机端已实现并有无硬件测试，但尚未由 STM 固件联签/实测；
3. `Hardware pending`：必须取得固件 revision、原始抓包和真机证据后才能通过。

`0x86 ChassisAttitudeState` 属于第 2 类。本文不能作为 STM 已实现或 Hardware Go 的证据。

## 1. 审计范围与权威来源

按“源码优先于旧文档”审计了以下实际入口：

| 责任 | 当前事实入口 |
| --- | --- |
| V2 C++ wire codec | `src/rm_serial_driver/{include,src}/competition_v2_protocol.*` |
| V2 C11 固件参考 codec | `src/rm_serial_driver/reference/competition_v2/competition_v2_codec.{h,c}` |
| V2 host transport | `src/rm_serial_driver/src/competition_v2_transport_node.cpp` |
| V2 无硬件 STM fixture | `src/rm_serial_driver/src/competition_v2_mock_lower_node.cpp` |
| legacy/HPM codec | `src/rm_serial_driver/{include/rm_serial_driver/protocol.hpp,src/protocol.cpp}` |
| legacy/HPM transport | `src/rm_serial_driver/src/serial_transport_node.cpp` |
| ROS interfaces | `src/rm_competition_interfaces/msg/` |
| codec/runtime/ACQ tests | `src/rm_serial_driver/test/` |
| Demo safety/bench/Phase 1 | `src/rm_demo_platform/`、`docs/operations/demo_phase1_checklist.md` |

审计没有在当前运行源码中发现另一套 V2 type registry、Radar 专用 V2 消息、Mission 串口命令，
也没有发现与 `0x86` 冲突的现有 V2 type。legacy/HPM 是独立 envelope，不共享 V2 type 空间。
历史研究文档、旧车抓包和候选分支不能覆盖上述当前源码事实。

## 2. V2 frame envelope

所有多字节整数为 unsigned little-endian；float 为 IEEE-754 binary32 little-endian。
不发送 C/C++ struct 内存布局。最大 payload 128 bytes。

| Offset | Size | Field | Rule |
| ---: | ---: | --- | --- |
| 0 | 1 | magic 0 | `0x52` (`R`) |
| 1 | 1 | magic 1 | `0x4d` (`M`) |
| 2 | 1 | version | `2` |
| 3 | 1 | message type | 见 type registry |
| 4 | 2 | payload length | `0..128` |
| 6 | 2 | frame sequence | uint16，发送方按 message type 独立递增 |
| 8 | N | payload | 类型固定布局 |
| 8+N | 2 | CRC | CRC16/Modbus，低字节先发 |

CRC 参数：poly `0xA001`、init `0xFFFF`，覆盖 `version` 到 payload 最后一个字节，
不覆盖两字节 magic。V2 CRC 与历史 HPM 的 payload-only CRC envelope 不兼容。

stream decoder 在版本、长度或 CRC 错误时丢弃一个字节并重新扫描相邻 `52 4d`；未知但 CRC
正确的 type 被完整消费并计数。host buffer 上限 4096 bytes。协议选择必须显式，禁止 malformed
V2 后自动回退 legacy。

## 3. V2 type 与 capability registry

| Type | Direction | Name | Payload | Capability |
| ---: | --- | --- | ---: | ---: |
| `0x01` | host → STM | ChassisCommand | 13 | bit 0 (`0x00000001`) |
| `0x02` | host → STM | PostureRequest | 11 | bit 1 (`0x00000002`) |
| `0x03` | both | Heartbeat | 13 | n/a |
| `0x81` | STM → host | PostureState | 13 | bit 1 |
| `0x82` | STM → host | GimbalState | 17 | bit 2 (`0x00000004`) |
| `0x83` | STM → host | RefereeState | 17 | bit 3 (`0x00000008`) |
| `0x84` | STM → host | OperatorNavigationTarget | 25 | bit 4 (`0x00000010`) |
| `0x85` | STM → host | ChassisHeadingState | 19 | bit 5 (`0x00000020`) |
| `0x86` | STM → host | ChassisAttitudeState | 55 | bit 6 (`0x00000040`) |

`0x86` 与 capability bit 6 是本分支的 **host-frozen recommendation**。STM 团队确认 type、
bit、字段语义和 golden vector 后才成为双方合同。不得用“最新空闲 type”自行替换；若固件发现
真实冲突，停止集成并共同修订 registry、codec、测试和本文。

## 4. 现有消息逐项定义

### 4.1 ChassisCommand `0x01`

| Offset | Size | Field |
| ---: | ---: | --- |
| 0 | 4 | float32 `vx_mps`，base +x 前 |
| 4 | 4 | float32 `vy_mps`，base +y 左 |
| 8 | 4 | float32 `wz_rad_s`，绕 +z 右手正向 |
| 12 | 1 | flags：bit0 enabled，bit1..7 reserved |

host timer 默认 100 Hz；命令 freshness 默认 0.2 s，均不是 MCU watchdog 承诺。production
transport 不订阅通用 `/cmd_vel`，没有正向 motion owner，持续发送 zero-disabled。
`motion_test_mode` 只能与 `dry_run=true` 同用。STM 独立 watchdog、停车时间和 re-arm 未验收。

### 4.2 PostureRequest `0x02`

`uint32 command_id`、`uint32 requester_boot_id`、`uint8 posture`、valid bit、`request_flags`，
共 11 bytes。六个 posture 值为 0..5；重复相同 tuple 是 desired-state 重发，必须幂等。
相同 requester boot + command ID 不得表达不同 posture。该执行器路径独立于底盘 motion guard。

### 4.3 Heartbeat `0x03`

| Offset | Size | Field |
| ---: | ---: | --- |
| 0 | 4 | uint32 `uptime_ms` |
| 4 | 4 | uint32 `boot_id`，必须非零 |
| 8 | 4 | uint32 capabilities |
| 12 | 1 | bit0 ready；bit1..7 reserved |

host 当前拒绝 zero boot、同一 boot 的 duplicate/out-of-order frame sequence、模半范围意义下
倒退的 uptime，以及本进程已退役 boot 的 heartbeat 重放。接受新 boot 后立即令 posture、gimbal、
heading、attitude 失效，并撤销已许可的 dry-run motion session。

这不是认证握手：host 刚启动时无法仅凭 wire 数据证明第一帧不是旧缓存；进程只保留最近 8 个
已退役 boot。STM 仍必须保证上电生成不复用的非零 boot、清空 TX 队列和旧授权。

### 4.4 PostureState `0x81`

13 bytes：ack command/requester boot、target、actual、valid/online/transitioning/fault、fault code。
ACK 不等于完成；host 还要求 ack tuple 与当前 request 相符、actual 达到、valid+online、
非 transitioning、无 fault。

### 4.5 GimbalState `0x82`

17 bytes：relative yaw、yaw rate、sample sequence、MCU time、valid/online。yaw 是机械相对底盘角，
不是 world heading。当前 wire **没有每样本 boot/reset**；host 把最新 heartbeat boot 写入 ACQ
关联，但不能证明延迟旧包属于该 boot。当前 host 也未对 gimbal sample sequence 做 replay window。
这是实机前待闭合限制。

### 4.6 RefereeState `0x83`

17 bytes：game progress、stage remaining、robot ID、current HP、红/蓝 outpost HP、17 mm allowance、
gold coin、diagnostic flags、valid。host 根据 robot ID 归一化 self/enemy；diagnostic bit 语义仍待
电控联签。它是状态，不是 mission owner。

### 4.7 OperatorNavigationTarget `0x84`

25 bytes：command ID、coordinate frame、alliance、valid/has-yaw、x/y/yaw、sample time、TTL。
`(0,0)` 是合法坐标，不表示无命令。transport 只发布 raw topic；未经过 frame/alliance/TTL、地图
边界和 mission authority gate 时，不能生成 Nav2 goal。当前没有独立 Radar V2 target 消息。

### 4.8 ChassisHeadingState `0x85`

19 bytes：world/local-initial yaw、yaw rate、sample sequence、MCU time、reset counter、valid/online。
它只包含 yaw，不是 full attitude，也不是 gimbal relative yaw。wire 没有 source boot；ROS 消息中的
boot 是 host 关联的最新 heartbeat。当前 host 未对 heading sample sequence/reset 做完整 replay
window；只有 stale/heartbeat reboot 失效。

## 5. ChassisAttitudeState `0x86`（host-frozen）

### 5.1 Wire layout

| Offset | Size | Field | Unit / semantic |
| ---: | ---: | --- | --- |
| 0 | 4 | `orientation_x` | quaternion x |
| 4 | 4 | `orientation_y` | quaternion y |
| 8 | 4 | `orientation_z` | quaternion z |
| 12 | 4 | `orientation_w` | quaternion w |
| 16 | 4 | `angular_velocity_x` | rad/s |
| 20 | 4 | `angular_velocity_y` | rad/s |
| 24 | 4 | `angular_velocity_z` | rad/s |
| 28 | 4 | `linear_acceleration_x` | m/s² |
| 32 | 4 | `linear_acceleration_y` | m/s² |
| 36 | 4 | `linear_acceleration_z` | m/s² |
| 40 | 4 | `sample_sequence` | uint32，每个真实新样本递增 |
| 44 | 4 | `mcu_time_ms` | uint32，采样时刻，与 heartbeat uptime 同一时钟域 |
| 48 | 4 | `source_boot_id` | uint32，必须非零且等于当前 heartbeat boot |
| 52 | 2 | `reset_counter` | uint16，估计器 reinit/re-zero 时递增 |
| 54 | 1 | flags | bit0 orientation valid；bit1 gyro valid；bit2 acc valid；bit3 online；bit4..7 reserved=0 |

payload 55 bytes，完整 frame 65 bytes。所有十个 float 必须 finite。orientation valid 时四元数
范数须满足 `abs(norm-1) <= 1e-3`；host 发布标准 Imu 前会归一化，ACQ 保留 raw quaternion。

### 5.2 坐标与物理语义

- 四元数字段顺序固定 `x,y,z,w`，表示 `R_reference_from_chassis_imu`，把 IMU frame 向量旋转到
  STM 的 reference frame；不是逆旋转。
- reference 是 STM 定义的本地重力对齐/启动世界系，**不自动等于** ROS `map` 或 `odom`；yaw
  零点、重力对齐算法和 reset 行为须由固件说明。
- gyro 与 acceleration 均表达在 `chassis_imu_link`，右手系。
- acceleration 固定为 accelerometer specific force，保留重力响应；静止、传感器 +z 向上时约
  `+9.81 m/s²`。禁止一端去重力、另一端按 specific force 解释。
- 当前合同不声明 bias compensation、标定质量或协方差。标准 Imu covariance 全零表示 unknown，
  不能解释成零噪声/高可信。
- `T_base_from_chassis_imu` 必须来自实测静态 TF；host transport 不旋转数据、不发布 TF，也不把
  该 IMU 自动融合为 canonical odometry。

### 5.3 Host publish contract

标准数据：`/chassis/imu`，`sensor_msgs/msg/Imu`，frame_id=`chassis_imu_link`。

生命周期：`/chassis/attitude_status`，`rm_competition_interfaces/msg/ChassisAttitudeStatus`。

只有同时满足下列条件才发布一条 Imu：

1. 已有 nonzero、ready、未超时 heartbeat 和 capability bit 6；
2. `source_boot_id` 等于当前 heartbeat boot；
3. online、orientation/gyro/acc 三个 valid 位全部为真；
4. quaternion 和全部标量通过 codec 检查；
5. 同一 boot/reset 内 sample sequence 模半范围严格更新；
6. MCU time 不倒退（相同 ms 允许，uint32 wrap 按模半范围处理）；
7. reset 改变时，新 counter 必须按 uint16 模半范围更新。

新 boot 或接受的新 reset 会开始新的 `lifecycle_epoch`；seq/tick 可从新值重新开始。duplicate、
乱序、旧 reset、boot mismatch、断流、offline 或 invalid 均不发布零值/旧值 Imu，只发布 invalid
status。默认 attitude stale timeout 为 0.2 s，现场 manifest 必须显式填写确认值。

frame sequence 与 sample sequence 是两个不同计数器：frame sequence 属于 envelope/发送行为；
sample sequence 属于传感器采样生命周期。`0x86` host replay gate 使用 sample sequence，不要求两者
相等，也不靠 packet sequence 替代 source sample identity。

## 6. 时间与 ACQ provenance

当前 host 映射为：

```text
offset = heartbeat_receive_ros_ns - heartbeat_uptime_ms * 1e6
sample_ros_ns = offset + sample_mcu_time_ms * 1e6
```

这只是最近 heartbeat 的单点 offset，不是 PTP、时钟驯服或延迟补偿。ACQ 可记录 handler 的
ROS/system/steady 时间、真实串口 read batch 完成时间、heartbeat basis、raw sample、boot/reset、
output header 和 publish API 返回后的 ROS/steady 观察时间。publish 观察时间不是 DDS 线速发送时刻。
mock RX 没有 read-batch timestamp。host clock step、USB 排队、tick wrap 必须按 epoch 分析。

## 7. Legacy / HPM profiles（独立协议，不占 V2 type）

| Profile | Envelope | Command/feedback | 已知限制 |
| --- | --- | --- | --- |
| `legacy_v1_no_crc` | `0x3e`, uint8 length, payload | 17-byte chassis/strategy command；完整 19 bytes | 无 CRC、无 type、无 seq/time/boot |
| `hpm_crc_v1` | 同 header/length，payload 后 CRC16/Modbus low-byte first | 同 17-byte command；完整 21 bytes | CRC 只覆盖 payload，不兼容 V2 CRC |
| `hpm_referee_v1` | 使用 HPM CRC frame | 41-byte feedback payload；完整 45 bytes | 无 type ID、source time、seq、valid lifecycle |

HPM feedback 当前可解出 gimbal yaw、target x/y、referee 数值、sentry info、keyboard command、
target distance、life、chassis error、redundancy。target 坐标缺少已确认单位/原点/alliance/new-command，
只能发布 untrusted raw；不能把它当作 V2 `0x84` 或直接迁为 mission goal。历史协议没有完整、
带 boot/reset/valid 的 chassis attitude 合同，也没有四轮独立 encoder 的已确认布局。

## 8. Safety 与 owner 边界

- V2 production transport 没有正向 motion admission；通用 `/cmd_vel` 不被订阅。
- dry-run test lease 不能打开物理串口；hardware launch 默认 zero nodes，需显式 IO/session 条件。
- heartbeat ready/capability、CRC 正确和 online 都不是 motion authorization。
- posture 是独立执行器路径，必须在 Phase 1 使用隔离 domain、物理禁能和专用禁用 topic。
- serial transport 不发布 TF、odometry 或 navigation goal。
- MID360 内部 IMU 与 chassis IMU 是不同 source/frame，不得合并或只改 frame_id。

## 9. 源码—旧文档差异审计

| 项目 | 审计发现 | 当前处理 |
| --- | --- | --- |
| frame sequence | 旧 V2 文档写“per-type sequence”，发送 fixture 确实分 type；但旧 host handlers 普遍不做 replay 检查 | 本次对 heartbeat 严格检查；`0x86` 用独立 sample seq；gimbal/heading 仍是 limitation |
| heartbeat boot nonzero | 文档要求 nonzero，基线 host 过去接受 zero | 本次 host 拒绝 zero，并测试 retired boot replay |
| output publish timestamp | 基线 ACQ 有 handler/header/basis，但没有 output publish 观察时刻 | 本次增加 ROS/steady publish-return timestamp |
| heading boot | ROS 消息带 boot，wire `0x85` 不带 boot | 明确标为 heartbeat 关联值，不宣称 per-sample binding |
| chassis attitude | 共同合同只有字段提案，wire/host/codec 均无实现 | 本次冻结 `0x86`、C++/C11 codec、Imu/status adapter 和生命周期测试；STM 仍 pending |
| Radar/mission | 历史说明易把 raw target 与导航目标混用 | 当前源码只有 `0x84` raw target 和 legacy raw target；无 Radar 专用 V2、无 serial mission owner |
| watchdog/stop | host 默认 0.2 s 容易被误写成 MCU watchdog | 继续明确为 host software default；实测停车与 re-arm 仍 blocked |

## 10. Golden vector：`0x86`

输入：frame seq `0x3344`；q=`[0,0,0,1]`；gyro=`[0.125,-0.25,0.5]`；
acc=`[0,0,9.80665]`；sample seq=`0x01020304`；tick=`1000`；
boot=`0x20270001`；reset=`2`；flags=`0x0f`。

```text
52 4d 02 86 37 00 44 33
00 00 00 00 00 00 00 00 00 00 00 00 00 00 80 3f
00 00 00 3e 00 00 80 be 00 00 00 3f
00 00 00 00 00 00 00 00 0a e8 1c 41
04 03 02 01 e8 03 00 00 01 00 27 20 02 00 0f
9d 96
```

最后两字节 `9d 96` 是 low-byte-first CRC `0x969d`。canonical C++ 与独立 C11 reference
测试都必须逐字节匹配该向量。STM 侧不要复制 host struct；使用参考 codec 或按 offset 写入。

## 11. STM 联签和 Phase 1A 前置

STM firmware 状态仍为 **Pending**。进入真机 Phase 1A 前必须提供：

1. firmware repository、commit/version、编译配置和实际 transport profile；
2. `0x86` type/capability/字段语义联签，C11 golden vector 在 MCU toolchain 通过；
3. 冷启动、reset/re-zero、uint16/uint32 wrap、duplicate/out-of-order/延迟旧包测试；
4. q 方向、reference/yaw zero、重力符号、gyro/acc frame、bias/滤波/饱和说明；
5. 实测 sample rate、允许 jitter/latency/stale threshold、串口带宽；
6. 原始正常/损坏/截断抓包与 boot/reset 事件证据；
7. `T_base_from_chassis_imu` 实测标定 revision；
8. 电机物理禁能，只接收状态，不发送任何运动授权。

Host software 可以进入 mock/bench 和接线前准备；没有上述证据时，Phase 1A 只能标为
`Ready with Limitations`，不能整体 PASS，更不能授予 Hardware Go。
