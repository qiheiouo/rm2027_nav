# Minimal localization raw acquisition

本 integration 分支只增加诊断记录。Instrumentation Go 不等于 READY_TO_COLLECT。
下一关是 Hardware Acquisition Preconditions Check：实际 DOF、独立机械角、实测外参/home、
heading 物理语义、source-clock/synchronization error evidence、目标设备录制连续性。
本入口不启动硬件、不授权运动，不要求 FAST-LIO、Z、Nav2 或完整整车 runtime。

## Task 1 冻结状态（2026-10-23）

**No real new-car hardware currently exists.** 当前无真实新车底盘、正式安装的 MID360、
真实 gimbal、可用 MCU/chassis、真实 serial/runtime，无法执行硬件前提检查。

```yaml
Task: Task 1 Localization Backend
State: FROZEN_WAIT_HARDWARE_DATA
Software baseline:
  main: d37f8d6e5ebe53533b7fc62aca8bb84392bee31b
Acquisition instrumentation:
  status: INSTRUMENTATION_GO
  rm_commit: 78cc03e9db0574d519f457e1939015b25aa41c32
  livox_commit: 24e989f43afd918f333ea20d526ed7ecc6615bb5
Acquisition readiness audit:
  status: MINOR_INSTRUMENTATION_REQUIRED
Real data admission:
  status: DATA_BLOCKED
Hardware readiness:
  status: HARDWARE_BLOCKED
Z candidate:
  status: FROZEN_WAIT_HARDWARE_DATA
```

上述 readiness audit 是上一轮静态审计结论；instrumentation 已软件验收，尚未取得实车
证据。冻结分支仅保存候选成果，不合入 production main。没有 hardware sanity、目标设备
recording、actual clock evidence 和 actual mechanical contract，不能升为 READY_TO_COLLECT。

真实新车硬件具备后的第一项任务固定为 **Hardware Acquisition Preconditions Check**：
确认 actual mechanical DOF、independent gimbal mechanical angle、chassis heading
semantics、real extrinsics、home/startup、Livox source clock、MCU source clock、
cross-source timing evidence、provenance hardware sanity 和 target-device recording
continuity。任何前提不通过即停止，不得直接采正式数据。

之后严格按以下顺序，不能跳级：

```text
Hardware Acquisition Preconditions Check
→ READY_TO_COLLECT
→ Session A — Prior Raw Acquisition
→ Session A integrity check
→ Session B — Independent Query Raw Acquisition
→ Data Admission
→ Task 1 Real Localization Baseline Replay
```

冻结期间不以 simulation、synthetic data 或旧车硬件替代。除非用户明确改变研究优先级，
不自动推进 ZLIO、loop closure、G-PROBE、UWB、body calibration、localization fail-closed
port、Task 2–5、更多 synthetic Z/Gazebo 定位验证、新 backend/factor graph 或历史 AUD
清理。DATA_BLOCKED 是合法结论，不需要制造其他软件工作填补等待期。下文采集操作只保留
为未来解冻入口，当前不执行。

## 启用和 session 绑定

两个 producer 默认 `enable_timestamp_provenance=false`。实采前显式启用，并将相同的
非空 `provenance_session_id` 传给 Livox 和 V2，且与 `session.yaml` 一致。每次真实
prior/query 使用新 ID。Livox 还需要 `provenance_device_identity`（实际传感器身份），
bridge 以 side 填 `provenance_sensor_role`。身份字符串限制 1..128 字节。

`single_mid360_driver.launch.py` 已透传参数；选 side=left 时 diagnostics remap 为
`/livox/left/timestamp_provenance`，xfer_format 必须为 1 或 4。
`serial_transport.launch.py` 显式选 `protocol_profile=competition_v2`，并透传上述开关和
session ID。capability 至少包含 gimbal(4) 和 heading(32)，mask=36（另要求 command
能力时为37）。此值不授予运动权限。真实 V2 仍发送 heartbeat/零禁用命令，不是被动嗅探器。

在已有独立硬件权限下选择传感器/V2 子组件。机械状态使用原始 `/gimbal/state`、
`use_input=true` 和唯一 RSP/实测模型；wrapper 对 use_input 的默认覆盖必须明确处理。
禁止 mock、零角、派生 yaw 或 identity map 来凑齐证据。本轮没有新增 hardware launch。

## 最小配置与未来 record 命令

安装目录 `rm_navigation_bringup/config/localization_acquisition/` 含 `topics.yaml`、
`qos.yaml`、`session.template.yaml`。新建本次唯一的绝对目录 `ACQ_RUN`，复制后两个文件，
填写 session ID/role 和软件/运行 metadata。`bag/` 必须尚不存在。记录实际 RM HEAD、
Livox HEAD（以及任何有意本地 diff）、ROS distro/RMW/domain、真实 launch 命令、
生效参数/config 快照和 topic/type/QoS/publisher inventory。HARDWARE_DEPENDENT 不填名义值。

下列仅为未来命令；本次不执行采集。前八项为固定 raw/geometry 白名单，后两项为新增证据：

```bash
: "${ACQ_RUN:?Set the absolute directory for this unique prior or query session}"
ros2 bag record -s sqlite3 -o "$ACQ_RUN/bag" \
  --qos-profile-overrides-path "$ACQ_RUN/qos.yaml" \
  /livox/left/lidar \
  /livox/lio_imu_raw \
  /gimbal/state \
  /chassis/heading \
  /serial/connection_state \
  /joint_states \
  /tf \
  /tf_static \
  /livox/left/timestamp_provenance \
  /serial/v2/timestamp_provenance
```

不使用 `-a`。`/tf_static` 为 reliable/transient_local，晚加入 recorder 仍可接收存活
RSP 的 latched TF。两个 diagnostics 为有界 best_effort/volatile，录制 QoS 与其匹配；
它们可能丢消息，必须在有效 startup 前完成 discovery。丢失关联、incomplete、sequence gap
和 recorder 告警必须检查，不能静默补时间或把缺失证据视为成功。保存全 bag 分片、metadata、
真实标定/model、startup/home、clock/boot/reset 附件。后续 replay 需过滤旧 canonical
TF/odom，避免重复 owner；这里不执行 replay。

## Livox 关联合同

每次实际 CustomMsg/IMU publish 后产生一个 `TimestampProvenance`。包含 session、随机
driver instance、配置的 device identity/role、resolved output topic、原 header、
每 topic output_sequence 和内容指纹。每包记录随同一条点云/IMU 队列流转，不以时间窗口猜
packet membership；点云记录 first_point/count span，IMU 一条记录。原始语义消息不增加 ID。

packet 字段包括 packet_id、device_handle、原始8字节 timestamp、time_type/data_type、
实际 selected timestamp、host_fallback、callback system/steady ns。false fallback
仅指旧 PTP/GPS 分支选择 packet stamp；true 指旧 high_resolution_clock fallback。
这不是同步锁定/准确度认证。callback 时刻不是 NIC/wire 到达时刻。
`publish_ros_ns/publish_steady_ns` 是输出 publish 后构造 diagnostic 的观察时刻，
不是精确 DDS 发出时刻。身份标签须用现场 serial/handle 映射核验，不能把 handle 当永久序列号。

每 cloud 最多512个 packet metadata，超出标 complete=false，原始点不截断；未捕获源记录
同样标 incomplete。完整关联指纹最多处理100000点；更大的 cloud 标 incomplete、指纹为0，
不截断原始点。不开诊断时无新 publisher，也不采 callback clocks。没有 UDP payload 副本。

指纹格式以 Livox `src/src/output_fingerprint.h` 为准：64位 FNV-1a，小端 scalar bytes；
header sec/nanosec/frame UTF-8+NUL；CustomMsg timebase/count/lidar_id 及每点
offset/XYZ/reflectivity/tag/line；IMU 六个 gyro/acc float64。只作内容关联，不是安全哈希。
离线在同 session/producer 内用 topic/header/fingerprint 和 per-topic order 关联。
要求 key 唯一，或重复 occurrence 的两个流均完整且顺序确定；丢消息/重启/碰撞导致多重匹配时
拒绝该窗口，不猜一个候选。output_sequence 在 topic 间独立，driver instance 重启必换。

## V2 关联合同

`V2TimestampProvenance` 覆盖 heartbeat、heading、gimbal、payload decode failure 及
transport 生成的 invalid heading/gimbal marker。包含 session/transport instance、
frame/sample sequence、原 MCU tick/reset/机械状态、实际 offset revision/value、该
revision 的 heartbeat frame/uptime/boot/ROS clock read，以及 handler ROS/system/steady
时刻、原输出 topic/header/sequence、有效位与原因。每个 sample 自带所用 heartbeat basis
快照，丢失 heartbeat diagnostic 时仍保留依据，但仍需报告丢消息和缺失关联。

```text
offset_ns = offset_heartbeat_ros_ns - offset_heartbeat_uptime_ms * 1,000,000
source_sample_header_ns = offset_ns + mcu_time_ms * 1,000,000
```

revision 在 transport_instance 内关联 heartbeat，输出按 topic/header/sample seq/tick、
状态及可用的 boot/reset 字段匹配；必要的顺序匹配要求完整流。revision0 表示无 heartbeat
basis，保留旧 now()/valid=false 语义。invalid marker 无源 sample/frame，不解释为物理零值。
真实 RX 的 `read_batch_completion` 为 polling read 批次结束，可能包含缓存/多个 frame；
mock RX 明确无该时间。handler/read completion 都不是 physical arrival。

boot 仅为最新 heartbeat 的关联值，不是每个 wire sample 的身份认证。记录不纠正旧队列
跨 boot、MCU wrap、host clock step、sensor sync 或 heading 物理参考。真实误差测量仍是
硬件前提；header 一致/full-base 的1ms research skew policy 不证明物理同步精度。

Prior/query 必须独立物理采集及 startup/home；可共享不可变标定/软件和 A-only prior，
不可共享 KF/局部地图/cache，不从 A 裁剪 B。本入口不推进 Task 1 backend。

## 本轮软件验证（2026-10-23）

Decision: `INSTRUMENTATION_GO`；验证范围为 Integration 的无硬件输入/输出合同。
这不提升上述 Hardware Acquisition Preconditions，也不代表 READY_TO_COLLECT。

复用已有 `rm2027_navigation:humble` 镜像，以 `--network none`、无宿主设备、源码只读
的临时容器运行；build/log/results 只写临时验证目录。使用 Debug（断言启用），
`CMAKE_BUILD_PARALLEL_LEVEL=2`。只构建以下三个包，没有 FAST 或全 workspace build：

```text
colcon build --base-paths <src>/rm_competition_interfaces <src>/rm_serial_driver <src>/livox_ros_driver2_humble/src
  --packages-select rm_competition_interfaces rm_serial_driver livox_ros_driver2
  --executor sequential --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Debug
```

| 实际检查 | 结果 |
|---|---|
| 三个受影响包局部编译 | PASS |
| Livox timestamp_provenance fixture | PASS：实际 callback→聚合→队列→ROS 输出；PTP/GPS/fallback、点跨度、IMU、512上限；开/关诊断正常输出一致 |
| Livox 原 lifecycle/publication/queue_concurrency 三个 fixture | PASS |
| V2 timestamp_provenance pytest | PASS：真实 dry-run decoder/handler；无 heartbeat、两次 offset 更新、boot/reset/invalid/decode failure；离线重建关联；开/关诊断全部正常输出字段及 header 一致 |
| 原 protocol / competition_v2_protocol / competition_v2_reference 三个 gtest target | PASS |
| 10-topic 名称/消息类型、QoS YAML、metadata 模板、launch AST/默认关闭 | PASS |
| record 命令 bash 语法、白名单一致性、Humble CLI 参数 | PASS；仅解析与 `--help`，没有实际 record |
| 原时间选择/消息构造/MCU header 估计函数静态对照 | PASS：未更改原算法/测量值/时间公式 |

一共通过8个直接相关 CTest target；不声称运行了全包 lint 或依赖其他 localization
节点的 runtime 矩阵。正常 localization、serial/control、timestamp 和 TF semantics
无变化；增加的只有 opt-in metadata/诊断发布和其有界资源开销。没有新增时钟修正、TF
owner 或控制入口。真实设备吞吐/连续录制和物理时间误差仍未验证。
