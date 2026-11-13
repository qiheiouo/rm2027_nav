# STM 与导航数据合同：Demo / 正式新车共用

状态：**Host chassis-attitude implementation ready，STM 联签/固件/实测 pending，Hardware Go = NO**。
适用于本 Demo 集成分支。
两车使用 RoboMaster 官方 STM 控制板，不代表板上固件已实现本协议。固件仓库、commit、
实际 profile、带时间的原始抓包均未知；禁止从板型号、旧车资料或 bench 推断。

本文件是共同确认清单；统一 type/offset/lifecycle 定义以
[STM V2 统一参考](stm_v2_protocol_reference.md) 和
`rm_serial_driver/{src,reference/competition_v2}` 为准。`0x86` 已在 host 冻结并实现，但在 STM
联签前仍是 host-frozen recommendation，不表示固件已接受。每项确认须附固件 revision、上位机
revision、配置、抓包/session、STM/导航负责人和日期；更换固件或校准后重新核对受影响项。

## 当前实现与未完成边界

| 项目 | 当前上位机可验证事实 | STM 待确认 / 扩展 |
| --- | --- | --- |
| Envelope | `52 4d`、version=2、type、uint16 payload length、uint16 per-type sequence，整数/float32 little-endian，最大 payload 128 | 同版实现、序号窗口/重复/乱序处理、UART/USB波特率及带宽 |
| CRC | CRC16/Modbus，poly `0xA001`，init `0xFFFF`，覆盖 version 至 payload，排除 magic，低字节先发 | 独立 golden vectors、损坏/截断/重同步抓包 |
| Relative yaw | `0x82` 17 bytes，角/角速度/seq/tick/valid/online | 安装零位、方向、连续旋转范围、估计器 reset 标识 |
| Chassis heading | `0x85` 19 bytes，world yaw/rate/seq/tick/reset_counter/flags | world参考、重置规则；**仅 yaw，不能代替完整姿态** |
| Chassis command | `0x01` 13 bytes，vx/vy/wz/enabled | 执行符号、限幅、独立 watchdog、实测制动、re-arm |
| Heartbeat | `0x03` 13 bytes，uptime_ms/boot_id/capabilities/ready | boot 唯一性、tick来源/回绕、ready及掉线定义 |
| Chassis attitude/IMU | host 已实现 `0x86` 55-byte codec、C11 reference、`/chassis/imu`、status/ACQ、boot/reset/seq/tick fail-closed lifecycle | STM type/bit/语义联签、固件实现、抓包、方向/重力/频率/时延/TF 实测；完成前 Phase 1A 不能整体PASS |
| Motion authority | production 模式始终 disabled；只存在隔离 dry-run test lease | 正式 task/session owner 与实车授权入口尚未实现；不得拿 test lease 上车 |

## 云台相对底盘 Yaw

共同消费语义是**测量时刻的机械相对关节角**。Demo：双 IMU → STM 解算；正式车：大 Yaw
电机绝对编码器 → STM。上位机均消费 `GimbalState`，不根据传感器来源切换含义。

| 字段 / 约定 | 统一目标及当前表示 | 待确认点 |
| --- | --- | --- |
| angle | `relative_yaw_rad`，rad；零位云台前向与底盘 +x 对齐；右手正向绕底盘局部 +z | 机械轴与base标定、零偏/编码器符号；斜坡上不能简单相减两个 world Euler yaw |
| wrap / 多圈 | 当前 wire 为 `[-π, π)` 单圈角，无圈数 | 是否连续旋转、物理限位、wrap附近连续性；需要多圈时另订字段，不把wrapped值当累计角 |
| rate | `yaw_rate_rad_s`，rad/s，与angle同一样本、同一正向 | 滤波延迟、估计方法及最大误差 |
| sample sequence | uint32 `sample_sequence` | 每次新采样递增，重复/乱序/回绕/重启规则；采样频率和允许丢包/延迟均待测 |
| source time | uint32 `mcu_time_ms`，毫秒 | 必须定义采样点与heartbeat uptime同一时钟、量化误差、wrap与暂停行为 |
| valid / online | 两个独立flags；离线或失效时不能复用最后一个有效角 | 双IMU任一失效、编码器故障、启动未收敛、重新置零时的flags与恢复条件 |
| boot / reset | `0x82` 与当前 `GimbalState` **没有**每样本boot/reset字段 | 补充样本epoch绑定或可证明的队列清空规则；无重启的置零必须显式事件，禁止静默跳变 |

Heartbeat boot 改变会令上位机旧状态失效；ACQ 记录的是当时最新 heartbeat 的
`associated_boot_id`，不是云台包自身携带的身份。不能据此证明延迟旧包属于新boot。
TF只有 `gimbal_state_adapter(use_input=true)` → `/joint_states` → RSP；失效/断流停止joint发布，
不回退假零角。TF关节为continuous只表示软件角度模型，不授权机械多圈旋转。

## 完整底盘姿态 / IMU：Host 冻结合同

host 已固定 `0x86 ChassisAttitudeState`、capability bit 6 和 55-byte payload；逐 offset 定义、
golden vector 和生命周期规则见 [STM 单页实现说明](stm_v2_chassis_attitude_implementation_guide.md)。
坡道场景必须保留 roll/pitch；禁止用 world yaw 或 `setRPY(0,0,yaw)` 代替。

| 字段 | Host 已冻结语义 | STM / 实机仍待确认 |
| --- | --- | --- |
| orientation quaternion | float32 `x,y,z,w`，`R_reference_from_chassis_imu`；q valid 时范数误差≤1e-3，host发布前归一化 | reference/yaw zero、重力对齐方法、误差/reset行为 |
| angular velocity | 三轴rad/s，右手，表达在`chassis_imu_link` | 量程、滤波、bias、饱和与实际延迟 |
| linear acceleration | 三轴m/s²，固定为specific force，静止+z向上约+g | 传感器型号、标定、g值、饱和/故障行为 |
| sample identity | uint32 sample seq、uint32 `mcu_time_ms`、每样本非零 source boot、uint16 reset counter | 采样点定义、频率、tick来源/回绕、TX队列清空 |
| validity | orientation/gyro/acc 独立valid + online；reserved flags必须0 | 初始化/失锁/故障/恢复的状态机 |
| ROS output | 完整有效时发布`/chassis/imu` (`sensor_msgs/Imu`)，frame=`chassis_imu_link`；covariance全零=unknown | 质量/协方差如需新增必须另订，不得伪造 |
| lifecycle output | `/chassis/attitude_status`给boot/reset/seq/tick/epoch/current_valid/stale/reason；异常不发布零值Imu | 真机cold boot/reset/re-zero/replay/dropout证据 |
| IMU → base_link | transport不旋转、不发TF；由实测`T_base_from_chassis_imu`静态TF定义 | 安装轴/位置和校准revision；有杆臂时明确补偿owner |

同一 boot/reset 内 host 只接受模半范围严格更新的 sample sequence，拒绝 MCU time 倒退；新 reset
须为更新 counter，新 boot 须先由 heartbeat 接受。断流默认软件阈值 0.2 s，但现场必须显式填写
固件确认值。host heartbeat 现拒绝 zero boot、duplicate/out-of-order、uptime倒退和本进程已退役boot
重放；它不是密码学认证，STM仍须确保新boot唯一并清空旧队列。

MID360内部IMU继续作为FAST-LIO输入；底盘IMU不自动成为第二个canonical odometry或TF owner。
Host software 状态为 Ready；STM firmware、物理语义和实机数据仍 Pending，因此 Phase 1A 只能
Ready with Limitations，未完成联签/抓包/TF/方向/频率证据前不能整体PASS。

## 底盘速度与运动 owner

`vx`沿base_link +x向前、`vy`沿+y向左，单位m/s；`wz`绕+z右手正向，单位rad/s。
`enabled=false`应使STM立即进入定义好的安全停车模式；零速度与disabled是两个不同条件，
必须分别验收，不得只看数值为零。`enabled=true`不能单独代表有授权。

轮序、全向轮逆解/速度环、限幅和电机输出由 **STM** 负责。Demo不新增轮级命令协议或第二套
轮级控制器。可选轮反馈仅用于诊断/速度响应验证，字段未确认前不假造 `/chassis/twist_raw`。
当前`0x01`只有envelope uint16 sequence，没有command timestamp、任务lease或boot绑定；
旧命令拒绝和re-arm不能仅靠该字段证明。后续正式授权设计必须与STM共同闭合。

现有transport源码默认chassis发送100Hz、命令freshness 0.2s、heartbeat 2Hz；这些是**软件默认**，
不是MCU频率/停车时间承诺。Demo manifest要求显式填写确认值，不能把0.2s写成已确认watchdog。
当前节点不订阅通用 `/cmd_vel`；`motion_test_mode=false`时持续发送zero-disabled。
`motion_test_mode=true`要求`dry_run=true`，不能打开真串口。ACQ不改变这些规则。

Posture request是独立执行器路径，不受底盘motion guard同等约束。Phase 1入口将其订阅隔离至
本session专用禁用topic，禁止外部发布者；需要独立ROS domain、物理运动抑制和受控接线。
这是防误接保护，不是安全认证或对恶意ROS参与者的访问控制。

## 时间、boot/reset 与 ACQ

当前MCU映射为 `offset = heartbeat_receive_ROS - uptime_ms`；样本header为
`offset + mcu_time_ms`。ACQ保存映射版本、heartbeat basis、handler ROS/system/steady时间，
真实串口另记一批read完成时间。它们不等于逐字节到达时间，不能证明USB时延或MCU时钟已同步。
`0x86` host 已实现boot/reset/seq/tick replay gate；gimbal/heading旧消息仍没有同等每样本boot绑定。
当前没有完整host clock step补偿，必须记录并分段分析，不能跨异常拟合。

Livox ACQ保留原始8字节timestamp、time_type、selected timestamp、host fallback、callback
system/steady、包到输出point span、publish之后的ROS/steady观察时间。保留原cloud/IMU值和
既有时钟选择；默认关闭诊断。输出`complete=false`、缺trace或队列丢数据均必须报告。
每条最多512个包记录，fingerprint是配对诊断，不是认证；记录时间不是精确DDS线速发送时间。

## 停车合同与证据表

每行需填：**触发时刻/时钟域、STM独立动作、最大允许停车时间、实测最坏时间、停车判据
（轮速/外部测量）、旧命令拒绝、人工re-arm、证据session/固件revision、PASS/FAIL**。
当前这些物理字段均为 **PENDING**。通信定时器失效不等于物理车已停。

| 故障 / 动作 | 当前已知上位机行为 | 必须验证的STM独立行为与恢复 |
| --- | --- | --- |
| ROS SIGINT / 正常退出 | 未承诺退出时最后一帧必达；不能依赖destructor | 无新命令后在确认时限内停车，重新启动不自动续跑 |
| SIGKILL / 进程崩溃 | 无清理机会 | MCU watchdog独立停车；旧缓冲帧不能重新使能 |
| USB/串口拔出 | 不再可靠TX/RX | 通信丢失停车；插回须新session/re-arm |
| Heartbeat丢失 | 连接状态失效；当前production仍为zero-disabled | 双向heartbeat策略、ready=false处理及恢复门明确 |
| Chassis command超时 | host freshness规则不能替代MCU timer | 即使heartbeat仍到达，也因命令超时停车 |
| MCU reboot | boot变化使上位机旧状态失效 | 上电disabled、清空旧授权、拒绝旧seq/队列，人工确认后恢复 |
| Host reboot | 原进程消失 | 独立停车；新的host boot不可继承上一任务 |
| 定位失效/断流 | bench能验证canonical输出暂停；当前没有production正授权 | 未来motion owner撤销/停止续约，STM按lease/watchdog停车；不能假定“无odom即停” |
| authorization撤销 | 现有测试guard可拒绝；不构成实车证据 | STM可观测禁用、拒绝迟到旧命令、新人工re-arm |
| ESTOP / manual disable | 不能由ROS检查代替 | 独立硬件/遥控机制、断电还是制动、优先级、释放不自动恢复均须实测 |

Phase 1B先做电机禁能/架空状态下的协议和看门狗观察；真正“运动中可靠停车”的证据仍需独立
受控电气测试并记录。本分支不能发送正向实车授权，禁止通过修改dry_run或test lease绕过。
未闭合这张表和授权实现，不进入Phase 1E/F。签署合同也不等于完成实车验收。
