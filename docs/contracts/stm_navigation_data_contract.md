# STM 与导航数据合同：Demo / 正式新车共用

状态：**Integration，字段约定待 STM 联签，Hardware Go = NO**。适用于本 Demo 集成分支。
两车使用 RoboMaster 官方 STM 控制板，不代表板上固件已实现本协议。固件仓库、commit、
实际 profile、带时间的原始抓包均未知；禁止从板型号、旧车资料或 bench 推断。

本文件是共同确认清单；已有字节定义仍以 [V2 wire specification](../competition_v2_protocol.md)
和 `rm_serial_driver/{src,reference/competition_v2}` 为准。未确认字段不占用 type、capability bit、
payload offset 或新增 ROS 消息。每项确认须附固件 revision、上位机 revision、配置、抓包/session、
STM/导航负责人和日期；更换固件或校准后重新核对受影响项。

## 当前实现与未完成边界

| 项目 | 当前上位机可验证事实 | STM 待确认 / 扩展 |
| --- | --- | --- |
| Envelope | `52 4d`、version=2、type、uint16 payload length、uint16 per-type sequence，整数/float32 little-endian，最大 payload 128 | 同版实现、序号窗口/重复/乱序处理、UART/USB波特率及带宽 |
| CRC | CRC16/Modbus，poly `0xA001`，init `0xFFFF`，覆盖 version 至 payload，排除 magic，低字节先发 | 独立 golden vectors、损坏/截断/重同步抓包 |
| Relative yaw | `0x82` 17 bytes，角/角速度/seq/tick/valid/online | 安装零位、方向、连续旋转范围、估计器 reset 标识 |
| Chassis heading | `0x85` 19 bytes，world yaw/rate/seq/tick/reset_counter/flags | world参考、重置规则；**仅 yaw，不能代替完整姿态** |
| Chassis command | `0x01` 13 bytes，vx/vy/wz/enabled | 执行符号、限幅、独立 watchdog、实测制动、re-arm |
| Heartbeat | `0x03` 13 bytes，uptime_ms/boot_id/capabilities/ready | boot 唯一性、tick来源/回绕、ready及掉线定义 |
| Chassis attitude/IMU | **无 wire payload、无本链路接收实现** | 下方字段级提案全部联签后另行实现 |
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

## 完整底盘姿态 / IMU：待确认的字段级提案

以下是拟定统一语义，**不是已经实现/双方接受的 wire 合同**。不分配新 type 或字节布局。
坡道场景必须保留 roll/pitch；禁止用 world yaw 或 `setRPY(0,0,yaw)` 代替。

| 字段 | 提案（联签前不得作为运行输入） |
| --- | --- |
| orientation quaternion | 归一化四元数；建议字段顺序 **x,y,z,w**；表示 `R_reference_from_chassis_imu`，把IMU向量旋转到参考frame；不是逆旋转；q与-q等价，禁止靠符号判reset |
| orientation reference | 明确命名 MCU 启动局部世界系或重力对齐局部系、yaw零点、是否磁北/启动朝向及reset行为；绝不自动等同ROS map/odom |
| gravity | 参考系采用右手 +z向上时，重力加速度矢量为 `[0,0,-g] m/s²`；g取值/估计方法待确认；启动是否重力对齐及误差须有证据 |
| angular velocity | 三轴rad/s，表达在 `chassis_imu_link`，右手；样本时刻、滤波延迟明确 |
| linear acceleration | 三轴m/s²；建议传加速度计specific force（含重力响应），静止且+z朝上时约+g；必须确认是否已去重力，不能模糊写“加速度” |
| 安装frame | `chassis_imu_link`的实际轴方向、安装位置、传感器ID；不是MID360内部IMU，不得混流 |
| IMU → base_link | 给出 **T_base_from_chassis_imu** 固定SE(3)、平移m和旋转约定/校准revision；适配器将角速度与加速度旋转，存在杆臂时另确认补偿；不可只改frame_id |
| valid / online | 分开表达姿态/gyro/acc是否有效（分量mask或明确的整体语义）；饱和、初始化、失锁、离线的规则 |
| sequence / tick | 每样本seq、采样MCU tick及单位/位宽、与heartbeat对应时钟域；打包时间与采样时间区分 |
| boot / reset | 每样本绑定boot_id与估计器reset_counter；重启、重新归零、时钟回绕及旧队列处理；跨epoch不得插值 |
| 质量 | quaternion范数容差、漂移/协方差/饱和标志或明确缺失策略；禁止将缺失当高可信 |
| 频率 / 时延 | 发布频率、允许间隔/抖动、端到端时延/同步误差、断流阈值全部待STM/实测确认，无默认数值 |

MID360内部IMU继续作为FAST-LIO输入；新增底盘IMU不自动成为第二个canonical odometry或TF owner。
本轮只准备后续接口位置；完整姿态未定义前Phase 1A只能标为部分采集，不能整体PASS。

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
当前没有完整tick回绕、排队旧样本和host clock step补偿，必须记录并分段分析，不能跨异常拟合。

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
