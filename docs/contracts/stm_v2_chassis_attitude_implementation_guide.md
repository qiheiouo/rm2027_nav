# 给 STM 组：V2 底盘姿态 `0x88` 实现单页说明

状态：**等待 STM 联签与固件实现；Hardware Go = NO**。完整定义见
[STM V2 统一参考](stm_v2_protocol_reference.md)。本页只用于固件实现对齐，不能替代抓包/实测。

## 需要实现什么

- V2 type：`0x88`，STM → host；capability bit 8 (`1u << 8`)；payload 55 bytes。
- heartbeat 必须使用非零、每次真实重启改变的 `boot_id`，并宣告 bit 8。
- 每一条 `0x88` 自带相同 `source_boot_id`，不能让旧 boot 缓冲包在新 boot 后发出。
- 使用仓库 C11 reference：
  `src/rm_serial_driver/reference/competition_v2/competition_v2_codec.{h,c}`，入口
  `rmcv2_encode_chassis_attitude_state()`。不要直接发送编译器 struct。

## 固定 payload

| Offset | Type | Field |
| ---: | --- | --- |
| 0/4/8/12 | 4×float32 LE | quaternion `x,y,z,w` |
| 16/20/24 | 3×float32 LE | gyro x/y/z，rad/s |
| 28/32/36 | 3×float32 LE | accelerometer specific force x/y/z，m/s² |
| 40 | uint32 LE | sample sequence |
| 44 | uint32 LE | sample time，ms，与 heartbeat uptime 同时钟 |
| 48 | uint32 LE | source boot ID，非零 |
| 52 | uint16 LE | estimator reset counter |
| 54 | uint8 | bit0 q valid；bit1 gyro valid；bit2 acc valid；bit3 online；bit4..7=0 |

四元数表示 `R_reference_from_chassis_imu`，字段顺序不是 `w,x,y,z`。gyro/acc 表达在
`chassis_imu_link`。acc 是 specific force：静止且传感器 +z 向上时 z 约 `+9.81 m/s²`。
reference frame、yaw 零点、安装轴、是否做 bias 补偿必须在联签记录中写清楚。

## 计数与故障规则

- `frame sequence`：uint16，`0x88` 自己独立递增；它不是 sample sequence。
- `sample_sequence`：每个新传感器样本 uint32 递增；重复发送同一样本不能伪装成新 sample。
- `mcu_time_ms`：采样点时间，不是 UART 发送时间。
- `reset_counter`：姿态估计器 reinit/re-zero 时 uint16 递增；该新 epoch 允许 sample/time 重新开始。
- reboot：改变 boot ID，清 TX/DMA 旧队列，reset 授权；不要复用旧 boot。
- 未收敛、饱和、传感器故障或任一分量不可用时，清对应 valid；不要填零后仍置 valid。
- offline 时清 online；恢复前先建立新的有效 sample 生命周期。
- 所有 float 必须 finite；q valid 时 `abs(norm(q)-1) <= 1e-3`。

## 固定 golden vector

输入：frame seq `0x3344`；q=`[0,0,0,1]`；gyro=`[0.125,-0.25,0.5]`；
acc=`[0,0,9.80665]`；sample seq=`0x01020304`；time=`1000`；
boot=`0x20270001`；reset=`2`；flags=`0x0f`。

预期 55-byte payload 必须是：

```text
00 00 00 00 00 00 00 00 00 00 00 00 00 00 80 3f
00 00 00 3e 00 00 80 be 00 00 00 3f
00 00 00 00 00 00 00 00 0a e8 1c 41
04 03 02 01 e8 03 00 00 01 00 27 20 02 00 0f
```

完整 65-byte frame 必须是：

```text
52 4d 02 88 37 00 44 33
00 00 00 00 00 00 00 00 00 00 00 00 00 00 80 3f
00 00 00 3e 00 00 80 be 00 00 00 3f
00 00 00 00 00 00 00 00 0a e8 1c 41
04 03 02 01 e8 03 00 00 01 00 27 20 02 00 0f
90 be
```

CRC 是 CRC16/Modbus，poly `0xA001`、init `0xffff`，覆盖 version 到 payload，排除 `52 4d`，
低字节先发；因此本向量 CRC 数值为 `0xbe90`，wire 为 `90 be`。

## STM 提交验收材料

1. firmware repo/commit/toolchain/profile；
2. reference C11 单测与 MCU toolchain golden vector 结果；
3. 正常、CRC 错误、截断、粘包/分包抓包；
4. 冷启动、手工 reset/re-zero、seq/time wrap、duplicate/out-of-order、旧 boot 延迟包结果；
5. 静止、已知 roll/pitch、绕三轴转动时 q/gyro/acc 对照；
6. 实测频率、最大间隔、端到端时延、带宽和 fault/valid 恢复规则。

Phase 1A 前仍保持电机物理禁能，只允许状态接收。`0x88` 通过不代表运动协议通过。
