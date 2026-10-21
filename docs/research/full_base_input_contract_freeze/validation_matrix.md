# 冻结验证矩阵（2026-10-08）

历史 19 组 / 14,454 个运动故障样本 / 217 项检查保持原样；本轮重放不冒充新物理试次。
新物理试次为下表九组，数值为最大误差；评分 truth 仅在候选输出后用于评分。

| 场景 / 阶段 | 样本 | 状态 | planar → full 位置 mm | planar → full 姿态 ° |
|---|---:|---|---:|---:|
| flat_A / base_yaw | 196 | {'SOLVED': 196} | 0.214923 → 0.030169 | 0.051066 → 0.008883 |
| flat_A / combined_wrap_multiturn | 735 | {'SOLVED': 735} | 0.072672 → 0.029848 | 0.009965 → 0.008883 |
| flat_A / lateral | 147 | {'SOLVED': 147} | 0.208340 → 0.093766 | 0.053061 → 0.008883 |
| flat_A / stationary_gimbal | 196 | {'SOLVED': 196} | 0.040379 → 0.029847 | 0.009507 → 0.008982 |
| flat_A / straight | 147 | {'SOLVED': 147} | 0.213760 → 0.092755 | 0.042172 → 0.008883 |
| flat_B / combined_wrap_multiturn | 735 | {'SOLVED': 735} | 0.159382 → 0.013018 | 0.013980 → 0.013100 |
| flat_B / lateral | 147 | {'SOLVED': 147} | 0.372199 → 0.124781 | 0.060964 → 0.013100 |
| flat_B / stationary_gimbal | 196 | {'SOLVED': 196} | 0.169906 → 0.013453 | 0.048990 → 0.013183 |
| flat_B / straight | 147 | {'SOLVED': 147} | 0.230720 → 0.123568 | 0.057489 → 0.013100 |
| up15_A / up15 | 881 | {'SOLVED': 881} | 68.565773 → 0.538702 | 15.089123 → 0.008883 |
| down15_B / down15 | 880 | {'SOLVED': 880} | 82.732832 → 0.747508 | 15.065854 → 0.013100 |
| bank2_B / bank2 | 489 | {'SOLVED': 489} | 0.450279 → 0.411768 | 2.017532 → 0.016382 |
| bank2_B / tilted_stationary | 147 | {'SOLVED': 147} | 0.096458 → 0.000011 | 2.015010 → 0.016530 |
| tilted_pitch11 / tilted_pitch_motion | 293 | {'SOLVED': 293} | 131.170188 → 0.159604 | 11.046335 → 0.017653 |
| tilted_pitch11 / tilted_pitch_stationary | 196 | {'SOLVED': 196} | 0.074732 → 0.000020 | 11.025811 → 0.017746 |
| frame_fault / nominal | 98 | {'SOLVED': 98} | 0.065044 → 0.000063 | 0.009521 → 0.008883 |
| frame_fault / recover | 98 | {'SOLVED': 98} | 0.075671 → 0.000063 | 0.009624 → 0.008883 |
| frame_fault / wrong_sensor | 98 | {'INVALID_INPUT': 98} | — → — | — → — |
| raw_epoch / epoch2 | 98 | {'INVALID_TIME': 98} | — → — | — → — |
| raw_epoch / identity_returns | 98 | {'INVALID_TIME': 98} | — → — | — → — |
| raw_epoch / nominal | 98 | {'SOLVED': 98} | 0.065044 → 0.000063 | 0.009521 → 0.008883 |
| source_skew / nominal | 98 | {'SOLVED': 98} | 0.064956 → 0.000063 | 0.009502 → 0.008883 |
| source_skew / offline | 97 | {'SOLVED': 1, 'INVALID_INPUT': 96} | — → 0.015521 | — → 0.458453 |
| source_skew / offset05ms | 98 | {'SOLVED': 98} | 0.084137 → 0.000062 | 0.014845 → 0.008883 |
| source_skew / offset10ms | 98 | {'INVALID_TIME': 98} | 0.088114 → — | 0.229367 → — |
| source_skew / offset1ms | 97 | {'SOLVED': 97} | 0.075542 → 0.000063 | 0.024856 → 0.008883 |
| source_skew / offset20ms_alias | 98 | {'INVALID_TIME': 1, 'SOLVED': 97} | 0.040052 → 0.015521 | 0.458460 → 0.458453 |
| source_skew / offset2ms | 98 | {'INVALID_TIME': 98} | 0.054089 → — | 0.046837 → — |
| source_skew / offset5ms | 98 | {'INVALID_TIME': 98} | 0.087283 → — | 0.114989 → — |
| source_skew / recover | 98 | {'SOLVED': 98} | 0.086285 → 0.000063 | 0.009629 → 0.008883 |

`offline` 阶段的 1 帧 SOLVED 是异步故障请求边界之前已进入缓存的周期 alias；保留计数和错误。
收到 invalid/offline marker 后不再复用旧有效缓存；实时 ROS 控制另行验证这一输入级条件。

## 验证层次

| 层次 | 结果 / 适用范围 |
|---|---|
| 输入单测 | 21 项通过 |
| 原数学单测 | 31 项，固定 Git blob 提取到本任务 build，全部通过 |
| C++ / 模型 | 原 15 项 canonical gtest 与 S2 geometry CTest 通过 |
| 冻结观测重放 | 19 组 / 14,454 个运动故障样本；正常记录 pose 差 <8e−16；旧 trace 无 sequence / arrival order，不声称复核原 transport history |
| 实时 ROS 合同控制 | 9 组 / 47 项检查通过；周期 alias 和隐藏值误接受是不可观测反例 |
| 原解析 ROS 边界 | continuum / near singular / two roots / time rewind 四组重新通过 |
| 新 Gazebo 回归 | 9 组 / 7000 个运动故障评分样本，完整 A/B shadow pipeline；不是实车 |

完整机器可读结果：[freeze_summary.json](evidence/freeze_summary.json)。
原实验与新试次分开保留；没有扩大硬件标定、LIO/deskew 或部署成功的证据范围。
