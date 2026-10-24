# Omni chassis research harness

状态：**FROZEN_WAIT_CHASSIS_PARAMETERS**。完整数学说明、冻结复核、复现命令和真实参数到位后的最小验证清单见
[研究记录](../../docs/research/omni_chassis_kinematics.md)。

本目录是由 `COLCON_IGNORE` 隔离的离线研究工具。`model.py`、`optimization.py`、
`uncertainty.py` 仅消费调用方给定参数；`run.py` / `stage2.py` 默认运行显式 synthetic fixtures。
Python QP 仅作 reference/oracle；没有 ROS、MCU、正式 TinyMPC/MPPI 接入或 Hardware Go。

保留配置、`test_*.py` 与两份关键 evidence。`outputs/` 和缓存可重建，不作为冻结资产。
