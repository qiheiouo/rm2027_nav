# Prediction Value Test（2026-10-13）

Research-only。稳定软件起点 `4541e3754242c5353e2e5316b44f181f5e9f2721`，
独立 `experiment/prediction-value-20261013`。不修改正式导航或冻结组件。
本目录 `COLCON_IGNORE`，无速度 publisher、串口或硬件入口。

## 公平边界

直接链接现有 Humble Nav2 1.1.20 的 MPPI 共享库，调用其原生 prepare、
generateNoisedTrajectories、七个标准 critics、updateControlSequence、fallback、
Savitzky–Golay 和 returned-offset 控制。薄研究子类只在标准评分与原生聚合
之间加入 R2 时间成本。没有复制或重写 MPPI，没有新 tracker、proposal 或 MPC。
R2 circle_clearance 原样复用；squared-hinge penalty 数学等价适配到阶段位置数组。
原 R2 从状态重算 CV 而不消费 prediction[]，所以 oracle 必须走此显式阶段适配，
不能仅改变旧消息的可视化点数组。

五组：A_proxy 当前占用；A' 相同时间消费者+零阶保持；CV 两次已观测位置差分；
B oracle；C 冻结 canonical KF 的 source-anchor CV。B/C只替换未来位置数组。
所有组有相同地图、参考、几何、原生参数、噪声 seed、计算上限、代理动力学及
观测噪声/范围/时序。每个 trial 重置 native 噪声；有效性检查确认同 seed 原生
指令与 rollout 精确重复。闭环之后状态不同，不能宣称每拍候选位置完全相同。

**A_proxy 不是完整 STVL+MPPI。** 当前地图直接栅格化带噪观测 disk，10Hz
刷新并施加相同 .5m inflation；不运行 STVL voxel/decay/frustum/3D 观测或
Nav2 action/BT/global replanning。安装版 STVL 可用，历史 A24–A26 有其真实
闭环证据；为最小信息干预，本轮主动选择受限 proxy。结论不能外推正式 STVL
性能，也不能声称已经建立新车完整动态基线。

代理机器人四全向轮 body-velocity 平面模型，继承既有仿真半尺寸 .325/.300，
统一 .03 padding；不是新车 CAD。实际速度按 .8/.8/2 加速度限幅、10ms物理步
中点积分。50ms输出延迟另加实际完整调用耗时；所有组都在这段时间继续执行
旧指令。Humble 1.1.20 原生 rollout 没有该加速度/执行延迟模型，差异如实记录。
不是理想无延迟机器人，更不是实车或数字孪生。

## 运行

需现有 `rm2027_navigation:humble` 离线镜像，包含 Nav2 1.1.20、STVL 2.3.4、
GCC11.4、CMake、nlohmann_json、Python3.10；不下载依赖。执行于本 worktree：

```bash
bash experiments/prediction_value/run.sh build
bash experiments/prediction_value/run.sh checks
bash experiments/prediction_value/run.sh p1
# P1 合同成立且结果可解释之后才运行：
bash experiments/prediction_value/run.sh p2
```

输出 `experiments/prediction_value/evidence`；cohort目录已存在时拒绝覆盖。
跨机器复制源码/镜像后相同命令；只读源码挂载，只有本轮 cache/evidence 可写，
network none、cap-drop ALL、无设备。`build/prediction_value_cache` 可完整重建，
不入 Git。小规模压缩逐周期输入、rollout、输出、执行状态和全部失败入 Git。
禁止拿本入口发实车速度。

## 输入、评价与来源

`protocol.json` 是结果之前冻结的口径；`scenes.py` 是环境，`providers.py` 不导入
环境或访问机器人真值。环境只生成延迟的当前观测、B未来接口和独立评价。
无遮挡、已知完整disk尺寸/质心的简化观测，对真实KF感知质量是乐观条件；
未复现可见面中心偏差、复杂数据关联或 LiDAR 静态剔除。

native stdin JSON：`state` 为当前测量 [x,y,yaw,vx,vy,wz]，`wall/radius` 为共同
代理几何，`current` 为当前观测，`prediction` 为 t+(k+1)*.1s 的30个位置或 null。
不接受 scene、未来机器人状态、碰撞标签。stdout 返回原生指令、延迟、实际
动态成本、候选条件安全数、聚合轨迹与预测净空。记录的 truth 标签仅在 Python
评价端添加；从不反馈进控制消费者。

实际碰撞为10ms采样signed body/disk或wall穿透<-1e-6；不是Gazebo contact。
净空硬门 .05m，数值容许3mm；success另要求4m目标位置/yaw容差。
成功和safe_success分列；失败时间不填到达时间。未计算任意“无意义等待”标签，
只统计实际低速WAIT；提前碰撞的WAIT和路径是截断值。消息丢弃、预测可信度
机制和ROS executor/DDS延迟没有运行，不能以代理全数据推送的零过期代替实测。

源码引用：

- KF 原样复制 `research/r4-hws-low-level-frozen-20261006` peeled
  `bdb8b8d00308be4384ea94e8b060d60fd2173d30` 的 canonical `core.py`；
  参数来自 public-v2 configuration，显示使用未衰减CV，不重写预测器。
- geometry 原样复制 archive `9e08122a6df758b7f0131a5e2de5813a17f0e97c`
  `src/rm_dynamic_obstacle_critic/include/rm_dynamic_obstacle_critic/geometry.hpp`。
- 时间惩罚来自同版本 `model.hpp::penalty/score`，只去ROS消息依赖，保持常量。
- Nav2 [1.1.20](https://github.com/ros-navigation/navigation2/tree/1.1.20/nav2_mppi_controller)
  Apache-2.0，动态适配范围仅此薄调用入口；移除本目录即可回退。
- 未引入其他上游实现；不修改 HWS、T-DT、NEU 或 Temporal MPC。

完整裁决、历史因果矩阵和资源见 `docs/research_history/prediction_value_20261013.md`。
