# 新车定位 L1 离线基线

仅 Research。源码固定 `bd6cf689d3d5aca5c8edd430ff760ca5ceb27496`；没有修改 `src/`。
结果和解释见 [L1 报告](../../docs/research/neu_localization_l1_baseline_20261007.md)。

`registration_probe.cpp` 使用 PCL GICP，复刻固定新车节点的基本门，并调用该版本真实
`relocalization_math.cpp` 和 `global_pose_gate.cpp`。假设点云已在 odom，odom/base 为单位变换。
T `e374894a` 的 quality 只从 Git 对象读取到忽略的 build 目录，作为**被动诊断**编译；
它既不成为新车 production dependency，也不参与新车 accept 决策。

本机验证环境：Ubuntu 24.04、ROS Jazzy、PCL 1.14.0、Eigen 3.4.0、Python 3.12.3。
这不是原 Humble 实车环境的性能验收。

有限扫描：

```bash
source /opt/ros/jazzy/setup.bash
python3 experiments/neu_localization_l1/run_baseline.py \
  --output experiments/neu_localization_l1/evidence
```

脚本从固定 522 点 fixture 确定性生成重复结构、稀疏点云和平面输入，构建离线 probe，
运行 34 个配置。没有随机采样。GT 不传给 C++；恒等真值只在 Python 中评分。
B1 附近 seed 是明确的上限测试例外，B2 的人为错误 / 无 seed 条件不由 GT 算法生成。
数学 oracle 预先固定 xy≤0.10 m、yaw≤0.10 rad、z≤0.10 m；不是实车容差。

未修改节点的代表核对：

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base build/l1_logs build \
  --base-paths src/rm_gicp_relocalization src/rm_relocalization_bridge \
  --build-base build/l1_ros --install-base build/l1_install \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-pipe \
  --parallel-workers 1
python3 experiments/neu_localization_l1/verify_native.py \
  --output experiments/neu_localization_l1/evidence --domain 198
```

先确保所选 domain 没有其他使用者。工具在私有 domain / 话题运行 GICP 节点，
将 odom/base 显式设为同一私有 frame 以构造单位变换；没有 TF broadcaster、
map→odom bridge、LIO、Nav2、底盘或 cmd_vel。节点一次一例，退出时终止。
该验证只覆盖合成身份假设下的 raw 输出，不证明真实时序 / 外参链。

保留资产：`manifest.json` 记录 pin、fixture hash、参数和输入；`results.csv` 记录
34 配置的有效点数、收敛、fitness、耗时、被动 overlap / 六个 eigen / condition、
两级接受、完整变换及评分；`native_crosscheck.json` 保存 5 例核对；
`bundle_validation.json` 和 `dataset_inventory.json` 保存地图与数据缺口。
`summary.json` 是简短结果索引。生成 PCD / 二进制 / 日志在 build，可由工具重建。

`T_counterfactual_gate` 是历史质量门的反事实诊断，不能解释为新车当前门。
`pose_gate_accept` 对候选单独评价；最终 `accepted_chain` 还要求 GICP 基础门通过。
无 seed / 点数不足没有产生估计，其最终误差留空。
`probe_total_ms` 包含加载和被动 quality；`solve_ms` 仅 PCL align，均不是实车延迟。
同一 fixture 的多 seed 和重复输入不计作独立 episode，不从这些配置计算实车成功率。

## L1-M 增量

```bash
source /opt/ros/jazzy/setup.bash
python3 experiments/neu_localization_l1/run_l1m.py \
  --output experiments/neu_localization_l1/evidence/l1m
```

使用原参数，17 个配置 / 4 个几何模板：正确 / 错误 seed、重复结构、
人工部分观测、只有正确地点含独有特征、退化平面、低点和 stale。
数据均为合成机制输入，不模拟 MID360 beam / occlusion / deskew。
`labels.json` 与配准输入分开；真值容差在求解前固定，GT 不传给 probe。
`partial*_seed0/seed12` 表示 seed 假设，不预先宣称最终解正确。
无配准 / 不收敛时，L1-M 的最终误差留空，输出矩阵不能当有效估计。

probe 保留原 CSV 接口，增加一个可选末尾 JSON 路径；导出完整 T point-to-plane
质量矩阵、post-fit 距离有效数、normal 有效数、去重 target 数和对应 residual。
`filtered_source_points` 是输入点数；`T_evaluated_source_points` 只在旁路计算时有值。
这些计数不冒充 GICP 内部最后迭代的对应数；T 矩阵也不是 GICP 真正 optimizer Hessian。
PCL fitness 用平方距离≤max_range，GICP 对应用平方距离<max_correspondence_distance²；
本轮都是 1，推广到其他参数需注意接口差异。所有门、参数、生产源码均未改动。

`new_car_cad_baseline.yaml` 复用用户认可的历史 CAD 值，明确轮轴 / 地面基准差异；
A 作为任意研究主雷达，B 保留。未确认 sensor rotation、IMU 原点或 left/right / IP 映射。
`geometry_audit.json` 只作地面高度、舍入、杆臂和坐标换基数值核对，不写入硬件配置。
`input_audit.json` 保留资料出处、固定 gitlink 和所查文件 hash，现场 profile / clocks 尚未核验。

`evidence/l1m/` 保留输入 manifest / 标签、17 配置 CSV、诊断、temporal 摘要、
结论和静态图。PCD 在 `build/l1m_inputs`，由固定 Git fixture 与脚本确定性重建；
不依赖唯一且未备份的实车原始数据。`temporal.json` 是不同部分云 / 理想 odom 的
事后一致性分析，没有实现新的 runtime gate、反馈或候选检索。
工具自动核对关键两例与原 L1 数字一致、H 的六 eigen 和 fitness 分母一致。
原 L1 evidence 不覆盖；只运行本段即可复现 L1-M，不需要启动 ROS 或全工程回归。
