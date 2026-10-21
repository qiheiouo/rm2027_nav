# 新车统一仿真 baseline：S2限定Research Go

2026-10-07 S2：**Go for navigation research simulation baseline，仅限明确的数值代理与测试域。** 已区分CAD车体、未知机械包络、物理collision与navigation margin；数值柔顺代理收口11°/15°坡顶侧滑，默认侧偏约11.6/13.3 mm；A/B轮速/动态TF、通行负对照和一目标Nav2复测通过。完整2026拓扑、多层自主导航、真实机械/LIO/STVL/硬件链仍Modify或未验证。

当前结论、失败保留、敏感性与复现见 [S2收口报告](s2_collision_passability.md)，全仓历史证据与仿真待办见 [验证状态审计](../navigation_validation_status_20261007.md)。以下保留`89e2e9fa`的S1历史记录；其中Modify、旧footprint数字和坡道失败描述属于S1，不是当前S2状态。

## S1历史记录：Modify

本轮建立了可运行、可复现的 **Research 候选 baseline**，但不足以宣称完整2026比赛场地、坡道动力学和真实MID360定位仿真均已可信。必须保留 **Modify**。

分支 `experiment/new-car-simulation-baseline`，基于 `origin/main-new-car@bd6cf689`；独立worktree `build/new_car_simulation_baseline`。只读核对远端确认main/main-new-car/main-old-car/狗洞分支HEAD与本地缓存一致，没有fetch场地大文件。没有merge历史研究分支、没有push。完整2026几何与实车标定也没有通过工程完整性或测试数量替代。

## 历史资产表

| asset | branch / commit | file | 内容 | 当前可用性 | 本轮复用 |
|---|---|---|---|---|---|
| 最早全向闭环 | `c03cb190`，已进入`origin/main-new-car` | `src/rm_simulation/worlds/phase1_omni.sdf` | 球形方向摩擦轮、MecanumDrive、ground truth、ROS桥 | 可作为历史闭环；轮半径0.075、轴距0.49、轮距0.43均placeholder | 借鉴关节接触建模和桥接边界；不沿用车辆数字或Mecanum驱动 |
| 狗洞任务与扰动 | `origin/feature/new-car-dog-hole-sim@3e7e0ae3`；前序`c0d7a252`、`4102e4fb` | `src/rm_simulation/launch/dog_hole_sim.launch.py`、`src/rm_dog_hole/` | 参数化洞、坡、专用穿越流程 | 已在main-new-car继承；旧版本存在spawn错位/绕洞统计失真 | 复用已经查明的物理spawn、独立穿越判定和实体洞顶经验；不merge整分支 |
| 修复后3D狗洞证据 | `3e7e0ae3` | `docs/validation/new_car_dog_hole_3d_geometry_20260804.md` | 0.22 m临时车体通过；pitch9.16°；最小净空4.27 mm | 对旧临时几何有效；不能移作CAD新车验收 | 保留历史并重新运行新CAD代理 |
| 旧八边形车体 | `7bfb0bbf`，在main-new-car | `src/rm_description/meshes/new_car_octagonal_chassis.obj` | 0.382/0.126 m边长假设，约0.56019 m见方 | 和后续CAD8点polygon不一致 | 新入口改为CAD8点polygon；旧文件保留 |
| 云台/合成LIO入口 | `origin/feature/new-car-rotating-gimbal-simulation@5b477fa9`；`5a3be2f1` | `src/rm_simulation/src/sim_chassis_heading_lio_source.cpp` | 从truth构造传感器轨迹、heading fusion、独立3D传感器truth | 是边界数学测试，不是LIO或真实MID360 | 复用动态几何合同；新入口用**实际关节状态**验证TF，oracle明确独立 |
| 用户记忆中的粗糙比赛场地 | `origin/main-new-car@bd6cf689`；由`ab17167e`引入；平行集成线`d2d5b554` | `src/rm_simulation/models/rmuc2026_navigation_field.sdf` | 25个box/cylinder碰撞，28×15边界、4×6中央deck、任意位置道路/基地 | visual/collision一致；部分维度有依据，但关键尺寸/拓扑也错 | 保留原资产，退出可信完整比赛场地候选；新入口用明确年份的2025native场地 |
| 双性能与坡道滤波入口 | `ab17167e`、`1e8f44a0`；docs `bd6cf689` | `src/rm_navigation_launch/launch/field_geometry_simulation*.launch.py` | lightweight/full、已知平面surface filter、在线costmap | filter只对旧synthetic revision成立 | 不迁移旧坡道mask到新场地；没有借滤波让新测试通过 |
| 旧导航地图 | main-new-car | `src/rm_nav_config/maps/phase1_empty.*`、`src/rm_map_tools/maps/phase2e_test/*` | 空地图和测试PCD/PGM | 不是比赛场地地图 | 不复用；本轮按每个scene重新生成 |
| CAD数值基线 | 只读`experiment/neu-localization-baseline`；文件源commit `4f438f7a` | `experiments/neu_localization_l1/new_car_cad_baseline.yaml` | 用户接受的轮轴、轮距、footprint、yaw轴、A/B数值 | 有提取记录；本轮没有原始SLDASM/冻结CAD | 仅复制这一份配置，注明来源；不改、不merge该研究分支 |
| 完整比赛场地mesh | 北极熊`rmu_gazebo_simulator@1feb76eadf` | `resource/models/rmuc_2025/meshes/rmuc_2025.stl` | 5.7 MB原生2025场地 | native米制几何；外墙最高18.55m需导航高度裁切 | 原STL/许可证入库，运行时裁去2.4m以上结构；保留2025年份 |

详见 [规则审计](rules_geometry_audit.md) 和 [开源引入记录](../../external/polarbear_rmuc2025_field.md)。所有STEP/STP内容均未下载、打开或转换。规则中的公差、700 mm局部收窄和双倾斜地面限制都保留为未通过项。

## 稳定入口

环境：本轮实际构建/运行于 **ROS 2 Jazzy + Gazebo Harmonic 8.10.0**。旧历史入口主要来自Humble/Fortress，新包不替换实车profile，也没有改变旧入口的行为。

在本worktree构建并载入环境后：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select rm_nav_config rm_new_car_simulation --symlink-install
source install/setup.bash

# 2026规则维度测试场；不是完整2026布局
ros2 launch rm_new_car_simulation baseline.launch.py

# 完整2025比赛场地参考；场地年份不能改为2026
ros2 launch rm_new_car_simulation baseline.launch.py field:=competition2025_reference

# 同一实现切换B雷达、3D几何扫描和导航
ros2 launch rm_new_car_simulation baseline.launch.py lidar:=B sensors:=true use_nav2:=true

# 组合坡道/狗洞：6cm台高、11°入口、15°出口均为明确fixture假设
ros2 launch rm_new_car_simulation baseline.launch.py field:=dog_slope

# 查看Gazebo；默认只运行server
ros2 launch rm_new_car_simulation baseline.launch.py headless:=false
```

默认`spawn:=auto`：fixture场在 `(0,0,0.002,0,0,0)`，完整2025场选择空地 `(-10,0,0.002,0,0,0)`，避免在中央高地下方出生。也可传米和弧度的6个逗号分隔数值。`output`目录保存world、URDF、CAD polygon OBJ、PCD、PGM/YAML、桥接配置和manifest。默认`sensors:=false`用于稳定几何/物理检查；显式true运行regular GPU lidar，它不是Livox时序、扫描图案或deskew模型。

复现实测：

```bash
ros2 run rm_new_car_simulation validate_baseline.py --output /tmp/new_car_flat_A --lidar A
ros2 run rm_new_car_simulation validate_baseline.py --output /tmp/new_car_flat_B --lidar B
ros2 run rm_new_car_simulation validate_baseline.py --output /tmp/new_car_dog_slope \
  --field dog_slope --mode drive --spawn=-1.5,4,.002,0,0,0 --duration 14 --gimbal 1.2
ros2 run rm_new_car_simulation smoke_baseline.py --output /tmp/new_car_smoke
```

并行运行不同场景须使用不同`ROS_DOMAIN_ID`、`GZ_PARTITION`和`output`，避免时钟/命令相互干扰。构建产物、生成大地图和全部临时日志保留在ignored `build/`；Git只保存少量决定性结果和来源mesh。

## CAD / 车辆几何审计

| 参数 | 历史值 | 新入口值 | 来源/可信等级 | 冲突与处理 |
|---|---:|---:|---|---|
| 轮半径 | 0.075 m | 0.06649 m | 用户接受CAD提取 / provisional | SDF接触球、URDF轮和运动学统一 |
| wheelbase | 0.49 m | 0.38923 m | 同上 | 四轮x=±0.194615 m；与原ideal中心的5 μm舍入差明确采用轴距一半 |
| track | 0.43 m | 0.38754 m | 同上 | 四轮y=±0.19377 m |
| footprint | 对称0.56019 m八边形；通用Nav2还存在0.6×0.5矩形 | CAD原8点，跨度0.5665×0.5670 m | 已保存8点数值 / provisional，但存在包含关系冲突 | SDF车体碰撞/生成OBJ/URDF保留8点；完整Nav2 footprint额外包络接触球轮与雷达扫掠，不缩小物理边界 |
| base_link | 地面投影 | 保持地面投影 | 现行TF合同 / 软件约定 | 新增`cad_wheel_center`固定Z=+0.06649，不静默更改base原点 |
| yaw轴XY | (0,0) | (-0.00194,-0.00002) m | CAD提取 / provisional | 动态TF与SDF一致 |
| yaw轴Z | 0.115 m | 0.115 m | **未知机械值；历史数值代理** | 不是可靠CAD高度；通过相对offset使CAD雷达home高度保持正确 |
| A原点 | 临时(0.12,0,0.18)地面系 | (0.13776,-0.09151,0.20761) m | CAD model origin / provisional | 轮轴系Z加半径；lever=(0.13970,-0.09149)。旧报告y=-0.09148差10 μm，采用几何一致的差值 |
| B原点 | 无CAD对应 | (-0.17344,0.16021,0.21270) m | 同上 | lever=(-0.17150,0.16023)，同一算法切换 |
| sensor RPY / sensor原点 | placeholder | RPY=0明确假设；frame表示CAD model origin | **待标定** | 不把CAD机械原点当成已知Livox/IMU外参 |
| internal IMU | 旧placeholder frame | 不生成虚构内部IMU transform | **未知** | 真正LIO仿真仍需内部外参与时间模型 |
| mass / CoM / inertia | 30 kg、COM z=0.0975 m及历史惯量 | mass/I沿用数值代理；新COM=(0,0,0)地面系显式假设；gimbal/wheel附加质量也为代理 | **低可信** | COM尚无CAD/测量依据；不把新代理当成实车参数或旧动力学等价模型 |
| 底部净空 / 上部包络 | 0.08/0.22/0.32 m假设 | 底部0.085；上部0.22或0.32；A实际proxy最高0.23761、B最高0.24270 m | **仿真假设** | 未知车高/变形机构；碰撞有意义但不能直接判实车可过 |
| wheel rolling axes / front | Mecanum proxy | 四轮对角切向omni假设 | **未与机械/MCU确认** | 平地验证软件轮序和符号；无法据此认定实车符号 |

独立读取SDF中的轮位置/轴方向重建接触矩阵，和驱动矩阵最大差`6.81e-12`；CAD polygon与SDF/OBJ误差为0。证据 [geometry_audit.json](evidence/geometry_audit.json)。

**CAD轮廓冲突必须保留：** 按同一坐标系解释，四个轮心分别在CAD8点polygon外约21.8–27.2 mm。该8点轮廓可能只包络某一车体部分，或提取的轴系/轮心含义需要复核；目前不能判断。接触球是明确的滚子代理，进一步增加其平面投影。新Nav2使用包含CAD车体、4个接触球和选定雷达完整yaw扫掠的保守凸包（B配置40点、最大半径0.34144 m），而不是把0.29587 m的CAD polygon外接半径当成整车碰撞半径。圆形代理用外切polygon表达，不会因多边形内切而缩小边界。详见 `field_and_footprint_audit.png`。

## 接口、TF和地图关系

`/cmd_vel`维持base_link中的`vx,vy,wz`，新Gazebo插件只设置四个wheel joint的速度，不设置base速度或pose。车体升降、侧滑、倾斜和阻挡来自接触求解。它是**理想滚子接触代理**，不是Mecanum插件、不是实物滚子动力学，也不是cmd_vel→base twist理想运动节点。

TF链：`map → odom → base_link → gimbal_yaw_link → sim_lidar_link`，额外`base_link → cad_wheel_center`。map→odom恒等和full SE(3) odom→base由独立`new_car_simulation_truth_tf` oracle发布；关节TF和静态mount由robot_state_publisher发布。`/joint_states`来自物理关节。`/simulation/sensor_truth`是独立物理link位姿，供验证使用。新oracle不能与lio_adapter/global localization同时运行；若做定位研究，关闭oracle后指定唯一定位TF源。

导航使用`/odometry/simulation`，没有把truth伪装成`/odometry/lio`；没有连接实车MID360、serial、referee或BT任务。Nav2 smoke只用静态几何地图+MPPI，不表示STVL已经接入、LIO已验证或T-DT/MPC已验收。

所有生成物：单位m；world=map；yaw=0；map→odom=identity。PCD是同一collision几何的空间采样，非真实扫描。occupancy原点`[-15,-8.5,0]`，分辨率0.05 m，600×340；图像行序翻转仅对应PGM格式，几何坐标不翻转。2025场景使用原STL中心坐标、地面top z=0；已取消上游SDF的(14.5,8,.2)平移并记录。

occupancy保守投影全部高于地面的结构：高地、road deck和屋顶也标占据。因此它与world几何坐标一致，但**不是支持多层通行/狗洞的完整导航地图**。不通过调origin或擅自放开屋顶投影来让规划过洞；后续需revision绑定的2.5D/分层通行模型。旧空PGM和测试PCD没有被冒充为真实先验。

## 实测及结论

| 关键工况 | 决定性结果 | 判定 |
|---|---|---|
| A/B平地运动矩阵 | 每套13工况：stationary+gimbal；+vx/+vy/+wz、vx+vy、vx+wz、vx+vy+wz及6个反向 | 软件假设下Go |
| 平地轮速/车体速度 | 最大轮速差4.81e-11 rad/s；最大twist分量差0.000631 m/s或rad/s | 理想接触运动学通过；不是已标定动力学 |
| 动态传感器/TF | A/B各1695个匹配样本；云台范围超过15.37 rad，覆盖多次wrap；平地最大位置差0.77 μm | 通过动态几何，不是静态底盘雷达 |
| B实际静态mount TF复核 | 511个stamp一致样本，组合实际`/tf`和`/tf_static`与独立物理sensor truth比对 | 通过；同一算法与A/B选择 |
| 11°坡+云台 | 最大z=0.27279 m；pitch=11.0029°、roll=3.41°；同命令侧向漂移0.75285 m | **Modify：侧滑明显**；TF误差<1e-12级 |
| 15°坡+云台 | 最大z=0.25963 m；pitch=15.0011°、roll=11.50°；侧向漂移0.77098 m | **Modify：侧滑/失去侧向支撑**；TF位置差0.53 μm、角差2.20e-6 rad |
| flat狗洞低包络 | 从x=-1.50实际到x=1.93，经过实体roof；Aproxy top0.23761<0.25 | 代理几何通过；B余量更小/实车未知 |
| flat狗洞高包络 | 0.32 m上部包络停在x≈-0.4556，未进入roof；发生接触后侧向运动 | roof真实阻挡；不能拿blocked时轮速度当车体速度 |
| 坡道狗洞+云台 | 6cm台高、11°入口、15°出口；实际通过到x=1.918；z=0.05743、pitch=8.8757° | 该局部fixture通过；不代表真实2026所有洞 |
| 0.8 m窄通道横向撞墙 | y停在4.11985，墙内表面4.4，车体上边≈4.28；未穿墙 | 实体约束通过 |
| 2025native地面/墙 | 地面base z≈0；围墙向外运动停在x≈-13.7194，未越过内侧-14 m；存在侧向滑动 | mesh参与实际碰撞；动力学仍为代理 |
| 最终Nav2/3D点云smoke | MPPI到达目标；终点距离0.1403 m；地图600×340/0.05 m/[-15,-8.5]；点云11条×2880点，10条timestamp TF查询通过 | 基本接口通过，使用完整碰撞代理footprint；不是STVL、LIO或多层地形验收 |

关键结果与对应launch日志位于 [evidence/](evidence/summary.json)。入库日志仅去除行尾空白，原始日志保留在ignored `build/evidence/`。最大已观测TF角误差约`2.49e-6 rad`，位置差约`1.24 μm`；发生接触时也保留样本，没有以缺失样本换小误差。

3D点云和Nav2 smoke结果见`evidence/nav2_smoke.json`与对应日志。最初smoke遇到Humble斜杠plugin名与Jazzy不兼容、通用bringup引入多余定位源，以及强制软件渲染时EGL设备选择冲突；最终采用最小Nav2节点图、Jazzy plugin名和正常渲染环境，未改系统GPU驱动。失败关键记录保留在`evidence/smoke_failure_summary.json`。GPU扫描是规则扫描网格，不能替代MID360/IMU时间戳、内部外参、采样轨迹或deskew。源码和机械参数未知不是靠点云数量能补齐的。

## Go / Modify / Stop 边界

**总体Modify。** 可用于：软件四轮运动学、CAD数值一致性、动态云台lever arm、full-pose TF、局部实体碰撞/通行研究，以及明确2025 revision的参考场地实验。

尚不足作为统一的完整2026 localization / planner / STVL+MPPI / MPC Research环境，主要缺口是：

1. 2025→2026每个模块的轮廓、位置、局部高度、地面双倾斜和通行拓扑尚未逐项升级；尤其A洞700 mm收窄、装配区和起伏地形。
2. 静态mesh collision仍保留很多原生三角形，未形成已审计的低负担分块collision；目标设备性能未认证。
3. 坡道显著侧滑未解决；CAD8点轮廓和轮心包含关系也冲突。不通过任意调摩擦系数或缩小footprint宣称真实四全向轮可信。需复核CAD提取、机械轮轴/滚子方向、轮胎接触、载荷、悬挂和摩擦依据。
4. occupancy仍保守封闭高地/道路/屋顶投影，多层通行需独立revision绑定的模型；没有真实PCD可对齐。
5. sensor mechanical origin→Livox sensor/IMU、完整RPY、云台zero/sign和时钟关系仍未知；目前LIO oracle关闭后的实际定位能力未验证。

等待实车/机械确认：前向与wheel order/sign、完整车高与变形扫掠包络、底部净空、质量/CoM/inertia、wheel load/suspension/friction、yaw轴Z、A/B到IP/driver左右映射、sensor与internal IMU外参、MCU/ROS/LIO时间同步。用户确认CAD数值可作为Research先验，但不是冻结实车标定。

本轮没有推进东北大学Task2、factor graph、Scan Context调参、UWB、TinyMPC、新LIO、预测/决策研究；未修改或merge三个指定定位研究分支。Go只针对表中有限软件/几何子项，任何结果都不是Deployment Go。
