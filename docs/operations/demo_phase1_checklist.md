# Demo Phase 1 执行与验收

适用：`integration/demo-navigation-validation`，Integration阶段。硬件尚未完成；当前只有
software bench / ACQ instrumentation通过。此入口不启动Nav2、不发布任务、不授予真实运动。
[STM共同合同](../contracts/stm_navigation_data_contract.md)是固件/停车验收依据。

## 0. 到车前 / 到车时准备

1. 中心圆盘图纸直径250 mm，四个全向轮外突未知；不得填`robot_radius=0.125`。
   实测整车外包络、轮位置/半径、制动距离、速度/加速度限制、base→yaw轴→MID360→内部IMU完整SE(3)。
2. 取得STM仓库/commit/version、实际profile、capabilities、CRC golden vectors、baud/rate、watchdog与ESTOP证据。
   若不是competition_v2，**停止该串口入口**，先按实际协议联合确认，不自动探测/回退。
3. MiniPC宿主Ubuntu24.04/26.04待核实；容器复用Humble依赖，宿主仍决定kernel、USB权限、网卡地址、
   PTP/时钟、CPU温控和Docker设备映射。镜像能启动不代表这些已验证。记录实际版本/image ID。
4. 实物禁能、急停可触达、独立ROS domain；不得让旧车/仿真/bench/其他命令发布者进入该domain。
   串口节点会发送heartbeat和zero-disabled；若需要完全不发包的监听，使用电气侧被动抓包，当前transport不是sniffer。
5. 在仓库外建立实测配置目录，复制`src/rm_demo_platform/config/demo_*.yaml`后填写。
   全部是**入口manifest**，不是可直接传给Nav2的ROS参数文件。null/false保持未完成，禁止填bench数值。
   `demo_lio.yaml`引用外部确认后的backend/adapter YAML；Nav2/MPPI仅保留禁用清单，不会生成猜测参数。

无设备验证与默认入口：

```bash
src/rm_demo_platform/scripts/run_bench_container.sh acq
src/rm_demo_platform/scripts/run_phase1_container.sh inventory
# 输出inventory，零节点、禁网、无设备。实际硬件不使用bench的domain或合成模型。
```

局部环境构建只包含原5包和Livox驱动。Phase 1C另需构建既有`rm_lio_bringup`、`fast_lio_multi`
及直接依赖到同一Humble overlay；本轮未把FAST-LIO实机执行列为通过。禁止加载宿主旧Jazzy overlay。

## 1. 准备每次session

在已source本分支Humble overlay的环境，或从源码设置
`PYTHONPATH="$PWD/src/rm_demo_platform"`后直接运行同名Python脚本：

```bash
# CONFIG_DIR、SESSION_DIR、OPERATOR是现场填写的绝对路径/姓名；输出目录必须尚不存在。
ros2 run rm_demo_platform demo_phase1.py prepare \
  --repo "$REPO" --config-dir "$CONFIG_DIR" --output "$SESSION_DIR" \
  --operator "$OPERATOR" --phase A
ros2 run rm_demo_platform demo_phase1.py preflight \
  --config-dir "$SESSION_DIR/config" --stage serial
```

准备器复用ACQ session schema、topics、QoS，role扩展为`demo_phase1`。自动保存：branch/HEAD、
tracked staged+unstaged binary diff、submodule revisions/diffs、dirty/untracked名称、操作者、UTC、
环境标识、实测配置快照，以及被引用的SDK JSON/LIO配置/安全证据副本。未跟踪源码仅列名称，
有这类修改时必须人工归档其实际内容再实验；不要只留一个无法复现的dirty标志。

`demo_session.yaml`填写MiniPC、镜像名/ID、地图bundle、具体case、启动命令、notes。
固件和标定revision由各manifest归入metadata。motion_authorization保持false，result初始NOT_RUN。
修改输入后重新prepare新session，不编辑已采集session的配置。采集器写入实际ROS/RMW/domain、
话题type/QoS/publisher GID清单、频率/间隔/序号/boot/reset观察记录和bag路径。
有效参数还需保存`ros2 param dump NODE`；hardware launch将实际参数和容器脚本提供的镜像identity保存到`launch_*.json`，
采集结束后归入session；手动启动未提供镜像环境信息时需人工补录。
安全结果、人工方向检查、map revision、启动/复位时刻需补入同一`session.yaml`，不另造平行报告。

## 1A. 无运动采集

入口按接线顺序可分别启动：

```bash
# 仅当上方条件已确认。脚本要求显式IO开关、隔离domain和准备好的session。
RM_DEMO_ENABLE_HARDWARE_IO=1 ROS_DOMAIN_ID="$DEMO_DOMAIN" \
  src/rm_demo_platform/scripts/run_phase1_container.sh serial "$SESSION_DIR"
# serial退出后，可启动tf；tf已经包含serial，不要重复启动两个串口节点。
RM_DEMO_ENABLE_HARDWARE_IO=1 ROS_DOMAIN_ID="$DEMO_DOMAIN" \
  src/rm_demo_platform/scripts/run_phase1_container.sh tf "$SESSION_DIR"
# lidar可与tf并行，各只有一个实例；共享相同session_id、domain和配置快照。
RM_DEMO_ENABLE_HARDWARE_IO=1 ROS_DOMAIN_ID="$DEMO_DOMAIN" \
  src/rm_demo_platform/scripts/run_phase1_container.sh lidar "$SESSION_DIR"
```

脚本只映射确认的串口设备，不使用privileged；物理阶段使用host网络以接收真实MID360。
USB重新插拔后重新核对设备映射并重启容器/session。IO开关不是motion授权。
`demo_measured.urdf.xacro`只含实测链，不含虚构footprint。未测量时先采serial/lidar，TF项为PENDING。
实际launch也可在已准备的MiniPC容器内直接运行：

```bash
ros2 launch rm_demo_platform demo_hardware.launch.py \
  stage:=tf enable_hardware_io:=true session_dir:="$SESSION_DIR"
# 在同一容器/ROS环境另一个终端采集；--record是显式bag写入，无该选项仅观察。
ros2 run rm_demo_platform demo_observe.py \
  --session-dir "$SESSION_DIR" --duration-sec "$DURATION_SEC" --record
```

跨容器发现/数据路径必须实际确认；优先在现有采集容器内进入第二终端，使用相同overlay/domain。
采集器只订阅显式白名单，不使用`-a`，不回放bag，不发布控制量。启动后先确认bag在写入再开始case。
退出会SIGINT停止其自己启动的recorder并检查metadata；强制终止/缺话题返回失败，不删除原始文件。
每次case/重启使用新session目录；在session内关联上一轮与boot/reset原因。

| 检查 | 保存证据与通过条件 |
| --- | --- |
| 固件 / profile / CRC | 固件identity + 原始抓包 + 正常/损坏帧统计；上位机codec过测不能替代MCU抓包 |
| boot / reset | 冷启动、STM reset、host/transport重启；记录前后boot、seq、tick、reset与缓存清空，无旧样本冒入新epoch |
| relative yaw | 手动转动/静止/跨wrap，比较已知机械方向与角度；valid/online/断流如合同；记录source stamp/ROS stamp |
| chassis attitude | 按联签后字段/ROS type录入session manifest，倾斜验证roll/pitch和gravity；**当前无载荷实现，此项BLOCKED，A不可整体PASS** |
| MID360点云 / IMU | 单颗真实ID/固件/SDK JSON、time_type、raw timestamp、host fallback、IMU物理frame lineage；禁止用改frame_id代替旋转 |
| 时间 / 丢包 | `observations.json`给接收频率、最大间隔、stamp非递增、序号缺口、trace incomplete与事件；按source/epoch分析ACQ，不能把计数缺口直接等同网络丢包率 |
| TF | RSP独占base→gimbal→MID→IMU；检查所有边及发布者；A无map→odom/odom→base owner，不用stub补齐 |
| bag | `ros2 bag info`、record log、metadata和配置快照齐全；实际数量/时间窗/静止启动窗口/trace关联完整；缺话题不能标PASS |

采集工具不自动验证物理单位/方向、真实同步误差、TF边owner或停车。记录这几项的人工结果与证据，
不得把`OBSERVATION_ONLY`改读为PASS。A的TF static来自RSP；B/C固定云台依然需要新鲜相对角反馈。

## 1B. 停车与独立禁能（无Nav2）

先禁能/架空验证zero + enabled=false、CRC正确，再按[停车表](../contracts/stm_navigation_data_contract.md#停车合同与证据表)
逐项记录：正常退出/SIGINT、指定transport PID的SIGKILL、USB拔出、单独丢heartbeat、
保持heartbeat但停chassis命令、MCU reboot、host reboot、定位输入丢失、授权撤销、ESTOP/manual disable。
不要使用全局kill命令；故障注入对象、时刻、预期、观察和恢复必须在case开始前写入session。

频率/故障注入需STM侧可控测试入口或电气测试工具；本轮未实现一个可绕过guard的上位机发令器。
每项给出允许停止时限及实测最坏值（都须现场填），观察轮速/外部运动，不只看ROS zero。
禁止自动恢复旧命令；re-arm应重新确认session/操作者/急停状态。
电机一直禁能的试验不能证明运动中的停车距离。受控电气运动/停车证据由独立已确认的手动流程取得，
本分支没有production正授权。因此B可能只获得部分证据；未完整通过不得进入E/F。

## 1C. 固定云台定位（先无动力人工搬动）

机械锁定云台，填写`demo_lio.yaml`及实测backend/adapter YAML；保持LIO内部标定与URDF一致。
`fixed_lio`组合原串口、Livox、RSP、gimbal/IMU adapter、既有FAST-LIO wrapper和canonical lio_adapter。
它禁止backend delayed zero-start、quarantine backend TF，原lio_adapter独占odom→base；无map→odom。

```bash
ros2 run rm_demo_platform demo_phase1.py preflight --config-dir "$SESSION_DIR/config" --stage fixed_lio
RM_DEMO_ENABLE_HARDWARE_IO=1 ROS_DOMAIN_ID="$DEMO_DOMAIN" \
  src/rm_demo_platform/scripts/run_phase1_container.sh fixed_lio "$SESSION_DIR"
```

用`--phase C`新建session，确认完整bag和raw/canonical/IMU/TF/ACQ均记录。
依次做：启动静止 → 倾斜启动 → 平地平移/旋转 → 坡道 → 长时静止/闭环漂移 → 输入断流 →
backend重启 → adapter重启。每个case记录初始姿态、外部参考、raw epoch、恢复后的first sample。
阈值来自测量精度/任务需求，未填时只有观察结果。

raw frame选择必须显式：`canonical_odom`要求输入已在声明的odom参考；`sensor_initial + first_source_sample`
会将首样本重基准，**不自动提供gravity/map对齐**。不得为“看起来归零”抹掉启动roll/pitch。
验证zero-start只发生在明确的新session边界；backend重启后不可让旧adapter悄悄沿用旧epoch，
出现坐标跳变立即结束该case，分段录包并重新建立session。当前不宣称自动重启恢复已通过。

## 1D. 重定位（无运动授权）

依次单独评估AMCL、GICP，复用`rm_relocalization_bridge`与已有backend/gate；不启动旧车整套launch，
避免加载旧几何、速度和TF owner。地图bundle、点云/scan链路及参数选定后传给已有组件。
先配置无TF后端，经唯一global-pose bridge发布map→odom；禁止同时运行stub、AMCL TF和bridge。

两种后端各跑：正确初值、错误初值、不同位置、重复结构、数据缺失、错误结果拒绝、恢复。
记录初值来源（不得用query真值偷偷初始化）、accepted/rejected原因、跳变/时延/有效期及map→odom owner。
`--phase D`复用session与collector，在其topics whitelist添加实际后端状态/候选/accepted topic及type再录包。
当前Demo没有已验收地图和参数，GICP registered-cloud依赖也须核对，故不提供假定可运行的D一键命令。

## 1E. 底盘基础运动

前置：A/B/C/D证据完整、物理ESTOP、完整STM合同、正式V2运动授权实现与真实lease/re-arm验收。
**当前分支不满足正向授权，入口保持禁用。** 不可用test lease、通用cmd_vel或legacy fallback绕过。

满足后沿用STM轮级解算，按实测低速/低加速度限制依次测试：+x/-x直行、+y/-y侧移、±wz旋转、
组合运动、停止；对比指令、独立实测速度、加减速时延、轮滑、制动距离、轴向符号。
准备`--phase E`session，增加实际反馈与authorization状态话题，逐case人工授权/撤销、记录结果。

## 1F. Nav2 / MPPI

A–E全部通过才填写并启用Demo专用参数；复用现有Nav2/MPPI和唯一motion owner，不引入Mission链。
`demo_nav2.yaml`/`demo_mppi.yaml`是禁用的需求清单：实际footprint或包含全部突出物的测量半径、
速度/加速度、制动距离、controller周期、horizon、地图bundle及Omni模型均需填入真实参数文件。

简单点到点 → 到点停止 → footprint边界与障碍物 → costmap更新 → MPPI可行性。
同session记录CPU、内存、controller实际周期/超时、MiniPC温度、点云/IMU丢帧和定位质量；
负载采样命令及温度传感器路径按实际MiniPC填写，不假设硬件路径。阈值预先声明，失败停止扩展。
此轮不执行E/F，也不将其标为软件已验收。

## 收尾规则

保留bag、原始抓包、同目录metadata/配置/观察结果和最小关键证据；不提交生成文件至Git。
人工填写case result、notes、故障原因、安全证据和后续动作。采集软件通过 ≠ Hardware Go。
实物到位且确认固件/profile/禁能后可从1A的接线、preflight和部分采集开始；完整1A通过仍受完整底盘姿态扩展阻塞。
