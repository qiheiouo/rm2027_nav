# RM2027 Navigation 新仓独立软件回归验收（2026-10-21）

## 结论

- 阶段：Integration，软件离线回归。
- 结论：**Integration Go**。
- 受测代码提交：`8566dab07e21860615fb074f6508ec98cbc116f8`。
- 受测 Git tree：`bc0a8b43fff3db1bdc7e0d66b865a2c1871ed5ee`。
- 13 个 Humble 目标包构建通过；`110 tests, 0 errors, 0 failures, 0 skipped`。
- AUD-003 九场景、AUD-059、AUD-020、AUD-050、新车八步验收 fixture 与关键无硬件软件入口均通过。
- 本结论只覆盖固定软件、合成输入和 mock transport；不包含真实传感器、MCU、底盘运动、场地地图或车型标定，不是 Deployment Go。

## 受测来源与隔离

- 持久新仓：`/home/qihei/projects/rm2027_navigation_clean`。
- 独立测试 checkout：`/home/qihei/projects/rm2027_navigation_clean_validation_8566dab`。
- 仓外结果：`/home/qihei/projects/rm2027_navigation_clean_artifacts/20261021_software_regression`，约 678 MiB；未把大型日志或构建产物提交到 Git。
- 测试 checkout 由新仓使用 `git clone --no-local` 建立；子模块从声明的 GitHub HTTPS URL 递归恢复，禁止 file protocol 和 submodule alternates。
- 主仓、独立 checkout 和三个子模块 `git fsck --full` 均通过；工作树 clean；未发现 `objects/info/alternates`。
- 容器内源码以只读方式挂载，网络关闭，无设备映射，私有 IPC，非 root，移除 capabilities，启用 `no-new-privileges`。
- 旧冻结提交 `3a2c03e9dd3b6b487c8fc7d0bb0863d5cb02fa95` 与新仓根提交 `1a592949dcebddaeaaba09cfee9186c8ea94a628` 的 tree 均为 `1ddb66884319a2ad6f9f549ab0701051421bb297`。
- 新仓根提交到受测提交之间，`src/` 与 `tests/` 无内容变化；正式源码、配置、测试和接口未因依赖迁移改变。

固定子模块：

| 依赖 | URL | SHA |
|---|---|---|
| FAST-LIO | `https://github.com/qiheiouo/FAST_LIO_MULTI_ROS2.git` | `9aa8c79ef064eafe0fae1308888248be7efa1d2e` |
| Livox ROS2 Driver | `https://github.com/qiheiouo/livox_ros_driver2_humble.git` | `ffb5c721c3356817a84f1c6eeaf2f66c680be24e` |
| ikd-Tree | `https://github.com/engcang/ikd-Tree` | `0438b0daa6ccfaaad9f65f45a3addc318b19ae7a` |

## 环境身份

- 镜像：`rm2027_navigation:humble`。
- 镜像 ID：`sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3`。
- 基础镜像：`ros:humble-ros-base-jammy@sha256:5417e56962ff6e15d4cf9b2f78a71a78f3901f47cd78696b575b7eecdb54eb78`。
- Livox-SDK2：`f5d9375f84efe2b15bc0a052d3e18482ed13adf4`（v1.3.1）。镜像内 `livox_lidar_api.h`、`livox_lidar_cfg.h`、`livox_lidar_def.h` 的 SHA256 分别为 `63b27c3...`、`fbeb3b91...`、`4245e58b...`，与依赖合同一致。
- CMake 3.22.1、GCC 11.4.0、PCL 1.12.1、Nav2 1.1.20；其余关键包版本与 `docs/dependency_and_acceptance_contract_20261021.md` 一致。
- 未重建或替换镜像，没有新增依赖，也没有用不同环境冒充冻结环境。

## 原始构建与测试命令

13 包构建和 110 项测试严格使用依赖合同中的包清单与参数：

```bash
bash tests/fastlio_deterministic/build_and_test.sh
source build/fastlio_validation/install_driver/setup.bash
colcon --log-base build/humble_log build \
  --base-paths src/rm_competition_interfaces src/rm_relocalization_bridge \
  src/rm_competition_mission src/rm_system_monitor src/rm_gicp_relocalization \
  src/rm_localization_adapters src/rm_serial_driver src/rm_lio_bringup \
  src/rm_mid360_driver_bridge src/rm_description src/rm_nav_config \
  src/rm_navigation_bringup src/rm_chassis_interface \
  --build-base build/humble_build --install-base build/humble_install \
  --executor sequential --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
colcon --log-base build/humble_test_log test \
  --base-paths src/rm_competition_interfaces src/rm_relocalization_bridge \
  src/rm_competition_mission src/rm_system_monitor src/rm_gicp_relocalization \
  src/rm_localization_adapters src/rm_serial_driver src/rm_lio_bringup \
  src/rm_mid360_driver_bridge src/rm_description src/rm_nav_config \
  src/rm_navigation_bringup src/rm_chassis_interface \
  --build-base build/humble_build --install-base build/humble_install \
  --executor sequential
colcon --log-base build/humble_result_log test-result \
  --test-result-base build/humble_build --verbose
```

最后一条只比历史合同多指定仓外 `--log-base`，因为受测源码按只读挂载，原命令会尝试在 `/workspace/log` 写默认日志。测试执行已经完成；该调整只影响结果汇总日志位置，没有重跑、跳过或改变测试。

其余定向入口：

```bash
bash tests/fastlio_deterministic/ros_and_original.sh
ctest --test-dir build/fastlio_validation/driver/livox_ros_driver2 \
  -R '^livox_driver_(lifecycle|publication|queue_concurrency)_fixture$' \
  --output-on-failure
python3 tests/fastlio_deterministic/ros_invalid_init_probe.py ...
python3 tests/aud003_final_motion/probe.py --case CASE ...
python3 tests/new_car_hardware_acceptance/run_fixture.py ...
python3 tests/new_car_hardware_acceptance/run_fixture.py --regress ...
python3 tests/new_car_integration/probe_software_entry.py
```

## 结果

### 构建与原 110 项

- FAST-LIO 与 Livox 独立构建通过，原生确定性 fixture 通过。
- 13/13 Humble 目标包顺序构建通过，用时约 6 分 14 秒。
- `rm_gicp_relocalization` 仅报告容器未启用 PCL 的可选 pcap/png I/O，不影响构建或测试。
- 13 包测试执行完成，汇总为 `110 tests, 0 errors, 0 failures, 0 skipped`，与旧冻结基线等价。

### FAST-LIO / AUD-059

- 原生 IMU fixture 三种模式均为 5 次更新，状态误差 `1.1345091532888403e-15`，协方差误差 `6.5369931689929217e-13`。
- measurement fixture：36 次调用，Jacobian 误差 `8.4276848610898014e-08`，reference/posterior error 为 0，预分配容量复用。
- 原始未修复 bundle：matched 场景按预期 `SIGABRT`（exit -6），unmatched 场景正常退出；堆栈定位到 `h_share_model` 的点云向量路径。
- 修复版 Debug 与 Release：matched/unmatched 均正常退出，各发布 6 条有限里程计；unmatched 均到达 `No Effective Points` 边界。
- 非法初始化：zero acceleration、NaN acceleration、Inf gyro 均被拒绝，非法阶段 0 条里程计；随后有效数据只初始化 1 次并发布 6 条有限里程计，正常退出。

### Livox / AUD-020 / AUD-050

- `livox_driver_lifecycle_fixture` 通过：验证退出生命周期。
- `livox_driver_publication_fixture` 通过：验证发布/FIFO 路径。
- `livox_driver_queue_concurrency_fixture` 通过：验证 SPSC 并发路径。
- 三项合计 `3/3` 通过，无失败。

### AUD-003 与软件安全入口

- AUD-003 九场景全部通过：unauthorized、normal、mission_cancel、localization_lost、authority_timeout、source_timeout、source_nonfinite、injection、recovery。
- 未授权及所有失效场景均编码零速度；非有限输入和旁路注入被过滤；旧授权不能在进程内复用，新会话可重新授权。
- 八步验收正向 fixture 判定 `GO`；时间回退 fixture 检出 IMU/odom 时间戳回退并按预期判定 `STOP`。
- 两个验收 fixture 都记录 `publishers_created=0`、`motion_commands_sent=0`。
- 无硬件软件入口通过：inventory 成功；缺少 opt-in 和 hardware 模式被拒绝；无输入时 0 条 raw odom；未确认对齐被拒绝；V2 初始禁用；通用 `/cmd_vel` 输入下 40 帧均为零；守卫保持禁用；无 controller/mission 节点；正常退出。
- `research_snapshots/` 在 colcon 包发现中为空，未进入正式构建或运行链。

## 编排差异与处理

1. 首次 110 项结果汇总因源码只读，`colcon test-result` 无法创建默认 `/workspace/log`；将其 `--log-base` 指向仓外 build 后，基于同一批 XML 汇总为 110/0/0/0。
2. 首次 Livox/AUD-059 批次在测试开始前，ROS setup 被 shell 的 `set -u` 拦截；调整为先 source 再启用严格模式后完整执行并通过。
3. 未修改源码、配置、Dockerfile、子模块 pin 或运动授权逻辑来取得绿色结果。

## 发布与硬件边界

- 从代码恢复、固定依赖、冻结环境和软件回归角度，已满足建立**正式私有 GitHub 主仓的软件技术条件**；仍应将其标记为 Integration 软件候选，而不是 Deployment Go。
- 根仓没有统一 LICENSE；`rm_chassis_interface`、`rm_description`、`rm_localization_adapters`、`rm_mid360_driver_bridge`、`rm_nav_config`、`rm_navigation_bringup`、`rm_navigation_launch` 仍为 `TODO`，`rm_serial_driver` 为 `Proprietary`，部分研究快照为 `NOASSERTION`。这些问题阻塞公开发布，也需要在任何超出团队授权范围的分发前解决。
- FAST-LIO 为 GPL2，Livox driver 为 MIT；第三方修改声明、fork 发布范围和长期维护责任仍需正式确认。
- apt 依赖没有逐包写死在 Dockerfile；当前结果依赖已存在的精确冻结镜像。未来重建若仓库版本漂移，不能宣称环境等价。
- 真实 `/data/rm27_maps`、地图/PCD、车型标定、footprint、双 MID360 拓扑、MCU/串口、底盘模式、急停和物理停止距离未验证；这些继续保持 Hardware-blocked。
- 原旧仓和 R4 dirty worktree 的 HEAD 与 dirty 文件清单在验收前后完全一致，本轮没有修改、stash、reset、clean 或 checkout 它们。

