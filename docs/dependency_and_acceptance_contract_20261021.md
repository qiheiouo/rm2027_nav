# RM2027 依赖与验收合同（2026-10-21）

## 状态与边界

- 阶段：Integration / Dependency Migration Candidate。
- 正式软件来源：旧仓 `3a2c03e9dd3b6b487c8fc7d0bb0863d5cb02fa95`；已验证源码父提交 `cc073ab17d85e9f29de87533c9aa4a06f59bd728`。
- 本文固化依赖恢复和后续回归入口，不表示本候选已重跑 110 项测试，也不构成 Deployment Go。
- 不启动硬件、不授权运动；真实地图、车型标定、footprint、底盘模式与物理停止数据仍是部署外部输入。

## 子模块恢复合同

| 依赖 | URL | 固定 SHA | 远端恢复状态 |
|---|---|---|---|
| FAST-LIO | `https://github.com/qiheiouo/FAST_LIO_MULTI_ROS2.git` | `9aa8c79ef064eafe0fae1308888248be7efa1d2e` | 主仓 gitlink；须精确 checkout |
| Livox ROS2 Driver | `https://github.com/qiheiouo/livox_ros_driver2_humble.git` | `ffb5c721c3356817a84f1c6eeaf2f66c680be24e` | 维护分支 `rm2027/aud020-aud050-20261020`；全新 HTTPS clone 已验证 |
| ikd-Tree | `https://github.com/engcang/ikd-Tree` | `0438b0daa6ccfaaad9f65f45a3addc318b19ae7a` | FAST-LIO 内嵌 gitlink；该远端 `lio` ref 包含固定对象 |

Livox 维护仓保留原父链：

```text
2a2029a6e62a2196b280be6ec00bb2418065b8e0  original intake
  -> 9bdedb484b1dd9bd7420423a728812dcf8cc8267  AUD-020
  -> ffb5c721c3356817a84f1c6eeaf2f66c680be24e  AUD-050
```

不得改写这些提交或用复制文件生成替代提交。主仓和子模块均不得依赖旧仓 alternates、旧 Gitee 主仓归档 ref 或已有子模块缓存。

## Humble 镜像与构建依赖

- 冻结验收镜像：`rm2027_navigation:humble`。
- 镜像 ID：`sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3`。
- 镜像创建时间：`2026-07-22T11:05:39.518996204Z`。
- 基础镜像：`ros:humble-ros-base-jammy@sha256:5417e56962ff6e15d4cf9b2f78a71a78f3901f47cd78696b575b7eecdb54eb78`；本机对应 image ID `sha256:151f70450bceb3669d183b2cc1ab203055eb68ffda3846794da043dcfb705a93`。
- Livox-SDK2：`https://github.com/Livox-SDK/Livox-SDK2.git`，固定 `f5d9375f84efe2b15bc0a052d3e18482ed13adf4`（tag `v1.3.1`）。

Livox-SDK2 pin 的确认链：冻结镜像构建历史记录 `LIVOX_SDK2_REF=master`；镜像在 2026-07-22 构建，官方提交图中此前的 `master` 顶端为 `f5d9375f…`，下一提交 `68ae1e1d…` 的提交时间为 2026-07-24；镜像内三个安装头文件 SHA256 与 `f5d9375f…` 逐字节一致，其中 `livox_lidar_api.h` 和 `livox_lidar_def.h` 与下一提交不同：

| 文件 | SHA256 |
|---|---|
| `/usr/local/include/livox_lidar_api.h` | `63b27c3dfd6b4464a052a57c02a1a2f3994a0e81115082d8e9ed4283a2371d58` |
| `/usr/local/include/livox_lidar_cfg.h` | `fbeb3b91897fad5b71b169e4e4f88bfb3b4b716cecb07e45aa08569cf159e9a5` |
| `/usr/local/include/livox_lidar_def.h` | `4245e58b212f051e5d02181e995f830d227cdf5caa4819d0281ad0458676b7b1` |

冻结镜像中与本验收直接相关的关键已安装版本：

| 依赖 | 版本 |
|---|---|
| CMake | `3.22.1-1ubuntu1.22.04.2` |
| GCC / G++ 11 | `11.4.0-1ubuntu1~22.04.3` |
| `libapr1-dev` / `libaprutil1-dev` | `1.7.0-8ubuntu0.22.04.2` / `1.6.1-5ubuntu4.22.04.2` |
| `libpcl-dev` | `1.12.1+dfsg-3build1` |
| `libqhull-dev` | `2020.2-4` |
| Nav2 | `1.1.20-1jammy.20260612.*` |
| `ros-humble-pcl-conversions` / `pcl-ros` | `2.4.5-2jammy.20260605.160413` / `2.4.5-2jammy.20260606.043532` |
| `ros-humble-openvdb-vendor` | `2.3.4-1jammy.20260304.075105` |
| `ros-humble-spatio-temporal-voxel-layer` | `2.3.4-1jammy.20260607.082704` |

Dockerfile 直接固定基础镜像 digest、Livox-SDK2 SHA、OpenVDB vendor 与 STVL；其余版本用于核验冻结镜像，不应被误称为 Dockerfile 已逐包锁定。若 apt 仓库不再提供相同版本，应先建立获准的包快照方案，不能自动升级后沿用“同一冻结环境”的表述。

## 13 包构建与 110 tests 合同

目标包按冻结验证顺序为：

1. `rm_competition_interfaces`
2. `rm_relocalization_bridge`
3. `rm_competition_mission`
4. `rm_system_monitor`
5. `rm_gicp_relocalization`
6. `rm_localization_adapters`
7. `rm_serial_driver`
8. `rm_lio_bringup`
9. `rm_mid360_driver_bridge`
10. `rm_description`
11. `rm_nav_config`
12. `rm_navigation_bringup`
13. `rm_chassis_interface`

冻结报告产生 110 tests 的命令为：

```bash
set -eo pipefail
ulimit -c 0
source /opt/ros/humble/setup.bash
mkdir -p build
bash tests/fastlio_deterministic/build_and_test.sh
source build/fastlio_validation/install_driver/setup.bash
export ROS_LOG_DIR=/workspace/build/humble_ros_logs
colcon --log-base build/humble_log build \
  --base-paths src/rm_competition_interfaces src/rm_relocalization_bridge \
  src/rm_competition_mission src/rm_system_monitor src/rm_gicp_relocalization \
  src/rm_localization_adapters src/rm_serial_driver src/rm_lio_bringup \
  src/rm_mid360_driver_bridge src/rm_description src/rm_nav_config \
  src/rm_navigation_bringup src/rm_chassis_interface \
  --build-base build/humble_build --install-base build/humble_install \
  --executor sequential --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
source build/humble_install/setup.bash
colcon --log-base build/humble_test_log test \
  --base-paths src/rm_competition_interfaces src/rm_relocalization_bridge \
  src/rm_competition_mission src/rm_system_monitor src/rm_gicp_relocalization \
  src/rm_localization_adapters src/rm_serial_driver src/rm_lio_bringup \
  src/rm_mid360_driver_bridge src/rm_description src/rm_nav_config \
  src/rm_navigation_bringup src/rm_chassis_interface \
  --build-base build/humble_build --install-base build/humble_install \
  --executor sequential
colcon test-result --test-result-base build/humble_build
```

冻结基线结果是 `110 tests, 0 errors, 0 failures`。后续只有在新仓独立递归恢复后，用上述镜像身份、子模块 pin 和命令重跑通过，才能把这一结果归因于新仓。

## 独立递归恢复记录

依赖迁移提交 `98af38e25ba79537fad75f701f2f0c18e241a9a4` 已从持久化候选以 `git clone --no-local` 恢复到全新临时目录，并使用禁止 file protocol、禁止 submodule alternates 的递归初始化。实际网络来源及结果为：

- FAST-LIO：GitHub fork，checkout `9aa8c79ef064eafe0fae1308888248be7efa1d2e`；
- Livox：GitHub 维护仓，checkout `ffb5c721c3356817a84f1c6eeaf2f66c680be24e`；
- ikd-Tree：`https://github.com/engcang/ikd-Tree`，checkout `0438b0daa6ccfaaad9f65f45a3addc318b19ae7a`；
- 主仓和三个子模块均 clean，四个仓库 `git fsck --full` 通过；
- 主仓 `.git` 及其递归子模块对象库没有 `objects/info/alternates`；
- 没有访问旧 Gitee 主仓归档 ref，也没有用旧仓或既有子模块对象缓存代替网络恢复。

该记录证明 Git 对象和固定依赖可恢复；不等同于编译、110 项测试、仿真或硬件验收。

## 后续验收门

1. 使用固定镜像重跑 FAST-LIO 确定性脚本、13 包 build 与 110 tests。
2. 重跑 AUD-003 九场景、新车八步验收工具和关键无硬件 smoke。
3. 发布前解决根仓许可证、一方 package 的 `TODO`/`Proprietary` 声明、研究快照 `NOASSERTION`、第三方修改声明与长期维护责任。

本阶段只关闭依赖恢复与验收合同记录，不创建正式 RM2027 GitHub 主仓、不宣称完整软件回归或 Deployment Go。
