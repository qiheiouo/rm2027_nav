# FAST-LIO covariance publication contract

2026-10-07，Research，独立分支 `experiment/fastlio-covariance-contract`。

**Go，仅限 FAST-LIO 原始 odometry 的单时刻 covariance publication contract 在固定源码 / C++ probe 层成立。**
三项候选均有独立反例；最小修复覆盖 bundle、adaptive、async 三个入口。
这不是 Task 1 Go，不进入 Task 2，也不改变其他定位 / 动态预测 / MPC 研究结论。

## 固定基线与隔离

| 项目 | 身份 / 核验 |
| --- | --- |
| Superproject base | `bd6cf689d3d5aca5c8edd430ff760ca5ceb27496`，直接派生于 `origin/main-new-car` |
| 基线是否前进 | 只读 `git ls-remote origin refs/heads/main-new-car` 得到同一 SHA；与定位研究固定源码一致 |
| 相关差异 | 基线相对固定 pin 的 `src/fast_lio_multi` gitlink 和整个 `src/rm_localization_adapters` 均无差异 |
| 本任务 worktree | `/home/qihei/rm2027_navigation/build/fastlio_covariance_contract` |
| Superproject branch | `experiment/fastlio-covariance-contract`；不从 `experiment/neu-localization-baseline` 派生 |
| Submodule old commit | `e7864a63a7e2a1ad62ec8fd75fcb9149db08e321` |
| Submodule experimental commit | `ea63e62b20c5b5cddcaf01df334ffa1da75b3f72`，parent 为上述 old commit |
| Submodule branch | 独立子模块仓库内的 `experiment/fastlio-covariance-contract` |
| Submodule Git directory | `/home/qihei/rm2027_navigation/.git/worktrees/fastlio_covariance_contract/modules/src/fast_lio_multi`；git-common-dir 与之相同，不共享其他 worktree 的子模块 working tree / refs |
| 其他工作区 | 首次检查发现主工作区处于动态研究分支且有未提交改动；未在其中编辑、切换、reset 或 clean |
| Build / 临时目录 | 仅本任务 worktree 内 `build/fastlio_covariance_contract_probe`；无 ROS / DDS 进程，无 Domain 占用 |

只有本实验的代码、测试、证据、报告和 gitlink 被提交；不修改 main / main-old-car，不合并实验分支，不 push。
机器身份及源码散列见 [revisions.json](../../experiments/fastlio_covariance_contract/evidence/revisions.json)。

## A：复核事实

使用真实 `use-ikfom.hpp`、MTK `boxplus`、ESKF `get_P()`、ROS message 类型，直接编译旧 / 候选源文件提取出的发布函数体。
这里的 publisher / broadcaster 保存消息副本；没有 ROS executor、点云求解、实车或 bag 回放。
[baseline.json](../../experiments/fastlio_covariance_contract/evidence/baseline.json) 保留旧版本的反例。

| 检查 | 源码及实测结果 |
| --- | --- |
| State layout | `use-ikfom.hpp` 的 position 在 index 0，rotation 在 index 3；之后依次为 extrinsic rotation / translation、velocity、gyro bias、acc bias、gravity。state DOF=23；gravity S2 使表示维数与 DOF 不同 |
| Covariance layout | ESKF `cov` 为 state DOF×DOF；真实 `get_P()` 是 23×23。前6维与 state `boxplus` 的 index 对应，取出的6×6是 pose marginal，不是信息矩阵 |
| Tangent | 真实 `boxplus` 验证 `p'=p+dp_W`、`R_WB'=R_WB Exp(dphi_B)`；位置是 world additive，旋转是 body right tangent，不能直接把两者当同一 frame |
| 发布时序 | 三个入口均先 publish 后填 P。已知 diagonal 输入首帧输出0；第二帧 pose 已更新，covariance 仍为上一轮且已交换的 diagonal |
| 索引 | 旧代码行列都使用 `[3,4,5,0,1,2]`，把 position-first 变成 rotation-first；ROS 消息要求位置3维在前，旋转3维在后 |
| zero-start | bundle 固定 `G=T_WB(t0)^-1`，之后发布 `G*T_WB`；原 covariance 路径未处理 G。连续两次相同输入消除 lag，再撤销已知排列，90°启动旋转的 xyz variance 仍是 `[.09,.01,.04]`，应为 `[.01,.09,.04]` |

旧版本入口位置：bundle `publish_odometry` 原行728，adaptive 原行807，async 原行589；
`SOn.hpp` 的 SO3 `boxplus` 右乘，ESKF 在 update 完成后返回本轮 state 与 P。
三个调用点均在 `update_iterated_dyn_share_modified` 和 `state_point=kf.get_x()` 之后，当前入口为 `rclcpp::spin` 单线程 executor；发布函数中没有再次更新 filter。
本合同不覆盖未来多线程同时修改 filter 的情形。

[ROS PoseWithCovariance](https://raw.githubusercontent.com/ros2/common_interfaces/jazzy/geometry_msgs/msg/PoseWithCovariance.msg)
定义36个 row-major 元素，顺序为位置 xyz 和固定轴 XYZ 旋转；
[Odometry](https://raw.githubusercontent.com/ros2/common_interfaces/jazzy/nav_msgs/msg/Odometry.msg)
将 pose 放在 `header.frame_id` 中，twist 使用 child frame。
本候选把旋转 covariance 明确定义为**nominal pose 附近、parent 固定轴的无穷小左旋转误差**，单位 rad；位置单位 m。
它与 [ROS tf2 的 covariance 父坐标变换](https://raw.githubusercontent.com/ros2/geometry2/jazzy/tf2_geometry_msgs/include/tf2_geometry_msgs/tf2_geometry_msgs.hpp)
对四个3×3 block 施加旋转的语义一致，并在 C++ 中调用安装的 tf2 实现作独立对照。
这不是全局 Euler-angle 数值差的 covariance，也不是 SE(3) right-log residual 的 covariance。

## B：最小候选与合同

令 `B` 为 native IMU/body 状态原点，`W` 为 filter 数值 world，`F` 为实际发布的 parent 数值基准。
已知且在本次线性化中固定的 `G=T_FW=(A,b)` 给出：

```text
p_F = A p_W + b
R_FB = A R_WB
e_native = [dp_W, dphi_B]
e_ROS    = [dp_F, dtheta_F]
dp_F     = A dp_W
dtheta_F = R_FB dphi_B
J        = diag(A, R_FB)
Sigma_ROS = J P_native[0:6,0:6] J^T
```

四个 block 分别是 `A P_pp A^T`、`A P_pφ R_FB^T`、`R_FB P_φp A^T`、`R_FB P_φφ R_FB^T`。
不丢弃 off-diagonal 或 position/rotation cross terms。
这里改变 parent basis，不改变被估计的 body 原点；parent translation b 不引入 lever-arm 项。
把整个 SE(3) adjoint 直接用在这里会混淆 additive position 与 Lie translation。

| 模式 / 阶段 | A | R_FB |
| --- | --- | --- |
| adaptive / async | I | 当前 `state_point.rot` |
| bundle，zero-start 关闭 / 尚未初始化 | I | 当前 `pub_quat` 的 rotation |
| bundle，zero-start 已初始化 | `T_world_to_zero` 的 rotation | 已 zeroed 的当前 `pub_quat` rotation |

[helper](../../src/fast_lio_multi/include/pose_covariance.hpp) 只有上述6×6转换和 row-major 写出。
三个入口在 publish 之前填充本轮 `kf.get_P()`；保持原 pose、TF、source stamp 生成路径。
不重构 filter，不改零起点逻辑、配置、点云、地图或 adapter。
helper 共33行，生产差异4个文件，48行增加 / 33行删除。

**zero-start 的范围必须区分：** 本输出只是把当前 native marginal 按已选定且冻结的数值 G 重表达。
G 虽由估计的启动 pose 选取，传播时不把它作为随机变量；不宣称已经求得真实统计条件分布，也不是相对随机启动状态的 covariance。
因此启动处 nominal pose 为 identity 时 covariance 仍可非零，不能人为归零。
若要求 `T(t0)^-1*T(t)` 的随机相对误差，必须有启动状态不确定性、cross-time covariance 及对应相关项；本轮没有这些量。
原 zero-start 延迟期间仍可能切换数值基准却沿用同一 frame 名；本轮只验证各单时刻重表达，不提供跨该切换的 epoch / 相对因子合同。

## C：验证

[contract.json](../../experiments/fastlio_covariance_contract/evidence/contract.json)
包含14组通过的检查；[复现入口](../../experiments/fastlio_covariance_contract/README.md)。

| 要求 | 覆盖 |
| --- | --- |
| 已知 diagonal | 三个真实发布入口 first emitted frame；identity basis 下逐值保留位置 / 旋转顺序 |
| 非 diagonal | 明确正定 dense fixture，36个字段、cross terms、旋转姿态、连续更新 |
| parent rotation | 复合非平面父坐标旋转；真实 `state.boxplus` 中心差分，以及真实 ROS tf2 变换对照 |
| consecutive updates | 三个入口改变 state / P / stamp 后，捕获当前帧完整消息和相同 pose / stamp 的 TF |
| first frame | 首次 publish 即包含当时 P，不继承全零数组 |
| zero-start | 未初始化、90°启动处、后续 dense 更新、关闭开关时忽略 stored transform |
| identity / rotated basis | 恒等变换、姿态旋转、父坐标旋转、native world 重表达的一致性；symmetry / positive definiteness |

中心差分 step `1e-6`；候选 covariance 与真实 state 扰动传播的最大元素误差 `2.7690755938225564e-11`，断言容差 `1e-9`。
ROS Jazzy 对照 headers 用 C++17；生产 helper 与真实 ESKF 类型另外通过 C++14 syntax check。
没有完整 FAST-LIO package 构建、DDS transport、传感器时间对齐或实车验证；这些不包含在本轮 Go 中。
测试先在旧 pin 上记录反例，再验证候选；源码散列核验将测试时尚未提交的候选明确对应到 `ea63e62`。

## D：子模块关系与恢复

子模块 `ea63e62` 是 `e7864a6` 的直接子 commit；superproject 的实验提交将 gitlink 指向 `ea63e62`。
最终研究分支和子模块 working tree clean。子模块实验 commit 未 push，普通远端 clone 不会自动取得它。
为此随主仓保存 [Git bundle](../../experiments/fastlio_covariance_contract/evidence/fast_lio_candidate.bundle)，
内含实验 branch / commit，prerequisite 为 old commit；已用 `git bundle verify` 验证。
另在本任务 build 内的临时仓库中只导入 old commit，确认 new commit 尚不存在，
再从 bundle 恢复 `ea63e62` 并核对 helper 内容；验证后移除这个一次性仓库。

在其他**专用** worktree 恢复实验时，先在 superproject baseline 上初始化 FAST-LIO 到 old commit，
再将本报告同仓的 bundle fetch 到该专用子模块：

```bash
git -C src/fast_lio_multi fetch /absolute/path/to/fast_lio_candidate.bundle refs/heads/experiment/fastlio-covariance-contract
```

之后切到 superproject 的 `experiment/fastlio-covariance-contract` 并执行
`git submodule update --init src/fast_lio_multi`，使 gitlink 与已导入对象一致。
`submodule update` 使用 detached HEAD 也正确；branch 是保留实验 commit 的名字，主仓依赖精确 SHA。
本任务子模块 origin 由本地 fixed-source repository 初始化；没有改原子模块或 canonical `.gitmodules` URL。

恢复旧行为时，在本任务**干净** worktree 中执行以下操作；不要对其他任务执行，也无需 reset / clean：

```bash
git switch --detach bd6cf689d3d5aca5c8edd430ff760ca5ceb27496
git submodule update --init src/fast_lio_multi
```

结果是 superproject 固定 baseline + submodule old commit，两者一致且实验分支仍保留。
恢复候选则 `git switch experiment/fastlio-covariance-contract` 后再次执行同一 submodule update。
以上恢复命令仅记录，本轮没有切走研究分支。

## 决策边界

**Go：原始 FAST-LIO 单时刻 marginal 的发布顺序、位置 / 旋转顺序、parent 固定轴 tangent，以及冻结 numerical-basis 传播，在源码 / 单测层成立。**

不表示 covariance 已实车标定、可直接作为 factor graph information、cross-time covariance 已解决，或 odom 与 map factor 独立。
不意味着整个 `/odometry/lio` canonical covariance 已正确；adapter 仍复制数组并只在 heading 模式替换 yaw diagonal，
其杆臂、parent 转换、heading / calibration 不确定性必须另有合同。
不进入 Task 2，不代表 Task 1 Go，不更改其他研究分支。
