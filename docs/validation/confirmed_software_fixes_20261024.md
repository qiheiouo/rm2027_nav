# 三项已确认软件缺陷修复（2026-10-24）

Integration；从 `main d37f8d6` 开始，依据 Needs-Evidence Closure `82bc206`，
严格按 016 → 052 → 056 独立提交。不重新审计，不修改 Mission/比赛策略。
运行环境复用 Humble/Nav2 1.1.20 镜像，禁网、私有 IPC、无设备、非 root、
cap-drop ALL/no-new-privileges、2 CPU/2 GiB。软件证据不能作为 Hardware Go。

## AUD-016

正确合同：costmap 不 current 时可被 cancel、server 停用或 context 退出打断；
取消在等待结束前被观察到后，旧请求不得继续计算路径/速度；终态不依赖地图恢复。
不承诺抢占一个已进入执行的 controller/planner plugin，不以 action ACK 冒充物理停止。

根因：planner 等待循环仅检查 currentness，调用方不再检查 cancel；controller 内层等待
也绕过外层取消检查，恢复 currentness 后先执行一次旧速度计算。

最小复现：真实安装 planner/Navfn + 原 `nav2_phase2f_deployment.yaml`，首图缺失，
cancel ACK 成功，0.804s 后仍无终态；地图恢复后旧 goal SUCCEEDED、18 poses。
同一正确性回归在未修复 Nav2 上两种规划 action 均失败（取消等待未完成）。

修改：固定源码 overlay 中为两类 planner 等待传入本 action server；等待中检查取消、
server/context；controller 等待循环及其返回边界检查同样条件并先发零再终结。
保留原 100Hz 轮询频率，使用 WallRate，避免依赖暂停的 ROS 时间；未加入取消超时。
[补丁来源/加载/回退](../../dependencies/nav2_humble/README.md)。系统包本身未改。

回归入口：[真实服务器回归](../../tests/confirmed_software_fixes/README.md)。
planner 覆盖 ToPose/ThroughPoses 的缺图取消和新请求恢复；controller 使用显式启用的
currentness layer、setPlan 事件和计算计数。fixture 初版未设 enabled_，层被忽略；
该次非零不计作有效缺陷证据，修正后重新跑 baseline/修复版本。

结果：修复后 3/3 CTest 通过（4.72s）；两类 planner 的取消终态分别约 8.88/9.07ms，
controller 约 10.43ms，均在地图/currentness 未恢复时完成。controller 计算 0 次、非零 0 次，
恢复后新 goal 计算 1 次并可取消；三次服务器清理退出码均 0。这是主机观测，非 SLA。
修正 fixture 的启用标志和更新区域后，未修复 controller 也稳定失败于取消仍阻塞。

兼容性：对本地 main 与全部既有候选 ref 逐个临时 index 应用检查。
main、software-debt/needs-evidence、mission-layer、intent-arbitration、posture、
localization-acquisition/z-integration/upstream-parity、omni 及治理 ref 均无补丁冲突。
dynamic-surface / hws-shadow 的代码补丁无冲突，但仍使用重构前 docs/runtime_profiles.md，
完整提交迁入需将文档说明映射到该路径；不为本任务改动这些分支。
未改比赛配置/目标选择/动作参数；未声称执行所有候选分支的完整联合运行。
建议进入 main 的软件修复候选，前提是构建并加载对应 overlay；仍无 Hardware Go。

Hardware gate 保留：目标机所选地图/传感器更新与负载下的取消/恢复延迟、原速度链、
真实串口/MCU 停车和重新授权验收。本次未运行 smoother/串口/MCU。
