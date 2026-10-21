# 东北大学研究串行路线

2026-10-07；按用户最新指令调整。属于 Research 路线，不改变正式 launch、TF / topic owner 或运动授权。当前只推进 **Task 1**；沿用唯一东北大学定位研究分支，其他研究入口冻结。

## 顺序与边界

| Task | 最小目标 | 进入条件 / 当前状态 |
| --- | --- | --- |
| 1 多源因子图定位后端 | 保留 FAST-LIO；离线 odom + scan-to-map 的 SE(3) 最小闭环；明确输入、权重及 map/odom 关系 | **Modify / waiting for real vehicle evidence**；[检查表与恢复入口](neu_factor_graph_task1_20261007.md#8-实车到位检查表与恢复入口) |
| 2 定位可信性、异常与恢复 | 在后端基础上复用 GICP quality、integrity、高转速 / recovery 历史；研究退化、漂移 / 发散、因子取舍、reset / epoch | Task 1 Go 后进入；未开始 |
| 3 前后台异步障碍地图 | 审计 perception 更新与 planner 访问的阻塞；STVL / obstacle layer 为 baseline | 定位后端及恢复研究 Go 后进入；未开始；不恢复动态预测 |
| 4 T-DT 轨迹输出收口 | 已迁移 A* / DP / Minimum-Snap 不重做；时间、系数 / 导数、corridor、heading、terminal 的 rich reference contract；保留 Nav2 Path baseline | 前序 Go 后进入；未开始 |
| 5 TinyMPC / 全向控制 | 合同明确后定义 state / input / frame / dt / horizon / 约束 / 下位机语义；先静态跟踪，与正确输入的 MPPI 公平 A/B | Task 4 Go 后进入；无收益则 Stop；未开始 |
| 6 系统闭环 | 前五项 Go 后联合验证 stale、LOST、reset / epoch、replan、timeout、旧轨迹失效、stop / resume / fallback | 未开始；不提前接未验证模块 |

每项按 `Audit → Minimal Research → Go / Modify / Stop` 执行。Go 的对象和证据范围必须写清楚：离线实现可运行，不自动意味着硬件有效或可以 Deployment。尚未达到该 Task 的核心效果标准时，不能用“软件 Go”绕过 Modify 进入下一项。

Modify 不进入无限阈值扫描。若缺真实数据 / 硬件，则留下最小复现入口与明确缺口并暂停该项。进入后续无依赖纯软件研究需要另行明确说明依赖为什么不被破坏；不得静默跳过本顺序。

## 已有资产如何处理

- [L0](neu_localization_controller_second_audit_20261007.md)、[L1 / L1-M](neu_localization_l1_baseline_20261007.md) 保留为历史证据；其中“先 L2、暂缓因子图”的旧任务顺序由本路线替代。
- [L2 global candidate prototype](neu_localization_l2_global_candidate_prototype_20261007.md) 冻结在 `976b249e`。不继续用 toy 调参；真实新车 A 数据与地图到位后，作为 Task 2 的 initialization / LOST 候选入口。manual / configured prior 与无 prior 三种入口保留。
- 新车研究继续选择 A，B 整套 CAD 假设保留；真实 topic / IP / role、外参和时钟仍未知，用户已经确认没有实车记录。
- UWB、loop closure、Scan Context 在 Task 1 只保留概念接入边界，不实现在线融合或检索。因子图不会自动消除重复地点的错误匹配，也不会使相同点云来源变成独立观测。
- 条件升级项：逐点自适应前端、多尺度概率平面 / iVox / octree、UWB / 雷达站 ego 融合、IPOPT、动态预测、决策、无图探索；仅在当前路线暴露具体瓶颈时再选。FAST-LIO 足够时不替换前端。

所有新资产留本地研究分支，后续按用户要求统一上传；main 不变。原自动继续任务保持暂停，本路线不自行恢复定时执行。
