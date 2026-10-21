# 冻结研究代码快照

本目录全部内容为 **Research / Frozen** 资产，只用于理解、复核或在获得新授权后恢复研究。它们不是正式 ROS 工作区的一部分，不得作为当前运行链、部署能力或安全验收结果引用。

顶层 `COLCON_IGNORE` 隔离整个目录；不要删除该隔离标记后直接混入正式 workspace。恢复研究时应使用独立工作区，并重新核对原固定提交、依赖和外部证据。

| 目录 | 固定来源 | 原路径 | 许可证状态 |
|---|---|---|---|
| `full_base_heading_adapter/` | `5733f126b71b52e7f4ddd2340b226c1589b4419b` | `experiments/full_base_heading_adapter/` | `NOASSERTION`；来源树无独立许可证文件 |
| `full_base_simulation_validation/` | `4d7f40cbe4645267c002e4dd9cab2f9f252b1257` | `experiments/full_base_simulation_validation/` | `NOASSERTION`；来源树无独立许可证文件 |
| `fastlio_covariance_contract/` | `04364d50a3db1f3a6dd79575e5532bb2f4130ab0` | `experiments/fastlio_covariance_contract/` | `NOASSERTION`；来源树无独立许可证文件 |
| `fastlio_covariance_ros_validation/` | `42deff2a803ecfddf0c6264007a6e7a2ba9303ea` | `experiments/fastlio_covariance_ros_validation/` | `NOASSERTION`；来源树无独立许可证文件；bundle 已排除 |
| `neu_factor_graph/` | `bfc235463eb621ac377c2226c0b988c4d901679f` | `experiments/neu_factor_graph/` | `NOASSERTION`；来源树无独立许可证文件 |
| `neu_localization_l1/` | `bfc235463eb621ac377c2226c0b988c4d901679f` | `experiments/neu_localization_l1/` | `NOASSERTION`；来源树无独立许可证文件 |
| `neu_localization_l2/` | `bfc235463eb621ac377c2226c0b988c4d901679f` | `experiments/neu_localization_l2/` | `NOASSERTION`；来源树无独立许可证文件 |
| `tdt_planner/` | `e374894a7a713ea84b82de9e92479a37041f39ff` | `experiments/tdt_planner/` | 保留 package `LICENSE`、vendor `LICENCE`、`UPSTREAM.json` 与 `LOCAL_CHANGES.patch` |
| `prediction_value/` | `4d59854fb8a2937ab9cf884f684cdc286842932c` | `experiments/prediction_value/` | `NOASSERTION`；来源树无独立许可证文件 |

所有 `evidence/`、日志、bundle、大型结果和原始输入均未复制。需要复现时，先按 `docs/migration_provenance.md` 和旧仓的 SHA256 manifest 恢复外部证据；缺少证据时不得把快照可读等同于结果可复现。
