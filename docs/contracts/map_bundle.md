# 地图 Bundle 与资产合同

> Current：适用于本研究归档分支的既有接口/profile；源自 `phase2e_map_bundle.md`。历史验收与待办章节只保留原条件，不提升实车结论。

## Goal

Phase 2E defines one reproducible artifact boundary for map assets. A map is
not accepted as a loose PCD/YAML/PGM collection.
It must have one `*.bundle.yaml` manifest that ties the files to the same
canonical `map` frame, revision and hashes.

This phase does not run small_gicp and does not claim that the synthetic test
map represents a RoboMaster field.

## Manifest Contract

Required root fields:

- `schema_version: 1` or `2`
- stable `map_id` and `revision`
- `deployment_status`: `test_only`, `candidate`, or `approved`
- `frame_id: map`
- source method and UTC creation time
- PCD path, frame and SHA-256 when `map_type` requires PCD
- occupancy YAML/image paths, frame and SHA-256 values
- explicit confirmation that PCD and occupancy coordinates share one map
  origin, or occupancy-origin review for an occupancy-only map

Schema version 1 remains the dual-artifact contract and is interpreted as
`map_type: occupancy_with_pcd`.

Phase 2J adds backward-compatible schema version 2. It requires an explicit
`map_type`:

- `occupancy_with_pcd`: retains the PCD and shared-origin requirements;
- `occupancy_only`: contains only occupancy YAML/image artifacts, forbids a
  fake PCD entry, and requires `alignment.occupancy_origin_reviewed`.

An approved occupancy-only bundle must set the origin-review flag to true.

All paths are relative to the manifest and must remain inside its bundle
directory. Absolute paths and `..` escapes are rejected.

An `approved` bundle is a human-reviewed deployment artifact. The validator
checks its structure and bytes; it cannot prove geometric alignment by itself.

## Validation Coverage

`rm_map_tools validate_map_bundle` checks:

1. schema, identifiers, deployment state and canonical frame;
2. path containment and artifact existence;
3. SHA-256 for required artifacts;
4. when present, PCD `FIELDS` containing x/y/z, positive dimensions, point
   count and data mode;
5. ASCII PCD row count when applicable;
6. occupancy resolution, origin, negate and threshold ordering;
7. occupancy YAML image reference matching the manifest;
8. P2/P5 PGM header and P2 pixel count;
9. approved-state and applicable alignment-review gates. An artifact labelled
   `approved` must satisfy its review flags even when inspected through a
   permissive experimental runtime policy.

Runtime acceptance is separate from structural validation. Candidate use does
not skip hashes, path containment, frame checks or artifact parsing; it only
allows `deployment_status: candidate` when the operator explicitly selects
`allow_candidate`.

## Synthetic Fixture

`src/rm_map_tools/maps/phase2e_test` contains test-only fixtures: the original
four-point/four-pixel bundle, an occupancy-only schema-2 bundle, and an
asymmetric 522-point GICP smoke-test bundle. None may be used for navigation.

Git attributes preserve PCD/PGM bytes and force map YAML to LF so manifest
hashes remain stable across Windows and Linux checkouts.

## Linux Validation

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select rm_map_tools rm_serial_driver
source install/setup.bash
colcon test --packages-select rm_map_tools rm_serial_driver \
  --event-handlers console_direct+
colcon test-result --verbose
```

Validate the installed fixture:

```bash
MANIFEST="$(ros2 pkg prefix rm_map_tools)/share/rm_map_tools/maps/phase2e_test/phase2e_test.bundle.yaml"
ros2 run rm_map_tools validate_map_bundle "$MANIFEST" --json
```

Expected: success, `frame_id=map`, four PCD points, and a 4x4 occupancy image.

The deployment gate must reject the fixture:

```bash
ros2 run rm_map_tools validate_map_bundle "$MANIFEST" --require-approved
```

Expected: non-zero exit with `map bundle is not approved for deployment`.

## small_gicp Boundary

A future adapted small_gicp launch must accept a bundle manifest, validate it
with `--require-approved`, and only then resolve the PCD path. It must not take
an unrecorded absolute `prior_pcd_file` as the production interface.

The backend still publishes `/localization/global_pose`; the existing Phase 2C
bridge remains the only canonical `map -> odom` publisher.

## Real Map Approval

Before changing a bundle to `approved`, record:

1. field and robot revision;
2. mapping trajectory and LIO configuration;
3. PCD cleaning/downsampling process;
4. occupancy-map generation method and resolution;
5. common origin/yaw verification in RViz and measured field landmarks;
6. repeated localization results from known initial poses;
7. artifact hashes and reviewer.

STEP/CAD-generated occupancy maps may be candidates, but they must be aligned
and compared against measured point clouds before approval.

Large competition PCD files should not be committed casually to the source
repository. Select Git LFS or an artifact store first, keep the manifest and
hash under review, and ensure deployment can reproduce the exact approved
bytes without relying on an engineer's home-directory path.

## Z-Localization prior ↔ Nav2 坐标合同（parity 候选，2026-10-24）

本节只约束隔离 Z 候选；不改变 main 的地图接受政策。采用右手系、米、弧度，
齐次列向量 `p_A = T_A_B p_B`，所有 SE(3) rotation 必须正交且 det=+1，不含 scale/reflection。
PCD 本身不携带可靠 ROS frame；`VIEWPOINT` 不是应再次作用于点的坐标变换。
frame、版本、文件身份和变换链必须由外部 metadata 明示，不能从文件名推断。

| 对象 | 正式语义 |
|---|---|
| mapping PCD / raw keyframe poses | mapping 运行的 gauge，记 M；没有绝对 Nav2 原点保证 |
| prior bundle / prior map.pcd | P；点、全体 keyframe poses、索引/descriptor 必须一致；`T_P_M` 记录整个 bundle 的离线变换 |
| Nav2 `map` | N；Z 候选运行前必须将整个 prior 烘焙到 N，最终 P=N=`map`、`T_N_P=I` |
| map.yaml `origin [x,y,yaw]` | **栅格局部坐标系 G 的左下外边界在 N 中的 pose**，不是 PCD 点的原点或一次额外全局 correction |
| occupancy image | 文件左上为 `(col=0,row=0)`；col 向右，row 向下；OccupancyGrid 的 y 向上 |
| CloudCompare | 仅删点时保留原数值/单位；global shift/scale、Apply transform、重采样各自记录；导出后以实际数值复核，不凭 GUI 外观 |

像素中心 `(u,v)`（整数 col,row），宽 W、高 H、resolution r：

```text
j = H - 1 - v
p_G = [(u+0.5)*r, (j+0.5)*r]
p_N = [origin.x, origin.y] + Rz(origin.yaw) * p_G
p_G = Rz(-origin.yaw) * (p_N - origin.xy)
u = floor(p_G.x/r); v = H-1-floor(p_G.y/r)
```

左/下边界包含，右/上边界不包含。不因图像行方向再对 PCD 翻转 Y。
PGM trinary 概率为 negate=0 时 `1-pixel/255`，negate=1 时 `pixel/255`；
严格 `probability > occupied_thresh` 为 occupied，`< free_thresh` 为 free，等阈值为 unknown。
依据 [Nav2 Humble map_io.cpp](https://github.com/ros-navigation/navigation2/blob/humble/nav2_map_server/src/map_io.cpp)
的 origin quaternion、阈值和行翻转实现，与现有 RM world/grid 数学保持一致。
本 checker 支持 P2/P5 8-bit PGM + trinary；其他编码必须明确转换并重新绑定文件身份。

### 唯一坐标进入路径

```text
mapping M --离线 T_P_M（整个 bundle）--> prior P
prior P --离线 T_N_P（整个 bundle；最终已烘焙）--> Nav2 N=map
ZLIO O→sensor + prior(N)配准 -> T_N_sensor(t)
T_N_base(t) = T_N_sensor(t) * inverse(T_base_sensor(t))
唯一 bridge: T_N_odom(t) = T_N_base(t) * inverse(T_odom_base(t))
```

候选 backend 的 `map` 输出已是 N；**不再乘 YAML origin**。
canonical odom 可保留 startup gauge，由 bridge 求 correction；不能把 raw correction直接改名。
不新增第二个静态 map→prior TF 来掩盖错位。checker 可计算非 identity `T_N_P` 的几何，
但必须返回 `RUNTIME_TRANSFORM_NOT_BAKED`，禁止将未烘焙 prior 用当前 runtime 直接 relabel。

### 离线流程与 metadata

1. 保留 raw mapping bundle、mapping bag、单位/机械/时钟/前端配置和 M 的定义。
2. CloudCompare 仅删点：导出 reference→删点→filter，所有点仍在同一系。
   如需调平、换原点或旋转：确定 `T_N_M`，用 upstream `transform_map_bundle` 对完整 bundle
   应用一次，再以 N 中 reference filter；不得只变 map.pcd 留旧 keyframes。
   CloudCompare scale≠1 不属于 SE(3)，先纠正单位并独立复核；禁止当刚体 metadata 接受。
3. PCD build 或 transform/filter 完成后重建所选 descriptor；记录 `T_P_M` 与 `T_N_P=I`。
   同一 prior 内容不可在线热替换；坐标/先验变化必须重启完整定位组。
4. 在同一 N 下生成 occupancy；保留 raster 原点/yaw/resolution 的独立生成记录。
   地图 YAML 的 crop/origin/旋转必须和实际图像一致；不能凭改 origin 使 RViz 看似重合。
5. 建立坐标 metadata：schema 1、dataset_kind（synthetic/new_car_real）、map_id/revision、
   frames(mapping/prior/nav2)、units=m、`T_prior_mapping`、`T_map_prior`、
   `expected_grid_origin`、`expected_resolution`、`pixel_convention=top_left_rows_down_cell_centers`、
   三个输入文件 SHA-256、在 map 中的 occupancy_z_band_m、acceptance 阈值，以及至少三个不共线
   独立 landmark。每项 landmark 明示 id、prior_xyz、map_xy、pixel=[col,row]、
   occupancy=occupied/free/unknown；可加 mapping_xyz 复核完整链。
   阈值记录 landmark/support/occupancy-sample 容差与 overlap 下限；不得按结果回调阈值。
6. 执行 `rm_map_tools check_zlocalization_coordinates`：输出 origin/orientation、SE(3)/frame、
   bounds、height-band point overlap、landmark point/pixel/PCD support、occupied sample correspondence
   和每项失败原因。exit 0 仅代表输入间软件 consistency 通过，exit 2 为失败。
   `real_map_accepted` 永远为 false；checker 不批准地图或 Hardware Go。
   PCD 支持 ASCII 和常见 PCL binary（额外 scalar/vector 字段可存在）；binary_compressed
   明确拒绝，先用 PCL 转为 binary 并重新记录文件身份，不静默猜测 LZF。

### 新车真实数据最小 acquisition / acceptance

- 获取独立 mapping bag 与 query bag；源 Livox/IMU、点时间/单位、原始 frame/session、
  动态 gimbal/home/heading、TF 和时钟可追溯。记录车型、机械 revision、前端配置与 prior ID。
- 提供未改 raw bundle、最终 prior、Nav2 YAML/PGM、全部离线变换/CloudCompare shift-scale
  导出记录和相同版本 hashes；至少三个不共线、覆盖地图空间的实测 landmark，避免只选同一面墙。
  landmark prior_xyz 必须在实际 PCD 有支持；map_xy 和 pixel 注记来自独立测量/栅格生成记录，
  不能由待检变换复制生成后声称独立证明。
- 先结构/hash validator，再本 checker；复核失败原因，校正产物/变换后重新生成同一新 revision。
  软件通过后人工交叉检查高度、障碍/自由空间、地图边界、原点/yaw 和至少三处已知车位/朝向。
- 在独立 query bag 上验证初始化、NORMAL→LOST→恢复、全局 relocalize 跳变、时间/重置隔离、
  动态杆臂与 canonical/TF 同刻性；包含实际高自转/云台运动/坡道与遮挡。逐项记录误差、掉定位、
  恢复时间、协方差语义及所有子进程退出码；门槛在测试前由地图/车型负责人确定。
- synthetic fixture 仅验证 checker；以上真实接受证据缺失时 M11 仍 Partial。
  地图接受、main 软件合入及实车授权是不同门槛；任何本节结果都不自动给 Hardware Go。
