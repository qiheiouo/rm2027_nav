# rm_map_tools

`validate_map_bundle` checks a versioned, reproducible map asset. Schema 1
contains a 3D PCD and Nav2 occupancy map. Schema 2 explicitly supports either
that dual artifact or an `occupancy_only` map with no fake PCD.

It validates:

- canonical `map` frame declarations;
- bundle-local artifact paths;
- SHA-256 hashes;
- PCD headers, dimensions and point count;
- occupancy YAML, PGM dimensions, thresholds and image references;
- deployment status and shared-origin confirmation.

The included `phase2e_test` bundle is synthetic and must never be deployed.
Real hardware bringup should eventually require an `approved` bundle.

```bash
ros2 run rm_map_tools validate_map_bundle \
  /path/to/map.bundle.yaml --require-approved
```

Runtime launch files use the stricter resolver:

```bash
ros2 run rm_map_tools resolve_map_bundle /path/to/map.bundle.yaml
```

It prints the approved occupancy YAML and optional PCD path consumed by
`rm_navigation_bringup`. The synthetic fixture requires an explicit offline
override:

```bash
ros2 run rm_map_tools resolve_map_bundle \
  /path/to/phase2e_test.bundle.yaml --allow-test-map
```

Never use `--allow-test-map` on a robot.

## Managed Mapping Session

`mapping_session_node` is the artifact boundary for real mapping. It consumes a
world-registered PointCloud2 and an OccupancyGrid, then saves one immutable
`candidate` bundle through `/mapping/save`.

```bash
ros2 service call /mapping/save std_srvs/srv/Trigger {}
```

The response contains the generated manifest path. The node also provides
`/mapping/start`, `/mapping/stop`, and `/mapping/reset`. It does not start a
sensor, publish TF, control the chassis, or approve a map.

See `docs/operations/managed_mapping.md` for the OctoMap projection and full
operator flow.

## Offline PCD/PGM Quality Diagnostics

`analyze_map_quality` validates an immutable bundle, reads the exporter ASCII
XYZ PCD and trinary PGM/YAML, and writes a new evidence directory containing:

- occupied-component CSV and pixel/coverage statistics;
- eight fixed Z-layer log-density PNGs;
- 0.05/0.10/0.25 m stable-PCD support statistics and overlay;
- the explicit world/grid/PGM row-direction contract;
- optional known-free, protected-obstacle and landmark hard-gate results;
- optional rosbag topic-coverage status.

It refuses binary PCDs instead of silently guessing their layout. It also
refuses existing output paths or paths outside `/tmp/rm27_pcd_pgm_diag`, so it
cannot overwrite a bundle:

```bash
ros2 run rm_map_tools analyze_map_quality \
  /path/to/map.bundle.yaml \
  --labels /external/evidence/map_quality_labels.yaml \
  --bag /external/evidence/mapping_bag \
  --output /tmp/rm27_pcd_pgm_diag/field01_baseline
```

The installed `map_quality_labels.example.yaml` is a template only. Replace
its map hash and all coordinates from field measurements; it must never be
treated as ground truth as shipped.

`sweep_map_projection plan` creates a baseline-plus-single-variable matrix.
It deliberately does not create a Cartesian product. A plan remains blocked
until at least two bags contain every required mapping topic. Complete topic
counts only advance the plan to message-level time-overlap and TF-coverage
preflight; metadata alone never marks a bag replay-ready:

```bash
ros2 run rm_map_tools sweep_map_projection plan \
  --baseline-config /path/to/mapping_octomap.yaml \
  --bag /external/evidence/closed_loop_1 \
  --bag /external/evidence/closed_loop_2 \
  --output /tmp/rm27_pcd_pgm_diag/sweep_plan
```

`sweep_map_projection rank` applies the over-filtering hard gates to analyzer
summaries. Missing labels, per-frame evidence, or two-dataset replication makes
a candidate ineligible; it never promotes a candidate or changes a deployment
manifest. Bag replay, ray evidence, TF timestamp/fallback attribution,
map-server reload, RViz review and robot validation remain separate stages.

After launching an isolated `map_deployment.launch.py`, verify that Nav2 loaded
the exact PGM/YAML contract, including every cell and lifecycle state:

```bash
ros2 run rm_map_tools verify_map_server /path/to/map.bundle.yaml --timeout 10
```

This command is read-only. A successful comparison proves serialization and
reload consistency; it does not prove obstacle truth, localization quality, or
approval readiness.

## Z prior / Nav2 coordinate consistency (manual, opt-in)

`check_zlocalization_coordinates` adds geometric checks to artifact validation:
origin/yaw/resolution, SE(3)/frame and baked-transform requirements, raster-local
bounds overlap, independent noncollinear landmarks (including PCD support and
pixel coordinates/class), and deterministic height-band occupied-cell samples.
It changes no map, launch, TF or approval flag.

```bash
ros2 run rm_map_tools check_zlocalization_coordinates \
  --prior-pcd /assets/prior/map.pcd --map-yaml /assets/nav2/map.yaml \
  --occupancy-image /assets/nav2/map.pgm --transform-metadata /assets/coordinates.yaml
```

JSON stdout includes every failed check and reason; exit 0 means input
consistency, exit 2 means failure. `real_map_accepted` and `hardware_go` are always
false. The metadata [template](config/zlocalization_coordinates.example.yaml)
requires replacement of all geometry, hashes, independent annotations and
predeclared acceptance thresholds; it intentionally cannot pass unfilled.
No example threshold is a new-car acceptance threshold.

Supports PCL ASCII/uncompressed binary PCD (extra/vector fields allowed), P2/P5
8-bit PGM, trinary negate/threshold semantics. Unsupported binary_compressed PCD,
image formats or modes fail explicitly; use existing PCL/image tools to convert
and bind the new bytes. No automatic alignment or guessed transform is applied.
A nonidentity prior→map transform may be evaluated but fails the current runtime
gate: bake it into the whole prior bundle first. YAML origin applies to raster
coordinates only, never a second time to Z localization poses.

The [coordinate contract and real acquisition/acceptance flow](../../docs/contracts/map_bundle.md#z-localization-prior--nav2-坐标合同parity-候选2026-10-24)
are authoritative. Synthetic tests verify checker behavior and cannot validate
an actual prior/map or grant Hardware Go.
