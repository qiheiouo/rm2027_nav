# rm_dynamic_obstacle_tracking

Status: **Integration / shadow-only, disabled by default**. This package was
selectively ported from legacy snapshot
`0d7c1bf82ac52a2143680ce8c41cc117b1fcfd63` as the minimum substrate for the
dynamic surface-member experiment. It does not restore the stopped HWS/R4
production route.

The offline `observed_surface_geometry.py` prototype keeps every selected scan
endpoint and original beam identity. Only consecutive beams with short spatial
and range gaps become nominal chords; gaps, isolated returns and hidden space
retain explicit unknown semantics. Chord interiors and short CV propagation are
uncertified assumptions. The ROS node does not invoke this prototype by default.

The optional `prediction.anchor_mode=last_observation_cv` mode retains the most
recent associated detection centroid, then advances it to the scan epoch using
the current filter velocity. It requires `prediction.velocity_decay_tau=0.0`
and `prediction.max_speed=0.0`. Default `filtered` mode and v1 data remain the
baseline. Neither mode certifies full object geometry or motion. The retained
legacy conclusions are indexed in the
[dynamic-obstacle research history](../../docs/dynamic_navigation/dynamic_obstacle_prediction_full_history_20261004.md).

This package provides an opt-in, shadow-only 2D dynamic-obstacle tracker. It
uses `/map` and timestamped `/local_scan` endpoints, subtracts static occupied
cells, clusters unexplained endpoints, tracks them with a constant-velocity
Kalman model, and visualizes current velocity and short-horizon predictions.
The shadow profile additionally requires a small observed displacement before a
track can become confirmed, so stationary map/projection residuals remain
tentative instead of being labeled as dynamic obstacles.
After endpoint subtraction, a second static-clearance check on each cluster
centroid removes compact wall residuals without discarding all points from a
person moving close to mapped structure. Multi-target association uses a gated
greedy assignment in the field profile; a gated global minimum-cost mode is
available for controlled A/B tests but remains disabled after the D05 field bag
showed no fragmentation improvement.

It intentionally does **not** publish TF, costmaps, plans, goals, or velocity
commands. Its outputs are:

- `/perception/dynamic_obstacles_shadow/predictions`
- `/perception/dynamic_obstacles_shadow/markers`
- `/perception/dynamic_obstacles_shadow/diagnostics`

The versioned prediction array is the only machine-readable consumer boundary.
It carries the scan source stamp, prediction step, state, visible extents and
the schema-selected anchors. `complete` indicates output-budget completeness; it does not
certify detection completeness or physical geometry. Display speed may be
clipped/decayed, while the published velocity remains the filter estimate.
The current public boundary is recorded in the
[topic contract](../../docs/contracts/topic_contract_2027.md).
Consumers must reject stale or wrong-frame data and must not parse markers or
diagnostic strings. Output is deterministically bounded by `prediction.max_tracks`;
if that bound truncates a frame, `complete=false` and consumers must not infer
clearance. The diagnostics and MarkerArray remain operator aids.

The launch defaults to disabled:

```bash
ros2 launch rm_dynamic_obstacle_tracking dynamic_obstacle_tracking_shadow.launch.py \
  enabled:=true
```

Use a timestamp-correct `/local_scan`. This selective port does not include a
vehicle-specific scan adapter, dynamic-clearance consumer, path annotation,
controller, owner/lease system, fallback, runtime-profile enablement or field
acceptance. Any future controller integration requires a separate reviewed
branch and the validation appropriate to its risk.
