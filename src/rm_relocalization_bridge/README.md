# rm_relocalization_bridge

This package owns the 2027 global-localization boundary. It converts a global
robot pose and canonical LIO odometry into the canonical dynamic transform:

```text
T_map_odom = T_map_base * inverse(T_odom_base)
```

Inputs:

- `/localization/global_pose` (`geometry_msgs/PoseWithCovarianceStamped`)
- `/odometry/lio` (`nav_msgs/Odometry`, `odom -> base_link` semantics)

Outputs:

- dynamic `map -> odom` on `/tf`
- `/localization/map_to_odom` for inspection
- `/localization/global_localization_valid`
- `/localization/reset_map_to_odom` reset service

The node never publishes an identity fallback. `map_odom_stub` and this package
must never run together. A future small_gicp, scan-to-map, or NDT backend must
publish `/localization/global_pose`; it must not publish canonical TF directly.

`fake_global_pose_publisher` is test-only. It derives synchronized fake global
poses from `/odometry/lio` and a configured correction, and publishes no TF.
The Phase 2C test launch uses one publication so reset behavior remains
observable. The test publisher uses transient-local QoS and supports a startup
delay; this is a test convenience, not the required QoS of a real backend.

Phase 2J adds `amcl_2d_backend.launch.py`. AMCL runs with `tf_broadcast=false`
and publishes backend-private `/localization/amcl_pose_raw`. The
`global_pose_gate_node` validates frame, timestamp, finite values, planar pose,
and covariance before publishing `/localization/global_pose`. The optional
PointCloud2 projection and AMCL itself remain replaceable upstream components.

The normal 2D profile remains `config/amcl_2d.yaml`. The separate
`config/amcl_2d_spin_robust_candidate.yaml` changes only the documented
high-speed-spin motion term (`alpha4=0.02`) and is paired with
`config/pointcloud_to_scan_2d_spin_robust_candidate.yaml`. It is experimental,
not a competition default, and must be launched through the no-motion
`old_car_2026_amcl_spin_candidate.launch.py` wrapper until field acceptance is
complete.

## Localization validity lease (AUD-001)

The selected gate publishes reliable **volatile** `LocalizationStatus` on
`<valid_topic>/status` at least every 100 ms while alive. The header is emission
time; `producer_id`, `started_at` and increasing `sequence` identify an instance.
`pose` is the last accepted map/base measurement, with its original timestamp.
`valid_until` is the source deadline upper bound: the existing 1.0 s AMCL /
1.5 s GICP input timeout plus the existing 0.1 s timeout timer period. The
steady-clock source timeout itself is unchanged. A heartbeat does not extend that deadline or manufacture a new
measurement. Fresh stamped observations with an unchanged pose remain legal.

When `upstream_valid_topic` is configured, the canonical bridge consumes that
status **including the pose**, rather than independently pairing a cached Bool
and an unqualified pose. It still exposes `/localization/global_pose` upstream
for inspection, but that topic cannot bypass the selected gate. The bridge is
still the only dynamic map/odom owner. It publishes `/localization/status`
with its own instance identity and the last accepted matched pose. The optional
correction gate below adds an admission step; backend quality thresholds stay separate.

The bridge and localization consumers require source validity, fresh emission
time, increasing sequence/epoch (equal emission timestamps are legal within one
simulated clock tick; reversed time is rejected) and a steady-clock receipt lease (default
`status_timeout_sec` / `localization_status_timeout_sec`: 1.0 s). This budget
reuses the AMCL stream budget and permits ten gate heartbeat periods, rather
than requiring each correction to change. The accepted source window is bounded
by the existing GICP 1.5 s budget plus the 0.1 s timeout timer period.
Old/pre-start messages, repeated or reversed timestamps, an expired source or
a silent producer close validity. Gate/bridge restart, reset and fault recovery
require a fresh acceptable source timestamp newer than the accepted-source
high-water mark; startup and explicit reset also impose their own time cutoff.
This admits fresh in-flight measurements rather than rejecting them merely
because a timeout callback ran before delivery.
A ROS clock reversal fails closed; restart in the new clock epoch is required
if the clock does not catch up. Simulated startup waits for positive ROS time.

`/localization/global_localization_valid` and backend Bool topics remain latched
**diagnostic snapshots**, with no independent lifetime semantics. Mission and
readiness now consume `/localization/status`; bridge death closes their
localization input within one lease plus their timer period (0.1/0.2 s).
Other Mission inputs are unchanged. Third-party Bool-only consumers must migrate
before relying on localization health. This change grants no serial authority
and does not prove a chassis stop.

Invalidation retains the last correction/pose in memory for diagnosis but
publishes false status and stops new TF/transform timestamps. Already cached TF
may still be visible to TF lookup; that is not localization health. Pending
poses are cleared on invalidation and their existing 0.2 s wait/0.5 s source-age
limits are checked before matching, so late odometry cannot reopen validity.

An unconfigured standalone external-pose bridge has no producer heartbeat; each
new pose provides only a finite 1.0 s lease. A one-shot Phase 2C fake pose now
expires automatically. For AMCL/GICP use the selected backend status chain.

### Optional LIO source-health prerequisite

With nonempty `source_health_topic`, the bridge consumes reliable/volatile
`LioSourceStatus` envelopes instead of bare `/odometry/lio`. It checks source
identity/start time, generation, sequence, original odometry time/frames/pose,
finite deadline, and `source_health_timeout` in both ROS and steady time. Retired
source epochs/generations and repeated sequences are inert, including old false
heartbeats arriving after recovery. A valid global pose cannot override an
invalid, missing, or expired LIO source. `valid_until` is the minimum of global
correction evidence and source-odometry evidence; neither heartbeat nor cached
TF renews source time. Configure the source timeout at least as large as the
adapter's maximum-age deadline and within the intended motion response budget.

A source invalidation clears pending global poses and odometry matches, withdraws
map/odom TF, publishes unified invalidity, and advances the bridge localization
session. A paused/reversed clock defers any epoch that cannot yet increase until
fresh source evidence and advancing time permit it; sequence is never reset
inside an unchanged epoch. ROS clock rollback otherwise retains its previous
high-water cutoff. Source recovery alone is insufficient: a newly accepted global
measurement after the invalidation cutoff must match a new healthy odometry
sample. Old corrections remain diagnostic only. The existing Mission session
handoff cancels and retires the old action before a new goal can run.

The selected AMCL/GICP gate may restart its sequence in a **new** epoch even with
the same producer ID. Duplicate/out-of-order sequences inside one epoch remain
invalid. With the optional correction gate below, a new backend epoch instead
requires explicit prepare. LIO health itself grants no autonomous recovery authority.
`/localization/status` remains the sole validity authority, the bridge remains the
sole map/odom owner, and the adapter remains the sole odom/base owner.

## Optional correction gate and explicit relocalization

`correction_gate.enabled=false` remains the default. When enabled, both a selected
`upstream_valid_topic` and atomic `source_health_topic` are mandatory. The bridge
pins one backend producer/admission epoch; another producer/epoch requires an
explicit transaction. AMCL and GICP remain mutually exclusive selected backends.
The backend is responsible for meaningful quality checks (AMCL covariance,
GICP convergence/fitness plus existing pose quality). Existing GICP covariance
is a clamped score-derived proxy, not calibrated measurement uncertainty; this
port neither creates nor upgrades that proxy. Small covariance does not establish
geometric correctness. The bridge never invents a score or covariance.

A candidate is OBSERVED, then ELIGIBLE only after source identity, monotonic
sequence/source time, finite pose, original source deadline, source quality and
healthy LIO match checks. Compute `C = T_map_base(t) * inverse(T_odom_base(t))`;
compare the observation with `C_trusted * T_odom_base(t)`. Residuals are 3D
position distance in metres and shortest wrapped yaw in radians. Using the
matched source time avoids treating ordinary robot motion as correction error,
and avoids comparing translation origins under a distant yaw pivot. The nearest
odometry match remains bounded by `max_pose_odom_dt_sec`; its timing error is
still a vehicle-dependent limitation, especially during spin.

- GATED_ACCEPT alone replaces map/odom and the accepted pose/deadline.
- REJECT retains the trusted correction and its **existing** finite lease;
  rejected observations and their heartbeats cannot renew that lease.
- `correction_gate.rejection_count` consecutive contradictory **eligible**
  observations cause FAULT_LATCHED. An accepted observation resets this count.
  Stale/invalid/source-quality failures are rejected or invalidate their existing
  source lease; they do not manufacture an innovation residual.
- FAULT_LATCHED withdraws unified validity and new map/odom TF timestamps. It does
  not change LIO health or publish commands. Mission cancels through its existing
  unified-validity input. Healthy LIO keeps its canonical odometry/TF.
- A good frame, source reconnect, `/initialpose`, or LIO recovery cannot clear
  the latch. There is no autonomous reseed, cooldown timer or motion-stop detector.

Startup and explicit relocalization use `transaction_samples` consecutive
consistent candidate transforms, permitting a large initial offset. A broken
eligibility/matching sequence or an inter-candidate gap exceeding
`status_timeout_sec` restarts verification. Explicit relocalization:

1. Call `/localization/reset_map_to_odom` (`Trigger`): **prepare**, reserve a newer
   bridge session, revoke old validity, clear pending/matching data and reset the
   gate for verification. It does not immediately grant validity.
2. After ROS time advances, call `<selected_valid_topic>/reset_session` (`Trigger`):
   reset the selected quality gate's admission epoch, discard prior accepted pose
   and reject source measurements at/before its cutoff. This neither seeds nor
   resets AMCL/GICP. Any required algorithm seed remains an explicit operation
   through the existing backend interface. Stop/drain old backend work before
   resetting admission, and retain original acquisition stamps.
3. Only the newer selected epoch's post-cutoff, healthy-LIO-matched, quality-valid
   and mutually consistent samples commit validity in the reserved session.
   The old action must terminate before Mission submits a replacement goal.

Until step 3 the transaction stays invalid. No automatic coordinator calls these
services. A backend restart without prepare cannot silently reset the baseline.
The gate requires strictly advancing epochs; a paused or reversed clock cannot
mint a new session. A single large accumulated drift needs this explicit entry;
gradual within-limit corrections may accumulate beyond the per-step threshold.

Raw `PoseWithCovarianceStamped` has no algorithm transaction identity. Admission
epochs cannot distinguish an old computation deliberately re-stamped as new.
This candidate therefore requires one controlled source, original acquisition
timestamps and a drained/explicitly seeded backend. It is not an authenticated
backend reset protocol or a general recovery solution. Source loss invalidates
its lease; good correction while LIO INVALID/RECOVERING is diagnostic eligibility
only and cannot queue a commit or drive recovery. Normal LIO reacquisition still
follows the existing health contract and then requires new correction evidence;
a correction latch, if set, persists.

`/localization/correction_state` (`std_msgs/String`, reliable transient-local)
is diagnostic only: OBSERVED / ELIGIBLE / ELIGIBLE_WAIT_LIO / VERIFYING /
GATED_ACCEPT / REJECT / FAULT_LATCHED / PREPARED. It is not a second validity or
motion authority. `/localization/status` and the existing TF owners are unchanged.

The explicit simulation profile uses 0.4 m, 0.3 rad, 3 contradictory samples and
3 transaction samples. These are integration-fixture values, **Hardware Blocked**,
including matching tolerances, source quality limits and motion uncertainty.
No real-vehicle launch enables this gate by default. See the
[port evidence](../../docs/validation/localization_correction_gate_port_20261028.md).
