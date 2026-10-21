# Full-base shadow system validation — frozen input contracts (2026-10-08)

**Conditional Simulation Go; freeze research pending new-car measurements.**
Branch `experiment/full-base-simulation-validation`, simulation parent `d5673e3a`,
contract revision parent `6c5e8227`. Only `scripts/adapter.py` was copied from
`experiment/full-base-heading-adapter@5733f126`; its bytes remain unchanged.
There was no whole-branch merge, push, NEU/covariance/control unfreeze, or Deployment Go.

The 20261007 19-case/4-boundary system evidence is immutable. The 20261008 additive
revision fixes observable heading cache/sequence/receipt-time and startup/epoch
contracts, plus two checks in the native planar node **on this validation branch**.
S2 geometry/physics and the mathematical candidate are unchanged. No default launch
starts this package; parent `experiments/COLCON_IGNORE` excludes default discovery.

The perfect declared Gazebo observation plugin supplies sensor-relative pose and
base +x azimuth to shared ingress. Actual joint/truth are scoring inputs only.
The candidate does not receive TF, joint, truth, or commands and does not publish TF
or commands. Planar has `publish_tf=false`. Public TF owners remain baseline oracle/RSP;
Nav2 localization is not started. The physics runner alone sends test motion commands.
The small contract fixtures send only raw/heading messages to four consumers:
old native planar / old full-base / current native planar / current full-base.

## Read-only regression verification

Run from this worktree using the existing system Python (NumPy/SciPy already available):

```bash
python3 experiments/full_base_simulation_validation/test_input_contract.py
python3 experiments/full_base_simulation_validation/verify_evidence.py
python3 experiments/full_base_simulation_validation/verify_contracts.py
```

`verify_evidence.py` recomputes the **historical** 217 checks without rewriting the
old summary, and compares the historical production sources at `6c5e8227` to `d5673e3a`.
It does not claim current C++ is unchanged. `verify_contracts.py` verifies the original
93 source/evidence hashes, protected refs, current results, mathematical byte identity,
and the additive frozen source/report/evidence manifest. The 79 historical evidence
files are checked in place; historical source bytes are read from `6c5e8227`.
Old traces did not record heading sequence/receipt events, so no fabricated transport
replay of all19 cases is claimed for the new guard.

## Rebuild and repeat the small live validation

Jazzy/Harmonic are installed. Select unused domain/partition IDs. Results below go
into ignored `build/` to avoid overwriting the committed measurements:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src/rm_new_car_simulation src/rm_localization_adapters \
  src/rm_competition_interfaces experiments/full_base_simulation_validation \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
export ROS_LOG_DIR=/tmp/full_base_contract_ros_logs ROS_HOME=/tmp/full_base_contract_ros_home
TASK_EVIDENCE="$PWD/build/contracts_reproduction/evidence"
ROS_DOMAIN_ID=94 python3 experiments/full_base_simulation_validation/run_contracts.py \
  --prepare-old --evidence-dir "$TASK_EVIDENCE"
ROS_DOMAIN_ID=95 python3 experiments/full_base_simulation_validation/run_boundaries.py \
  --evidence-dir "$TASK_EVIDENCE/boundaries"
ROS_DOMAIN_ID=96 GZ_PARTITION=full_base_contract_reproduction \
  python3 experiments/full_base_simulation_validation/run_system.py --case flat_B \
  --evidence-dir "$TASK_EVIDENCE/physics"
python3 experiments/full_base_simulation_validation/verify_contracts.py \
  --evidence-dir "$TASK_EVIDENCE" --write-summary
ctest --test-dir build/rm_localization_adapters --output-on-failure
ctest --test-dir build/rm_new_car_simulation --output-on-failure
```

`--prepare-old` extracts only the adapter package and three old candidate files from
`6c5e8227` into ignored `build/contracts_20261008`; it builds an isolated old executable
against the unchanged interface underlay. It does not switch branches or source an old
install over the current workspace. Without this flag it reuses the previously built
old executable. `run_contracts.py --case timing` (or another recorded group name) runs
one group. Scripts stop their own process groups; do not write one result directory
from two concurrent runs. Wall scheduling can change sample counts/physical contact
scores, so repeat validation of gates, not byte-identical ROS measurements.

`run_system.py` retains all19 original case definitions; `--case` selects any original
case. Default new measurements go under `evidence/contracts_20261008/physics` and
boundary results under `.../boundaries`; the 20261007 files are never default write targets.
`replay_sensitivity.py` remains an optional offline fixed-rate perturbation, now writing
under the additive folder or to explicit `--output`. This is not another physical trial.

To intentionally replace/freeze this revision's recorded measurements after an authorized
rerun, use `verify_contracts.py --write-summary`, then `freeze_evidence.py`. The latter
only writes `contracts_20261008/evidence_sha256.json`; it preserves the old SHA manifest.
Read-only verification does not need a ROS domain or Gazebo.

## Frozen timing / identity / startup contract

Math rejects >1 ms source skew. The wrapper preserves stamps, waits up to60 ms wall
clock for exact matches, then uses nearest without retimestamping/interpolation/rate
propagation. Cached heading and raw first-receipt times must be within that same60 ms
budget. This is a research transport policy, not a hardware time bound or absolute age
proof. Legitimate heading-first/raw-first and heading-stream reordering remain supported.

Heading sequence is checked only within its boot/reset epoch, using uint32 modular
ordering. Recognizable duplicates do not refresh their receipt; conflicting sequence/
stamp metadata is rejected. Invalid/offline/reference-invalid heading clears the valid
cache. Valid new samples can recover ordinary input faults. Cache depth is100.

Raw parent, sensor child and heading reference must match the declared inputs. Startup
roll/pitch defaults to missing; `startup_home_confirmed` defaults false. Research launch
explicitly supplies fixture roll/pitch and home q=0. Startup is committed only after a
unique valid solve. During initialization upstream must hold/confirm home and maintain
that complete reference. No independent attitude/home measurement timestamps are inferred.

Observed raw epoch changes, heading boot/reset changes (even invalid/unmatched packets),
or raw time rollback invalidate the active full-base startup until explicit restart and
re-home. Same-name raw epoch reuse without any observable identity/time contradiction
cannot be detected. Planar only gains strict child validation and active raw-parent latch;
its30 ms rate-propagated heading policy and invalid-heading epoch handling stay distinct.

Hidden old yaw with fresh consistent metadata, periodic stamp alias, jointly delayed
streams, wrong but internally consistent startup/home/calibration, and physical MCU/
LiDAR/ROS clock mappings remain unobservable/uncertified. Do not add guessed thresholds,
truth-assisted roots, or continuity priors to make those cases appear solved.
A/B share the same candidate and guard; their CAD origins are not measured IMU origins.
Research Odometry remains a pose carrier with unvalidated covariance/twist; do not wire it
into public TF, localization or control.

See [contract repair/freeze report](../../docs/research/full_base_input_contract_20261008.md),
[additive results](../../docs/research/full_base_simulation_validation/evidence/contracts_20261008/summary.json),
and [historical system A/B](../../docs/research/full_base_simulation_validation_20261007.md).
