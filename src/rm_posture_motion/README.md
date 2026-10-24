# rm_posture_motion

Consumes **effective** `PostureState.posture_actual` and selects six independent
upper motion profiles. `MOVE`/`ENHANCED_MOVE` in the existing wire enum map to
`mobile`/`enhanced_mobile`; the other four native values remain distinct.
UNKNOWN/STALE select `fallback`. Requested posture, ACK and `completed` never
select motion limits. The node issues no posture request or lifecycle decision.

## Opt-in production composition

```bash
ros2 launch rm_posture_motion posture_motion.launch.py \
  enable_posture_constraints:=true nav2_params:=/absolute/path/to/selected_nav2.yaml
```

Default: disabled. This starts only a parameter/diagnostic consumer; it starts
no Nav2, MCU, mock, Mission or motion-authority source. The selected active
Nav2 profile must use MPPI and match its smoother's velocity bounds.
`profiles.yaml` gives all six slots and fallback independent names, inheriting
that exact baseline. Physical values are **HARDWARE_TBD**. No power-to-motion
inference is made. Test-only concrete numbers live under `test/fixtures/`, are
not installed as production configuration and require explicit `source_mode=test`.
The production launch fixes the real topic and production file.

At startup the adapter waits for active controller/smoother and verifies their
live limits against the selected baseline **before any write**. On a changed
numeric profile it updates the smoother's `max_velocity`, `min_velocity`,
`max_accel`, `max_decel` in one atomic parameter request, then MPPI's
`vx_max`, `vx_min`, `vy_max`, `wz_max` in another. Humble MPPI 1.1.20 exposes no
matching independent acceleration/deceleration parameters, so none are invented.
Identical numeric profiles avoid writes while preserving distinct state,
profile selection and generations. MPPI/smoother must have no competing limit writer.

Freshness: source header receipt stamp and steady-clock progress lease, 0.5 s;
normal posture changes debounce for 0.05 s, invalid/stale immediately select
fallback. There are no 5 s/3 min/15 s posture lifecycle timers. Changes arriving
during RPCs coalesce to the latest selection. Diagnostics on
`/posture_motion/status` distinguish selected and confirmed applied profile,
generation, revision, effective posture, freshness and fallback/error reason.

Parameter application across two nodes is not a distributed atomic transaction.
During a change the smoother bounds first; its normal deceleration can briefly
leave output above the newly reduced velocity cap. MCU remains the final hard
clamp and motion authority remains independent. On a rejection/timeout,
application is reported **unconfirmed**, with ERROR, and no further RPCs are
issued. Resolve the cause and restart this selector together with a known
baseline; backend/selector lifecycle recovery is not automatic. A parameter
control-plane failure cannot be repaired by claiming that fallback was applied.

The selector never publishes Twist, calls Nav2 actions, changes a target,
requests home or grants an AUD-003 owner/session lease. Host profile limits do
not replace final MCU clamping or host final authorization.

[Contract and audit](../../docs/contracts/posture_motion_constraints_2027.md) ·
[Controlled validation](../../docs/validation/posture_motion_constraints_20261024.md)
