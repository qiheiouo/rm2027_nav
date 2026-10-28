# rm_pursuit

`pursuit_goal_planner` validates `/perception/target_track`, transforms the
track into `map`, predicts a short horizon, and publishes a standoff pose on
`/mission/pursuit_goal` with `/mission/pursuit_goal_valid`.

It deliberately does not call Nav2 and never publishes `/cmd_vel`. The
competition mission executor decides whether a valid candidate may become a
`NavigateToPose` action. A stale target immediately invalidates the candidate.

No-hardware boundary test:

```bash
ros2 launch rm_pursuit pursuit.launch.py \
  enable_pursuit_boundary:=true use_mock_target:=true
```

The launch default is disabled. The mock target is never a claim that the
auto-aim/lower-controller target protocol is implemented.

## Radar and research input policy

`radar_only=true` fixes input to `/radar/target_track`; it does not subscribe to
generic teammate/perception targets or use the target mock. `radar_only=false`
retains `/perception/target_track` (or the research `input_topic` override).
Both paths publish `/mission/pursuit_candidate` with stable `track_id`, source,
original sample stamp, `valid_until` and validity in one message. Mission uses
this candidate; the old pose/Bool remain for diagnostics.

The existing 0.3 s track budget is preserved, including a steady-clock bound.
Republishing the same sample cannot renew it. A real radar-link producer is
still absent. No serial format, new tracker or prediction model was added.
See [Target Arbitration](../../docs/contracts/navigation_target_arbitration_2027.md).
