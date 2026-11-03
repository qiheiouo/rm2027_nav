# rm_competition_interfaces

This package defines data contracts only. It contains no runtime node and owns
no TF, odometry, navigation action, chassis command, serial device or hardware
protocol.

- `RefereeState` is the normalized, validated match-state boundary.
- `ChassisMode` keeps lower-controller authority separate from referee data.
- `TargetTrack` is the perception-to-pursuit boundary with frame, timestamp,
  confidence and velocity.
- `PursuitCandidate` carries an atomic map goal, original target timestamp/deadline,
  source policy, stable target identity and validity; it grants no Nav2 authority.
- `TargetIntent` carries independent Local/Radar/Operator map targets with stable
  identity, per-source sequence, original stamp/TTL and distinct valid/clear semantics;
  see the [arbitration contract](../../docs/contracts/navigation_target_arbitration_2027.md).
- `MissionState` reports decision execution without becoming a control topic.
- `OperatorNavigationTarget` carries an untrusted raw target from the serial
  boundary. Its coordinate system and command semantics must be confirmed
  before a separate adapter may offer it to mission logic.
- `SystemReadiness` reports missing runtime requirements without granting
  motion authority.
- `RobotPosture`, `PostureRequest` and `PostureState` define the single
  six-posture enum, desired-state request, session-safe ACK and completion
  boundary for `competition_v2`.
- `GimbalState` carries timestamped mechanical yaw relative to the chassis; it
  is not an INS world heading.
- `ChassisHeadingState` carries timestamped lower-controller world yaw,
  estimator reset identity and source boot identity. It is an optional input
  to the sensor/chassis heading-fusion candidate, not a TF owner.
- `SerialConnectionState` reports protocol version, peer capabilities/restart,
  freshness and parser counters without granting mission authority.
- `SetMissionMode` is the explicit enable/mode gate for competition mission
  execution.
- `LocalizationStatus.transaction_phase` / `committed_at`, `BeginRebase` and
  `LocalizationBarrier` describe the opt-in map-only transaction. Receipts do
  not grant independent TF/motion authority; see the
  [bridge contract](../rm_relocalization_bridge/README.md#explicit-map-only-rebase-transaction).
