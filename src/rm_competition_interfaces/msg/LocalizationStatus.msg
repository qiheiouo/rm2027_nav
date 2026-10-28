# Periodic localization evidence; Bool topics are diagnostic snapshots only.
# header.stamp is emission time, never the time of a new measurement.
std_msgs/Header header
# Unique per process; started_at and sequence prevent old-instance/replay reuse.
string producer_id
builtin_interfaces/Time started_at
uint64 sequence
bool valid
# ROS-time upper bound from the accepted source, not extended by heartbeats.
# Consumers also require a bounded steady-clock receipt lease.
builtin_interfaces/Time valid_until
# Last gate-accepted map->base pose (may be retained while valid=false).
geometry_msgs/PoseWithCovarianceStamped pose
# Explicit map-only transaction; zero preserves the ordinary evidence contract.
uint8 ACTIVE=0
uint8 DRAIN_OLD_NAV=1
uint8 PRIME_NEW_SESSION=2
uint8 REBASE_FAILED=3
uint8 transaction_phase
# TF changes at/after this ROS time; map-frame observations before it are retired.
builtin_interfaces/Time committed_at
