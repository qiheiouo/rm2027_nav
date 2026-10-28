# Source evidence, NOT motion authorization. Sole localization authority remains
# /localization/status. Atomic odometry prevents cross-topic health/pose mixing.
std_msgs/Header header
string producer_id
builtin_interfaces/Time started_at
uint64 generation
uint64 sequence
bool healthy
string reason
builtin_interfaces/Time valid_until
# Present only when healthy; source timestamp is never refreshed by a heartbeat.
nav_msgs/Odometry odometry
