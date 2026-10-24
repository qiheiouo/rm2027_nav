# One totally ordered backend/owner output. State notices carry no measurement.
builtin_interfaces/Time emitted_at
string producer_id
uint64 epoch
uint64 sequence
uint64 state_generation
uint64 pose_revision
bool has_odometry
bool valid
nav_msgs/Odometry odometry
geometry_msgs/PoseWithCovarianceStamped global_pose
