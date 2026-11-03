# High-level map-frame point/pose only. Never a motion or posture command.
uint8 SOURCE_NONE=0
uint8 SOURCE_LOCAL=1
uint8 SOURCE_RADAR=2
uint8 SOURCE_OPERATOR=3
std_msgs/Header header
uint8 source
geometry_msgs/Pose pose
# Strictly increasing per source for this producer/arbiter process lifetime.
# Repeat the identical sequence/stamp/payload without renewing its lease.
uint64 sequence
# Stable logical target identity; fresh samples may update its pose.
string target_id
builtin_interfaces/Duration ttl
# valid=false withdraws availability; clear=true explicitly cancels this source.
# clear requires valid=false. Neither requests a robot stop.
bool valid
bool clear
