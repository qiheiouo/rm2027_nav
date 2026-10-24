# One atomic candidate. Header is the original target sample time in map frame.
# Re-publication must never extend valid_until beyond that sample's freshness budget.
uint8 SOURCE_RESEARCH=0
uint8 SOURCE_RADAR=1
std_msgs/Header header
geometry_msgs/Pose pose
builtin_interfaces/Time valid_until
string track_id
uint8 source
bool valid
