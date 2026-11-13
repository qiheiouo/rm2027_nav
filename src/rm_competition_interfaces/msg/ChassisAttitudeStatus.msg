# Lifecycle and validity companion for /chassis/imu. This message never carries
# substitute attitude data; consumers use it to gate the standard Imu stream.
std_msgs/Header header
uint16 frame_sequence
uint32 sample_sequence
uint32 mcu_time_ms
uint32 source_boot_id
uint16 reset_counter
uint64 lifecycle_epoch
bool online
bool orientation_valid
bool angular_velocity_valid
bool linear_acceleration_valid
bool current_valid
bool stale
string reason
