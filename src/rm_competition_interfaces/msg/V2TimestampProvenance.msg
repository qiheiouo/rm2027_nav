# Diagnostic only; never a clock correction or motion/localization input.
uint8 HEARTBEAT=0
uint8 HEADING=1
uint8 GIMBAL=2
uint8 INVALID_HEADING=3
uint8 INVALID_GIMBAL=4
string session_id
string transport_instance
uint32 local_boot_id
uint64 record_sequence
uint8 kind
bool has_frame
uint8 frame_type
uint16 frame_sequence
bool payload_decoded
int64 handler_ros_ns
int64 handler_system_ns
int64 handler_steady_ns
# End of the polling read batch, not individual frame/wire arrival. Absent in mock RX.
bool has_read_batch_completion
int64 read_batch_completion_ros_ns
int64 read_batch_completion_steady_ns

bool offset_available
uint64 offset_revision
int64 offset_ns
# Self-contained copy of the heartbeat basis actually used by this record.
uint16 offset_heartbeat_frame_sequence
uint32 offset_heartbeat_uptime_ms
uint32 associated_boot_id
int64 offset_heartbeat_ros_ns
int64 offset_heartbeat_steady_ns
uint32 heartbeat_capabilities
bool heartbeat_ready

uint32 sample_sequence
uint32 mcu_time_ms
uint16 heading_reset_counter
float64 raw_yaw_rad
float64 raw_yaw_rate_rad_s
bool raw_online
bool raw_valid
bool has_output
string output_topic
uint64 output_sequence
std_msgs/Header output_header
bool output_online
bool output_valid
string invalid_reason
