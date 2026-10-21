// Use the COMPLETE original source, its real publisher function and messages.
// Rename its sensor-driven main; that original binary is also built and smoked.
#define main fastlio_sensor_main
#include FASTLIO_SOURCE_FILE
#undef main
#include <std_msgs/msg/float64_multi_array.hpp>

class PublicationBoundary : public rclcpp::Node
{
public:
    PublicationBoundary() : Node("fastlio_publication_boundary")
    {
        map_frame = declare_parameter<std::string>("parent_frame", "odom");
        output_ = create_publisher<nav_msgs::msg::Odometry>(
            declare_parameter<std::string>("raw_topic", "/covariance_validation/raw"), 10);
        broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        input_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            declare_parameter<std::string>("command_topic", "/covariance_validation/native_fixture"), 10,
            [this](std_msgs::msg::Float64MultiArray::ConstSharedPtr command) {publishFixture(*command);});
    }
private:
    void publishFixture(const std_msgs::msg::Float64MultiArray &command)
    {
        // [source time, p_W xyz, quaternion xyzw, zero-start flag, P6 row-major]
        // Fixture P is assigned to the REAL 23x23 ESKF, with no fake get_P().
        if (command.data.size() != 45) throw std::runtime_error("fixture must contain 45 values");
        const auto &d = command.data;
        state_ikfom native;
        native.pos = Eigen::Vector3d(d[1],d[2],d[3]);
        const Eigen::Quaterniond quaternion(d[7],d[4],d[5],d[6]);
        native.rot = SO3(quaternion.toRotationMatrix());
        auto covariance = kf.get_P();
        covariance.setIdentity();
        for(int row=0;row<6;++row) for(int col=0;col<6;++col)
            covariance(row,col) = d[9+6*row+col];
        kf.change_x(native);
        kf.change_P(covariance);
        state_point = kf.get_x();
        lidar_end_time = d[0];
#ifdef COVARIANCE_BUNDLE
        publish_lidar_time = d[0];
        const bool requested_zero = d[8] != 0.;
        if (requested_zero && !zero_start_pose) {
            zero_pose_initialized = false;
            processed_scans_for_zero = 0;
            zero_start_delay_scans = 2;
            T_world_to_zero.setIdentity();
        }
        zero_start_pose = requested_zero;
        // Actual unchanged production block, including RCLCPP_INFO.
#include "zero_bundle.inc"
#else
        // These two variants have no zero-start implementation.
        if (d[8] != 0.) throw std::runtime_error("zero-start is bundle-only");
        geoQuat.x = quaternion.x(); geoQuat.y = quaternion.y();
        geoQuat.z = quaternion.z(); geoQuat.w = quaternion.w();
#ifdef FASTLIO_ADAPTIVE
        publish_lidar_time = d[0];
#endif
#endif
        publish_odometry(output_, broadcaster_);
    }
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr output_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr input_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<PublicationBoundary>());
    rclcpp::shutdown();
    return 0;
}
