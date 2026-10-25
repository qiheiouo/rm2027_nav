#pragma once
#include <optional>
#include <random>
#include "rm_competition_interfaces/localization_session.hpp"
#include "rm_competition_interfaces/pose_covariance.hpp"
#include "rm_competition_interfaces/msg/localization_frame.hpp"
#include "rm_competition_interfaces/srv/select_localization_source.hpp"

// Opt-in input of the existing LioAdapter owner. All callbacks use its default
// mutually-exclusive group. TF may arrive independently, but only a current
// work ticket can publish. It never subscribes to unversioned odometry.
class SessionCanonicalInput
{
  using Frame = rm_competition_interfaces::msg::LocalizationFrame;
  using Steady = std::chrono::steady_clock;
public:
  SessionCanonicalInput(rclcpp::Node & node, tf2_ros::Buffer & tf,
    tf2_ros::TransformBroadcaster & broadcaster,
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom,
    rm_localization_adapters::PoseTwistEstimator & twist,
    std::string input, std::string sensor, std::string base, std::string odom_frame,
    bool publish_tf, double wait, const std::array<double, 6> & twist_variance)
  : node_(node), tf_(tf), broadcaster_(broadcaster), odom_pub_(odom), twist_(twist),
    fence_(node.now().nanoseconds()), input_frame_(input), sensor_(sensor), base_(base),
    odom_frame_(odom_frame), publish_tf_(publish_tf), wait_(wait), twist_variance_(twist_variance)
  {
    boot_floor_ns_ = node.now().nanoseconds();
    id_ = std::to_string(std::random_device{}())+":"+std::to_string(std::random_device{}());
    output_ = node.create_publisher<Frame>("/zlocalization/canonical_frame", 100);
    subscription_ = node.create_subscription<Frame>("/zlocalization/session_input", 100,
      [this](Frame::ConstSharedPtr message) {receive(*message);});
    selection_ = node.create_service<rm_competition_interfaces::srv::SelectLocalizationSource>(
      "/zlocalization/canonical/select_source", [this](
        const std::shared_ptr<rm_competition_interfaces::srv::SelectLocalizationSource::Request> request,
        std::shared_ptr<rm_competition_interfaces::srv::SelectLocalizationSource::Response> response) {
        const auto old = fence_.boundary();
        response->success = fence_.select(request->producer_id, request->minimum_epoch, node_.now().nanoseconds());
        if (fence_.boundary() != old) {boundary();}
        response->canonical_epoch = fence_.boundary();
        response->message = response->success ? "source selected; awaiting fresh observation and TF" :
          "retired source or regressing epoch rejected";
      });
    timer_ = node.create_wall_timer(std::chrono::milliseconds(10), [this]() {tick();});
  }
private:
  static std::int64_t ns(const builtin_interfaces::msg::Time & stamp)
  {return static_cast<std::int64_t>(stamp.sec)*1000000000LL+stamp.nanosec;}
  static bool poseValid(const geometry_msgs::msg::Pose & pose)
  {
    const auto & p = pose.position; const auto & q = pose.orientation;
    const auto norm = q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
      std::isfinite(norm) && std::abs(norm-1.) < 1e-3;
  }
  static tf2::Transform transform(const geometry_msgs::msg::Pose & pose)
  {
    const auto & p = pose.position; const auto & q = pose.orientation;
    return tf2::Transform(tf2::Quaternion(q.x,q.y,q.z,q.w), tf2::Vector3(p.x,p.y,p.z));
  }
  static void pose(const tf2::Transform & t, geometry_msgs::msg::Pose & p)
  {
    p.position.x=t.getOrigin().x(); p.position.y=t.getOrigin().y(); p.position.z=t.getOrigin().z();
    const auto & q=t.getRotation();
    p.orientation.x=q.x(); p.orientation.y=q.y(); p.orientation.z=q.z(); p.orientation.w=q.w();
  }
  void send(Frame frame)
  {
    frame.emitted_at=node_.now(); frame.producer_id=id_; frame.epoch=fence_.boundary();
    frame.sequence=++sequence_;
    // Downstream versions belong to this owner process, not to a retired backend.
    frame.state_generation=sequence_; frame.pose_revision=sequence_;
    output_->publish(frame);
  }
  void notice()
  {
    Frame message;
    message.state_generation=last_generation_; message.pose_revision=last_revision_;
    send(message);
  }
  void boundary()
  {
    pending_.reset(); reference_.reset(); twist_.reset(); global_valid_=false;
    notice();  // ordered before any new-basis odometry or TF
  }
  void invalidate()
  {
    fence_.revoke(); pending_.reset(); global_valid_=false;
    notice();
  }
  void receive(const Frame & message)
  {
    checkClock();
    // Capture admission time after the clock check: passing an earlier snapshot
    // back to the fence would falsely look like a wall-clock backwards jump.
    const auto now=node_.now().nanoseconds();
    if (boot_floor_ns_ == 0 && now > 0) {boot_floor_ns_ = now;}
    const auto emitted=ns(message.emitted_at);
    if (message.emitted_at.sec < 0 || message.emitted_at.nanosec >= 1000000000U ||
      emitted < boot_floor_ns_ || emitted > now+100000000LL || now-emitted > 500000000LL) {return;}
    const auto stamp=message.has_odometry ? ns(message.odometry.header.stamp) : 0;
    if (message.has_odometry && (message.odometry.header.stamp.sec < 0 ||
      message.odometry.header.stamp.nanosec >= 1000000000U || stamp <= 0 ||
      stamp > now+100000000LL || now-stamp > 500000000LL ||
      message.odometry.header.frame_id != input_frame_ ||
      message.odometry.child_frame_id != sensor_ || !poseValid(message.odometry.pose.pose))) {return;}
    const auto previous=fence_.boundary();
    const auto accepted=fence_.observe(message.producer_id, message.epoch, message.sequence,
      message.state_generation, message.pose_revision, stamp, message.has_odometry, message.valid, now);
    if (fence_.boundary() != previous) {boundary();}
    if (!accepted) {return;}
    const bool version_changed = last_generation_ != message.state_generation ||
      last_revision_ != message.pose_revision;
    last_generation_=message.state_generation; last_revision_=message.pose_revision;
    pending_.reset();
    if (!message.valid) {global_valid_=false; notice();}
    if (!message.has_odometry) {
      if (message.valid && version_changed) {invalidate();}
      if (!message.valid) {twist_.reset();}
      return;
    }
    if (message.valid &&
      !rm_competition_interfaces::validatePoseCovariance(message.global_pose.pose.covariance)) {
      invalidate(); return;
    }
    pending_=Pending{message, fence_.ticket(), Steady::now()};
    process();
  }
  void process()
  {
    if (!pending_) {return;}
    const auto work=*pending_;
    if (!fence_.current(work.ticket) ||
      std::chrono::duration<double>(Steady::now()-work.received).count() > wait_) {
      pending_.reset(); return;
    }
    const auto & frame=work.frame;
    const auto stamp=ns(frame.odometry.header.stamp);
    if (node_.now().nanoseconds()-stamp > 500000000LL) {pending_.reset(); return;}
    tf2::Transform base_sensor;
    try {
      const auto mechanical=tf_.lookupTransform(base_, sensor_, rclcpp::Time(frame.odometry.header.stamp),
        rclcpp::Duration::from_seconds(0));
      const auto & t=mechanical.transform.translation; const auto & q=mechanical.transform.rotation;
      base_sensor=tf2::Transform(tf2::Quaternion(q.x,q.y,q.z,q.w),tf2::Vector3(t.x,t.y,t.z));
    } catch (const tf2::TransformException &) {return;}
    checkClock();
    if (!fence_.current(work.ticket)) {pending_.reset(); return;}
    const auto raw=transform(frame.odometry.pose.pose);
    if (!reference_) {reference_=base_sensor*raw.inverse();}
    const auto canonical=rm_localization_adapters::compute_base_transform(*reference_*raw,base_sensor);
    const auto estimate=twist_.update(canonical,stamp);
    pending_.reset();
    if (estimate.status != rm_localization_adapters::TwistEstimateStatus::kValid) {return;}
    Frame output=frame;
    auto & odom=output.odometry;
    odom.header.frame_id=odom_frame_; odom.child_frame_id=base_;
    pose(canonical,odom.pose.pose); odom.pose.covariance.fill(0.);  // missing, not a relabeled covariance
    odom.twist.twist=estimate.twist; odom.twist.covariance.fill(0.);
    for (int i=0;i<6;++i) {odom.twist.covariance[7*i]=twist_variance_[i];}
    if (frame.valid) {
      if (frame.global_pose.header.frame_id != "map" ||
        frame.global_pose.header.stamp != frame.odometry.header.stamp ||
        !poseValid(frame.global_pose.pose.pose) ||
        !rm_competition_interfaces::validatePoseCovariance(frame.global_pose.pose.covariance)) {invalidate(); return;}
      const auto map_sensor=transform(frame.global_pose.pose.pose);
      const auto map_base=rm_localization_adapters::compute_base_transform(map_sensor,base_sensor);
      pose(map_base,output.global_pose.pose.pose);
      output.global_pose.pose.covariance=rm_localization_adapters::shift_pose_covariance(
        frame.global_pose.pose.covariance,map_base.getOrigin()-map_sensor.getOrigin());
      if (!rm_competition_interfaces::validatePoseCovariance(output.global_pose.pose.covariance)) {
        invalidate(); return;
      }
      global_valid_=true; evidence_received_=Steady::now(); evidence_stamp_=stamp;
    } else {output.global_pose=geometry_msgs::msg::PoseWithCovarianceStamped();}
    checkClock();
    if (!fence_.current(work.ticket)) {return;}
    odom_pub_->publish(odom);
    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped tf_message;
      tf_message.header=odom.header; tf_message.child_frame_id=base_;
      tf_message.transform.translation.x=odom.pose.pose.position.x;
      tf_message.transform.translation.y=odom.pose.pose.position.y;
      tf_message.transform.translation.z=odom.pose.pose.position.z;
      tf_message.transform.rotation=odom.pose.pose.orientation;
      broadcaster_.sendTransform(tf_message);
    }
    send(output);
  }
  void checkClock()
  {
    const auto previous=fence_.boundary();
    fence_.clock(node_.now().nanoseconds());
    if (previous != fence_.boundary()) {boundary();}
  }
  void tick()
  {
    checkClock();
    if (global_valid_ && (std::chrono::duration<double>(Steady::now()-evidence_received_).count() >= .5 ||
      node_.now().nanoseconds()-evidence_stamp_ >= 500000000LL)) {invalidate();}
    process();
  }
  struct Pending {Frame frame; std::uint64_t ticket; Steady::time_point received;};
  rclcpp::Node & node_; tf2_ros::Buffer & tf_; tf2_ros::TransformBroadcaster & broadcaster_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rm_localization_adapters::PoseTwistEstimator & twist_;
  rm_competition_interfaces::LocalizationSession fence_;
  std::string input_frame_,sensor_,base_,odom_frame_,id_;
  bool publish_tf_,global_valid_{false}; double wait_;
  std::array<double,6> twist_variance_;
  std::optional<Pending> pending_;
  std::optional<tf2::Transform> reference_;
  std::uint64_t sequence_{0},last_generation_{0},last_revision_{0};
  Steady::time_point evidence_received_{}; std::int64_t evidence_stamp_{0},boot_floor_ns_{0};
  rclcpp::Publisher<Frame>::SharedPtr output_; rclcpp::Subscription<Frame>::SharedPtr subscription_;
  rclcpp::Service<rm_competition_interfaces::srv::SelectLocalizationSource>::SharedPtr selection_;
  rclcpp::TimerBase::SharedPtr timer_;
};
