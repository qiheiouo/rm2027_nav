#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <random>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "rm_competition_interfaces/msg/localization_status.hpp"
#include "rm_competition_interfaces/msg/localization_barrier.hpp"
#include "rm_competition_interfaces/srv/begin_rebase.hpp"
#include "rm_competition_interfaces/msg/lio_source_status.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/transform_broadcaster.h"

#include "rm_relocalization_bridge/relocalization_math.hpp"
#include "rm_relocalization_bridge/correction_gate.hpp"

namespace
{

tf2::Transform poseToTransform(const geometry_msgs::msg::Pose & pose)
{
  tf2::Quaternion rotation(
    pose.orientation.x,
    pose.orientation.y,
    pose.orientation.z,
    pose.orientation.w);
  rotation.normalize();

  tf2::Transform transform;
  transform.setOrigin(tf2::Vector3(pose.position.x, pose.position.y, pose.position.z));
  transform.setRotation(rotation);
  return transform;
}

geometry_msgs::msg::Transform transformToMessage(const tf2::Transform & transform)
{
  geometry_msgs::msg::Transform message;
  message.translation.x = transform.getOrigin().x();
  message.translation.y = transform.getOrigin().y();
  message.translation.z = transform.getOrigin().z();
  message.rotation.x = transform.getRotation().x();
  message.rotation.y = transform.getRotation().y();
  message.rotation.z = transform.getRotation().z();
  message.rotation.w = transform.getRotation().w();
  return message;
}

}  // namespace

class MapOdomFromGlobalPose : public rclcpp::Node
{
public:
  MapOdomFromGlobalPose()
  : Node("map_odom_from_global_pose"),
    odom_cache_(static_cast<std::size_t>(
        std::max<std::int64_t>(
          1, declare_parameter<std::int64_t>("odom_cache_size", 500))))
  {
    started_at_ = now();
    recovery_after_ns_ = started_at_.nanoseconds();
    last_clock_ns_ = recovery_after_ns_;
    producer_id_ = std::to_string(std::random_device{}()) + ":" +
      std::to_string(std::random_device{}());
    status_timeout_sec_ = declare_parameter<double>("status_timeout_sec", 1.0);
    max_future_sec_ = declare_parameter<double>("max_future_sec", 0.1);
    if (!std::isfinite(status_timeout_sec_) || status_timeout_sec_ <= 0.0 ||
      !std::isfinite(max_future_sec_) || max_future_sec_ < 0.0)
    {
      throw std::invalid_argument("localization status time limits are invalid");
    }
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    global_pose_topic_ = declare_parameter<std::string>(
      "global_pose_topic", "/localization/global_pose");
    upstream_valid_topic_ = declare_parameter<std::string>("upstream_valid_topic", "");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odometry/lio");
    output_topic_ = declare_parameter<std::string>(
      "map_to_odom_topic", "/localization/map_to_odom");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 20.0);
    max_pose_odom_dt_sec_ = declare_parameter<double>("max_pose_odom_dt_sec", 0.05);
    max_global_pose_age_sec_ = declare_parameter<double>("max_global_pose_age_sec", 0.5);
    pending_pose_max_wait_sec_ = declare_parameter<double>("pending_pose_max_wait_sec", 0.2);
    pending_pose_max_size_ = static_cast<std::size_t>(
      std::max<std::int64_t>(
        1, declare_parameter<std::int64_t>("pending_pose_max_size", 20)));

    if (publish_rate_hz_ <= 0.0) {
      throw std::invalid_argument("publish_rate_hz must be positive");
    }
    if (
      max_pose_odom_dt_sec_ < 0.0 || max_global_pose_age_sec_ < 0.0 ||
      pending_pose_max_wait_sec_ < 0.0)
    {
      throw std::invalid_argument("time tolerances must not be negative");
    }

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    transform_pub_ = create_publisher<geometry_msgs::msg::TransformStamped>(output_topic_, 10);

    auto valid_qos = rclcpp::QoS(1).reliable().transient_local();
    valid_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/localization/global_localization_valid", valid_qos);

    status_pub_ = create_publisher<rm_competition_interfaces::msg::LocalizationStatus>(
      "/localization/status", rclcpp::QoS(1).reliable());
    source_health_topic_ = declare_parameter<std::string>("source_health_topic", "");
    source_health_timeout_ = declare_parameter<double>("source_health_timeout", 0.3);
    if (!std::isfinite(source_health_timeout_) || source_health_timeout_ <= 0) {
      throw std::invalid_argument("source health timeout must be positive");
    }
    if (source_health_topic_.empty()) {
      odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, rclcpp::QoS(100).reliable(),
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {handleOdometry(*msg);});
    } else {
      source_sub_ = create_subscription<rm_competition_interfaces::msg::LioSourceStatus>(
        source_health_topic_, rclcpp::QoS(10).reliable(),
        [this](const rm_competition_interfaces::msg::LioSourceStatus::SharedPtr msg) {
          handleSourceHealth(*msg);
        });
    }
    correction_enabled_ = declare_parameter<bool>("correction_gate.enabled", false);
    rm_relocalization_bridge::CorrectionLimits limits;
    limits.translation_m = declare_parameter<double>("correction_gate.translation_m", 0.4);
    limits.yaw_rad = declare_parameter<double>("correction_gate.yaw_rad", 0.3);
    const int rejects = declare_parameter<int>("correction_gate.rejection_count", 3);
    const int samples = declare_parameter<int>("correction_gate.transaction_samples", 3);
    if (rejects < 1 || samples < 2) {throw std::invalid_argument("invalid correction sample counts");}
    limits.rejection_count = rejects;
    limits.transaction_samples = samples;
    correction_limits_ = limits;
    correction_gate_ = std::make_unique<rm_relocalization_bridge::CorrectionGate>(limits);
    rebase_enabled_ = declare_parameter<bool>("rebase.enabled", false);
    rebase_timeout_ = declare_parameter<double>("rebase.timeout_sec", 8.0);
    if (rebase_enabled_ && (!correction_enabled_ || !std::isfinite(rebase_timeout_) || rebase_timeout_ <= 0)) {
      throw std::invalid_argument("rebase requires correction gate and positive timeout");
    }
    if (correction_enabled_ && (upstream_valid_topic_.empty() || source_health_topic_.empty())) {
      throw std::invalid_argument("correction gate requires selected backend status and atomic LIO health");
    }
    correction_state_pub_ = create_publisher<std_msgs::msg::String>(
      "/localization/correction_state", rclcpp::QoS(1).reliable().transient_local());
    upstream_valid_required_ = !upstream_valid_topic_.empty();
    if (upstream_valid_required_) {
      upstream_status_sub_ = create_subscription<rm_competition_interfaces::msg::LocalizationStatus>(
        upstream_valid_topic_ + "/status", rclcpp::QoS(1).reliable(),
        [this](const rm_competition_interfaces::msg::LocalizationStatus::SharedPtr msg) {
          handleUpstreamStatus(*msg);
        });
    } else {
      // Direct external-pose fixtures have a finite correction lease, no cached Bool authority.
      global_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        global_pose_topic_, rclcpp::QoS(10).reliable(),
        [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          checkValidityLocked();
          handleGlobalPoseLocked(*msg, now() + rclcpp::Duration::from_seconds(status_timeout_sec_));
        });
    }

    if (rebase_enabled_) {
      begin_rebase_ = create_service<rm_competition_interfaces::srv::BeginRebase>(
        "/localization/begin_rebase", [this](
          const std::shared_ptr<rm_competition_interfaces::srv::BeginRebase::Request> request,
          std::shared_ptr<rm_competition_interfaces::srv::BeginRebase::Response> response) {
          std::lock_guard<std::mutex> lock(mutex_);
          beginRebase(*request, *response);
        });
      nav_barrier_ = create_subscription<rm_competition_interfaces::msg::LocalizationBarrier>(
        "/localization/navigation_drained", 10, [this](
          const rm_competition_interfaces::msg::LocalizationBarrier & message) {
          std::lock_guard<std::mutex> lock(mutex_);
          if (rebase_phase_ == Status::DRAIN_OLD_NAV && barrierMatches(message)) {
            nav_drained_ = true;
          }
        });
      map_barrier_ = create_subscription<rm_competition_interfaces::msg::LocalizationBarrier>(
        "/localization/map_cache_primed", 10, [this](
          const rm_competition_interfaces::msg::LocalizationBarrier & message) {
          std::lock_guard<std::mutex> lock(mutex_);
          if (rebase_phase_ == Status::PRIME_NEW_SESSION && barrierMatches(message) &&
            rclcpp::Time(message.header.stamp) > committed_at_) {map_primed_ = true;}
        });
    }
    reset_service_ = create_service<std_srvs::srv::Trigger>(
      "/localization/reset_map_to_odom",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (rebase_enabled_ && has_committed_) {
          response->success = false;
          response->message = "use begin_rebase with a verified candidate; legacy reset cannot bypass drain";
          return;
        }
        if (correction_enabled_) {
          if (now().nanoseconds() <= started_at_.nanoseconds() || now().nanoseconds() < last_clock_ns_) {
            response->success = false;
            response->message = "prepare requires advancing ROS time";
            return;
          }
          started_at_ = now();
          sequence_ = 0;
          correction_gate_->prepare();
          await_new_backend_ = true;
          transaction_epoch_floor_ = std::max(upstream_epoch_ns_, started_at_.nanoseconds());
          odom_cache_.clear();
          correctionState("PREPARED: reset selected backend session, then supply verified observations");
        }
        recovery_after_ns_ = std::max(recovery_after_ns_, now().nanoseconds());
        invalidateLocked();
        response->success = true;
        response->message = correction_enabled_ ? "relocalization prepared; awaiting new backend epoch" :
          "map->odom correction invalidated";
      });

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    publish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() {publishCurrentTransform();});

    publishValid(false);
    RCLCPP_INFO(
      get_logger(),
      "Global-pose bridge ready: %s + %s -> dynamic %s -> %s. "
      "It publishes no identity fallback.",
      global_pose_topic_.c_str(), odom_topic_.c_str(), map_frame_.c_str(), odom_frame_.c_str());
    if (upstream_valid_required_) {
      RCLCPP_INFO(
        get_logger(), "map->odom validity follows upstream gate %s.",
        upstream_valid_topic_.c_str());
    }
  }

private:
  using Status = rm_competition_interfaces::msg::LocalizationStatus;
  struct RebaseSample {
    tf2::Transform map_base, odom_base;
    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    rclcpp::Time deadline;
    std::chrono::steady_clock::time_point received;
  };

  bool barrierMatches(const rm_competition_interfaces::msg::LocalizationBarrier & message) const
  {
    if (message.header.stamp.sec < 0 || message.header.stamp.nanosec >= 1000000000U ||
      message.started_at.sec < 0 || message.started_at.nanosec >= 1000000000U) {return false;}
    const auto emitted = rclcpp::Time(message.header.stamp);
    return message.producer_id == producer_id_ && rclcpp::Time(message.started_at) == started_at_ &&
      emitted >= started_at_ && emitted <= now() && (now() - emitted).seconds() < 0.5;
  }

  bool sampleFresh(const RebaseSample & sample) const
  {
    return source_healthy_ && upstream_valid_ && sample.deadline > now() &&
      (now() - rclcpp::Time(sample.pose.header.stamp)).seconds() <= max_global_pose_age_sec_ &&
      std::chrono::duration<double>(std::chrono::steady_clock::now() - sample.received).count() <=
      max_global_pose_age_sec_;
  }

  std::unique_ptr<rm_relocalization_bridge::CorrectionGate> verifiedWindow() const
  {
    if (rebase_samples_.size() != correction_limits_.transaction_samples) {return nullptr;}
    auto gate = std::make_unique<rm_relocalization_bridge::CorrectionGate>(correction_limits_);
    for (const auto & sample : rebase_samples_) {
      if (!sampleFresh(sample)) {return nullptr;}
      gate->evaluate(sample.map_base, sample.odom_base);
    }
    return gate->preparing() || gate->latched() ? nullptr : std::move(gate);
  }

  void beginRebase(const rm_competition_interfaces::srv::BeginRebase::Request & request,
    rm_competition_interfaces::srv::BeginRebase::Response & response)
  {
    checkValidityLocked();
    response.accepted = false;
    response.detail = "busy, stale session/candidate, unhealthy source or inconsistent candidate window";
    if (request.expected_session.sec < 0 || request.expected_session.nanosec >= 1000000000U ||
      request.candidate_stamp.sec < 0 || request.candidate_stamp.nanosec >= 1000000000U ||
      !has_committed_ || request.producer_id != producer_id_ ||
      rclcpp::Time(request.expected_session) != started_at_ || now() <= started_at_ ||
      now().nanoseconds() < last_clock_ns_ ||
      rebase_phase_ == Status::DRAIN_OLD_NAV || rebase_phase_ == Status::PRIME_NEW_SESSION) {return;}
    auto gate = verifiedWindow();
    const bool observed = std::any_of(rebase_samples_.begin(), rebase_samples_.end(),
      [&request](const RebaseSample & sample) {return sample.pose.header.stamp == request.candidate_stamp;});
    if (!gate || !observed) {return;}
    rebase_anchor_ = gate->trusted();
    rebase_backend_epoch_ = upstream_epoch_ns_;
    rebase_backend_id_ = upstream_id_;
    rebase_source_generation_ = source_generation_;
    rebase_source_epoch_ = source_epoch_;
    started_at_ = now(); sequence_ = 0;
    rebase_began_ = std::chrono::steady_clock::now();
    rebase_phase_ = Status::DRAIN_OLD_NAV;
    nav_drained_ = map_primed_ = false;
    // Keep the verified candidates and old correction. The ordinary gate remains latched
    // until the explicit transaction really commits; no seed/reset authority is implied.
    valid_ = false;
    pending_global_poses_.clear();
    publishValid(false);
    response.accepted = true;
    response.transaction_session = started_at_;
    response.detail = "verified candidate; old navigation must drain before map->odom commit";
    correctionState("DRAIN_OLD_NAV");
  }

  void failRebase()
  {
    if (rebase_phase_ == Status::DRAIN_OLD_NAV || rebase_phase_ == Status::PRIME_NEW_SESSION) {
      rebase_phase_ = Status::REBASE_FAILED;
      valid_ = false;
      correctionState("REBASE_FAILED: explicit fresh request required");
    }
  }

  void advanceRebase()
  {
    if (rebase_phase_ != Status::DRAIN_OLD_NAV && rebase_phase_ != Status::PRIME_NEW_SESSION) {return;}
    if (std::chrono::duration<double>(std::chrono::steady_clock::now() - rebase_began_).count() >
      rebase_timeout_ || !source_healthy_ || !upstream_valid_ ||
      upstream_epoch_ns_ != rebase_backend_epoch_ || upstream_id_ != rebase_backend_id_ ||
      source_generation_ != rebase_source_generation_ || source_epoch_ != rebase_source_epoch_) {
      failRebase(); return;
    }
    auto gate = verifiedWindow();
    if (!gate) {return;}
    const auto & sample = rebase_samples_.back();
    auto anchor_check = rm_relocalization_bridge::CorrectionGate(correction_limits_);
    for (std::size_t i = 0; i < correction_limits_.transaction_samples; ++i) {
      anchor_check.evaluate(rebase_anchor_ * sample.odom_base, sample.odom_base);
    }
    if (anchor_check.evaluate(sample.map_base, sample.odom_base).decision !=
      rm_relocalization_bridge::CorrectionDecision::Accept) {failRebase(); return;}
    if (rebase_phase_ == Status::DRAIN_OLD_NAV && nav_drained_) {
      map_to_odom_ = gate->trusted();
      correction_gate_ = std::move(gate);
      accepted_pose_ = sample.pose;
      evidence_until_ = sample.deadline;
      last_evidence_received_ = sample.received;
      committed_at_ = now();
      rebase_tf_published_ = false;
      rebase_phase_ = Status::PRIME_NEW_SESSION;
      correctionState("PRIME_NEW_SESSION");
    } else if (rebase_phase_ == Status::PRIME_NEW_SESSION && map_primed_) {
      // Require post-commit global evidence as well as the map-cache receipt.
      if (rclcpp::Time(sample.pose.header.stamp) <= committed_at_) {return;}
      // Cache prime covers the committed geometry. A later correction must also
      // pass the ordinary gate against THAT C, not only the original request anchor.
      auto ordinary = *correction_gate_;
      if (ordinary.evaluate(sample.map_base, sample.odom_base).decision !=
        rm_relocalization_bridge::CorrectionDecision::Accept) {failRebase(); return;}
      *correction_gate_ = ordinary;
      map_to_odom_ = ordinary.trusted();
      accepted_pose_ = sample.pose;
      evidence_until_ = sample.deadline;
      last_evidence_received_ = sample.received;
      rebase_phase_ = Status::ACTIVE;
      valid_ = true;
      correctionState("REBASE_ACTIVE");
    }
  }

  void correctionState(const std::string & text)
  {
    if (!correction_enabled_) {return;}
    std_msgs::msg::String message;
    message.data = text;
    correction_state_pub_->publish(message);
  }

  void rejectCandidate(const char * reason)
  {
    if (correction_enabled_) {
      correction_gate_->break_consensus();
      correctionState(std::string("REJECT: ") + reason);
    } else {invalidateLocked();}
  }

  bool validatePose(const geometry_msgs::msg::Pose & pose) const
  {
    const double quaternion_norm_squared =
      pose.orientation.x * pose.orientation.x +
      pose.orientation.y * pose.orientation.y +
      pose.orientation.z * pose.orientation.z +
      pose.orientation.w * pose.orientation.w;
    if (
      !std::isfinite(pose.position.x) ||
      !std::isfinite(pose.position.y) ||
      !std::isfinite(pose.position.z) ||
      !std::isfinite(quaternion_norm_squared) ||
      quaternion_norm_squared <= 1.0e-12)
    {
      return false;
    }
    const tf2::Transform transform = poseToTransform(pose);
    return rm_relocalization_bridge::isFiniteTransform(transform);
  }

  void invalidateLocked()
  {
    failRebase();
    rebase_samples_.clear();
    if (correction_enabled_) {correction_gate_->break_consensus();}
    valid_ = false;
    upstream_valid_ = false;
    recovery_after_ns_ = std::max(recovery_after_ns_, std::max(last_pose_ns_, upstream_pose_ns_));
    pending_global_poses_.clear();
    publishValid(false);
  }

  void invalidateSourceLocked()
  {
    if (source_healthy_) {
      // New validity session immediately retires Mission's old action callbacks.
      if (now() > started_at_) {
        started_at_ = now();
        sequence_ = 0;
      } else {
        // A paused/reversed ROS clock cannot mint a newer time-based epoch yet.
        source_epoch_pending_ = true;
      }
      recovery_after_ns_ = std::max(recovery_after_ns_, started_at_.nanoseconds());
    }
    source_healthy_ = false;
    odom_cache_.clear();
    invalidateLocked();
  }

  void handleSourceHealth(const rm_competition_interfaces::msg::LioSourceStatus & msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    checkValidityLocked();
    const auto well = [](const builtin_interfaces::msg::Time & t) {
        return t.sec >= 0 && t.nanosec < 1000000000U;
      };
    if (!well(msg.header.stamp) || !well(msg.started_at) || !well(msg.valid_until) ||
      !well(msg.odometry.header.stamp)) {invalidateSourceLocked(); return;}
    const auto epoch = rclcpp::Time(msg.started_at).nanoseconds();
    const auto emitted = rclcpp::Time(msg.header.stamp).nanoseconds();
    const auto t = rclcpp::Time(msg.odometry.header.stamp).nanoseconds();
    const auto current = now().nanoseconds();
    // Late traffic from a retired source generation is inert, including invalid heartbeats.
    if (epoch < source_epoch_ || (epoch == source_epoch_ && source_epoch_ != 0 &&
      (msg.producer_id != source_id_ || msg.generation < source_generation_ ||
      (msg.generation == source_generation_ && msg.sequence <= source_sequence_)))) {return;}
    if (msg.producer_id.empty() || epoch <= 0 || epoch > emitted || msg.generation == 0 ||
      msg.sequence == 0 || (current - emitted)*1e-9 > source_health_timeout_ ||
      (emitted - current)*1e-9 > max_future_sec_)
    {invalidateSourceLocked(); return;}
    if (epoch != source_epoch_ || msg.generation != source_generation_) {
      invalidateSourceLocked();
      source_stamp_ = 0;
    }
    source_epoch_ = epoch;
    source_id_ = msg.producer_id;
    source_generation_ = msg.generation;
    source_sequence_ = msg.sequence;
    if (!msg.healthy) {invalidateSourceLocked(); return;}
    const auto deadline = rclcpp::Time(msg.valid_until);
    if (t <= source_stamp_ || t <= recovery_after_ns_ || t < epoch ||
      t > current + static_cast<int64_t>(max_future_sec_ * 1e9) ||
      (current - t)*1e-9 > source_health_timeout_ || deadline.nanoseconds() <= current ||
      (deadline.nanoseconds() - t)*1e-9 > source_health_timeout_ + 1e-6 ||
      msg.odometry.header.frame_id != odom_frame_ || msg.odometry.child_frame_id != base_frame_ ||
      !validatePose(msg.odometry.pose.pose))
    {invalidateSourceLocked(); return;}
    if (source_epoch_pending_) {
      if (current <= started_at_.nanoseconds()) {return;}
      started_at_ = now();
      sequence_ = 0;
      source_epoch_pending_ = false;
    }
    source_stamp_ = t;
    source_until_ = deadline;
    source_received_ = std::chrono::steady_clock::now();
    source_healthy_ = true;
    if (!odom_cache_.add({t, poseToTransform(msg.odometry.pose.pose)})) {
      invalidateSourceLocked(); return;
    }
    processPendingGlobalPoses();
  }

  void checkValidityLocked()
  {
    const auto clock_ns = now().nanoseconds();
    if (started_at_.nanoseconds() <= 0 && clock_ns > 0) {
      started_at_ = now();
      recovery_after_ns_ = clock_ns;
    }
    const bool clock_reversed = clock_ns < last_clock_ns_;
    if (!source_health_topic_.empty() && source_healthy_ &&
      (clock_reversed || clock_ns > source_until_.nanoseconds() ||
      std::chrono::duration<double>(std::chrono::steady_clock::now() - source_received_).count() >
      source_health_timeout_)) {invalidateSourceLocked();}
    last_clock_ns_ = std::max(last_clock_ns_, clock_ns);
    const double silence = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_evidence_received_).count();
    if (clock_reversed || (valid_ &&
      (silence > status_timeout_sec_ || clock_ns > evidence_until_.nanoseconds() ||
      (upstream_valid_required_ &&
      (clock_ns - upstream_emitted_ns_) * 1.0e-9 > status_timeout_sec_))))
    {
      if (clock_reversed) {recovery_after_ns_ = last_clock_ns_;}
      invalidateLocked();
    }
  }

  void handleUpstreamStatus(const rm_competition_interfaces::msg::LocalizationStatus & msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    checkValidityLocked();
    const auto well_formed = [](const builtin_interfaces::msg::Time & stamp) {
        return stamp.sec >= 0 && stamp.nanosec < 1000000000U;
      };
    if (!well_formed(msg.header.stamp) || !well_formed(msg.started_at) ||
      !well_formed(msg.valid_until) || !well_formed(msg.pose.header.stamp))
    {
      rejectCandidate("malformed source stamp");
      return;
    }
    const auto current = now();
    const auto emitted = rclcpp::Time(msg.header.stamp);
    const auto epoch = rclcpp::Time(msg.started_at).nanoseconds();
    const auto deadline = rclcpp::Time(msg.valid_until);
    const double age = (current - emitted).seconds();
    if (correction_enabled_ &&
      ((await_new_backend_ && epoch <= transaction_epoch_floor_) ||
      (!await_new_backend_ && !upstream_id_.empty() &&
      (epoch != upstream_epoch_ns_ || msg.producer_id != upstream_id_))))
    {rejectCandidate("unselected or retired backend epoch"); return;}
    if (msg.producer_id.empty() || msg.sequence == 0 || epoch <= 0 ||
      epoch > emitted.nanoseconds() || emitted.nanoseconds() <= started_at_.nanoseconds() ||
      age < -max_future_sec_ || age > status_timeout_sec_ || epoch < upstream_epoch_ns_ ||
      (epoch == upstream_epoch_ns_ && !upstream_id_.empty() && msg.producer_id != upstream_id_) ||
      (msg.producer_id == upstream_id_ && epoch == upstream_epoch_ns_ &&
      (msg.sequence <= upstream_sequence_ || emitted.nanoseconds() < upstream_emitted_ns_)))
    {
      rejectCandidate("stale, future or reordered backend status");
      return;
    }
    if (msg.producer_id != upstream_id_ || epoch != upstream_epoch_ns_) {
      if (!upstream_id_.empty()) {invalidateLocked();}
      upstream_id_ = msg.producer_id;
      upstream_epoch_ns_ = epoch;
      upstream_pose_ns_ = 0;
      await_new_backend_ = false;
    }
    upstream_sequence_ = msg.sequence;
    upstream_emitted_ns_ = emitted.nanoseconds();
    if (!msg.valid || deadline <= current ||
      (deadline - emitted).seconds() > 1.6)
    {
      invalidateLocked();
      return;
    }
    upstream_valid_ = true;
    const auto source_ns = rclcpp::Time(msg.pose.header.stamp).nanoseconds();
    if (correction_enabled_ && source_ns <= upstream_epoch_ns_) {
      rejectCandidate("measurement predates backend admission epoch");
    } else if (source_ns > upstream_pose_ns_) {
      upstream_pose_ns_ = source_ns;
      handleGlobalPoseLocked(msg.pose, deadline);
    } else if (source_ns < upstream_pose_ns_) {
      rejectCandidate("reordered backend measurement");
    } else if (valid_ && source_ns == rclcpp::Time(accepted_pose_.header.stamp).nanoseconds()) {
      // Only the accepted measurement may prove liveness, never a rejected pose.
      last_evidence_received_ = std::chrono::steady_clock::now();
      evidence_until_ = std::min(evidence_until_, deadline);
    }
    // A heartbeat for the held pose cannot restore validity after invalidation.
    publishValid(valid_ && upstream_valid_);
  }

  void handleOdometry(const nav_msgs::msg::Odometry & msg)
  {
    if (msg.header.frame_id != odom_frame_ || msg.child_frame_id != base_frame_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejecting odometry with frames '%s' -> '%s'; expected '%s' -> '%s'.",
        msg.header.frame_id.c_str(), msg.child_frame_id.c_str(),
        odom_frame_.c_str(), base_frame_.c_str());
      return;
    }
    const rclcpp::Time stamp(msg.header.stamp);
    if (stamp.nanoseconds() <= 0 || !validatePose(msg.pose.pose)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejecting odometry with zero timestamp or invalid pose.");
      return;
    }

    const auto transform = poseToTransform(msg.pose.pose);
    std::lock_guard<std::mutex> lock(mutex_);
    checkValidityLocked();
    const bool monotonic = odom_cache_.add({stamp.nanoseconds(), transform});
    if (!monotonic) {
      invalidateLocked();
      RCLCPP_WARN(
        get_logger(),
        "Odometry timestamp reset detected. Cleared cache and invalidated map->odom.");
      return;
    }
    processPendingGlobalPoses();
  }

  void handleGlobalPoseLocked(
    const geometry_msgs::msg::PoseWithCovarianceStamped & msg, const rclcpp::Time & deadline)
  {
    correctionState("OBSERVED");
    if (msg.header.stamp.sec < 0 || msg.header.stamp.nanosec >= 1000000000U) {
      rejectCandidate("malformed measurement stamp"); return;
    }
    if (msg.header.frame_id != map_frame_) {
      rejectCandidate("frame, finite pose or measurement time");
      RCLCPP_WARN(
        get_logger(), "Rejecting global pose in frame '%s'; expected '%s'.",
        msg.header.frame_id.c_str(), map_frame_.c_str());
      return;
    }

    const rclcpp::Time stamp(msg.header.stamp);
    if (stamp.nanoseconds() <= 0 || !validatePose(msg.pose.pose)) {
      rejectCandidate("frame, finite pose or measurement time");
      RCLCPP_WARN(get_logger(), "Rejecting global pose with zero timestamp or invalid pose.");
      return;
    }

    const auto current_time = now();
    const double age = (current_time - stamp).seconds();
    if (stamp.nanoseconds() <= recovery_after_ns_ || stamp.nanoseconds() <= last_pose_ns_ ||
      age < -max_future_sec_ || age > max_global_pose_age_sec_) {
      rejectCandidate("frame, finite pose or measurement time");
      RCLCPP_WARN(
        get_logger(), "Rejecting stale, future, replayed or pre-session global pose: age %.3f s, age limit %.3f s.",
        age, max_global_pose_age_sec_);
      return;
    }

    if (!source_health_topic_.empty() && !source_healthy_) {
      correctionState("ELIGIBLE_WAIT_LIO: no commit or recovery authority");
      correction_gate_->break_consensus();
      return;
    }
    if (correction_enabled_ && correction_gate_->latched() && !rebase_enabled_) {
      correctionState("FAULT_LATCHED: explicit prepare required");
      return;
    }
    if (correction_enabled_ && correction_gate_->preparing() && last_pose_ns_ > 0 &&
      (stamp.nanoseconds() - last_pose_ns_) * 1e-9 > status_timeout_sec_) {
      // Do not accumulate transaction confidence across a source silence gap.
      correction_gate_->break_consensus();
    }
    correctionState("ELIGIBLE");
    const auto map_to_base = poseToTransform(msg.pose.pose);
    const auto tolerance_ns = static_cast<std::int64_t>(max_pose_odom_dt_sec_ * 1.0e9);

    last_pose_ns_ = stamp.nanoseconds();
    const auto odom_sample = odom_cache_.nearest(stamp.nanoseconds(), tolerance_ns);
    if (!odom_sample.has_value()) {
      if (pending_global_poses_.size() >= pending_pose_max_size_) {
        pending_global_poses_.pop_front();
        RCLCPP_WARN(get_logger(), "Global-pose pending queue full; dropped oldest pose.");
      }
      pending_global_poses_.push_back({
        stamp.nanoseconds(), map_to_base, std::chrono::steady_clock::now(), msg, deadline});
      return;
    }

    acceptCorrection(map_to_base, *odom_sample, msg, deadline, std::chrono::steady_clock::now());
  }

  struct PendingGlobalPose
  {
    std::int64_t stamp_nanoseconds;
    tf2::Transform map_to_base;
    std::chrono::steady_clock::time_point received_at;
    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    rclcpp::Time deadline;
  };

  void processPendingGlobalPoses()
  {
    const auto tolerance_ns = static_cast<std::int64_t>(max_pose_odom_dt_sec_ * 1.0e9);
    const auto now_steady = std::chrono::steady_clock::now();
    auto pending = pending_global_poses_.begin();
    while (pending != pending_global_poses_.end()) {
      const double waiting_seconds =
        std::chrono::duration<double>(now_steady - pending->received_at).count();
      const double source_age = (now().nanoseconds() - pending->stamp_nanoseconds) * 1.0e-9;
      if (waiting_seconds > pending_pose_max_wait_sec_ || source_age > max_global_pose_age_sec_ ||
        pending->stamp_nanoseconds <= recovery_after_ns_ ||
        (upstream_valid_required_ && !upstream_valid_))
      {
        if (correction_enabled_) {correction_gate_->break_consensus();}
        pending = pending_global_poses_.erase(pending);
        continue;
      }
      const auto odom_sample = odom_cache_.nearest(pending->stamp_nanoseconds, tolerance_ns);
      if (odom_sample.has_value()) {
        const auto matched = *pending;
        pending_global_poses_.erase(pending);
        acceptCorrection(matched.map_to_base, *odom_sample, matched.pose, matched.deadline, matched.received_at);
        pending = pending_global_poses_.begin();
        continue;
      }
      ++pending;
    }
  }

  void acceptCorrection(
    const tf2::Transform & map_to_base,
    const rm_relocalization_bridge::TimedTransform & odom_sample,
    const geometry_msgs::msg::PoseWithCovarianceStamped & source_pose,
    const rclcpp::Time & deadline, const std::chrono::steady_clock::time_point & received)
  {
    if (rclcpp::Time(source_pose.header.stamp).nanoseconds() <= recovery_after_ns_ ||
      rclcpp::Time(source_pose.header.stamp) <= rclcpp::Time(accepted_pose_.header.stamp) ||
      deadline <= now() || (correction_enabled_ && !source_healthy_)) {
      rejectCandidate("expired or unhealthy at commit"); return;
    }
    const auto candidate = rm_relocalization_bridge::computeMapToOdom(
      map_to_base, odom_sample.transform);
    if (!rm_relocalization_bridge::isFiniteTransform(candidate)) {
      RCLCPP_ERROR(get_logger(), "Computed non-finite map->odom; correction rejected.");
      invalidateLocked();
      return;
    }

    if (rebase_enabled_) {
      rebase_samples_.push_back({map_to_base, odom_sample.transform, source_pose, deadline, received});
      while (rebase_samples_.size() > correction_limits_.transaction_samples) {rebase_samples_.pop_front();}
      if (rebase_phase_ != Status::ACTIVE) {return;}
    }
    if (correction_enabled_) {
      const auto result = correction_gate_->evaluate(map_to_base, odom_sample.transform);
      using Decision = rm_relocalization_bridge::CorrectionDecision;
      if (result.decision == Decision::Latched) {
        if (valid_) {invalidateLocked();}
        correctionState("FAULT_LATCHED: explicit prepare required");
        return;
      }
      if (result.decision != Decision::Accept) {
        correctionState(result.decision == Decision::Verifying ? "VERIFYING: consecutive correction samples" :
          "REJECT: residual " + std::to_string(result.translation_m) + " m / " + std::to_string(result.yaw_rad) + " rad");
        return;
      }
      correctionState("GATED_ACCEPT");
    }
    has_committed_ = true;
    map_to_odom_ = candidate;
    accepted_pose_ = source_pose;
    last_evidence_received_ = received;
    evidence_until_ = deadline;
    valid_ = true;
    publishValid(valid_ && (!upstream_valid_required_ || upstream_valid_));
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Accepted global pose at %.6f; map->odom correction updated.",
      rclcpp::Time(source_pose.header.stamp).seconds());
  }

  void publishValid(bool value)
  {
    std_msgs::msg::Bool message;
    value = value && (source_health_topic_.empty() || source_healthy_);
    message.data = value;
    valid_pub_->publish(message);
    rm_competition_interfaces::msg::LocalizationStatus status;
    status.header.stamp = now();
    status.header.frame_id = map_frame_;
    status.producer_id = producer_id_;
    status.started_at = started_at_;
    status.sequence = ++sequence_;
    status.valid = value;
    status.valid_until = !source_health_topic_.empty() && source_until_ < evidence_until_ ?
      source_until_ : evidence_until_;
    status.pose = accepted_pose_;
    status.transaction_phase = rebase_phase_;
    status.committed_at = committed_at_;
    status_pub_->publish(status);
  }

  void publishCurrentTransform()
  {
    tf2::Transform transform;
    rclcpp::Time stamp = now();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      checkValidityLocked();
      advanceRebase();
      publishValid(valid_ && (!upstream_valid_required_ || upstream_valid_));
      if ((!valid_ && rebase_phase_ != Status::PRIME_NEW_SESSION) || (upstream_valid_required_ && !upstream_valid_) ||
        (!source_health_topic_.empty() && !source_healthy_)) {
        return;
      }
      transform = map_to_odom_;
      if (rebase_phase_ == Status::PRIME_NEW_SESSION && !rebase_tf_published_) {
        stamp = committed_at_;
        rebase_tf_published_ = true;
      }
    }

    geometry_msgs::msg::TransformStamped message;
    message.header.stamp = stamp;
    message.header.frame_id = map_frame_;
    message.child_frame_id = odom_frame_;
    message.transform = transformToMessage(transform);
    transform_pub_->publish(message);
    if (publish_tf_) {
      tf_broadcaster_->sendTransform(message);
    }
  }

  bool rebase_enabled_{false}, has_committed_{false}, nav_drained_{false}, map_primed_{false};
  bool rebase_tf_published_{false};
  uint8_t rebase_phase_{Status::ACTIVE};
  double rebase_timeout_{8.0};
  rm_relocalization_bridge::CorrectionLimits correction_limits_;
  std::deque<RebaseSample> rebase_samples_;
  tf2::Transform rebase_anchor_;
  int64_t rebase_backend_epoch_{0}, rebase_source_epoch_{0};
  uint64_t rebase_source_generation_{0};
  std::string rebase_backend_id_;
  rclcpp::Time committed_at_{0, 0, RCL_ROS_TIME};
  std::chrono::steady_clock::time_point rebase_began_;
  rclcpp::Service<rm_competition_interfaces::srv::BeginRebase>::SharedPtr begin_rebase_;
  rclcpp::Subscription<rm_competition_interfaces::msg::LocalizationBarrier>::SharedPtr nav_barrier_, map_barrier_;
  bool correction_enabled_{false}, await_new_backend_{false};
  int64_t transaction_epoch_floor_{0};
  std::unique_ptr<rm_relocalization_bridge::CorrectionGate> correction_gate_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr correction_state_pub_;
  std::string source_health_topic_, source_id_;
  double source_health_timeout_{0.3};
  bool source_healthy_{false}, source_epoch_pending_{false};
  int64_t source_epoch_{0}, source_stamp_{0};
  uint64_t source_generation_{0}, source_sequence_{0};
  rclcpp::Time source_until_{0, 0, RCL_ROS_TIME};
  std::chrono::steady_clock::time_point source_received_;
  rclcpp::Subscription<rm_competition_interfaces::msg::LioSourceStatus>::SharedPtr source_sub_;
  std::string producer_id_, upstream_id_;
  rclcpp::Time started_at_{0, 0, RCL_ROS_TIME}, evidence_until_{0, 0, RCL_ROS_TIME};
  int64_t recovery_after_ns_{0}, last_clock_ns_{0}, last_pose_ns_{0};
  int64_t upstream_epoch_ns_{0}, upstream_emitted_ns_{0}, upstream_pose_ns_{0};
  uint64_t sequence_{0}, upstream_sequence_{0};
  double status_timeout_sec_{1.0}, max_future_sec_{0.1};
  std::chrono::steady_clock::time_point last_evidence_received_;
  geometry_msgs::msg::PoseWithCovarianceStamped accepted_pose_;
  rclcpp::Publisher<rm_competition_interfaces::msg::LocalizationStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<rm_competition_interfaces::msg::LocalizationStatus>::SharedPtr upstream_status_sub_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string global_pose_topic_;
  std::string upstream_valid_topic_;
  std::string odom_topic_;
  std::string output_topic_;
  bool publish_tf_;
  double publish_rate_hz_;
  double max_pose_odom_dt_sec_;
  double max_global_pose_age_sec_;
  double pending_pose_max_wait_sec_;
  std::size_t pending_pose_max_size_;

  std::mutex mutex_;
  rm_relocalization_bridge::TimedTransformCache odom_cache_;
  std::deque<PendingGlobalPose> pending_global_poses_;
  tf2::Transform map_to_odom_;
  bool valid_ = false;
  bool upstream_valid_required_ = false;
  bool upstream_valid_ = false;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr transform_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr valid_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
  global_pose_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapOdomFromGlobalPose>());
  rclcpp::shutdown();
  return 0;
}
