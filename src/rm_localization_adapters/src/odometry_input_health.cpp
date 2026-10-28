#include "rm_localization_adapters/odometry_input_health.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include "tf2/LinearMath/Quaternion.h"

namespace rm_localization_adapters
{
namespace
{
int64_t stamp(const nav_msgs::msg::Odometry & m)
{return int64_t(m.header.stamp.sec) * 1000000000LL + m.header.stamp.nanosec;}
double translation(const nav_msgs::msg::Odometry & a, const nav_msgs::msg::Odometry & b)
{
  const auto & p = a.pose.pose.position;
  const auto & q = b.pose.pose.position;
  return std::hypot(std::hypot(p.x - q.x, p.y - q.y), p.z - q.z);
}
double rotation(const nav_msgs::msg::Odometry & a, const nav_msgs::msg::Odometry & b)
{
  const auto & p = a.pose.pose.orientation;
  const auto & q = b.pose.pose.orientation;
  tf2::Quaternion x(p.x, p.y, p.z, p.w), y(q.x, q.y, q.z, q.w);
  x.normalize(); y.normalize();
  return x.angleShortestPath(y);
}
double norm(const geometry_msgs::msg::Vector3 & v)
{return std::hypot(std::hypot(v.x, v.y), v.z);}
bool finite(const nav_msgs::msg::Odometry & m)
{
  const auto & p = m.pose.pose;
  const auto & t = m.twist.twist;
  for (const double v : {p.position.x, p.position.y, p.position.z,
      p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w,
      t.linear.x, t.linear.y, t.linear.z, t.angular.x, t.angular.y, t.angular.z})
  {if (!std::isfinite(v)) {return false;}}
  const auto & q = p.orientation;
  const double n = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
  return std::isfinite(n) && n > 1e-12;
}
}  // namespace

OdometryInputHealth::OdometryInputHealth(InputHealthConfig c) : config_(c)
{
  for (double v : {c.max_age, c.max_silence, c.max_speed, c.max_angular_speed,
      c.recovery_translation, c.recovery_rotation, c.speed_error, c.angular_error,
      c.frozen_duration})
  {if (!std::isfinite(v) || v <= 0) {throw std::invalid_argument("positive health limits required");}}
  for (double v : {c.max_future, c.translation_slack, c.rotation_slack})
  {if (!std::isfinite(v) || v < 0) {throw std::invalid_argument("nonnegative health limits required");}}
  if (c.recovery_samples < 2) {throw std::invalid_argument("recovery requires at least two samples");}
}

void OdometryInputHealth::fault(const std::string & reason, int64_t now)
{
  if (healthy_) {++generation_;}
  healthy_ = false;
  recovery_count_ = 0;
  have_previous_ = false;
  last_candidate_steady_ = 0;
  frozen_since_ = 0;
  recovery_after_ = std::max({recovery_after_, now, stamp_highwater_});
  reason_ = reason;
}

void OdometryInputHealth::tick(int64_t ros_now, int64_t steady_now)
{
  if (ros_now < clock_highwater_) {fault("clock_rollback", clock_highwater_);}
  clock_highwater_ = std::max(clock_highwater_, ros_now);
  if (last_candidate_steady_ > 0 &&
    (steady_now - last_candidate_steady_) * 1e-9 > config_.max_silence)
  {fault("output_silence", ros_now);}
}

bool OdometryInputHealth::evaluate(
  const nav_msgs::msg::Odometry & m, int64_t now, int64_t steady)
{
  tick(now, steady);
  const auto reject = [&](const char * why) {fault(why, now); return false;};
  if (now <= 0 || now < clock_highwater_) {return reject("clock_rollback");}
  if (m.header.stamp.sec < 0 || m.header.stamp.nanosec >= 1000000000U || stamp(m) <= 0)
  {return reject("malformed_stamp");}
  const auto t = stamp(m);
  const double age = (now - t) * 1e-9;
  if (age < -config_.max_future) {return reject("future");}
  if (age > config_.max_age) {return reject("stale");}
  if (t <= stamp_highwater_ || t <= recovery_after_) {return reject("replayed_or_pre_fault");}
  if (!finite(m)) {return reject("nonfinite_pose_or_twist");}
  // A bad/future sample must never advance the accepted timestamp watermark.
  if (have_anchor_ && (m.header.frame_id != anchor_.header.frame_id ||
    m.child_frame_id != anchor_.child_frame_id)) {return reject("frame_changed");}
  if (!healthy_ && have_anchor_ &&
    (translation(m, anchor_) > config_.recovery_translation ||
    rotation(m, anchor_) > config_.recovery_rotation)) {return reject("recovery_anchor_mismatch");}
  if (!healthy_ && have_anchor_ && config_.trust_twist &&
    ((translation(m, anchor_) < 1e-4 && norm(m.twist.twist.linear) > 0.1) ||
    (rotation(m, anchor_) < 1e-4 && norm(m.twist.twist.angular) > 0.3)))
  {return reject("frozen_recovery");}
  if (have_previous_) {
    const double dt = (t - stamp(previous_)) * 1e-9;
    const double distance = translation(m, previous_), angle = rotation(m, previous_);
    if (distance > config_.translation_slack + config_.max_speed * dt ||
      angle > config_.rotation_slack + config_.max_angular_speed * dt)
    {return reject("pose_jump");}
    if (config_.trust_twist) {
      const double v = norm(m.twist.twist.linear), w = norm(m.twist.twist.angular);
      if ((distance < 1e-4 && v > 0.1) || (angle < 1e-4 && w > 0.3)) {
        if (frozen_since_ == 0) {frozen_since_ = stamp(previous_);}
        if ((t - frozen_since_) * 1e-9 >= config_.frozen_duration)
        {return reject("frozen_pose_with_motion_evidence");}
      } else {frozen_since_ = 0;}
      const auto & cq = m.pose.pose.orientation;
      const auto & pq = previous_.pose.pose.orientation;
      tf2::Quaternion current(cq.x, cq.y, cq.z, cq.w), prior(pq.x, pq.y, pq.z, pq.w);
      current.normalize(); prior.normalize();
      const auto & cp = m.pose.pose.position;
      const auto & pp = previous_.pose.pose.position;
      const tf2::Vector3 dp(cp.x-pp.x, cp.y-pp.y, cp.z-pp.z);
      const auto body_velocity = tf2::quatRotate(current.inverse(), dp / dt);
      auto dq = prior.inverse() * current;
      if (dq.w() < 0) {dq = tf2::Quaternion(-dq.x(), -dq.y(), -dq.z(), -dq.w());}
      const auto angular_velocity = angle > 1e-9 ? dq.getAxis() * (angle / dt) : tf2::Vector3(0, 0, 0);
      const auto & linear = m.twist.twist.linear;
      const auto & angular = m.twist.twist.angular;
      if ((body_velocity - tf2::Vector3(linear.x, linear.y, linear.z)).length() > config_.speed_error ||
        (angular_velocity - tf2::Vector3(angular.x, angular.y, angular.z)).length() > config_.angular_error)
      {return reject("pose_twist_inconsistent");}
    }
  }
  previous_ = m;
  have_previous_ = true;
  stamp_highwater_ = t;
  last_candidate_steady_ = steady;
  if (!healthy_ && ++recovery_count_ < config_.recovery_samples) {
    reason_ = "recovering";
    return false;
  }
  healthy_ = true;
  anchor_ = m;
  have_anchor_ = true;
  accepted_stamp_ = t;
  reason_ = frozen_since_ > 0 ? "suspect_frozen_pose" : "healthy";
  return true;
}
}  // namespace rm_localization_adapters
