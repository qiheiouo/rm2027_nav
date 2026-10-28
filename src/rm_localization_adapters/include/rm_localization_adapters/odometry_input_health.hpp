#pragma once

#include <cstdint>
#include <string>
#include "nav_msgs/msg/odometry.hpp"

namespace rm_localization_adapters
{
// Values are integration fixtures, not new-car calibrated limits.
struct InputHealthConfig
{
  double max_age{0.3}, max_future{0.05}, max_silence{0.3};
  unsigned recovery_samples{5};
  double max_speed{4.0}, translation_slack{0.15};
  double max_angular_speed{12.0}, rotation_slack{0.15};
  double recovery_translation{0.8}, recovery_rotation{0.8};
  // Only legal when raw twist has independently established base-frame semantics.
  bool trust_twist{false};
  double speed_error{2.0}, angular_error{3.0}, frozen_duration{0.4};
};

class OdometryInputHealth
{
public:
  explicit OdometryInputHealth(InputHealthConfig config);
  bool evaluate(const nav_msgs::msg::Odometry & sample, int64_t ros_now, int64_t steady_now);
  void fault(const std::string & reason, int64_t ros_now);
  void tick(int64_t ros_now, int64_t steady_now);
  bool healthy() const {return healthy_;}
  const std::string & reason() const {return reason_;}
  unsigned recovery_count() const {return recovery_count_;}
  uint64_t generation() const {return generation_;}
  int64_t accepted_stamp() const {return accepted_stamp_;}
  const InputHealthConfig & config() const {return config_;}

private:
  InputHealthConfig config_;
  bool healthy_{false}, have_anchor_{false}, have_previous_{false};
  unsigned recovery_count_{0};
  uint64_t generation_{1};
  int64_t clock_highwater_{0}, stamp_highwater_{0}, recovery_after_{0};
  int64_t accepted_stamp_{0}, last_candidate_steady_{0}, frozen_since_{0};
  nav_msgs::msg::Odometry anchor_, previous_;
  std::string reason_{"initializing"};
};
}  // namespace rm_localization_adapters
