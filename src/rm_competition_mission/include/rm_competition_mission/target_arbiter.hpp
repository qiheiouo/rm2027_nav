#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rm_competition_interfaces/msg/target_intent.hpp"

namespace rm_competition_mission
{
// A latest-value store and selector, with no ROS publishers or action clients.
class TargetArbiter
{
public:
  using Intent = rm_competition_interfaces::msg::TargetIntent;
  using Clock = std::chrono::steady_clock;

  void revoke()
  {
    // Preserve sequence watermarks, but never carry a cached value across an
    // authority boundary (including a sample within the future-clock tolerance).
    for (auto & slot : slots_) {slot.intent.valid = false;}
  }

  bool update(const Intent & input, const rclcpp::Time & now,
    Clock::time_point received, const std::string & frame, double max_ttl)
  {
    if (input.source < Intent::SOURCE_LOCAL || input.source > Intent::SOURCE_OPERATOR ||
      input.sequence == 0 || !wellFormed(input.header.stamp) ||
      input.ttl.sec < 0 || input.ttl.nanosec >= 1000000000U ||
      (input.clear && input.valid)) {return false;}
    const auto stamp = rclcpp::Time(input.header.stamp);
    const double ttl = rclcpp::Duration(input.ttl).seconds();
    const double age = (now - stamp).seconds();
    if (stamp.nanoseconds() <= 0 || age < -0.05 || age >= ttl ||
      ttl <= 0.0 || ttl > max_ttl) {return false;}
    if (input.valid && (input.target_id.empty() || !validPose(input, frame))) {return false;}
    auto & slot = slots_[input.source];
    // Validate before changing watermarks. Equal sequence is an idempotent no-op,
    // including conflicting payloads; neither can refresh or revive a target.
    if (input.sequence <= slot.intent.sequence ||
      stamp < rclcpp::Time(slot.intent.header.stamp)) {return false;}
    slot.intent = input;
    slot.deadline = received + std::chrono::duration_cast<Clock::duration>(
      std::chrono::duration<double>(ttl - std::max(0.0, age)));
    return true;
  }

  std::optional<Intent> select(const rclcpp::Time & now, Clock::time_point steady,
    const rclcpp::Time & not_before, const std::optional<Intent> & radar = {},
    const std::optional<Intent> & local = {}) const
  {
    for (const auto source : {Intent::SOURCE_OPERATOR, Intent::SOURCE_RADAR, Intent::SOURCE_LOCAL}) {
      const auto & slot = slots_[source];
      const auto & i = slot.intent;
      if (i.valid && !i.clear && rclcpp::Time(i.header.stamp) > not_before &&
        now >= rclcpp::Time(i.header.stamp) &&
        now < rclcpp::Time(i.header.stamp) + rclcpp::Duration(i.ttl) && steady < slot.deadline)
      {
        return i;
      }
      // The track adapter and built-in Local decision are already freshness checked
      // by Mission; they never compete with another producer for the same slot.
      if (source == Intent::SOURCE_RADAR && radar) {return radar;}
      if (source == Intent::SOURCE_LOCAL && local) {return local;}
    }
    return std::nullopt;
  }

private:
  static bool wellFormed(const builtin_interfaces::msg::Time & t)
  {return t.sec >= 0 && t.nanosec < 1000000000U;}

  static bool validPose(const Intent & i, const std::string & frame)
  {
    const auto & p = i.pose;
    const double norm = p.orientation.z * p.orientation.z + p.orientation.w * p.orientation.w;
    return i.header.frame_id == frame && std::isfinite(p.position.x) &&
      std::isfinite(p.position.y) && std::isfinite(p.position.z) && std::abs(p.position.z) < 1e-6 &&
      std::isfinite(p.orientation.x) && std::abs(p.orientation.x) < 1e-6 &&
      std::isfinite(p.orientation.y) && std::abs(p.orientation.y) < 1e-6 &&
      std::isfinite(norm) && std::abs(norm - 1.0) < 0.01;
  }

  struct Slot {Intent intent; Clock::time_point deadline;};
  std::array<Slot, 4> slots_;
};
}  // namespace rm_competition_mission
