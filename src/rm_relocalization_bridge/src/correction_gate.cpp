#include "rm_relocalization_bridge/correction_gate.hpp"
#include <cmath>
#include <stdexcept>
#include "tf2/LinearMath/Matrix3x3.h"
#include "rm_relocalization_bridge/relocalization_math.hpp"

namespace rm_relocalization_bridge
{
namespace
{
CorrectionResult residual(const tf2::Transform & prediction, const tf2::Transform & observed)
{
  double r, p, before, after;
  tf2::Matrix3x3(prediction.getRotation()).getRPY(r, p, before);
  tf2::Matrix3x3(observed.getRotation()).getRPY(r, p, after);
  return {CorrectionDecision::Reject, (observed.getOrigin() - prediction.getOrigin()).length(),
    std::abs(std::atan2(std::sin(after-before), std::cos(after-before)))};
}
}  // namespace
CorrectionGate::CorrectionGate(CorrectionLimits limits) : limits_(limits)
{
  if (!std::isfinite(limits.translation_m) || limits.translation_m <= 0 ||
    !std::isfinite(limits.yaw_rad) || limits.yaw_rad <= 0 || limits.yaw_rad > M_PI ||
    limits.rejection_count == 0 || limits.transaction_samples < 2)
  {throw std::invalid_argument("invalid correction limits/confirmation counts");}
}
void CorrectionGate::prepare()
{
  latched_ = false;
  preparing_ = true;
  rejected_ = confirmed_ = 0;
}
void CorrectionGate::break_consensus() {confirmed_ = 0;}
CorrectionResult CorrectionGate::evaluate(
  const tf2::Transform & map_base, const tf2::Transform & odom_base)
{
  if (!isFiniteTransform(map_base) || !isFiniteTransform(odom_base)) {
    break_consensus();
    return {CorrectionDecision::Reject};
  }
  if (latched_) {return {CorrectionDecision::Latched};}
  const auto candidate = computeMapToOdom(map_base, odom_base);
  if (!isFiniteTransform(candidate)) {break_consensus(); return {CorrectionDecision::Reject};}
  if (preparing_) {
    if (confirmed_ == 0) {provisional_ = candidate;}
    const auto delta = residual(provisional_ * odom_base, map_base);
    if (!std::isfinite(delta.translation_m) || !std::isfinite(delta.yaw_rad) ||
      delta.translation_m > limits_.translation_m || delta.yaw_rad > limits_.yaw_rad) {
      confirmed_ = 0;
      return delta;
    }
    if (++confirmed_ < limits_.transaction_samples) {return {CorrectionDecision::Verifying};}
    preparing_ = false;
  } else {
    // Compare at the SAME robot observation time. A moving/spinning robot is
    // not a correction residual; this also avoids map-origin lever-arm errors.
    auto delta = residual(trusted_ * odom_base, map_base);
    if (!std::isfinite(delta.translation_m) || !std::isfinite(delta.yaw_rad) ||
      delta.translation_m > limits_.translation_m || delta.yaw_rad > limits_.yaw_rad) {
      if (++rejected_ >= limits_.rejection_count) {
        latched_ = true;
        delta.decision = CorrectionDecision::Latched;
      }
      return delta;
    }
  }
  rejected_ = 0;
  trusted_ = candidate;
  return {CorrectionDecision::Accept};
}
}  // namespace rm_relocalization_bridge
