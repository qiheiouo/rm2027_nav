#pragma once

#include <cstddef>
#include "tf2/LinearMath/Transform.h"

namespace rm_relocalization_bridge
{
struct CorrectionLimits
{
  // Integration fixture values; vehicle/sensor calibration is still required.
  double translation_m{0.4};
  double yaw_rad{0.3};
  std::size_t rejection_count{3};
  std::size_t transaction_samples{3};
};
enum class CorrectionDecision {Accept, Reject, Latched, Verifying};
struct CorrectionResult
{
  CorrectionDecision decision;
  double translation_m{0};
  double yaw_rad{0};
};
// Only eligible, fresh, source-bound, LIO-matched candidates enter this class.
// No clocks, ROS IO, source selection, TF publication, reseed or motion commands.
class CorrectionGate
{
public:
  explicit CorrectionGate(CorrectionLimits limits);
  CorrectionResult evaluate(const tf2::Transform & map_base, const tf2::Transform & odom_base);
  void prepare();
  void break_consensus();
  bool latched() const {return latched_;}
  bool preparing() const {return preparing_;}
  const tf2::Transform & trusted() const {return trusted_;}
private:
  CorrectionLimits limits_;
  tf2::Transform trusted_{tf2::Transform::getIdentity()};
  tf2::Transform provisional_{tf2::Transform::getIdentity()};
  bool latched_{false}, preparing_{true};
  std::size_t rejected_{0}, confirmed_{0};
};
}  // namespace rm_relocalization_bridge
