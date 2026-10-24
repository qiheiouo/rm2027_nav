#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

#include "gtest/gtest.h"

#include "rm_relocalization_bridge/global_pose_gate.hpp"

namespace
{

geometry_msgs::msg::PoseWithCovarianceStamped validPose()
{
  geometry_msgs::msg::PoseWithCovarianceStamped message;
  message.header.frame_id = "map";
  message.header.stamp.sec = 10;
  message.pose.pose.orientation.w = 1.0;
  message.pose.covariance[0] = 0.1;
  message.pose.covariance[7] = 0.1;
  message.pose.covariance[35] = 0.05;
  return message;
}

constexpr std::int64_t kNow = 10100000000LL;

}  // namespace

TEST(GlobalPoseGate, AcceptsFiniteRecentPlanarPose)
{
  std::string reason;
  EXPECT_TRUE(rm_relocalization_bridge::validateGlobalPose(
      validPose(), kNow, {}, &reason));
  EXPECT_TRUE(reason.empty());
}

TEST(GlobalPoseGate, RejectsWrongFrame)
{
  auto message = validPose();
  message.header.frame_id = "odom";
  EXPECT_FALSE(rm_relocalization_bridge::validateGlobalPose(message, kNow, {}));
}

TEST(GlobalPoseGate, RejectsStaleOrFutureTimestamp)
{
  auto stale = validPose();
  stale.header.stamp.sec = 8;
  EXPECT_FALSE(rm_relocalization_bridge::validateGlobalPose(stale, kNow, {}));

  auto future = validPose();
  future.header.stamp.sec = 11;
  EXPECT_FALSE(rm_relocalization_bridge::validateGlobalPose(future, kNow, {}));
}

TEST(GlobalPoseGate, RejectsZeroTimestamp)
{
  auto message = validPose();
  message.header.stamp.sec = 0;
  EXPECT_FALSE(rm_relocalization_bridge::validateGlobalPose(message, kNow, {}));
}

TEST(GlobalPoseGate, RejectsNonFinitePose)
{
  auto message = validPose();
  message.pose.pose.position.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(rm_relocalization_bridge::validateGlobalPose(message, kNow, {}));
}

TEST(GlobalPoseGate, RejectsExcessiveCovariance)
{
  auto message = validPose();
  message.pose.covariance[7] = 2.0;
  EXPECT_FALSE(rm_relocalization_bridge::validateGlobalPose(message, kNow, {}));
}

TEST(GlobalPoseGate, RejectsNonPlanarOrientation)
{
  auto message = validPose();
  const double half_roll = 0.2;
  message.pose.pose.orientation.x = std::sin(half_roll);
  message.pose.pose.orientation.w = std::cos(half_roll);
  EXPECT_FALSE(rm_relocalization_bridge::validateGlobalPose(message, kNow, {}));
}

TEST(GlobalPoseGate, RejectsAsymmetricFullCovariance)
{
  auto message = validPose();
  message.pose.covariance[1] = 0.01;
  EXPECT_FALSE(rm_relocalization_bridge::validateGlobalPose(message, kNow, {}));
}

TEST(GlobalPoseGate, RejectsIndefinitePositiveDiagonalCovariance)
{
  auto message = validPose();
  message.pose.covariance[1] = message.pose.covariance[6] = 0.2;
  EXPECT_FALSE(rm_relocalization_bridge::validateGlobalPose(message, kNow, {}));
  // Every 2x2 principal block passes, but the 3x3 x/y/yaw block is indefinite.
  message = validPose();
  message.pose.covariance[35] = 0.1;
  message.pose.covariance[1] = message.pose.covariance[6] = 0.08;
  message.pose.covariance[5] = message.pose.covariance[30] = 0.08;
  message.pose.covariance[11] = message.pose.covariance[31] = -0.08;
  EXPECT_FALSE(rm_relocalization_bridge::validateGlobalPose(message, kNow, {}));
}

TEST(GlobalPoseGate, AcceptsCorrelatedAndRankDeficientCovariance)
{
  auto message = validPose();
  message.pose.covariance[1] = message.pose.covariance[6] = 0.08;
  EXPECT_TRUE(rm_relocalization_bridge::validateGlobalPose(message, kNow, {}));
  message.pose.covariance[1] = message.pose.covariance[6] = 0.1;
  EXPECT_TRUE(rm_relocalization_bridge::validateGlobalPose(message, kNow, {}));
}

TEST(GlobalPoseGate, ChecksUnobservedDimensionsToo)
{
  auto message = validPose();
  message.pose.covariance[2] = message.pose.covariance[12] = 0.01;
  EXPECT_FALSE(rm_relocalization_bridge::validateGlobalPose(message, kNow, {}));
  message = validPose();
  message.pose.covariance[14] = -0.1;
  EXPECT_FALSE(rm_relocalization_bridge::validateGlobalPose(message, kNow, {}));
}
