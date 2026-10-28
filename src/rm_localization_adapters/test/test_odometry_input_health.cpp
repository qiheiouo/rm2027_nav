#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include "rm_localization_adapters/odometry_input_health.hpp"

using rm_localization_adapters::OdometryInputHealth;
using rm_localization_adapters::InputHealthConfig;
namespace
{
nav_msgs::msg::Odometry sample(int64_t t, double x = 0, double yaw = 0)
{
  nav_msgs::msg::Odometry m;
  m.header.stamp.sec = t / 1000000000LL;
  m.header.stamp.nanosec = t % 1000000000LL;
  m.header.frame_id = "odom"; m.child_frame_id = "base_link";
  m.pose.pose.position.x = x;
  m.pose.pose.orientation.z = std::sin(yaw / 2);
  m.pose.pose.orientation.w = std::cos(yaw / 2);
  return m;
}
struct HealthTest : testing::Test
{
  OdometryInputHealth gate{InputHealthConfig{}};
  int64_t t{1000000000LL};
  bool next(double x = 0, double yaw = 0) {
    t += 50000000;
    return gate.evaluate(sample(t, x, yaw), t, t);
  }
  void warm() {for (int i = 0; i < 4; ++i) {EXPECT_FALSE(next());} EXPECT_TRUE(next());}
};
TEST_F(HealthTest, NormalAndDebouncedStartup) {
  warm();
  for (int i = 0; i < 100; ++i) {EXPECT_TRUE(next(i * .02));}
  EXPECT_EQ(gate.generation(), 1U);
}
TEST_F(HealthTest, SilenceAndRecoveryCreateGeneration) {
  warm(); t += 400000000; gate.tick(t, t);
  EXPECT_FALSE(gate.healthy()); EXPECT_EQ(gate.generation(), 2U);
  for (int i = 0; i < 4; ++i) {EXPECT_FALSE(next());}
  EXPECT_TRUE(next()); EXPECT_EQ(gate.generation(), 2U);
}
TEST_F(HealthTest, FrozenRosClockStillExpiresBySteadyTime) {
  warm(); gate.tick(t, t + 400000000);
  EXPECT_FALSE(gate.healthy());
}
TEST_F(HealthTest, ReorderedDuplicateAndOldCallbackCannotReauthorize) {
  warm(); const auto old = sample(t);
  t += 50000000; EXPECT_FALSE(gate.evaluate(old, t, t));
  for (int i = 0; i < 4; ++i) {EXPECT_FALSE(next());}
  EXPECT_TRUE(next()); const auto gen = gate.generation();
  EXPECT_FALSE(gate.evaluate(old, t, t));
  EXPECT_FALSE(gate.healthy()); EXPECT_EQ(gate.generation(), gen + 1);
}
TEST_F(HealthTest, FutureDoesNotPoisonRecoveryWatermark) {
  warm(); t += 50000000;
  EXPECT_FALSE(gate.evaluate(sample(t + 5000000000LL), t, t));
  for (int i = 0; i < 4; ++i) {EXPECT_FALSE(next());}
  EXPECT_TRUE(next());
}
TEST_F(HealthTest, NanInfAndMalformedStampFailClosed) {
  warm(); t += 50000000; auto m = sample(t);
  m.twist.twist.angular.z = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(gate.evaluate(m, t, t));
  t += 50000000; m = sample(t); m.pose.pose.position.x = std::nan("");
  EXPECT_FALSE(gate.evaluate(m, t, t));
  t += 50000000; m = sample(t); m.header.stamp.nanosec = 1000000000U;
  EXPECT_FALSE(gate.evaluate(m, t, t));
}
TEST_F(HealthTest, JumpNeverMovesTrustedAnchor) {
  warm(); EXPECT_FALSE(next(10));
  for (int i = 0; i < 10; ++i) {EXPECT_FALSE(next(10));}
  for (int i = 0; i < 4; ++i) {EXPECT_FALSE(next());}
  EXPECT_TRUE(next());
}
TEST_F(HealthTest, HighSpinIsAllowedButAngularJumpIsNot) {
  warm();
  for (int i = 1; i < 30; ++i) {EXPECT_TRUE(next(0, i * .35));}
  EXPECT_FALSE(next(0, 29 * .35 + 2));
}
TEST_F(HealthTest, ClockRollbackHoldsUntilBeyondPreviousTime) {
  warm(); const auto high = t; gate.tick(t - 200000000, t);
  EXPECT_FALSE(gate.healthy());
  EXPECT_FALSE(gate.evaluate(sample(high - 100000000), high - 100000000, t));
  for (int i = 0; i < 4; ++i) {EXPECT_FALSE(next());}
  EXPECT_TRUE(next());
}
TEST_F(HealthTest, FrameContinuityAndStaleCannotPass) {
  warm(); t += 50000000; auto m = sample(t); m.child_frame_id = "sensor";
  EXPECT_FALSE(gate.evaluate(m, t, t));
  EXPECT_FALSE(gate.evaluate(sample(t - 500000000), t, t));
}
TEST(HealthEvidence, TrustedTwistDetectsFreezeAndCannotRecoverWhileFrozen) {
  InputHealthConfig c; c.trust_twist = true; OdometryInputHealth g(c);
  int64_t t = 1000000000;
  for (int i = 0; i < 20; ++i) {
    t += 50000000; auto m = sample(t, i * .025); m.twist.twist.linear.x = .5;
    EXPECT_EQ(g.evaluate(m, t, t), i >= 4);
  }
  bool rejected = false;
  for (int i = 0; i < 30; ++i) {
    t += 50000000; auto m = sample(t, .475); m.twist.twist.linear.x = .5;
    const bool valid = g.evaluate(m, t, t);
    if (rejected) {EXPECT_FALSE(valid);}
    rejected |= !valid;
  }
  EXPECT_TRUE(rejected);
  for (int i = 0; i < 5; ++i) {
    t += 50000000; EXPECT_EQ(g.evaluate(sample(t, .475), t, t), i == 4);
  }
}
TEST(HealthEvidence, TrustedHighSpinConsistencyAndBadAngularOutput) {
  InputHealthConfig c; c.trust_twist = true; OdometryInputHealth g(c);
  int64_t t = 1000000000;
  for (int i = 0; i < 50; ++i) {
    t += 50000000; auto m = sample(t, 0, i * .35); m.twist.twist.angular.z = 7;
    EXPECT_EQ(g.evaluate(m, t, t), i >= 4);
  }
  t += 50000000; auto m = sample(t, 0, 50 * .35); m.twist.twist.angular.z = -7;
  EXPECT_FALSE(g.evaluate(m, t, t));
}
TEST(HealthEvidence, UnknownCovarianceIsNotAQualitySignal) {
  OdometryInputHealth g(InputHealthConfig{});
  for (int i = 0; i < 6; ++i) {
    const int64_t t = 1000000000LL + i * 50000000;
    auto m = sample(t); m.pose.covariance.fill(1e8);
    EXPECT_EQ(g.evaluate(m, t, t), i >= 4);
  }
}
TEST(HealthEvidence, InvalidParametersRejected) {
  InputHealthConfig c; c.recovery_samples = 1;
  EXPECT_THROW(OdometryInputHealth g(c), std::invalid_argument);
  c.recovery_samples = 5; c.max_speed = std::nan("");
  EXPECT_THROW(OdometryInputHealth g(c), std::invalid_argument);
}
}  // namespace
