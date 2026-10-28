#include <gtest/gtest.h>
#include <limits>
#include "rm_relocalization_bridge/correction_gate.hpp"
using namespace rm_relocalization_bridge;
namespace {
tf2::Transform pose(double x=0, double y=0, double yaw=0) {
  tf2::Quaternion q; q.setRPY(0,0,yaw); return tf2::Transform(q,tf2::Vector3(x,y,0));
}
void establish(CorrectionGate & g, const tf2::Transform & c=pose()) {
  EXPECT_EQ(g.evaluate(c,pose()).decision, CorrectionDecision::Verifying);
  EXPECT_EQ(g.evaluate(c,pose()).decision, CorrectionDecision::Verifying);
  EXPECT_EQ(g.evaluate(c,pose()).decision, CorrectionDecision::Accept);
}
}
TEST(CorrectionGate, StartupLargeNeedsConsensus) {
  CorrectionGate g({}); establish(g,pose(30,-20,2)); EXPECT_FALSE(g.preparing());
}
TEST(CorrectionGate, IsolatedOutlierNeverChangesTrusted) {
  CorrectionGate g({}); establish(g);
  EXPECT_EQ(g.evaluate(pose(2),pose()).decision,CorrectionDecision::Reject);
  EXPECT_DOUBLE_EQ(g.trusted().getOrigin().x(),0);
  EXPECT_EQ(g.evaluate(pose(.1),pose()).decision,CorrectionDecision::Accept);
}
TEST(CorrectionGate, ConsecutiveOutliersLatchUntilExplicitPrepare) {
  CorrectionGate g({}); establish(g);
  for(int i=0;i<2;++i) EXPECT_EQ(g.evaluate(pose(2),pose()).decision,CorrectionDecision::Reject);
  EXPECT_EQ(g.evaluate(pose(2),pose()).decision,CorrectionDecision::Latched);
  EXPECT_EQ(g.evaluate(pose(),pose()).decision,CorrectionDecision::Latched);
  g.prepare(); establish(g,pose(20,3,2)); EXPECT_FALSE(g.latched());
}
TEST(CorrectionGate, AlternatingOutliersDoNotFlickerOrPollute) {
  CorrectionGate g({}); establish(g);
  for(int i=0;i<20;++i) {
    EXPECT_EQ(g.evaluate(pose(2),pose()).decision,CorrectionDecision::Reject);
    EXPECT_EQ(g.evaluate(pose(.01*i),pose()).decision,CorrectionDecision::Accept);
  }
  EXPECT_FALSE(g.latched());
}
TEST(CorrectionGate, AccumulatedDriftCanExceedSingleStepLimit) {
  CorrectionGate g({}); establish(g);
  for(int i=1;i<=30;++i) EXPECT_EQ(g.evaluate(pose(.1*i),pose()).decision,CorrectionDecision::Accept);
  EXPECT_NEAR(g.trusted().getOrigin().x(),3,1e-9);
}
TEST(CorrectionGate, YawWrapUsesShortestAngle) {
  CorrectionGate g({}); establish(g,pose(0,0,3.13));
  EXPECT_EQ(g.evaluate(pose(0,0,-3.13),pose()).decision,CorrectionDecision::Accept);
}
TEST(CorrectionGate, MovingSpinningRobotAndDistantOrigin) {
  CorrectionGate g({}); auto c=pose(20,-10,1.5); establish(g,c);
  for(int i=0;i<40;++i) {
    auto odom=pose(100+i*2,200+i*3,i*.7);
    EXPECT_EQ(g.evaluate(c*odom,odom).decision,CorrectionDecision::Accept);
  }
  // Rotation around the CURRENT robot, not the arbitrary map/odom origin.
  auto odom=pose(1000,2000,0); auto observed=c*odom;
  auto q=observed.getRotation()*pose(0,0,.1).getRotation(); observed.setRotation(q);
  EXPECT_EQ(g.evaluate(observed,odom).decision,CorrectionDecision::Accept);
}
TEST(CorrectionGate, YawOnlyOutlierAndInvalidNumbers) {
  CorrectionGate g({}); establish(g);
  EXPECT_EQ(g.evaluate(pose(0,0,1),pose()).decision,CorrectionDecision::Reject);
  for(double x : {std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity()})
    EXPECT_EQ(g.evaluate(pose(x),pose()).decision,CorrectionDecision::Reject);
  EXPECT_DOUBLE_EQ(g.trusted().getOrigin().x(),0);
}
TEST(CorrectionGate, BrokenTransactionConsensusMustRestart) {
  CorrectionGate g({});
  EXPECT_EQ(g.evaluate(pose(20),pose()).decision,CorrectionDecision::Verifying);
  g.break_consensus(); establish(g,pose(-20));
  g.prepare();
  EXPECT_EQ(g.evaluate(pose(20),pose()).decision,CorrectionDecision::Verifying);
  EXPECT_EQ(g.evaluate(pose(-20),pose()).decision,CorrectionDecision::Reject);
  establish(g,pose(-20));
}
TEST(CorrectionGate, ConfigurationMustBeFiniteAndConfirmTransaction) {
  CorrectionLimits limits; limits.transaction_samples=1;
  EXPECT_THROW(CorrectionGate g(limits),std::invalid_argument);
  limits={};limits.translation_m=std::numeric_limits<double>::infinity();
  EXPECT_THROW(CorrectionGate g(limits),std::invalid_argument);
}
