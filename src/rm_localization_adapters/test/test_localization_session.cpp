#include <gtest/gtest.h>
#include "rm_competition_interfaces/localization_session.hpp"
#include "rm_localization_adapters/canonical_odometry.hpp"
using rm_competition_interfaces::LocalizationSession;

TEST(LocalizationSession, ResetFencesOldEpochAndCallbackBeforeAcceptingNewEvidence)
{
  LocalizationSession s(100);
  ASSERT_TRUE(s.observe("A",0,1,1,1,110,true,true,110));
  const auto work=s.ticket(), old_epoch=s.boundary();
  ASSERT_TRUE(s.observe("A",1,2,2,2,0,false,false,120));
  EXPECT_GT(s.boundary(),old_epoch);
  EXPECT_FALSE(s.current(work));
  EXPECT_FALSE(s.observe("A",0,99,3,3,130,true,true,130));
  EXPECT_FALSE(s.observe("A",1,3,2,2,110,true,true,131));
  ASSERT_TRUE(s.observe("A",1,4,2,2,132,true,true,132));
  EXPECT_FALSE(s.observe("A",1,3,2,2,133,true,true,133));
}

TEST(LocalizationSession, LossAndExpiryCannotResurrectHeldPose)
{
  LocalizationSession s(100);
  ASSERT_TRUE(s.observe("A",0,1,1,1,110,true,true,110));
  const auto work=s.ticket();
  ASSERT_TRUE(s.observe("A",0,2,2,1,0,false,false,111));
  EXPECT_FALSE(s.current(work));
  EXPECT_FALSE(s.observe("A",0,3,1,1,112,true,true,112)); // old generation
  EXPECT_FALSE(s.observe("A",0,4,3,2,110,true,true,112)); // recovery heartbeat, old pose
  ASSERT_TRUE(s.observe("A",0,5,3,2,113,true,true,113));
  const auto new_work=s.ticket();
  EXPECT_FALSE(s.observe("A",0,100,4,3,113,true,true,113));
  EXPECT_TRUE(s.current(new_work)); // an invalid late packet cannot cancel current work
  s.revoke(); // steady lease expiry while ROS time is paused
  EXPECT_FALSE(s.current(new_work));
  EXPECT_FALSE(s.observe("A",0,6,3,2,113,true,true,113));
  EXPECT_TRUE(s.observe("A",0,7,3,2,114,true,true,114));
}

TEST(LocalizationSession, ClockRollbackRequiresNewEpochAndHighWaterCrossing)
{
  LocalizationSession s(100);
  ASSERT_TRUE(s.observe("A",0,1,1,1,110,true,true,110));
  const auto work=s.ticket();
  EXPECT_FALSE(s.clock(90));
  EXPECT_FALSE(s.current(work));
  EXPECT_FALSE(s.observe("A",0,2,1,1,120,true,true,120)); // catching up alone is insufficient
  ASSERT_TRUE(s.observe("A",1,3,2,2,0,false,false,120));
  EXPECT_FALSE(s.observe("A",1,4,2,2,110,true,true,120));
  EXPECT_TRUE(s.observe("A",1,5,2,2,121,true,true,121));
}

TEST(LocalizationSession, ExplicitProducerSelectionRetiresOldSession)
{
  LocalizationSession s(100);
  ASSERT_TRUE(s.observe("A",4,10,10,10,110,true,true,110));
  const auto old_work=s.ticket();
  EXPECT_FALSE(s.observe("unknown",0,1,1,1,120,true,true,120));
  EXPECT_TRUE(s.current(old_work));
  ASSERT_TRUE(s.select("B",0,120));
  EXPECT_FALSE(s.current(old_work));
  EXPECT_FALSE(s.observe("A",999,999,999,999,122,true,true,122));
  EXPECT_TRUE(s.observe("B",0,1,1,1,123,true,true,123));
  EXPECT_FALSE(s.select("A",1000,124));
}

TEST(LocalizationSession, RapidPreparedResetsRemainIdempotentAndRejectStaleCommit)
{
  LocalizationSession s(100);
  ASSERT_TRUE(s.select("A",0,100));
  const auto start=s.boundary();
  for (uint64_t epoch=1;epoch<=1000;++epoch) {
    const auto ticket=s.ticket();
    ASSERT_TRUE(s.select("A",epoch,100+epoch));
    const auto prepared=s.boundary();
    ASSERT_TRUE(s.select("A",epoch,100+epoch));
    ASSERT_EQ(s.boundary(),prepared); // duplicate command does not create phantom sessions
    ASSERT_FALSE(s.current(ticket));
    ASSERT_FALSE(s.observe("A",epoch-1,2*epoch,epoch,epoch,101+epoch,true,true,101+epoch));
    ASSERT_TRUE(s.observe("A",epoch,2*epoch+1,epoch,epoch,0,false,false,101+epoch));
  }
  EXPECT_EQ(s.boundary(),start+1000);
  EXPECT_FALSE(s.select("A",999,1200));
  EXPECT_TRUE(s.observe("A",1000,3000,1000,1000,1201,true,true,1201));
}

TEST(LocalizationSession, DeterministicTenThousandEpochStressRejectsLateWork)
{
  LocalizationSession s(1);
  std::uint64_t sequence=0;
  for (uint64_t epoch=0;epoch<10000;++epoch) {
    const auto stamp=static_cast<int64_t>(10+3*epoch);
    ASSERT_TRUE(s.observe("A",epoch,++sequence,epoch+1,epoch+1,0,false,false,stamp));
    const auto old=s.ticket();
    ASSERT_TRUE(s.observe("A",epoch,++sequence,epoch+1,epoch+1,stamp+1,true,true,stamp+1));
    ASSERT_FALSE(s.current(old));
    const auto current=s.ticket();
    if (epoch>0) {
      ASSERT_FALSE(s.observe("A",epoch-1,sequence+100,epoch,epoch,stamp+2,true,true,stamp+2));
      ASSERT_TRUE(s.current(current));
    }
    s.revoke();
    ASSERT_FALSE(s.current(current));
    ASSERT_FALSE(s.observe("A",epoch,++sequence,epoch+2,epoch+2,stamp+1,true,true,stamp+2));
  }
}

TEST(LocalizationSession, LeverArmCovarianceRetainsCrossTermsWithSamePoseTransform)
{
  const std::array<double,6> source{1,2,3,4,5,6}, expected{4,-4,6,4,5,6};
  std::array<double,36> covariance{};
  for (int i=0;i<6;++i) {for(int j=0;j<6;++j) {covariance[6*i+j]=source[i]*source[j];}}
  const auto shifted=rm_localization_adapters::shift_pose_covariance(covariance,tf2::Vector3(1,2,3));
  for (int i=0;i<6;++i) {for(int j=0;j<6;++j) {
      EXPECT_NEAR(shifted[6*i+j],expected[i]*expected[j],1e-12);
    }}
}
