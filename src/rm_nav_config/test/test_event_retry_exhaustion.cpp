// Exercise the shipped XML and Nav2 RecoveryNode, with controlled action leaves.
// RUNNING on attempt four is not a fourth FAILURE or a wall-clock deadline.
#include <gtest/gtest.h>
#include <behaviortree_cpp/bt_factory.h>
#include <nav2_behavior_tree/plugins/control/recovery_node.hpp>

namespace
{
struct Actions
{
  int plans{0}, follows{0}, waits{0}, halted{0};
  bool last_plan_fails{false};
  BT::NodeStatus last_follow{BT::NodeStatus::RUNNING};
};

class Follow : public BT::StatefulActionNode
{
public:
  Follow(const std::string & name, const BT::NodeConfig & config, Actions & actions)
  : BT::StatefulActionNode(name, config), actions_(actions) {}
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("path"), BT::InputPort<std::string>("controller_id"),
      BT::OutputPort<int>("error_code_id")};
  }
  BT::NodeStatus onStart() override
  {
    EXPECT_EQ(getInput<std::string>("path").value(), std::to_string(actions_.plans));
    ++actions_.follows;
    return actions_.follows < 4 ? BT::NodeStatus::FAILURE : actions_.last_follow;
  }
  BT::NodeStatus onRunning() override {return actions_.last_follow;}
  void onHalted() override {++actions_.halted;}
private:
  Actions & actions_;
};

class EventRetry : public ::testing::Test
{
protected:
  Actions actions;
  BT::BehaviorTreeFactory factory;
  BT::Tree tree;
  void SetUp() override
  {
    // Link the installed Nav2 implementation; do not reimplement its retry loop.
    factory.registerNodeType<nav2_behavior_tree::RecoveryNode>("RecoveryNode");
    factory.registerNodeType<Follow>("FollowPath", std::ref(actions));
    factory.registerSimpleAction("ComputePathToPose", [this](BT::TreeNode & node) {
      ++actions.plans;
      node.setOutput("path", std::to_string(actions.plans));
      return actions.last_plan_fails && actions.plans == 4 ?
             BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
    }, {BT::InputPort<std::string>("goal"), BT::OutputPort<std::string>("path"),
      BT::InputPort<std::string>("planner_id"), BT::OutputPort<int>("error_code_id")});
    factory.registerSimpleAction("Wait", [this](BT::TreeNode & node) {
      EXPECT_DOUBLE_EQ(node.getInput<double>("wait_duration").value(), 1.5);
      ++actions.waits;
      return BT::NodeStatus::SUCCESS;
    }, {BT::InputPort<double>("wait_duration")});
    tree = factory.createTreeFromFile(EVENT_RETRY_XML);
  }
  void expect_budget(int follows = 4)
  {
    EXPECT_EQ(actions.plans, 4);
    EXPECT_EQ(actions.follows, follows);
    EXPECT_EQ(actions.waits, 3);
  }
};

TEST_F(EventRetry, FourthFailureTerminatesWithoutAnotherPlanOrWait)
{
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  expect_budget();
  // No timer or unobserved planner event can consume an extra retry.
  for (int i = 0; i < 100; ++i) {
    ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  }
  expect_budget();
  actions.last_follow = BT::NodeStatus::FAILURE;
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
  expect_budget();
  // The action already returned terminal; a second cancel is unnecessary.
  EXPECT_EQ(actions.halted, 0);
  const auto * root = dynamic_cast<const BT::ControlNode *>(tree.rootNode());
  ASSERT_NE(root, nullptr);
  for (const auto & child : root->children()) {
    EXPECT_EQ(child->status(), BT::NodeStatus::IDLE);
  }
}

TEST_F(EventRetry, FourthPlannerFailureTerminatesWithoutFollowingStalePath)
{
  actions.last_plan_fails = true;
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
  expect_budget(3);
}

TEST_F(EventRetry, FourthSuccessStillSucceeds)
{
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  actions.last_follow = BT::NodeStatus::SUCCESS;
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  expect_budget();
}

TEST_F(EventRetry, OwnerCancellationHaltsPendingFourthFollow)
{
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  tree.haltTree();
  EXPECT_EQ(tree.rootNode()->status(), BT::NodeStatus::IDLE);
  EXPECT_EQ(actions.halted, 1);
  expect_budget();
}
}  // namespace
