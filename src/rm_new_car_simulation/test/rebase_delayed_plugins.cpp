// Test-only timing decorators around the installed Navfn/MPPI implementations.
#include <chrono>
#include <thread>
#include "nav2_core/global_planner.hpp"
#include "nav2_core/controller.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
namespace rm_new_car_simulation
{
class DelayedPlanner : public nav2_core::GlobalPlanner
{
  pluginlib::ClassLoader<nav2_core::GlobalPlanner> loader_{"nav2_core", "nav2_core::GlobalPlanner"};
  nav2_core::GlobalPlanner::Ptr delegate_;
  bool first_{true};
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map) override
  {
    delegate_ = loader_.createSharedInstance("nav2_navfn_planner::NavfnPlanner");
    delegate_->configure(parent, name, tf, map);
  }
  void cleanup() override {delegate_->cleanup();}
  void activate() override {delegate_->activate();}
  void deactivate() override {delegate_->deactivate();}
  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal, std::function<bool()> cancel) override
  {
    auto path = delegate_->createPlan(start, goal, cancel);
    if (first_) {first_ = false; std::this_thread::sleep_for(std::chrono::seconds(2));}
    return path;  // deliberately deliver the old computed path after cancel
  }
};
class DelayedController : public nav2_core::Controller
{
  pluginlib::ClassLoader<nav2_core::Controller> loader_{"nav2_core", "nav2_core::Controller"};
  nav2_core::Controller::Ptr delegate_;
  std::chrono::steady_clock::time_point cancel_at_{};
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map) override
  {
    delegate_ = loader_.createSharedInstance("nav2_mppi_controller::MPPIController");
    delegate_->configure(parent, name, tf, map);
  }
  void cleanup() override {delegate_->cleanup();}
  void activate() override {delegate_->activate();}
  void deactivate() override {delegate_->deactivate();}
  void setPlan(const nav_msgs::msg::Path & path) override {delegate_->setPlan(path);}
  void setSpeedLimit(const double & limit, const bool & percentage) override {delegate_->setSpeedLimit(limit, percentage);}
  void reset() override {cancel_at_ = {}; delegate_->reset();}
  geometry_msgs::msg::TwistStamped computeVelocityCommands(const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity, nav2_core::GoalChecker * checker) override
  {return delegate_->computeVelocityCommands(pose, velocity, checker);}
  bool cancel() override
  {
    if (cancel_at_ == std::chrono::steady_clock::time_point{}) {cancel_at_ = std::chrono::steady_clock::now();}
    if (std::chrono::steady_clock::now() - cancel_at_ < std::chrono::seconds(1)) {return false;}
    return delegate_->cancel();
  }
};
}
PLUGINLIB_EXPORT_CLASS(rm_new_car_simulation::DelayedPlanner, nav2_core::GlobalPlanner)
PLUGINLIB_EXPORT_CLASS(rm_new_car_simulation::DelayedController, nav2_core::Controller)
