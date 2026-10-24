#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include <atomic>
#include <algorithm>
namespace fixture {
class Layer : public nav2_costmap_2d::Layer {
  std::atomic<bool> ready_{false};
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
public:
  void onInitialize() override {
    enabled_ = true;
    auto node = node_.lock();
    service_ = node->create_service<std_srvs::srv::SetBool>("/fixture/current",
      [this](std::shared_ptr<std_srvs::srv::SetBool::Request> req,
             std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
        ready_ = req->data;
        res->success = true;
      });
    current_ = false;
  }
  void updateBounds(double x, double y, double, double * min_x, double * min_y,
    double * max_x, double * max_y) override {
    *min_x = std::min(*min_x, x - 1.0); *min_y = std::min(*min_y, y - 1.0);
    *max_x = std::max(*max_x, x + 1.0); *max_y = std::max(*max_y, y + 1.0);
  }
  void updateCosts(nav2_costmap_2d::Costmap2D &, int, int, int, int) override {current_ = ready_.load();}
  void reset() override {ready_ = false; current_ = false;}
  bool isClearable() override {return true;}
};
class Controller : public nav2_core::Controller {
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr events_;
  void event(const char * text) {std_msgs::msg::String m; m.data = text; events_->publish(m);}
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string,
    std::shared_ptr<tf2_ros::Buffer>, std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override {
    events_ = parent.lock()->create_publisher<std_msgs::msg::String>("/fixture/events", 10);
  }
  void cleanup() override {events_.reset();}
  void activate() override {events_->on_activate();}
  void deactivate() override {events_->on_deactivate();}
  void setPlan(const nav_msgs::msg::Path &) override {event("set_plan");}
  geometry_msgs::msg::TwistStamped computeVelocityCommands(const geometry_msgs::msg::PoseStamped &,
    const geometry_msgs::msg::Twist &, nav2_core::GoalChecker *) override {
    event("compute");
    geometry_msgs::msg::TwistStamped m; m.twist.linear.x = 0.1; return m;
  }
  void setSpeedLimit(const double &, const bool &) override {}
};
}
PLUGINLIB_EXPORT_CLASS(fixture::Layer, nav2_costmap_2d::Layer)
PLUGINLIB_EXPORT_CLASS(fixture::Controller, nav2_core::Controller)
