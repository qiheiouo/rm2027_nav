#include <gtest/gtest.h>
#include <thread>
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_ros/transform_listener.h"
#include "../src/rebase_obstacle_layer.cpp"

class InspectLayer : public rm_nav_config::RebaseObstacleLayer
{
public:
  void addBuffer(const std::shared_ptr<nav2_costmap_2d::ObservationBuffer> & buffer)
  {observation_buffers_.push_back(buffer);marking_buffers_.push_back(buffer);}
};

TEST(RebaseObstacleLayer, RetiresOldObservationsAndPreservesStaticMasterCosts)
{
  rclcpp::init(0, nullptr);
  auto parent=std::make_shared<nav2_util::LifecycleNode>("rebase_layer_test", "", rclcpp::NodeOptions{});
  parent->declare_parameter("track_unknown_space", false);
  parent->declare_parameter("transform_tolerance", 0.1);
  parent->declare_parameter("obstacle.observation_sources", std::string(""));
  parent->declare_parameter("obstacle.footprint_clearing_enabled", false);
  auto probe=std::make_shared<rclcpp::Node>("rebase_layer_probe");
  auto publisher=probe->create_publisher<rm_competition_interfaces::msg::LocalizationStatus>("/localization/status",10);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(parent->get_node_base_interface());executor.add_node(probe);
  const auto spin=[&]() {for (int i=0;i<10;++i) {executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(10));}};
  tf2_ros::Buffer tf(parent->get_clock());
  tf2_ros::TransformListener listener(tf);
  nav2_costmap_2d::LayeredCostmap layered("map",false,false);
  layered.resizeMap(100,100,.1,-5,-5);
  InspectLayer layer;
  layer.initialize(&layered,"obstacle",&tf,parent,parent->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive));
  auto buffer=std::make_shared<nav2_costmap_2d::ObservationBuffer>(parent,"fixture",0.,0.,0.,2.,8.,0.,10.,0.,tf,"map","map",tf2::durationFromSec(.1));
  layer.addBuffer(buffer);
  nav2_costmap_2d::Costmap2D master(100,100,.1,-5,-5,0);
  master.setCost(90,90,nav2_costmap_2d::LETHAL_OBSTACLE);  // independent static layer result
  const auto cloud=[&](double x,const builtin_interfaces::msg::Time & stamp) {
      sensor_msgs::msg::PointCloud2 message;
      message.header.frame_id="map";message.header.stamp=stamp;
      sensor_msgs::PointCloud2Modifier modifier(message);modifier.setPointCloud2FieldsByString(1,"xyz");modifier.resize(1);
      sensor_msgs::PointCloud2Iterator<float> px(message,"x"),py(message,"y"),pz(message,"z");
      *px=x;*py=0;*pz=.3;buffer->bufferCloud(message);
    };
  const auto update=[&]() {
      double x=1e9,y=1e9,mx=-1e9,my=-1e9;
      layer.updateBounds(0,0,0,&x,&y,&mx,&my);
      // The layered master is reconstructed from preserved static and current dynamic layers.
      master.resetMap(0,0,100,100);master.setCost(90,90,nav2_costmap_2d::LETHAL_OBSTACLE);
      layer.updateCosts(master,0,0,100,100);
    };
  spin();
  builtin_interfaces::msg::Time old=probe->now();cloud(1.,old);update();
  EXPECT_EQ(master.getCost(60,50),nav2_costmap_2d::LETHAL_OBSTACLE);
  rm_competition_interfaces::msg::LocalizationStatus status;
  status.producer_id="test-bridge";status.started_at=probe->now();status.sequence=1;
  status.header.stamp=probe->now();status.committed_at=probe->now();
  status.transaction_phase=status.PRIME_NEW_SESSION;publisher->publish(status);spin();update();
  EXPECT_NE(master.getCost(60,50),nav2_costmap_2d::LETHAL_OBSTACLE);
  cloud(2.,probe->now());update();
  EXPECT_EQ(master.getCost(70,50),nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(master.getCost(90,90),nav2_costmap_2d::LETHAL_OBSTACLE);
  cloud(3.,old);update();  // old queued observation must not repopulate the new map
  EXPECT_NE(master.getCost(80,50),nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_FALSE(layer.isCurrent());
  status.started_at=probe->now();status.committed_at=probe->now();status.header.stamp=probe->now();status.sequence=1;
  publisher->publish(status);spin();cloud(-1.,probe->now());update();
  EXPECT_NE(master.getCost(70,50),nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(master.getCost(40,50),nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(master.getCost(90,90),nav2_costmap_2d::LETHAL_OBSTACLE);
  rclcpp::shutdown();
}
