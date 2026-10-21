// Research harness: calls installed Nav2 1.1.20 optimizer operations unchanged.
// The sole addition between native scoring and native aggregation is the R2
// time-score adapter. No ROS velocity publisher and no hardware interface.
#include <iostream>
#include <chrono>
#include <nlohmann/json.hpp>
#include <xtensor/xrandom.hpp>
#include "nav2_mppi_controller/optimizer.hpp"
#include "vendor/geometry.hpp"
using json = nlohmann::json;
namespace dyn = rm_dynamic_obstacle_critic;
using Clock = std::chrono::steady_clock;

// Port of frozen R2 model.hpp::penalty; constants are frozen in protocol.json.
double penalty(double d) {
  double near = std::max(0., (.4-d)/.4);
  double danger = std::max(0., (.02-d)/.02);
  return near*near + 10000.*danger*danger;
}
class Consumer : public mppi::Optimizer {
public:
  std::vector<dyn::Point> fp{{-.355,-.330},{-.355,.330},{.355,.330},{.355,-.330}};
  json metadata() const {return {{"model_dt",settings_.model_dt},{"time_steps",settings_.time_steps},{"batch_size",settings_.batch_size},{"shift",settings_.shift_control_sequence}};}
  json step(const json &in) {
    auto start = Clock::now();
    auto s=in.at("state");
    costmap_->resizeMap(160,160,.05,s[0].get<double>()-4,s[1].get<double>()-4);
    auto current=in.at("current");
    double wall=in.at("wall"), radius=in.at("radius");
    for(unsigned y=0;y<160;y++) for(unsigned x=0;x<160;x++) {
      double wx,wy;costmap_->mapToWorld(x,y,wx,wy);
      double d=wall-std::abs(wy);
      if(!current.is_null()) d=std::min(d,std::hypot(wx-current[0].get<double>(),wy-current[1].get<double>())-radius);
      unsigned char cost=d<=0?254:d<=.33?253:d<=.5?static_cast<unsigned char>(252*std::exp(-6*(d-.33))):0;
      costmap_->setCost(x,y,cost);
    }
    geometry_msgs::msg::PoseStamped pose;pose.header.frame_id="map";
    pose.pose.position.x=s[0];pose.pose.position.y=s[1];
    double yaw=s[2];pose.pose.orientation.z=sin(yaw/2);pose.pose.orientation.w=cos(yaw/2);
    geometry_msgs::msg::Twist speed;speed.linear.x=s[3];speed.linear.y=s[4];speed.angular.z=s[5];
    nav_msgs::msg::Path path;path.header=pose.header;
    // Deterministic straight global reference, trimmed at current progress.
    double begin=std::clamp(s[0].get<double>(),0.,3.99);
    for(double x=begin;x<4;x+=.05) {auto p=pose;p.pose.position.x=x;p.pose.position.y=0;p.pose.orientation.z=0;p.pose.orientation.w=1;path.poses.push_back(p);}
    auto goal=pose;goal.pose.position.x=4;goal.pose.position.y=0;goal.pose.orientation.z=0;goal.pose.orientation.w=1;path.poses.push_back(goal);
    const auto &pred=in.at("prediction");
    if(!pred.is_null() && pred.size()!=30) throw std::runtime_error("prediction grid must be 30x100ms");
    prepare(pose,speed,path,nullptr);
    size_t retries=0, safe=0;double lo=0,hi=0,best_clear=0;
    do {
      generateNoisedTrajectories();
      critic_manager_.evalTrajectoriesScores(critics_data_);
      lo=INFINITY;hi=0;safe=0;best_clear=-INFINITY;
      for(size_t i=0;i<settings_.batch_size;i++) {
        double extra=0,min_d=INFINITY;
        if(!pred.is_null()) for(size_t k=0;k<30;k++) {
          double d=dyn::circle_clearance(dyn::transform(fp,generated_trajectories_.x(i,k),generated_trajectories_.y(i,k),generated_trajectories_.yaws(i,k)),{pred[k][0],pred[k][1]},radius);
          extra+=penalty(d)*.1;min_d=std::min(min_d,d);
        }
        costs_(i)+=extra;
        if(min_d>.02) safe++;
        best_clear=std::max(best_clear,min_d);lo=std::min(lo,extra);hi=std::max(hi,extra);
      }
      updateControlSequence();
      if(critics_data_.fail_flag) retries++;
    } while(fallback(critics_data_.fail_flag));
    mppi::utils::savitskyGolayFilter(control_sequence_,control_history_,settings_);
    auto command=getControlFromSequenceAsTwist(path.header.stamp);
    auto aggregate=getOptimizedTrajectory();
    double agg_clear=INFINITY;
    json rollout=json::array();
    for(size_t k=0;k<30;k++) {
      rollout.push_back({aggregate(k,0),aggregate(k,1),aggregate(k,2)});
      if(!pred.is_null()) agg_clear=std::min(agg_clear,dyn::circle_clearance(dyn::transform(fp,aggregate(k,0),aggregate(k,1),aggregate(k,2)),{pred[k][0],pred[k][1]},radius));
    }
    if(settings_.shift_control_sequence) shiftControlSequence();
    double ms=std::chrono::duration<double,std::milli>(Clock::now()-start).count();
    return {{"command",{command.twist.linear.x,command.twist.linear.y,command.twist.angular.z}},
      {"compute_ms",ms},{"retries",retries},{"dynamic_cost_min",lo},{"dynamic_cost_max",hi},
      {"safe_candidate_count",pred.is_null()?json(nullptr):json(safe)},
      {"best_candidate_clearance",pred.is_null()?json(nullptr):json(best_clear)},
      {"aggregate_predicted_clearance",pred.is_null()?json(nullptr):json(agg_clear)},
      {"rollout",rollout}};
  }
};
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  auto node=std::make_shared<rclcpp_lifecycle::LifecycleNode>("controller_server");
  auto map=std::make_shared<nav2_costmap_2d::Costmap2DROS>("pvt_costmap");
  map->set_parameter(rclcpp::Parameter("plugins",std::vector<std::string>{}));
  map->set_parameter(rclcpp::Parameter("global_frame",std::string("map")));
  map->set_parameter(rclcpp::Parameter("footprint_padding",0.0));
  map->set_parameter(rclcpp::Parameter("footprint",std::string("[[-0.355,-0.33],[-0.355,0.33],[0.355,0.33],[0.355,-0.33]]")));
  map->configure();
  std::vector<geometry_msgs::msg::Point> fp(4);
  fp[0].x=-.355;fp[0].y=-.33;fp[1].x=-.355;fp[1].y=.33;
  fp[2].x=.355;fp[2].y=.33;fp[3].x=.355;fp[3].y=-.33;map->setRobotFootprint(fp);
  auto handler=std::make_unique<mppi::ParametersHandler>(node);
  auto optimizer=std::make_unique<Consumer>();
  optimizer->initialize(node,"FollowPath",map,handler.get());
  std::string line;
  while(std::getline(std::cin,line)) {
    try {
      auto in=json::parse(line);
      if(in.contains("reset")) {
        xt::random::seed(in.at("reset").get<unsigned>());optimizer->reset();
        std::cout<<json({{"ready",true},{"metadata",optimizer->metadata()}}).dump()<<std::endl;
      } else std::cout<<optimizer->step(in).dump()<<std::endl;
    } catch(const std::exception &e) {std::cout<<json({{"error",e.what()}}).dump()<<std::endl;}
  }
  optimizer.reset();handler.reset();map->cleanup();map.reset();node.reset();rclcpp::shutdown();
}
