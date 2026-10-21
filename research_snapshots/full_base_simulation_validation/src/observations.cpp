// Perfect *declared simulated observations*, not FAST-LIO or physical MCU.
// Independently reads sensor/base entities, never subscribes to truth topics.
// No joint angle, startup attitude, branch choice, or TF is exported.
#include <chrono>
#include <cmath>
#include <memory>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/odometry.pb.h>
#include <gz/msgs/Utility.hh>
namespace rm_shadow {
class Observations : public gz::sim::System, public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPostUpdate {
  gz::sim::Entity base{}, sensor{};
  gz::transport::Node node;
  gz::transport::Node::Publisher pub;
  bool initialized{false};
  gz::math::Pose3d sensor0;
  double lastHeading{};
  std::chrono::steady_clock::duration last{};
public:
  void Configure(const gz::sim::Entity & e, const std::shared_ptr<const sdf::Element> &,
    gz::sim::EntityComponentManager & ecm, gz::sim::EventManager &) override {
    gz::sim::Model m(e);base=m.LinkByName(ecm,"base_link");sensor=m.LinkByName(ecm,"sim_lidar_link");
    pub=node.Advertise<gz::msgs::Odometry>("/model/new_car/shadow_observations");
  }
  void PostUpdate(const gz::sim::UpdateInfo & info, const gz::sim::EntityComponentManager & ecm) override {
    if(info.paused || info.simTime<std::chrono::seconds(2) ||
      (initialized && info.simTime-last<std::chrono::milliseconds(20))) return;
    const auto s=gz::sim::worldPose(sensor,ecm);
    const auto forward=gz::sim::worldPose(base,ecm).Rot().RotateVector(gz::math::Vector3d(1,0,0));
    const double heading=std::atan2(forward.Y(),forward.X());
    double rate=0;
    if(initialized) { const double d=heading-lastHeading;
      rate=std::atan2(std::sin(d),std::cos(d))/std::chrono::duration<double>(info.simTime-last).count();
    } else {sensor0=s;initialized=true;}
    gz::msgs::Odometry packet;
    const auto ns=std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
    packet.mutable_header()->mutable_stamp()->set_sec(ns/1000000000);
    packet.mutable_header()->mutable_stamp()->set_nsec(ns%1000000000);
    auto h=packet.mutable_header()->add_data();h->set_key("frame_id");h->add_value("sensor_epoch_1");
    h=packet.mutable_header()->add_data();h->set_key("child_frame_id");h->add_value("sim_lidar_link");
    gz::msgs::Set(packet.mutable_pose(),sensor0.Inverse()*s);
    // Transport envelope only: angular.z=azimuth, angular.y=azimuth rate.
    packet.mutable_twist()->mutable_angular()->set_z(heading);
    packet.mutable_twist()->mutable_angular()->set_y(rate);
    pub.Publish(packet);last=info.simTime;lastHeading=heading;
  }
}; }
GZ_ADD_PLUGIN(rm_shadow::Observations,gz::sim::System,rm_shadow::Observations::ISystemConfigure,
 rm_shadow::Observations::ISystemPostUpdate)
