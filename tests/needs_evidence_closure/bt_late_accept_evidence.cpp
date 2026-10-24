// Adapted from the historical AUD-056 real ROS client fixture.
// Passing characterizes a known dependency defect; it is not a fix gate.
#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/spin.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <atomic>
#include <cassert>
#include <chrono>
#include <fstream>
#include <iostream>
#include <thread>
using namespace std::chrono_literals;
using Action = nav2_msgs::action::Spin;
using Handle = rclcpp_action::ServerGoalHandle<Action>;
using Clock = std::chrono::steady_clock;
class Client : public nav2_behavior_tree::BtActionNode<Action> {
public:
  Client(const std::string &name, const BT::NodeConfiguration &c)
      : BtActionNode("audit_spin", name, c) {}
  void on_tick() override {
    goal_.target_yaw = 1.;
    goal_.time_allowance.sec = 5;
  }
  bool ready() const { return bool(goal_handle_); }
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto &out = std::cout;
  out << "{\"cases\":[";
  for (int delay : {0, 80, 240}) {
    auto server_node =
        std::make_shared<rclcpp::Node>("audit_delayed_spin_server");
    auto client_node = std::make_shared<rclcpp::Node>("audit_bt_client");
    std::atomic<int> accepted{0}, cancel{0}, done{0};
    std::atomic<double> accepted_ms{0};
    std::thread worker;
    auto now_ns = []() {
      return std::chrono::duration_cast<std::chrono::nanoseconds>(
                 Clock::now().time_since_epoch())
          .count();
    };
    std::atomic<int64_t> start_ns{now_ns()};
    std::string name = "/audit/bt_accept/spin_" + std::to_string(delay);
    auto server = rclcpp_action::create_server<Action>(
        server_node, name,
        [delay](const rclcpp_action::GoalUUID &,
                std::shared_ptr<const Action::Goal>) {
          std::this_thread::sleep_for(std::chrono::milliseconds(delay));
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [&cancel](std::shared_ptr<Handle>) {
          cancel++;
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [&](std::shared_ptr<Handle> h) {
          accepted_ms = (now_ns() - start_ns.load()) / 1e6;
          accepted++;
          worker = std::thread([&, h] {
            auto end = Clock::now() + 250ms;
            while (Clock::now() < end && !h->is_canceling())
              std::this_thread::sleep_for(1ms);
            auto r = std::make_shared<Action::Result>();
            if (h->is_canceling())
              h->canceled(r);
            else
              h->succeed(r);
            done++;
          });
        });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(server_node);
    std::thread exec([&] { executor.spin(); });
    auto bb = BT::Blackboard::create();
    bb->set("node", client_node);
    bb->set("server_timeout", 20ms);
    bb->set("bt_loop_duration", 10ms);
    bb->set("wait_for_service_timeout", 2000ms);
    BT::NodeConfiguration config;
    config.blackboard = bb;
    Client client(name, config);
    std::this_thread::sleep_for(100ms);
    start_ns = now_ns();
    auto status = BT::NodeStatus::IDLE;
    auto deadline = Clock::now() + 1s;
    while (Clock::now() < deadline) {
      status = client.executeTick();
      if (status == BT::NodeStatus::FAILURE || client.ready())
        break;
      std::this_thread::sleep_for(2ms);
    }
    bool ready = client.ready();
    double halted_ms = (now_ns() - start_ns.load()) / 1e6;
    client.halt();
    auto end = Clock::now() + 1s;
    while (Clock::now() < end && done.load() == 0)
      std::this_thread::sleep_for(1ms);
    assert(done == 1 && accepted == 1);
    if (worker.joinable())
      worker.join();
    executor.cancel();
    exec.join();
    if (delay == 0)
      assert(ready && status == BT::NodeStatus::RUNNING && cancel == 1);
    else
      assert(!ready && status == BT::NodeStatus::FAILURE && cancel == 0 &&
             accepted_ms > halted_ms);
    if (delay)
      out << ',';
    out << "{\"delay_ms\":" << delay
        << ",\"server_timeout_ms\":20,\"ready_before_halt\":"
        << (ready ? "true" : "false") << ",\"bt_failure_before_halt\":"
        << (status == BT::NodeStatus::FAILURE ? "true" : "false")
        << ",\"halt_time_ms\":" << halted_ms
        << ",\"server_accepted_ms\":" << accepted_ms
        << ",\"accepted_goals\":" << accepted
        << ",\"cancel_requests\":" << cancel << ",\"server_completed\":" << done
        << '}';
  }
  out << "],\"scope\":\"Actual installed unchanged BtActionNode original "
         "tick/halt and real ROS client. Derived on_tick only fills legal Spin "
         "goal. Controlled ActionServer timing/execute is a fixture, not Nav2 "
         "Spin/controller or full BT/robot motion.\"}\n";
  rclcpp::shutdown();
}
