#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/spin.hpp"
#include <atomic>
#include "behaviortree_cpp_v3/bt_factory.h"
#include <cassert>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <thread>
using namespace std::chrono_literals;
using Action = nav2_msgs::action::Spin;
using Handle = rclcpp_action::ServerGoalHandle<Action>;
using Clock = std::chrono::steady_clock;
// All scheduling is event-controlled. Deadlines only fail a broken test.
template<class F> bool until(F predicate) {
  auto deadline = Clock::now() + 2s;
  while (Clock::now() < deadline) {if (predicate()) return true; std::this_thread::yield();}
  return predicate();
}
class Client : public nav2_behavior_tree::BtActionNode<Action> {
public:
  unsigned successes = 0;
  Client(const std::string & name, const BT::NodeConfiguration & config)
    : BtActionNode("regression_spin", name, config) {}
  void on_tick() override {goal_.target_yaw = 1.; goal_.time_allowance.sec = 5;}
  BT::NodeStatus on_success() override {++successes; return BT::NodeStatus::SUCCESS;}
  bool ready() const {return bool(goal_handle_);}
  BT::NodeStatus on_cancelled() override {
    assert(result_.result && result_.code == rclcpp_action::ResultCode::CANCELED);
    return BT::NodeStatus::SUCCESS;
  }
  void update() {goal_updated_ = true;}
};
struct Server {
  std::mutex mutex;
  std::condition_variable changed;
  unsigned received = 0, release = 0;
  bool reject = false, refuse_cancel = false;
  std::vector<std::shared_ptr<Handle>> handles;
  std::atomic<unsigned> cancels{0};
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("event_action_server");
  rclcpp_action::Server<Action>::SharedPtr server;
  rclcpp::executors::SingleThreadedExecutor executor;
  std::thread thread;
  explicit Server(const std::string & name) {
    server = rclcpp_action::create_server<Action>(node, name,
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal>) {
        std::unique_lock<std::mutex> lock(mutex);
        const auto request = ++received; changed.notify_all();
        changed.wait(lock, [&] {return release >= request;});
        return reject ? rclcpp_action::GoalResponse::REJECT : rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](std::shared_ptr<Handle>) {
        ++cancels; return refuse_cancel ? rclcpp_action::CancelResponse::REJECT : rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](std::shared_ptr<Handle> h) {std::lock_guard<std::mutex> lock(mutex); handles.push_back(h); changed.notify_all();});
    executor.add_node(node); thread = std::thread([this] {executor.spin();});
  }
  ~Server() {
    allow(100); executor.cancel(); if (thread.joinable()) thread.join();
  }
  void allow(unsigned count) {std::lock_guard<std::mutex> lock(mutex); release = count; changed.notify_all();}
  unsigned requests() {std::lock_guard<std::mutex> lock(mutex); return received;}
  std::shared_ptr<Handle> handle(unsigned index) {
    std::lock_guard<std::mutex> lock(mutex); return handles.size() > index ? handles[index] : nullptr;
  }
};
BT::NodeConfiguration config(const std::string & suffix) {
  BT::NodeConfiguration c; c.blackboard = BT::Blackboard::create();
  c.blackboard->set("node", std::make_shared<rclcpp::Node>("event_bt_client_" + suffix));
  c.blackboard->set("server_timeout", 20ms); c.blackboard->set("bt_loop_duration", 10ms);
  c.blackboard->set("wait_for_service_timeout", 2000ms); return c;
}
void healthy(Client & client, Server & server, unsigned next) {
  server.reject = false; server.refuse_cancel = false; server.allow(100);
  assert(until([&] {client.executeTick(); return server.requests() == next && client.ready();}));
  assert(until([&] {return bool(server.handle(next - 1));}));
  auto h = server.handle(next - 1); assert(h);
  h->succeed(std::make_shared<Action::Result>());
  assert(until([&] {return client.executeTick() == BT::NodeStatus::SUCCESS;}));
  assert(client.successes == 1); client.halt();
}
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  const std::string mode = argc > 1 ? argv[1] : "timeout";
  const std::string name = mode == "plugin" ? "/spin" : "/fixture/late_goal/" + mode;
  Server server(name);
  if (mode == "plugin") {
    BT::BehaviorTreeFactory factory;
    assert(std::getenv("NAV2_SPIN_PLUGIN"));
    factory.registerFromPlugin(std::getenv("NAV2_SPIN_PLUGIN"));
    auto c = config(mode);
    auto tree = factory.createTreeFromText(
      "<root main_tree_to_execute='MainTree'><BehaviorTree ID='MainTree'>"
      "<Spin spin_dist='1.0' time_allowance='5.0' is_recovery='false'/>"
      "</BehaviorTree></root>", c.blackboard);
    assert(tree.tickRoot() == BT::NodeStatus::RUNNING);
    assert(until([&] {return server.requests() == 1;}));
    assert(until([&] {return tree.tickRoot() == BT::NodeStatus::FAILURE;}));
    tree.haltTree(); server.allow(1);
    assert(until([&] {return server.cancels.load() == 1;}));
    auto h = server.handle(0); assert(h);
    assert(until([&] {return h->is_canceling();}));
    assert(tree.tickRoot() == BT::NodeStatus::FAILURE && server.requests() == 1);
    h->canceled(std::make_shared<Action::Result>());
    server.allow(100);
    assert(until([&] {tree.tickRoot(); return bool(server.handle(1));}));
    server.handle(1)->succeed(std::make_shared<Action::Result>());
    assert(until([&] {return tree.tickRoot() == BT::NodeStatus::SUCCESS;}));
    tree.haltTree(); std::cout << "PASS actual Spin plugin" << std::endl;
    rclcpp::shutdown(); return 0;
  }
  auto client = std::make_unique<Client>(name, config(mode));
  if (mode == "known" || mode == "update" || mode == "completed") {
    server.allow(1);
    assert(until([&] {client->executeTick(); return client->ready();}));
    assert(until([&] {return bool(server.handle(0));}));
    if (mode == "completed") {
      server.handle(0)->succeed(std::make_shared<Action::Result>());
      client->halt();
      assert(client->successes == 0);
      healthy(*client, server, 2);
    } else {
      unsigned count = 1;
      if (mode == "update") {
        client->update();
        client->executeTick();
        assert(until([&] {return server.requests() == 2;}));
        assert(until([&] {return client->executeTick() == BT::NodeStatus::FAILURE;}));
        count = 2;
      }
      client->halt(); server.allow(count);
      assert(until([&] {return server.cancels.load() == count;}));
      assert(client->executeTick() == BT::NodeStatus::FAILURE && server.requests() == count);
      for (unsigned i = 0; i < count; ++i) {
        auto h = server.handle(i); assert(h);
        assert(until([&] {return h->is_canceling();}));
        h->publish_feedback(std::make_shared<Action::Feedback>());
        h->canceled(std::make_shared<Action::Result>());
      }
      healthy(*client, server, count + 1);
    }
    std::cout << "PASS " << mode << std::endl;
    client.reset(); rclcpp::shutdown(); return 0;
  }
  assert(client->executeTick() == BT::NodeStatus::RUNNING);
  assert(until([&] {return server.requests() == 1;}));
  if (mode != "halt" && mode != "destroy") {
    assert(until([&] {return client->executeTick() == BT::NodeStatus::FAILURE;}));
  }
  if (mode == "destroy") client.reset();
  else client->halt();
  if (mode == "reject") server.reject = true;
  if (mode == "refuse") server.refuse_cancel = true;
  server.allow(1); // acceptance/rejection occurs only after timeout/halt/destruction
  if (mode == "reject") {
    assert(until([&] {return client->executeTick() == BT::NodeStatus::RUNNING && server.requests() == 2;}));
    server.reject = false; server.allow(100);
    assert(until([&] {client->executeTick(); return client->ready();}));
    assert(until([&] {return bool(server.handle(0));}));
    server.handle(0)->succeed(std::make_shared<Action::Result>());
    assert(until([&] {return client->executeTick() == BT::NodeStatus::SUCCESS;}));
    assert(client->successes == 1);
  } else {
    assert(until([&] {return bool(server.handle(0));}));
    const bool canceled = until([&] {return server.cancels.load() == 1;});
    auto h = server.handle(0);
    if (canceled && mode != "refuse") assert(until([&] {return h->is_canceling();}));
    if (client && canceled) {
      // A repeated tick may not replace an unresolved abandoned request.
      assert(client->executeTick() == BT::NodeStatus::FAILURE);
      assert(server.requests() == 1 && client->successes == 0);
    }
    if (mode == "shutdown") {
      assert(canceled);
      client.reset();
      rclcpp::shutdown();  // terminal result deliberately withheld: pre-shutdown must drain cleanup
      std::cout << "PASS shutdown with unresolved result" << std::endl;
      return 0;
    }
    if (h->is_canceling()) h->canceled(std::make_shared<Action::Result>());
    else h->succeed(std::make_shared<Action::Result>());
    assert(canceled && "late accepted request escaped cancellation");
    if (client) healthy(*client, server, 2);
  }
  std::cout << "PASS " << mode << " cancel_requests=" << server.cancels.load() << std::endl;
  if (client) client->halt();
  client.reset(); rclcpp::shutdown();
}
