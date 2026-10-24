#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/spin.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rm_competition_interfaces/msg/chassis_mode.hpp"
#include "rm_competition_interfaces/msg/mission_state.hpp"
#include "rm_competition_interfaces/msg/localization_status.hpp"
#include "rm_competition_interfaces/msg/referee_state.hpp"
#include "rm_competition_interfaces/msg/pursuit_candidate.hpp"
#include "rm_competition_interfaces/msg/v2_motion_guard_state.hpp"
#include "rm_competition_interfaces/srv/set_mission_mode.hpp"
#include "rm_competition_interfaces/srv/set_temporary_mission.hpp"
#include "std_msgs/msg/bool.hpp"

namespace
{

double yawFromPose(const geometry_msgs::msg::Pose & pose)
{
  return 2.0 * std::atan2(pose.orientation.z, pose.orientation.w);
}

geometry_msgs::msg::PoseStamped makePose(
  const std::string & frame, double x, double y, double yaw)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = frame;
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.orientation.z = std::sin(yaw * 0.5);
  pose.pose.orientation.w = std::cos(yaw * 0.5);
  return pose;
}

bool validMode(const std::string & mode)
{
  return mode == "hold" || mode == "home" || mode == "patrol" ||
         mode == "pursuit" || mode == "auto";
}

}  // namespace

class CompetitionMissionNode : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  using TemporaryCommand = rm_competition_interfaces::srv::SetTemporaryMission;
  using Candidate = rm_competition_interfaces::msg::PursuitCandidate;
  using GuardState = rm_competition_interfaces::msg::V2MotionGuardState;
  using Spin = nav2_msgs::action::Spin;
  using SpinGoalHandle = rclcpp_action::ClientGoalHandle<Spin>;

  CompetitionMissionNode()
  : Node("competition_mission_node")
  {
    localization_start_ns_ = now().nanoseconds();
    localization_last_clock_ns_ = localization_start_ns_;
    localization_status_timeout_sec_ = declare_parameter<double>(
      "localization_status_timeout_sec", 1.0);
    if (!std::isfinite(localization_status_timeout_sec_) || localization_status_timeout_sec_ <= 0.0) {
      throw std::invalid_argument("localization_status_timeout_sec must be positive");
    }
    tree_xml_ = declare_parameter<std::string>("tree_xml", "");
    startup_enabled_ = declare_parameter<bool>("startup_enabled", false);
    requested_mode_ = declare_parameter<std::string>("default_mode", "hold");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    navigate_action_ = declare_parameter<std::string>(
      "navigate_action", "/navigate_to_pose");
    spin_action_ = declare_parameter<std::string>("spin_action", "/spin");
    tick_rate_hz_ = declare_parameter<double>("tick_rate_hz", 10.0);
    require_localization_valid_ = declare_parameter<bool>(
      "require_localization_valid", true);
    require_referee_state_ = declare_parameter<bool>("require_referee_state", true);
    require_game_running_ = declare_parameter<bool>("require_game_running", true);
    require_chassis_mode_ = declare_parameter<bool>("require_chassis_mode", true);
    allow_pursuit_ = declare_parameter<bool>("allow_pursuit", false);
    radar_only_ = declare_parameter<bool>("radar_only", true);
    require_motion_guard_ = declare_parameter<bool>("require_motion_guard", false);
    input_timeout_sec_ = declare_parameter<double>("input_timeout_sec", 0.5);
    if (!std::isfinite(input_timeout_sec_) || input_timeout_sec_ <= 0.0) {
      throw std::invalid_argument("input_timeout_sec must be positive");
    }
    game_running_value_ = declare_parameter<int>("game_running_value", 4);
    retreat_hp_threshold_ = declare_parameter<int>("retreat_hp_threshold", 150);
    goal_update_distance_ = declare_parameter<double>("goal_update_distance", 0.35);
    goal_update_yaw_ = declare_parameter<double>("goal_update_yaw", 0.35);
    minimum_goal_update_sec_ = declare_parameter<double>("minimum_goal_update_sec", 0.5);
    goal_failure_retry_sec_ = declare_parameter<double>("goal_failure_retry_sec", 2.0);
    max_consecutive_goal_failures_ = declare_parameter<int>(
      "max_consecutive_goal_failures", 3);
    patrol_spin_angular_velocity_ = declare_parameter<double>(
      "patrol_spin_angular_velocity", 1.2);
    patrol_spin_duration_sec_ = declare_parameter<double>(
      "patrol_spin_duration_sec", 10.0);
    spin_time_allowance_sec_ = declare_parameter<double>(
      "spin_time_allowance_sec", 15.0);
    spin_failure_retry_sec_ = declare_parameter<double>(
      "spin_failure_retry_sec", 2.0);
    max_consecutive_spin_failures_ = declare_parameter<int>(
      "max_consecutive_spin_failures", 3);
    supply_values_ = declare_parameter<std::vector<double>>("supply_pose", std::vector<double>{});
    recovery_values_ = declare_parameter<std::vector<double>>("recovery_pose", std::vector<double>{});
    maximum_temporary_mission_sec_ = declare_parameter<double>("maximum_temporary_mission_sec", 300.0);
    if (!std::isfinite(maximum_temporary_mission_sec_) || maximum_temporary_mission_sec_ <= 0.0) {
      throw std::invalid_argument("maximum_temporary_mission_sec must be positive");
    }
    home_values_ = declare_parameter<std::vector<double>>(
      "home_pose", std::vector<double>{});
    patrol_values_ = declare_parameter<std::vector<double>>(
      "patrol_waypoints", std::vector<double>{});

    if (tree_xml_.empty()) {
      throw std::invalid_argument("tree_xml must point to a behavior tree file");
    }
    if (!validMode(requested_mode_)) {
      throw std::invalid_argument("default_mode must be hold, home, patrol, pursuit or auto");
    }
    if (tick_rate_hz_ <= 0.0 || goal_update_distance_ < 0.0 ||
      goal_update_yaw_ < 0.0 || minimum_goal_update_sec_ < 0.0 ||
      goal_failure_retry_sec_ < 0.0 || max_consecutive_goal_failures_ < 1 ||
      patrol_spin_angular_velocity_ == 0.0 || patrol_spin_duration_sec_ <= 0.0 ||
      spin_time_allowance_sec_ < patrol_spin_duration_sec_ ||
      spin_failure_retry_sec_ < 0.0 || max_consecutive_spin_failures_ < 1)
    {
      throw std::invalid_argument("mission timing and goal thresholds are invalid");
    }
    loadWaypoints();
    operator_enabled_ = startup_enabled_;

    registerTreeNodes();
    tree_ = std::make_unique<BT::Tree>(factory_.createTreeFromFile(tree_xml_));
    navigation_client_ = rclcpp_action::create_client<NavigateToPose>(this, navigate_action_);
    spin_client_ = rclcpp_action::create_client<Spin>(this, spin_action_);

    auto latched = rclcpp::QoS(1).reliable().transient_local();
    mission_state_pub_ = create_publisher<rm_competition_interfaces::msg::MissionState>(
      "/mission/state", latched);
    localization_sub_ = create_subscription<rm_competition_interfaces::msg::LocalizationStatus>(
      "/localization/status", rclcpp::QoS(1).reliable(),
      [this](rm_competition_interfaces::msg::LocalizationStatus::SharedPtr message) {
        handleLocalizationStatus(*message);
      });
    referee_valid_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/referee/state_valid", latched,
      [this](std_msgs::msg::Bool::SharedPtr message) {referee_valid_ = message->data;});
    referee_sub_ = create_subscription<rm_competition_interfaces::msg::RefereeState>(
      "/referee/state", latched,
      [this](rm_competition_interfaces::msg::RefereeState::SharedPtr message) {
        referee_state_ = *message;
        referee_received_ = true;
        referee_received_at_ = std::chrono::steady_clock::now();
      });
    chassis_sub_ = create_subscription<rm_competition_interfaces::msg::ChassisMode>(
      "/chassis/mode", latched,
      [this](rm_competition_interfaces::msg::ChassisMode::SharedPtr message) {
        chassis_mode_ = *message;
        chassis_received_ = true;
        chassis_received_at_ = std::chrono::steady_clock::now();
      });
    pursuit_sub_ = create_subscription<Candidate>(
      "/mission/pursuit_candidate", 10,
      [this](Candidate::SharedPtr message) {
        const auto stamp = rclcpp::Time(message->header.stamp);
        if (stamp < rclcpp::Time(pursuit_candidate_.header.stamp)) {return;}
        if (stamp > rclcpp::Time(pursuit_candidate_.header.stamp)) {
          pursuit_received_at_ = std::chrono::steady_clock::now();
        }
        pursuit_candidate_ = *message;
      });
    motion_guard_sub_ = create_subscription<GuardState>(
      "/motion/v2_guard/state", rclcpp::QoS(1).reliable(),
      [this](GuardState::SharedPtr message) {
        motion_guard_ = *message;
        motion_guard_received_ = true;
        motion_guard_received_at_ = std::chrono::steady_clock::now();
      });

    set_mode_service_ = create_service<rm_competition_interfaces::srv::SetMissionMode>(
      "/mission/set_mode",
      [this](
        const std::shared_ptr<rm_competition_interfaces::srv::SetMissionMode::Request> request,
        std::shared_ptr<rm_competition_interfaces::srv::SetMissionMode::Response> response)
      {
        if (!validMode(request->mode)) {
          response->accepted = false;
          response->message = "mode must be hold, home, patrol, pursuit or auto";
          return;
        }
        finishTemporary("mode_changed");
        return_home_latched_ = false;
        operator_enabled_ = request->enable;
        requested_mode_ = request->mode;
        failed_goal_.reset();
        consecutive_goal_failures_ = 0;
        consecutive_spin_failures_ = 0;
        response->accepted = true;
        response->message = operator_enabled_ ? "mission enabled" : "mission disabled";
      });

    temporary_service_ = create_service<TemporaryCommand>(
      "/mission/set_temporary",
      [this](const std::shared_ptr<TemporaryCommand::Request> request,
        std::shared_ptr<TemporaryCommand::Response> response) {
        handleTemporary(*request, *response);
      });

    const auto period = std::chrono::duration<double>(1.0 / tick_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() {tick();});
    RCLCPP_WARN(
      get_logger(),
      "competition mission ready: enabled=%s mode=%s pursuit=%s. "
      "Only this node may convert mission choices into NavigateToPose or Spin goals.",
      operator_enabled_ ? "true" : "false", requested_mode_.c_str(),
      allow_pursuit_ ? "true" : "false");
  }

private:
  struct GoalRequest
  {
    geometry_msgs::msg::PoseStamped pose;
    std::string source;
    std::string track_id{};
  };

  struct TemporaryMission
  {
    GoalRequest goal;
    rclcpp::Time deadline;
    std::chrono::steady_clock::time_point steady_deadline;
  };

  static bool validPlanarPose(const geometry_msgs::msg::PoseStamped & pose,
    const std::string & frame)
  {
    const auto & p = pose.pose;
    const double norm = p.orientation.z * p.orientation.z + p.orientation.w * p.orientation.w;
    return pose.header.frame_id == frame &&
      std::isfinite(p.position.x) && std::isfinite(p.position.y) &&
      std::isfinite(p.position.z) && std::abs(p.position.z) < 1e-6 &&
      std::isfinite(p.orientation.x) && std::abs(p.orientation.x) < 1e-6 &&
      std::isfinite(p.orientation.y) && std::abs(p.orientation.y) < 1e-6 &&
      std::isfinite(norm) && std::abs(norm - 1.0) < 0.01;
  }

  bool temporaryFresh() const
  {
    return temporary_.has_value() && now() < temporary_->deadline &&
      std::chrono::steady_clock::now() < temporary_->steady_deadline;
  }

  void finishTemporary(const std::string & reason)
  {
    if (!temporary_) {return;}
    temporary_status_ = temporary_->goal.source + ":" + temporary_->goal.track_id + ":" + reason;
    temporary_.reset();
    pursuit_not_before_ = now();
  }

  bool shouldReturnHome() const
  {
    const bool low_hp = referee_valid_ && referee_received_ &&
      inputFresh(referee_state_.header.stamp, referee_received_at_) &&
      referee_state_.current_hp <= retreat_hp_threshold_;
    return return_home_latched_ || requested_mode_ == "home" || low_hp;
  }

  void handleTemporary(const TemporaryCommand::Request & command,
    TemporaryCommand::Response & response)
  {
    response.accepted = false;
    if (command.sequence == 0 || command.mission_id.empty()) {
      response.message = "nonzero sequence and mission_id are required"; return;
    }
    if (command.sequence <= last_temporary_sequence_) {
      response.accepted = command.sequence == last_temporary_sequence_ &&
        last_temporary_command_.has_value() && command == *last_temporary_command_;
      response.message = response.accepted ? "duplicate; no action or lease renewal" :
        "old sequence or reused sequence with different payload";
      return;
    }
    if (command.cancel) {
      if (!temporary_ || temporary_->goal.track_id != command.mission_id) {
        response.message = "no matching current temporary mission"; return;
      }
      finishTemporary("canceled");
    } else {
      if (!operator_enabled_ || requested_mode_ == "hold" || shouldReturnHome()) {
        response.message = "temporary mission requires enabled non-hold/non-home mode"; return;
      }
      const auto & stamp = command.valid_until;
      if (stamp.sec < 0 || stamp.nanosec >= 1000000000U) {
        response.message = "malformed valid_until"; return;
      }
      const rclcpp::Time deadline(stamp);
      const double remaining = (deadline - now()).seconds();
      if (remaining <= 0.0 || remaining > maximum_temporary_mission_sec_) {
        response.message = "expired or excessive temporary deadline"; return;
      }
      geometry_msgs::msg::PoseStamped target;
      if (command.kind == "supply" || command.kind == "recovery") {
        const auto & configured = command.kind == "supply" ? supply_pose_ : recovery_pose_;
        if (!configured) {response.message = "missing configured " + command.kind + " pose"; return;}
        target = *configured;
      } else if (command.kind == "special") {
        target = command.target;
      } else {
        response.message = "kind must be supply, recovery or special"; return;
      }
      if (!validPlanarPose(target, map_frame_)) {
        response.message = "target must be a finite planar map-frame pose"; return;
      }
      // A higher transport sequence may repeat the same logical command, but
      // cannot mutate its identity/target/deadline or resurrect a finished one.
      if (last_temporary_payload_ && command.mission_id == last_temporary_payload_->mission_id) {
        auto payload = command;
        payload.sequence = last_temporary_payload_->sequence;
        if (!(payload == *last_temporary_payload_)) {
          response.message = "mission id payload is immutable; use a new id"; return;
        }
        last_temporary_sequence_ = command.sequence;
        last_temporary_command_ = command;
        response.accepted = true;
        response.message = "same mission; no action or lease renewal"; return;
      }
      finishTemporary("replaced");
      temporary_ = TemporaryMission{GoalRequest{target, command.kind, command.mission_id}, deadline,
        std::chrono::steady_clock::now() + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(remaining))};
      last_temporary_payload_ = command;
      temporary_status_ = command.kind + ":" + command.mission_id + ":accepted";
      pursuit_not_before_ = now();
      failed_goal_.reset();
      consecutive_goal_failures_ = 0;
    }
    last_temporary_sequence_ = command.sequence;
    last_temporary_command_ = command;
    response.accepted = true;
    response.message = command.cancel ? "temporary mission canceled" : "temporary mission accepted";
  }

  void loadWaypoints()
  {
    const auto load_optional = [this](const std::vector<double> & values,
        const std::string & name) -> std::optional<geometry_msgs::msg::PoseStamped> {
        if (values.empty()) {return std::nullopt;}
        if (values.size() != 3 || !std::all_of(values.begin(), values.end(),
          [](double v) {return std::isfinite(v);}))
        {
          throw std::invalid_argument(name + " must be empty or finite [x, y, yaw]");
        }
        return makePose(map_frame_, values[0], values[1], values[2]);
      };
    supply_pose_ = load_optional(supply_values_, "supply_pose");
    recovery_pose_ = load_optional(recovery_values_, "recovery_pose");
    if (!home_values_.empty() && home_values_.size() != 3) {
      throw std::invalid_argument("home_pose must be empty or [x, y, yaw]");
    }
    if (patrol_values_.size() % 3 != 0) {
      throw std::invalid_argument("patrol_waypoints must contain x, y, yaw triples");
    }
    if (home_values_.size() == 3) {
      home_pose_ = makePose(
        map_frame_, home_values_[0], home_values_[1], home_values_[2]);
    }
    for (std::size_t index = 0; index < patrol_values_.size(); index += 3) {
      patrol_poses_.push_back(makePose(
          map_frame_, patrol_values_[index], patrol_values_[index + 1],
          patrol_values_[index + 2]));
    }
  }

  void registerTreeNodes()
  {
    factory_.registerSimpleCondition(
      "MissionEnabled", [this](BT::TreeNode &) {
        return operator_enabled_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
      });
    factory_.registerSimpleCondition(
      "SafetyReady", [this](BT::TreeNode &) {
        return safetyReady() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
      });
    factory_.registerSimpleCondition(
      "ShouldReturnHome", [this](BT::TreeNode &) {
        return shouldReturnHome() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
      });
    factory_.registerSimpleCondition(
      "HasTemporaryMission", [this](BT::TreeNode &) {
        return temporaryFresh() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
      });
    factory_.registerSimpleAction(
      "SelectTemporaryMission", [this](BT::TreeNode &) {
        if (!temporaryFresh()) {return BT::NodeStatus::FAILURE;}
        desired_goal_ = temporary_->goal;
        active_branch_ = temporary_->goal.source;
        return BT::NodeStatus::SUCCESS;
      });
    factory_.registerSimpleCondition(
      "CanPursue", [this](BT::TreeNode &) {
        const bool mode_allows = requested_mode_ == "pursuit" || requested_mode_ == "auto";
        return (allow_pursuit_ && mode_allows && pursuitFresh()) ?
               BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
      });
    factory_.registerSimpleCondition(
      "CanPatrol", [this](BT::TreeNode &) {
        const bool mode_allows = requested_mode_ == "patrol" || requested_mode_ == "auto";
        return (mode_allows && !patrol_poses_.empty()) ?
               BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
      });
    factory_.registerSimpleAction(
      "SelectHome", [this](BT::TreeNode &) {return selectHome();});
    factory_.registerSimpleAction(
      "SelectPursuit", [this](BT::TreeNode &) {return selectPursuit();});
    factory_.registerSimpleAction(
      "SelectPatrol", [this](BT::TreeNode &) {return selectPatrol();});
    factory_.registerSimpleAction(
      "SelectPatrolWithSpin",
      [this](BT::TreeNode &) {return selectPatrolWithSpin();});
    factory_.registerSimpleAction(
      "SelectHold", [this](BT::TreeNode &) {
        active_branch_ = "hold";
        desired_goal_.reset();
        return BT::NodeStatus::SUCCESS;
      });
  }

  void handleLocalizationStatus(const rm_competition_interfaces::msg::LocalizationStatus & msg)
  {
    const auto well_formed = [](const builtin_interfaces::msg::Time & stamp) {
        return stamp.sec >= 0 && stamp.nanosec < 1000000000U;
      };
    if (!well_formed(msg.header.stamp) || !well_formed(msg.started_at) ||
      !well_formed(msg.valid_until))
    {
      localization_valid_ = false;
      return;
    }
    const auto emitted = rclcpp::Time(msg.header.stamp);
    const auto epoch = rclcpp::Time(msg.started_at).nanoseconds();
    const auto deadline = rclcpp::Time(msg.valid_until);
    const double age = (now() - emitted).seconds();
    if (msg.producer_id.empty() || msg.sequence == 0 || epoch <= 0 ||
      epoch > emitted.nanoseconds() || emitted.nanoseconds() <= localization_start_ns_ ||
      age < -0.1 || age > localization_status_timeout_sec_ || epoch < localization_epoch_ns_ ||
      (epoch == localization_epoch_ns_ && !localization_id_.empty() && msg.producer_id != localization_id_) ||
      (msg.producer_id == localization_id_ &&
      (msg.sequence <= localization_sequence_ || emitted.nanoseconds() < localization_emitted_ns_)))
    {
      localization_valid_ = false;
      return;
    }
    localization_id_ = msg.producer_id;
    localization_epoch_ns_ = epoch;
    localization_sequence_ = msg.sequence;
    localization_emitted_ns_ = emitted.nanoseconds();
    localization_deadline_ = deadline;
    localization_received_ = std::chrono::steady_clock::now();
    localization_valid_ = msg.valid && deadline > now() &&
      (deadline - emitted).seconds() <= 1.6;
  }

  bool localizationFresh() const
  {
    return localization_valid_ && now() >= rclcpp::Time(localization_emitted_ns_, RCL_ROS_TIME) &&
      now() <= localization_deadline_ &&
      (now().nanoseconds() - localization_emitted_ns_) * 1.0e-9 <= localization_status_timeout_sec_ &&
      std::chrono::duration<double>(std::chrono::steady_clock::now() -
      localization_received_).count() <= localization_status_timeout_sec_;
  }

  bool inputFresh(
    const builtin_interfaces::msg::Time & stamp,
    const std::chrono::steady_clock::time_point & received) const
  {
    const double age = (now() - rclcpp::Time(stamp)).seconds();
    return rclcpp::Time(stamp).nanoseconds() > 0 && age >= -0.05 &&
      age <= input_timeout_sec_ &&
      std::chrono::duration<double>(std::chrono::steady_clock::now() - received).count()
      <= input_timeout_sec_;
  }

  bool pursuitFresh() const
  {
    const auto & c = pursuit_candidate_;
    const auto & p = c.pose;
    const double norm = p.orientation.x * p.orientation.x + p.orientation.y * p.orientation.y +
      p.orientation.z * p.orientation.z + p.orientation.w * p.orientation.w;
    return c.valid && !c.track_id.empty() && c.header.frame_id == map_frame_ &&
      (!radar_only_ || c.source == Candidate::SOURCE_RADAR) &&
      inputFresh(c.header.stamp, pursuit_received_at_) &&
      rclcpp::Time(c.header.stamp) > pursuit_not_before_ &&
      rclcpp::Time(c.valid_until) > now() &&
      (rclcpp::Time(c.valid_until) - rclcpp::Time(c.header.stamp)).seconds() <= input_timeout_sec_ &&
      std::isfinite(p.position.x) && std::isfinite(p.position.y) && std::isfinite(p.position.z) &&
      std::isfinite(norm) && std::abs(norm - 1.0) < 0.01;
  }

  bool safetyReady() const
  {
    if (require_localization_valid_ && !localizationFresh()) {
      return false;
    }
    if (require_referee_state_ && (!referee_valid_ || !referee_received_ ||
      !inputFresh(referee_state_.header.stamp, referee_received_at_))) {
      return false;
    }
    if (require_game_running_ &&
      (!referee_received_ || referee_state_.game_progress != game_running_value_))
    {
      return false;
    }
    if (require_chassis_mode_ &&
      (!chassis_received_ || !inputFresh(chassis_mode_.header.stamp, chassis_received_at_) ||
      !chassis_mode_.online ||
      !chassis_mode_.autonomous_enabled || chassis_mode_.emergency_stop))
    {
      return false;
    }
    if ((require_motion_guard_ || motion_guard_received_) &&
      (!motion_guard_received_ || motion_guard_.phase != GuardState::PERMITTED ||
      !inputFresh(motion_guard_.header.stamp, motion_guard_received_at_)))
    {
      return false;
    }
    return true;
  }

  BT::NodeStatus selectHome()
  {
    return_home_latched_ = true;
    finishTemporary("preempted_home");
    pursuit_not_before_ = now();
    if (!home_pose_.has_value()) {
      active_branch_ = "hold_missing_home";
      desired_goal_.reset();
      return BT::NodeStatus::SUCCESS;
    }
    desired_goal_ = GoalRequest{*home_pose_, "home"};
    active_branch_ = "home";
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus selectPursuit()
  {
    if (!pursuitFresh()) {
      return BT::NodeStatus::FAILURE;
    }
    geometry_msgs::msg::PoseStamped pose;
    pose.header = pursuit_candidate_.header;
    pose.pose = pursuit_candidate_.pose;
    desired_goal_ = GoalRequest{pose, "pursuit", pursuit_candidate_.track_id};
    active_branch_ = "pursuit";
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus selectPatrol()
  {
    if (patrol_poses_.empty()) {
      return BT::NodeStatus::FAILURE;
    }
    const auto index = patrol_index_ % patrol_poses_.size();
    desired_goal_ = GoalRequest{patrol_poses_[index], "patrol"};
    active_branch_ = "patrol";
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus selectPatrolWithSpin()
  {
    if (patrol_poses_.empty()) {
      return BT::NodeStatus::FAILURE;
    }
    patrol_spin_selected_ = true;
    if (patrol_spin_pending_ || spin_goal_sent_ || active_spin_goal_handle_) {
      desired_goal_.reset();
      active_branch_ = "patrol_spin";
      return BT::NodeStatus::SUCCESS;
    }
    const auto index = patrol_index_ % patrol_poses_.size();
    desired_goal_ = GoalRequest{patrol_poses_[index], "patrol_spin"};
    active_branch_ = "patrol";
    return BT::NodeStatus::SUCCESS;
  }

  void tick()
  {
    const auto clock_ns = now().nanoseconds();
    if (clock_ns < localization_last_clock_ns_) {
      localization_valid_ = false;
      finishTemporary("clock_rollback");
    }
    localization_last_clock_ns_ = std::max(localization_last_clock_ns_, clock_ns);
    if (temporary_ && !temporaryFresh()) {finishTemporary("expired");}
    desired_goal_.reset();
    patrol_spin_selected_ = false;
    if (!operator_enabled_ || requested_mode_ == "hold" || !safetyReady()) {
      // A new target observation is required after a hold/revocation, even if
      // the previous candidate still has a short remaining lease.
      pursuit_not_before_ = now();
      active_branch_ = "hold";
    } else {
      tree_->tickRoot();
    }
    if (patrol_spin_selected_ &&
      (patrol_spin_pending_ || spin_goal_sent_ || active_spin_goal_handle_))
    {
      cancelNavigation("patrol spin", false);
      reconcileSpin();
    } else {
      cancelSpin("mission branch changed");
      reconcileNavigation();
    }
    publishState();
  }

  bool goalChanged(const GoalRequest & left, const GoalRequest & right) const
  {
    if (left.source != right.source || left.track_id != right.track_id) {
      return true;
    }
    const double dx = left.pose.pose.position.x - right.pose.pose.position.x;
    const double dy = left.pose.pose.position.y - right.pose.pose.position.y;
    constexpr double kTwoPi = 6.28318530717958647692;
    const double dyaw = std::remainder(
      yawFromPose(left.pose.pose) - yawFromPose(right.pose.pose), kTwoPi);
    return std::hypot(dx, dy) > goal_update_distance_ || std::abs(dyaw) > goal_update_yaw_;
  }

  bool navigationOutstanding() const
  {
    return navigation_pending_ || active_goal_handle_ != nullptr;
  }

  bool spinOutstanding() const
  {
    return spin_goal_sent_ || active_spin_goal_handle_ != nullptr;
  }

  void reconcileNavigation()
  {
    if (!desired_goal_.has_value()) {
      cancelNavigation("mission hold or safety gate");
      return;
    }
    if (spinOutstanding()) {
      navigation_status_ = "waiting_for_spin_cancel";
      return;
    }
    if (navigationOutstanding()) {
      if (!navigation_cancel_requested_ && last_sent_goal_.has_value() &&
        goalChanged(*desired_goal_, *last_sent_goal_))
      {
        const bool new_intent = desired_goal_->source != last_sent_goal_->source ||
          desired_goal_->track_id != last_sent_goal_->track_id;
        if (new_intent || (now() - last_goal_send_time_).seconds() >= minimum_goal_update_sec_) {
          cancelNavigation("intent changed");
        }
      }
      return;
    }
    if (completed_goal_.has_value() && !goalChanged(*desired_goal_, *completed_goal_)) {
      navigation_status_ = "goal_complete";
      return;
    }
    if (failed_goal_.has_value()) {
      if (goalChanged(*desired_goal_, *failed_goal_)) {
        failed_goal_.reset();
        consecutive_goal_failures_ = 0;
      } else if (consecutive_goal_failures_ >= max_consecutive_goal_failures_) {
        navigation_status_ = "goal_retry_limit";
        return;
      } else if ((now() - last_goal_send_time_).seconds() < goal_failure_retry_sec_) {
        navigation_status_ = "goal_retry_backoff";
        return;
      }
    }
    if (completed_goal_.has_value() && desired_goal_->source == "pursuit" &&
      completed_goal_->source == "pursuit" &&
      desired_goal_->track_id == completed_goal_->track_id &&
      (now() - last_goal_send_time_).seconds() < minimum_goal_update_sec_)
    {
      return;
    }
    sendGoal(*desired_goal_);
  }

  void sendGoal(GoalRequest request)
  {
    if (!navigation_client_->action_server_is_ready()) {
      navigation_status_ = "waiting_for_nav2";
      return;
    }
    navigation_pending_ = true;
    navigation_cancel_requested_ = false;
    navigation_cancel_sent_ = false;
    request.pose.header.stamp = now();
    NavigateToPose::Goal goal;
    goal.pose = request.pose;
    const std::uint64_t generation = ++goal_generation_;
    last_sent_goal_ = request;
    completed_goal_.reset();
    last_goal_send_time_ = now();
    navigation_status_ = "goal_pending";

    auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    options.goal_response_callback =
      [this, generation](GoalHandle::SharedPtr handle) {
        if (generation != goal_generation_) {
          if (handle) {
            navigation_client_->async_cancel_goal(handle);
          }
          return;
        }
        navigation_pending_ = false;
        active_goal_handle_ = handle;
        if (navigation_cancel_requested_) {
          if (handle) {
            cancelNavigation("late acceptance");
          } else {
            navigation_cancel_requested_ = false;
            last_sent_goal_.reset();
          }
          return;
        }
        navigation_status_ = handle ? "goal_active" : "goal_rejected";
        if (!handle) {
          failed_goal_ = last_sent_goal_;
          ++consecutive_goal_failures_;
          last_goal_send_time_ = now();
          last_sent_goal_.reset();
        }
      };
    options.result_callback =
      [this, generation](const GoalHandle::WrappedResult & result) {
        if (generation != goal_generation_) {
          return;
        }
        active_goal_handle_.reset();
        navigation_pending_ = false;
        if (navigation_cancel_requested_) {
          navigation_cancel_requested_ = false;
          last_sent_goal_.reset();
          navigation_status_ = "goal_cancel_complete";
          return;
        }
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          navigation_status_ = "goal_succeeded";
          failed_goal_.reset();
          consecutive_goal_failures_ = 0;
          completed_goal_ = last_sent_goal_;
          if (temporary_ && last_sent_goal_ &&
            temporary_->goal.source == last_sent_goal_->source &&
            temporary_->goal.track_id == last_sent_goal_->track_id)
          {
            finishTemporary(temporaryFresh() ? "completed" : "expired");
          }
          if (last_sent_goal_.has_value() && !patrol_poses_.empty()) {
            if (last_sent_goal_->source == "patrol") {
              patrol_index_ = (patrol_index_ + 1) % patrol_poses_.size();
              completed_goal_.reset();
            } else if (last_sent_goal_->source == "patrol_spin") {
              patrol_spin_pending_ = true;
              navigation_status_ = "spin_pending";
              completed_goal_.reset();
              last_sent_goal_.reset();
            }
          }
        } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
          navigation_status_ = "goal_canceled";
          failed_goal_ = last_sent_goal_;
          ++consecutive_goal_failures_;
          last_goal_send_time_ = now();
        } else {
          navigation_status_ = "goal_failed";
          failed_goal_ = last_sent_goal_;
          ++consecutive_goal_failures_;
          last_goal_send_time_ = now();
          last_sent_goal_.reset();
        }
        last_sent_goal_.reset();
      };
    navigation_client_->async_send_goal(goal, options);
  }

  void reconcileSpin()
  {
    if (!patrol_spin_pending_) {
      return;
    }
    if (spinOutstanding() || navigationOutstanding()) {
      return;
    }
    if (consecutive_spin_failures_ >= max_consecutive_spin_failures_) {
      navigation_status_ = "spin_retry_limit";
      return;
    }
    if (last_spin_send_time_.nanoseconds() > 0 &&
      (now() - last_spin_send_time_).seconds() < spin_failure_retry_sec_)
    {
      navigation_status_ = "spin_retry_backoff";
      return;
    }
    sendSpin();
  }

  void sendSpin()
  {
    if (!spin_client_->action_server_is_ready()) {
      navigation_status_ = "waiting_for_spin_server";
      return;
    }
    Spin::Goal goal;
    goal.target_yaw = static_cast<float>(
      patrol_spin_angular_velocity_ * patrol_spin_duration_sec_);
    goal.time_allowance = static_cast<builtin_interfaces::msg::Duration>(
      rclcpp::Duration::from_seconds(spin_time_allowance_sec_));

    const std::uint64_t generation = ++spin_generation_;
    spin_goal_sent_ = true;
    spin_cancel_requested_ = false;
    spin_cancel_sent_ = false;
    last_spin_send_time_ = now();
    navigation_status_ = "spin_goal_pending";

    auto options = rclcpp_action::Client<Spin>::SendGoalOptions();
    options.goal_response_callback =
      [this, generation](SpinGoalHandle::SharedPtr handle) {
        if (generation != spin_generation_) {
          if (handle) {
            spin_client_->async_cancel_goal(handle);
          }
          return;
        }
        active_spin_goal_handle_ = handle;
        if (spin_cancel_requested_) {
          if (handle) {
            cancelSpin("late acceptance");
          } else {
            spin_goal_sent_ = false;
            spin_cancel_requested_ = false;
          }
          return;
        }
        navigation_status_ = handle ? "spin_active" : "spin_rejected";
        if (!handle) {
          spin_goal_sent_ = false;
          ++consecutive_spin_failures_;
          last_spin_send_time_ = now();
        }
      };
    options.result_callback =
      [this, generation](const SpinGoalHandle::WrappedResult & result) {
        if (generation != spin_generation_) {
          return;
        }
        active_spin_goal_handle_.reset();
        spin_goal_sent_ = false;
        if (spin_cancel_requested_) {
          spin_cancel_requested_ = false;
          navigation_status_ = "spin_cancel_complete";
          return;
        }
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          navigation_status_ = "spin_succeeded";
          patrol_spin_pending_ = false;
          consecutive_spin_failures_ = 0;
          if (!patrol_poses_.empty()) {
            patrol_index_ = (patrol_index_ + 1) % patrol_poses_.size();
          }
        } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
          navigation_status_ = "spin_canceled";
          ++consecutive_spin_failures_;
          last_spin_send_time_ = now();
        } else {
          navigation_status_ = "spin_failed";
          ++consecutive_spin_failures_;
          last_spin_send_time_ = now();
        }
      };
    spin_client_->async_send_goal(goal, options);
  }

  void cancelSpin(const std::string & reason)
  {
    patrol_spin_pending_ = false;
    consecutive_spin_failures_ = 0;
    if (!spinOutstanding()) {return;}
    spin_cancel_requested_ = true;
    if (active_spin_goal_handle_ && !spin_cancel_sent_) {
      spin_cancel_sent_ = true;
      spin_client_->async_cancel_goal(active_spin_goal_handle_);
    }
    navigation_status_ = "spin_canceling: " + reason;
  }

  void cancelNavigation(const std::string & reason, bool clear_completed = true)
  {
    if (clear_completed) {completed_goal_.reset();}
    if (!navigationOutstanding()) {return;}
    navigation_cancel_requested_ = true;
    if (active_goal_handle_ && !navigation_cancel_sent_) {
      navigation_cancel_sent_ = true;
      navigation_client_->async_cancel_goal(active_goal_handle_);
    }
    // Keep the pending/active handle until its terminal result. A cancel ACK
    // only means CANCELING; it does not grant ownership to a replacement.
    navigation_status_ = "goal_canceling: " + reason;
  }

  void publishState()
  {
    rm_competition_interfaces::msg::MissionState state;
    state.header.stamp = now();
    state.header.frame_id = map_frame_;
    state.enabled = operator_enabled_;
    state.requested_mode = requested_mode_;
    state.active_branch = active_branch_;
    state.navigation_status = navigation_status_;
    state.detail = std::string(safetyReady() ? "safety_ready" : "safety_gate_closed") +
      "; temporary=" + temporary_status_ +
      "; home_latched=" + (return_home_latched_ ? "true" : "false");
    mission_state_pub_->publish(state);
  }

  std::optional<TemporaryMission> temporary_;
  std::optional<TemporaryCommand::Request> last_temporary_command_;
  std::optional<TemporaryCommand::Request> last_temporary_payload_;
  std::uint64_t last_temporary_sequence_{0};
  std::string temporary_status_{"none"};
  bool return_home_latched_{false};
  double maximum_temporary_mission_sec_;
  std::vector<double> supply_values_, recovery_values_;
  std::optional<geometry_msgs::msg::PoseStamped> supply_pose_, recovery_pose_;
  rclcpp::Service<TemporaryCommand>::SharedPtr temporary_service_;

  std::string tree_xml_;
  bool startup_enabled_;
  bool operator_enabled_{false};
  std::string requested_mode_;
  std::string map_frame_;
  std::string navigate_action_;
  std::string spin_action_;
  double tick_rate_hz_;
  bool require_localization_valid_;
  bool require_referee_state_;
  bool require_game_running_;
  bool require_chassis_mode_;
  bool allow_pursuit_;
  bool radar_only_;
  bool require_motion_guard_;
  double input_timeout_sec_;
  bool motion_guard_received_{false};
  GuardState motion_guard_;
  Candidate pursuit_candidate_;
  std::chrono::steady_clock::time_point pursuit_received_at_;
  std::chrono::steady_clock::time_point referee_received_at_;
  std::chrono::steady_clock::time_point chassis_received_at_;
  std::chrono::steady_clock::time_point motion_guard_received_at_;
  rclcpp::Time pursuit_not_before_{0, 0, RCL_ROS_TIME};
  bool navigation_pending_{false};
  bool navigation_cancel_requested_{false};
  bool navigation_cancel_sent_{false};
  bool spin_cancel_requested_{false};
  bool spin_cancel_sent_{false};
  int game_running_value_;
  int retreat_hp_threshold_;
  double goal_update_distance_;
  double goal_update_yaw_;
  double minimum_goal_update_sec_;
  double goal_failure_retry_sec_;
  int max_consecutive_goal_failures_;
  double patrol_spin_angular_velocity_;
  double patrol_spin_duration_sec_;
  double spin_time_allowance_sec_;
  double spin_failure_retry_sec_;
  int max_consecutive_spin_failures_;
  std::vector<double> home_values_;
  std::vector<double> patrol_values_;
  std::optional<geometry_msgs::msg::PoseStamped> home_pose_;
  std::vector<geometry_msgs::msg::PoseStamped> patrol_poses_;
  std::size_t patrol_index_{0};
  bool patrol_spin_selected_{false};
  bool patrol_spin_pending_{false};

  bool localization_valid_{false};
  double localization_status_timeout_sec_{1.0};
  int64_t localization_start_ns_{0}, localization_last_clock_ns_{0};
  int64_t localization_epoch_ns_{0}, localization_emitted_ns_{0};
  uint64_t localization_sequence_{0};
  std::string localization_id_;
  rclcpp::Time localization_deadline_{0, 0, RCL_ROS_TIME};
  std::chrono::steady_clock::time_point localization_received_;
  bool referee_valid_{false};
  bool referee_received_{false};
  bool chassis_received_{false};
  rm_competition_interfaces::msg::RefereeState referee_state_;
  rm_competition_interfaces::msg::ChassisMode chassis_mode_;

  BT::BehaviorTreeFactory factory_;
  std::unique_ptr<BT::Tree> tree_;
  std::optional<GoalRequest> desired_goal_;
  std::optional<GoalRequest> last_sent_goal_;
  std::optional<GoalRequest> completed_goal_;
  std::optional<GoalRequest> failed_goal_;
  int consecutive_goal_failures_{0};
  int consecutive_spin_failures_{0};
  bool spin_goal_sent_{false};
  std::string active_branch_{"hold"};
  std::string navigation_status_{"holding"};
  rclcpp::Time last_goal_send_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_spin_send_time_{0, 0, RCL_ROS_TIME};
  std::uint64_t goal_generation_{0};
  std::uint64_t spin_generation_{0};
  GoalHandle::SharedPtr active_goal_handle_;
  SpinGoalHandle::SharedPtr active_spin_goal_handle_;

  rclcpp_action::Client<NavigateToPose>::SharedPtr navigation_client_;
  rclcpp_action::Client<Spin>::SharedPtr spin_client_;
  rclcpp::Publisher<rm_competition_interfaces::msg::MissionState>::SharedPtr
    mission_state_pub_;
  rclcpp::Subscription<rm_competition_interfaces::msg::LocalizationStatus>::SharedPtr localization_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr referee_valid_sub_;
  rclcpp::Subscription<rm_competition_interfaces::msg::RefereeState>::SharedPtr
    referee_sub_;
  rclcpp::Subscription<rm_competition_interfaces::msg::ChassisMode>::SharedPtr
    chassis_sub_;
  rclcpp::Subscription<Candidate>::SharedPtr pursuit_sub_;
  rclcpp::Subscription<GuardState>::SharedPtr motion_guard_sub_;
  rclcpp::Service<rm_competition_interfaces::srv::SetMissionMode>::SharedPtr
    set_mode_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CompetitionMissionNode>());
  rclcpp::shutdown();
  return 0;
}
