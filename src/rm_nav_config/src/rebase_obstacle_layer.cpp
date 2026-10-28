// Selective map-frame invalidation. Reuses Nav2 marking/raytracing; no TF or motion authority.
#include <algorithm>
#include <mutex>
#include <stdexcept>
#include <vector>
#include "nav2_costmap_2d/obstacle_layer.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rm_competition_interfaces/msg/localization_status.hpp"
#include "rm_competition_interfaces/msg/localization_barrier.hpp"

namespace rm_nav_config
{
class RebaseObstacleLayer : public nav2_costmap_2d::ObstacleLayer
{
  using Status = rm_competition_interfaces::msg::LocalizationStatus;
public:
  void onInitialize() override
  {
    ObstacleLayer::onInitialize();
    if (global_frame_ != "map") {throw std::invalid_argument("rebase layer is only for map-frame obstacles");}
    auto node = node_.lock();
    receipt_ = node->create_publisher<rm_competition_interfaces::msg::LocalizationBarrier>(
      "/localization/map_cache_primed", 10);
    status_ = node->create_subscription<Status>("/localization/status", 10,
      [this](const Status & message) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (message.started_at.sec < 0 || message.started_at.nanosec >= 1000000000U ||
          message.header.stamp.sec < 0 || message.header.stamp.nanosec >= 1000000000U ||
          message.committed_at.sec < 0 || message.committed_at.nanosec >= 1000000000U) {return;}
        if (rclcpp::Time(message.started_at) < rclcpp::Time(session_.started_at) ||
          (message.started_at == session_.started_at &&
          (message.producer_id != session_.producer_id || message.sequence <= session_.sequence))) {return;}
        session_ = message;
      });
  }

  void updateBounds(double x, double y, double yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    ready_ = false;
    if (session_.transaction_phase == Status::DRAIN_OLD_NAV ||
      session_.transaction_phase == Status::REBASE_FAILED) {current_ = false; return;}
    const auto cutoff = rclcpp::Time(session_.committed_at).nanoseconds();
    if (cutoff > cutoff_) {
      cutoff_ = cutoff;
      // Rebuild only this obstacle grid. StaticLayer and odom-frame local layers survive.
      resetMaps();
      addExtraBounds(getOriginX(), getOriginY(),
        getOriginX() + getSizeInMetersX(), getOriginY() + getSizeInMetersY());
      for (auto & notifier : observation_notifiers_) {notifier->clear();}
    }
    if (cutoff_ == 0) {ObstacleLayer::updateBounds(x, y, yaw, min_x, min_y, max_x, max_y); return;}
    // Lock through base processing: callbacks cannot replace a checked buffer with
    // a delayed pre-commit observation between validation and raytracing.
    for (auto & buffer : observation_buffers_) {buffer->lock();}
    const auto marking = marking_buffers_, clearing = clearing_buffers_;
    std::vector<std::shared_ptr<nav2_costmap_2d::ObservationBuffer>> rejected;
    for (auto & buffer : observation_buffers_) {
      std::vector<nav2_costmap_2d::Observation> observations;
      buffer->getObservations(observations);
      if (observations.empty() || !buffer->isCurrent() || std::any_of(observations.begin(), observations.end(),
        [this](const auto & observation) {
          return rclcpp::Time(observation.cloud_->header.stamp).nanoseconds() <= cutoff_;
        })) {rejected.push_back(buffer);}
    }
    const auto remove_rejected = [&rejected](auto & buffers) {
        buffers.erase(std::remove_if(buffers.begin(), buffers.end(), [&rejected](const auto & buffer) {
          return std::find(rejected.begin(), rejected.end(), buffer) != rejected.end();
        }), buffers.end());
      };
    remove_rejected(marking_buffers_); remove_rejected(clearing_buffers_);
    ObstacleLayer::updateBounds(x, y, yaw, min_x, min_y, max_x, max_y);
    marking_buffers_ = marking; clearing_buffers_ = clearing;
    for (auto & buffer : observation_buffers_) {buffer->unlock();}
    ready_ = rejected.empty() && !observation_buffers_.empty();
    current_ = current_ && ready_;
  }

  void updateCosts(nav2_costmap_2d::Costmap2D & master, int min_i, int min_j, int max_i, int max_j) override
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    ObstacleLayer::updateCosts(master, min_i, min_j, max_i, max_j);
    current_ = current_ && (cutoff_ == 0 || ready_);
    if (ready_ && session_.transaction_phase == Status::PRIME_NEW_SESSION) {
      rm_competition_interfaces::msg::LocalizationBarrier message;
      message.header.stamp = clock_->now();
      message.producer_id = session_.producer_id;
      message.started_at = session_.started_at;
      receipt_->publish(message);
    }
  }
private:
  std::mutex state_mutex_;
  Status session_;
  int64_t cutoff_{0};
  bool ready_{false};
  rclcpp::Subscription<Status>::SharedPtr status_;
  rclcpp::Publisher<rm_competition_interfaces::msg::LocalizationBarrier>::SharedPtr receipt_;
};
}  // namespace rm_nav_config
PLUGINLIB_EXPORT_CLASS(rm_nav_config::RebaseObstacleLayer, nav2_costmap_2d::Layer)
