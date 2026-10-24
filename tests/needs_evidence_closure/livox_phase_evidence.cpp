// Compile the complete pinned production pub_handler.cpp. SDK observation
// registration is a carrier only; no SDK initialization or devices are used.
#include "comm/pub_handler.h"
#include <cassert>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>
using namespace livox_ros;
static LivoxLidarPointCloudObserver observer = nullptr;
static void* context = nullptr;
extern "C" uint16_t LivoxLidarAddPointCloudObserver(LivoxLidarPointCloudObserver cb, void* ctx) {
  observer = cb; context = ctx; return 1;
}
extern "C" void LivoxLidarRemovePointCloudObserver(uint16_t) { observer = nullptr; context = nullptr; }
static void packet(uint64_t stamp, uint8_t time_type) {
  std::vector<uint8_t> bytes(offsetof(LivoxLidarEthernetPacket, data) + 2 * sizeof(LivoxLidarCartesianHighRawPoint), 0);
  auto* p = reinterpret_cast<LivoxLidarEthernetPacket*>(bytes.data());
  p->length = bytes.size(); p->data_type = kLivoxLidarCartesianCoordinateHighData;
  p->time_type = time_type; p->dot_num = 2; p->time_interval = 2000;
  std::memcpy(p->timestamp, &stamp, sizeof(stamp));
  LivoxLidarCartesianHighRawPoint raw{}; raw.x = 1000;
  std::memcpy(p->data, &raw, sizeof(raw)); std::memcpy(p->data + sizeof(raw), &raw, sizeof(raw));
  assert(observer); observer(101, kLivoxLidarTypeMid360, p, context);
}
struct Observation {
  std::mutex mutex; std::condition_variable cv;
  size_t callbacks = 0, points = 0, largest_frame = 0;
};
int main() {
  for (int rate : {10, 50}) for (int phase = 0; phase < 10; ++phase) {
    Observation o; PubHandler handler;
    handler.SetPointCloudsCallback([](PointFrame* frame, void* ptr) {
      auto& o = *static_cast<Observation*>(ptr);
      assert(frame->lidar_num == 1);
      const auto& lidar = frame->lidar_point[0];
      assert(lidar.handle == 101 && lidar.points_num > 0);
      for (size_t i = 0; i < lidar.points_num; ++i) assert(lidar.points[i].x == 1);
      std::lock_guard<std::mutex> lock(o.mutex);
      ++o.callbacks; o.points += lidar.points_num;
      o.largest_frame = std::max(o.largest_frame, size_t(lidar.points_num)); o.cv.notify_all();
    }, &o);
    handler.SetPointCloudConfig(rate);
    // Two seconds of source time, deliberately accelerated; not physical cadence.
    for (uint64_t k = 0; k < 200; ++k)
      packet(1000000000ULL + phase * 1000000ULL + k * 10000000ULL, kTimestampTypeGptpOrPtp);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    size_t before, points_before;
    {
      std::lock_guard<std::mutex> lock(o.mutex); before = o.callbacks; points_before = o.points;
      if (phase == 0) assert(before > 0 && points_before > 0);
      else assert(before == 0 && points_before == 0);
    }
    // An aligned packet is also a completion barrier: all 402 points must emerge.
    packet(3200000000ULL, kTimestampTypeGptpOrPtp);
    {
      std::unique_lock<std::mutex> lock(o.mutex);
      assert(o.cv.wait_for(lock, std::chrono::seconds(2), [&] { return o.points == 402; }));
    }
    handler.Uninit();
    assert(o.points == 402);
    if (phase != 0) assert(o.callbacks == 1 && o.largest_frame == 402);
    std::cout << "EVIDENCE {\"rate\":" << rate << ",\"phase_ms\":" << phase
      << ",\"callbacks_before_aligned_packet\":" << before << ",\"published_points_before\":" << points_before
      << ",\"total_points\":" << o.points << ",\"largest_frame\":" << o.largest_frame << "}" << std::endl;
  }
}
