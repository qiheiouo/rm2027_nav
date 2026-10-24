// Test-only access; actual callbacks, preprocessing, sync and FOV methods.
#define main inherited_imu_fixture_main
#include "../fastlio_deterministic/imu_fixture.cpp"
#undef main
#define main inherited_backend_main
#include BUNDLE_VISIBLE
#undef main
#include <sys/resource.h>

livox_ros_driver2::msg::CustomMsg::UniquePtr cloud(double t) {
  auto msg = std::make_unique<livox_ros_driver2::msg::CustomMsg>();
  stamp(msg->header.stamp, t);
  msg->timebase = llround(t * 1e9);
  for (unsigned i = 0; i < 900; ++i) {
    livox_ros_driver2::msg::CustomPoint p;
    p.x = 5; p.y = -3 + (i % 30) * .2; p.z = -3 + (i / 30) * .2;
    p.offset_time = i * 100000; p.line = i % 4; p.tag = 0; p.reflectivity = 50;
    msg->points.push_back(p);
  }
  msg->point_num = msg->points.size();
  return msg;
}
void imu_sample(double t) {
  auto msg = std::make_unique<sensor_msgs::msg::Imu>();
  stamp(msg->header.stamp, t); msg->linear_acceleration.z = 9.81;
  imu_cbk(std::move(msg));
}
void checkpoint(const char* mode, unsigned injected) {
  rusage usage{}; getrusage(RUSAGE_SELF, &usage);
  size_t points = 0; for (const auto& c : lidar_buffer) points += c->size();
  std::cout << "EVIDENCE {\"mode\":\"" << mode << "\",\"injected\":" << injected
    << ",\"imu_queue\":" << imu_buffer.size() << ",\"cloud_queue\":" << lidar_buffer.size()
    << ",\"retained_points\":" << points << ",\"max_rss_kib\":" << usage.ru_maxrss << "}" << std::endl;
}
int main(int argc, char** argv) {
  assert(argc >= 2); const std::string mode = argv[1];
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LaserMappingBundleNode>();
  if (mode == "fov") {
    // This is a conditional exclusion proof, not an enforced displacement fence.
    assert(cube_len == 1000 && DET_RANGE == 100);
    const double threshold = cube_len / 2 - MOV_THRESHOLD * DET_RANGE;
    assert(threshold == 350);
    unsigned checks = 0;
    for (int axis = 0; axis < 3; ++axis) for (int sign : {-1, 1}) {
      Localmap_Initialized = false; pos_lid.setZero(); lasermap_fov_segment();
      pos_lid(axis) = sign * (threshold - .01); lasermap_fov_segment();
      assert(cub_needrm.empty());
      pos_lid(axis) = sign * threshold; lasermap_fov_segment();
      assert(cub_needrm.size() == 1); ++checks;
    }
    std::cout << "EVIDENCE {\"mode\":\"fov\",\"boundary_m\":350,\"directions\":" << checks
      << ",\"inside_delete_calls\":0,\"boundary_boxes_per_direction\":1}" << std::endl;
  } else {
    if (mode == "dual") multi_lidar = true;
    else assert(mode == "imu" || mode == "lidar");
    // Positive control through the same production preprocessing and sync path.
    livox_pcl_cbk(cloud(100)); imu_sample(100); imu_sample(100.1);
    if (mode != "dual") assert(sync_packages(Measures));
    else assert(!sync_packages(Measures));
    const auto initial_imu = imu_buffer.size(), initial_cloud = lidar_buffer.size();
    checkpoint(mode.c_str(), 0);
    const unsigned count = mode == "imu" ? 20000 : 200;
    for (unsigned k = 1; k <= count; ++k) {
      if (mode == "imu") imu_sample(100.1 + k * .002);
      else {
        livox_pcl_cbk(cloud(100.1 + k * .1));
        if (mode == "dual") imu_sample(100.1 + k * .1);
      }
      assert(!sync_packages(Measures));
      assert(!input_epoch_blocked);
      if (k == count / 2 || k == count) checkpoint(mode.c_str(), k);
    }
    if (mode == "imu") assert(imu_buffer.size() == initial_imu + count);
    else assert(lidar_buffer.size() == initial_cloud + count);
    assert(lidar_buffer.size() == time_buffer.size());
  }
  node.reset(); rclcpp::shutdown();
}
