// Real production ImuProcess + ESKF; independent constant-world-velocity/yaw oracle.
#define main inherited_imu_fixture_main
#include "../fastlio_deterministic/imu_fixture.cpp"
#undef main
unsigned cases = 0;
void check(int mode, double speed, double yaw_rate, std::vector<double> offsets) {
  Filter filter; initialize(filter);
  auto state = filter.get_x();
  state.pos.setZero(); state.vel = V3D(speed, 0, 0);
  state.grav = S2(V3D(0, 0, -G_m_s2));
  // S2 fixes |g| at 9.809 while preprocessing scales acceleration to 9.81.
  // Set the known vertical bias so the full 3D constant-velocity oracle is exact.
  state.ba = V3D(0, 0, G_m_s2 + state.grav[2]);
  filter.change_x(state);
  ImuProcess process; configure(process);
  process.mean_acc = V3D(0, 0, G_m_s2);
  process.last_lidar_end_time_ = 100.;
  auto sample = [yaw_rate](double t) {
    auto msg = std::make_shared<sensor_msgs::msg::Imu>();
    stamp(msg->header.stamp, t); msg->linear_acceleration.z = G_m_s2;
    msg->angular_velocity.z = yaw_rate; return msg;
  };
  process.last_imu_ = sample(100.);
  MeasureGroup m; m.lidar_beg_time = m.lidar_beg_time2 = 100.;
  m.lidar_end_time = m.lidar_end_time2 = 100.1;
  m.imu.push_back(sample(100.05)); m.imu.push_back(sample(100.1));
  m.lidar.reset(new PointCloudXYZI); m.lidar2.reset(new PointCloudXYZI);
  for (double offset : offsets) {
    PointType p{}; p.x = 1; p.curvature = offset; m.lidar->push_back(p);
    p.x = 2; m.lidar2->push_back(p);
  }
  PointCloudXYZI out;
  if (mode == 0) process.UndistortPclMultiLiDAR(m, filter, out);
  else process.UndistortPcl(m, filter, out, mode);
  assert(out.size() == (mode == 0 ? 2 : 1) * offsets.size());
  assert(std::abs(filter.get_x().pos.x() - speed * .1) < 1e-7);
  assert((filter.get_x().rot.toRotationMatrix() -
    Eigen::AngleAxisd(yaw_rate * .1, V3D::UnitZ()).toRotationMatrix()).norm() < 1e-7);
  std::sort(offsets.begin(), offsets.end());
  double max_error = 0;
  for (size_t i = 0; i < out.size(); ++i) {
    const double t = offsets[i % offsets.size()] / 1000.;
    const double source_x = (mode == 0 && i >= offsets.size()) ? 2 : 1;
    const V3D expected = Eigen::AngleAxisd(-yaw_rate * .1, V3D::UnitZ()) *
      (Eigen::AngleAxisd(yaw_rate * t, V3D::UnitZ()) * V3D(source_x, 0, 0) +
       V3D(speed * (t - .1), 0, 0));
    const V3D actual(out[i].x, out[i].y, out[i].z);
    max_error = std::max(max_error, (actual - expected).norm());
  }
  std::cout << "REGRESSION mode=" << mode << " speed=" << speed << " yaw=" << yaw_rate
    << " points=" << offsets.size() << " max_error_m=" << max_error << std::endl;
  assert(max_error < 1e-6);
  ++cases;
}
int main() {
  // Original 24-case closure grid, plus the second single-lidar entry.
  for (int mode : {0, 1, 2}) for (double speed : {0., .1})
    for (double first : {0., 5., 25., 50., 75., 100.}) check(mode, speed, 0, {first, 100.});
  for (int mode : {0, 1, 2}) {
    check(mode, .1, 0, {0.});
    check(mode, .1, 0, {75.});
    check(mode, .1, .4, {0., 50., 75., 100.});
    check(mode, .1, 0, {100., 0., 0., 50., 75.});
    check(mode, .1, 0, {});
  }
  assert(cases == 51);
}
