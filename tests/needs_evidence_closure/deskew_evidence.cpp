// Real current ImuProcess and ESKF, with an analytic constant-velocity oracle.
#define main inherited_imu_fixture_main
#include "../fastlio_deterministic/imu_fixture.cpp"
#undef main
int main() {
  unsigned failures = 0, controls = 0;
  for (bool dual : {false, true}) for (double speed : {0., .1})
    for (double earliest_ms : {0., 5., 25., 50., 75., 100.}) {
      Filter filter; initialize(filter);
      auto state = filter.get_x();
      state.pos.setZero(); state.vel = V3D(speed, 0, 0);
      // The manifold's default S2 direction is not an initialized gravity state.
      state.grav = S2(V3D(0, 0, -G_m_s2)); filter.change_x(state);
      ImuProcess process; configure(process);
      process.mean_acc = V3D(0, 0, G_m_s2);
      process.last_lidar_end_time_ = 100.;
      auto sample = [](double t) {
        auto msg = std::make_shared<sensor_msgs::msg::Imu>();
        stamp(msg->header.stamp, t); msg->linear_acceleration.z = G_m_s2; return msg;
      };
      process.last_imu_ = sample(100.);
      MeasureGroup m; m.lidar_beg_time = m.lidar_beg_time2 = 100.;
      m.lidar_end_time = m.lidar_end_time2 = 100.1;
      m.imu.push_back(sample(100.05)); m.imu.push_back(sample(100.1));
      m.lidar.reset(new PointCloudXYZI); m.lidar2.reset(new PointCloudXYZI);
      for (double offset : {earliest_ms, 100.}) {
        PointType p{}; p.x = 1; p.curvature = offset; m.lidar->push_back(p);
        p.x = 2; m.lidar2->push_back(p);
      }
      PointCloudXYZI out;
      if (dual) process.UndistortPclMultiLiDAR(m, filter, out);
      else process.UndistortPcl(m, filter, out, 1);
      assert(out.size() == (dual ? 4 : 2));
      assert(std::abs(filter.get_x().pos.x() - speed * .1) < 1e-7);
      double max_error = 0;
      for (size_t i = 0; i < out.size(); ++i) {
        const double expected = (i < 2 ? 1 : 2) + speed * ((i % 2 == 0 ? earliest_ms : 100.) / 1000 - .1);
        max_error = std::max(max_error, std::abs(out[i].x - expected));
      }
      // 100.05-100 is slightly below .05 as a double: the exact 50 ms
      // boundary also revisits begin(), then applies the translation twice.
      const bool known_failure = speed != 0 &&
        (earliest_ms == 0 || earliest_ms == 50 || earliest_ms == 75);
      std::cout << "EVIDENCE {\"dual\":" << dual << ",\"speed\":" << speed
        << ",\"earliest_ms\":" << earliest_ms << ",\"max_error_m\":" << max_error
        << ",\"known_error\":" << known_failure << "}" << std::endl;
      assert((max_error > 1e-6) == known_failure);
      if (known_failure) {
        const double expected_error = earliest_ms == 0 ? .01 :
          (earliest_ms == 50 ? .005 : .0025);
        assert(std::abs(max_error - expected_error) < 1e-6);
      }
      failures += known_failure; controls += !known_failure;
    }
  assert(failures == 6 && controls == 18);
}
