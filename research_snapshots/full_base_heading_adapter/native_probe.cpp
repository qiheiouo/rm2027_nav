// Compile with the UNCHANGED canonical_odometry.cpp; no ROS runtime.
#include <iomanip>
#include <iostream>
#include "rm_localization_adapters/canonical_odometry.hpp"

tf2::Transform read_transform()
{
  double a[16];
  for (double & v : a) {std::cin >> v;}
  return tf2::Transform(
    tf2::Matrix3x3(a[0], a[1], a[2], a[4], a[5], a[6], a[8], a[9], a[10]),
    tf2::Vector3(a[3], a[7], a[11]));
}

void write_transform(const tf2::Transform & T)
{
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      if (i || j) {std::cout << ',';}
      std::cout << (i == 3 ? (j == 3 ? 1.0 : 0.0) :
        (j == 3 ? T.getOrigin()[i] : T.getBasis()[i][j]));
    }
  }
}

int main()
{
  int n;
  std::cin >> n;
  std::cout << std::setprecision(17) << '[';
  for (int i = 0; i < n; ++i) {
    auto start = read_transform(), current = read_transform(), home = read_transform();
    auto current_extrinsic = read_transform();
    double x, y, z, h0, h;
    std::cin >> x >> y >> z >> h0 >> h;
    auto result = rm_localization_adapters::compute_base_transform_from_chassis_heading(
      start, current, home, tf2::Vector3(x, y, z), h0, h);
    if (i) {std::cout << ',';}
    std::cout << "{\"pose\":[";
    write_transform(result.base_initial_to_base);
    std::cout << "],\"joint_delta_rad\":" << result.gimbal_yaw_rad;
    std::cout << ",\"sensor_initial_dynamic_pose\":[";
    write_transform(rm_localization_adapters::compute_base_transform_from_sensor_initial(
      start.inverse() * current, current_extrinsic));
    std::cout << "]}";
  }
  std::cout << ']';
}
