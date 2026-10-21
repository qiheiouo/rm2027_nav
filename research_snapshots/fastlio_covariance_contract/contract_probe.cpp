// Probe actual MTK state/ESKF, ROS messages, and source-extracted publishers.
#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <omp.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "use-ikfom.hpp"
#ifdef TEST_CANDIDATE
#include "pose_covariance.hpp"
#endif

using Matrix6 = Eigen::Matrix<double, 6, 6>;
using Filter = esekfom::esekf<state_ikfom, 12, input_ikfom>;
Filter kf;
state_ikfom state_point;
std::string map_frame = "probe_parent";
double publish_lidar_time = 1.0, lidar_end_time = 1.0;
geometry_msgs::msg::Quaternion geoQuat;
bool zero_start_pose = false, zero_pose_initialized = false;
int zero_start_delay_scans = 2, processed_scans_for_zero = 0;
Eigen::Matrix4d T_world_to_zero = Eigen::Matrix4d::Identity();
Eigen::Vector3d pub_pos = Eigen::Vector3d::Zero();
Eigen::Quaterniond pub_quat = Eigen::Quaterniond::Identity();
#undef RCLCPP_INFO
#define RCLCPP_INFO(...) ((void)0)

builtin_interfaces::msg::Time get_ros_time(double value) {
  builtin_interfaces::msg::Time out;
  out.sec = static_cast<int>(value);
  out.nanosec = static_cast<unsigned>((value - out.sec) * 1e9);
  return out;
}
struct CapturePublisher {
  std::vector<nav_msgs::msg::Odometry> copies;
  void publish(const nav_msgs::msg::Odometry &msg) { copies.push_back(msg); }
};
struct CaptureBroadcaster {
  std::vector<geometry_msgs::msg::TransformStamped> copies;
  void sendTransform(const geometry_msgs::msg::TransformStamped &msg) { copies.push_back(msg); }
};
#include "old_publishers.inc"
#ifdef TEST_CANDIDATE
#include "candidate_publishers.inc"
#endif

void require(bool condition, const std::string &name) {
  if (!condition) throw std::runtime_error(name);
}
template<class A, class B>
double difference(const A &a, const B &b) { return (a - b).cwiseAbs().maxCoeff(); }
template<class A, class B>
void near(const A &a, const B &b, double tolerance, const std::string &name) {
  require(difference(a, b) < tolerance, name);
}
Matrix6 matrix(const std::array<double, 36> &data) {
  return Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(data.data());
}
Eigen::Matrix3d rotation(const geometry_msgs::msg::Quaternion &q) {
  return Eigen::Quaterniond(q.w, q.x, q.y, q.z).toRotationMatrix();
}
Eigen::Vector3d log_rotation(const Eigen::Matrix3d &R) {
  Eigen::AngleAxisd angle(R);
  return angle.axis() * angle.angle();
}
state_ikfom state(const Eigen::Matrix3d &R, const Eigen::Vector3d &p) {
  state_ikfom out;
  out.pos = p;
  out.rot = SO3(R);
  return out;
}
void configure(const state_ikfom &x, const Matrix6 &P, double timestamp = 1.0) {
  auto full = Filter::cov::Identity().eval();
  full.block<6,6>(0,0) = P;
  kf.change_P(full);
  auto current = x;
  kf.change_x(current);
  state_point = kf.get_x();
  pub_pos = x.pos;
  pub_quat = Eigen::Quaterniond(x.rot.toRotationMatrix());
  geoQuat.x = pub_quat.x(); geoQuat.y = pub_quat.y();
  geoQuat.z = pub_quat.z(); geoQuat.w = pub_quat.w();
  publish_lidar_time = lidar_end_time = timestamp;
}
Matrix6 diagonal() {
  Matrix6 P = Matrix6::Zero();
  P.diagonal() << .09, .01, .04, .0001, .0004, .0009;
  return P;
}
Matrix6 dense() {
  Matrix6 L;
  L << 1,0,0,0,0,0, 2,3,0,0,0,0, -1,4,5,0,0,0,
       6,-2,1,7,0,0, 3,8,-4,2,9,0, -5,1,3,4,-2,11;
  return .001 * L * L.transpose();
}
Matrix6 numerical_jacobian(const state_ikfom &x, const Eigen::Matrix4d &G) {
  // Perturb the actual 23-DOF state using its real boxplus, not a mirror.
  // Output error: additive position and left infinitesimal fixed-parent axes.
  Matrix6 J;
  const Eigen::Matrix3d A = G.block<3,3>(0,0);
  const Eigen::Matrix3d nominal = A * x.rot.toRotationMatrix();
  for (int i = 0; i < 6; ++i) {
    Eigen::Matrix<double, state_ikfom::DOF, 1> d;
    d.setZero(); d(i) = 1e-6;
    auto plus = x, minus = x;
    plus.boxplus(d); d *= -1.; minus.boxplus(d);
    Eigen::Matrix<double,6,1> yp, ym;
    yp.head<3>() = A * (plus.pos - x.pos);
    ym.head<3>() = A * (minus.pos - x.pos);
    yp.tail<3>() = log_rotation(A * plus.rot.toRotationMatrix() * nominal.transpose());
    ym.tail<3>() = log_rotation(A * minus.rot.toRotationMatrix() * nominal.transpose());
    J.col(i) = (yp - ym) / 2e-6;
  }
  return J;
}
void diagonal_json(const Matrix6 &P) {
  std::cout << '[';
  for (int i=0;i<6;++i) { if (i) std::cout << ','; std::cout << P(i,i); }
  std::cout << ']';
}
using Publish = void (*)(const std::shared_ptr<CapturePublisher>&, std::unique_ptr<CaptureBroadcaster>&);

void baseline_witness(Publish publish, nav_msgs::msg::Odometry &buffer, const char *mode) {
  auto publisher = std::make_shared<CapturePublisher>();
  auto broadcaster = std::unique_ptr<CaptureBroadcaster>(new CaptureBroadcaster);
  buffer = nav_msgs::msg::Odometry();
  configure(state(Eigen::Matrix3d::Identity(), Eigen::Vector3d(1,2,3)), diagonal());
  publish(publisher, broadcaster);
  configure(state(Eigen::Matrix3d::Identity(), Eigen::Vector3d(4,5,6)), 10*diagonal(), 2.);
  publish(publisher, broadcaster);
  const auto first = matrix(publisher->copies[0].pose.covariance);
  const auto second = matrix(publisher->copies[1].pose.covariance);
  Matrix6 swapped = Matrix6::Zero();
  swapped.diagonal() << .0001,.0004,.0009,.09,.01,.04;
  near(first, Matrix6::Zero(), 1e-15, "legacy first default covariance");
  near(second, swapped, 1e-15, "legacy second lagged/swapped covariance");
  require(publisher->copies[1].pose.pose.position.x == 4., "legacy current pose");
  std::cout << "{\"mode\":\"" << mode << "\",\"first_published_diagonal\":";
  diagonal_json(first);
  std::cout << ",\"second_published_diagonal\":";
  diagonal_json(second);
  std::cout << ",\"current_buffer_diagonal\":";
  diagonal_json(matrix(buffer.pose.covariance));
  std::cout << '}';
}

void baseline_zero_witness() {
  old_bundle::odomAftMapped = nav_msgs::msg::Odometry();
  auto publisher = std::make_shared<CapturePublisher>();
  auto broadcaster = std::unique_ptr<CaptureBroadcaster>(new CaptureBroadcaster);
  zero_start_pose = true; zero_pose_initialized = false;
  processed_scans_for_zero = 0; zero_start_delay_scans = 1;
  T_world_to_zero.setIdentity();
  const auto R = Eigen::AngleAxisd(M_PI/2,Eigen::Vector3d::UnitZ()).toRotationMatrix();
  configure(state(R, Eigen::Vector3d(2,-3,1)), diagonal());
  old_zero_pose_update();
  old_bundle::publish_odometry(publisher, broadcaster);
  old_bundle::publish_odometry(publisher, broadcaster);
  // Eliminate the known delay and undo only the known index permutation.
  // The remaining parent-basis error must still be visible independently.
  const auto published = matrix(publisher->copies.back().pose.covariance);
  Matrix6 unswapped;
  for(int i=0;i<6;++i) for(int j=0;j<6;++j)
    unswapped(i,j) = published((i+3)%6,(j+3)%6);
  Matrix6 expected = diagonal(); expected(0,0) = .01; expected(1,1) = .09;
  near(unswapped, diagonal(), 1e-14, "legacy unpropagated native covariance");
  require(difference(unswapped, expected) > .07, "basis error after undoing lag and swap");
  std::cout << ",\"legacy_zero_start\":{\"lag_removed_and_indices_unswapped_diagonal\":";
  diagonal_json(unswapped);
  std::cout << ",\"expected_conditioned_basis_diagonal\":";
  diagonal_json(expected);
  std::cout << '}';
  zero_start_pose = false; zero_pose_initialized = false;
}

#ifdef TEST_CANDIDATE
std::vector<std::string> passed;
double maximum_fd_covariance_error = 0.;
void candidate_witness(Publish publish, nav_msgs::msg::Odometry &buffer, const char *mode) {
  auto publisher = std::make_shared<CapturePublisher>();
  auto broadcaster = std::unique_ptr<CaptureBroadcaster>(new CaptureBroadcaster);
  buffer = nav_msgs::msg::Odometry();
  const auto x = state(Eigen::Matrix3d::Identity(), Eigen::Vector3d(1,2,3));
  configure(x, diagonal());
  publish(publisher, broadcaster);
  near(matrix(publisher->copies.back().pose.covariance), diagonal(), 1e-14, "first covariance");
  passed.push_back(std::string(mode) + ":first_frame_known_diagonal");
  const auto R = (Eigen::AngleAxisd(.7,Eigen::Vector3d::UnitZ()) *
                  Eigen::AngleAxisd(-.4,Eigen::Vector3d::UnitY()) *
                  Eigen::AngleAxisd(.3,Eigen::Vector3d::UnitX())).toRotationMatrix();
  const auto next = state(R, Eigen::Vector3d(-2,4,.5));
  configure(next, dense(), 2.);
  publish(publisher, broadcaster);
  const auto J = numerical_jacobian(next, Eigen::Matrix4d::Identity());
  const auto expected = (J*dense()*J.transpose()).eval();
  const auto &out = publisher->copies.back();
  const double error = difference(matrix(out.pose.covariance), expected);
  maximum_fd_covariance_error = std::max(maximum_fd_covariance_error, error);
  require(error < 1e-9, "rotated dense covariance incl cross blocks");
  near(rotation(out.pose.pose.orientation), R, 1e-14, "consecutive current orientation");
  require(out.pose.pose.position.x == -2. && out.header.stamp.sec == 2, "current pose/time");
  const auto &tf = broadcaster->copies.back();
  require(out.header.frame_id == map_frame && out.child_frame_id == "body", "frame identifiers");
  require(tf.header.stamp == out.header.stamp && tf.child_frame_id == out.child_frame_id &&
          tf.header.frame_id == out.header.frame_id, "TF frame/time consistency");
  near(rotation(tf.transform.rotation), R, 1e-14, "TF orientation consistency");
  require(tf.transform.translation.x == -2., "TF position consistency");
  passed.push_back(std::string(mode) + ":consecutive_updates_dense_rotated_body_tf");
}

void transformation_checks() {
  const auto R = (Eigen::AngleAxisd(.6,Eigen::Vector3d::UnitX()) *
                  Eigen::AngleAxisd(-.9,Eigen::Vector3d::UnitZ())).toRotationMatrix();
  const auto x = state(R, Eigen::Vector3d(3,-4,.2));
  Eigen::Matrix4d G = Eigen::Matrix4d::Identity();
  G.block<3,3>(0,0) = (Eigen::AngleAxisd(-.5,Eigen::Vector3d::UnitY()) *
                       Eigen::AngleAxisd(1.1,Eigen::Vector3d::UnitZ())).toRotationMatrix();
  G.block<3,1>(0,3) = Eigen::Vector3d(10,20,-30);
  for (const auto &P : {diagonal(), dense()}) {
    const auto J = numerical_jacobian(x, G);
    const auto candidate = matrix(fast_lio::native_pose_covariance(P, G.block<3,3>(0,0), G.block<3,3>(0,0)*R));
    maximum_fd_covariance_error = std::max(maximum_fd_covariance_error, difference(candidate, J*P*J.transpose()));
    near(candidate, J*P*J.transpose(), 1e-9, "parent rotation finite difference");
    const auto world = fast_lio::native_pose_covariance(P, Eigen::Matrix3d::Identity(), R);
    tf2::Transform tf;
    tf.setOrigin(tf2::Vector3(10,20,-30));
    const auto A = G.block<3,3>(0,0);
    tf.setBasis(tf2::Matrix3x3(A(0,0),A(0,1),A(0,2),A(1,0),A(1,1),A(1,2),A(2,0),A(2,1),A(2,2)));
    near(candidate, matrix(tf2::transformCovariance(world, tf)), 1e-14, "official tf2 parent propagation");
    near(candidate, candidate.transpose(), 1e-14, "symmetry preserved");
    Eigen::SelfAdjointEigenSolver<Matrix6> eigenvalues(candidate);
    require(eigenvalues.eigenvalues().minCoeff() > 0., "positive covariance preserved");
    const auto identity = matrix(fast_lio::native_pose_covariance(P, Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Identity()));
    near(identity, P, 1e-14, "identity chart and row major");
  }
  passed.push_back("parent_rotation_diagonal_and_dense_actual_boxplus_fd");
  passed.push_back("official_ros_tf2_transform_covariance_crosscheck");
  passed.push_back("identity_basis_all_36_fields_symmetry_positive_definite");
  // Re-express native dp under a changed numerical world; right body dphi stays unchanged.
  Matrix6 K = Matrix6::Identity(); K.block<3,3>(0,0) = G.block<3,3>(0,0);
  const Matrix6 changed = K*dense()*K.transpose();
  const auto direct = matrix(fast_lio::native_pose_covariance(dense(), G.block<3,3>(0,0), G.block<3,3>(0,0)*R));
  const auto via_changed_world = matrix(fast_lio::native_pose_covariance(changed, Eigen::Matrix3d::Identity(), G.block<3,3>(0,0)*R));
  near(direct, via_changed_world, 1e-14, "numerical world basis change");
  passed.push_back("native_numerical_basis_reexpression");
}

void zero_start_checks() {
  old_bundle::odomAftMapped = nav_msgs::msg::Odometry();
  candidate_bundle::odomAftMapped = nav_msgs::msg::Odometry();
  auto old_pub = std::make_shared<CapturePublisher>();
  auto new_pub = std::make_shared<CapturePublisher>();
  auto br = std::unique_ptr<CaptureBroadcaster>(new CaptureBroadcaster);
  zero_start_pose = true; zero_pose_initialized = false;
  processed_scans_for_zero = 0; zero_start_delay_scans = 2;
  T_world_to_zero.setIdentity();
  const Eigen::Matrix3d R0 = Eigen::AngleAxisd(M_PI/2,Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const auto anchor = state(R0, Eigen::Vector3d(2,-3,1));
  configure(anchor, diagonal());
  candidate_zero_pose_update();
  candidate_bundle::publish_odometry(new_pub, br);
  const auto J0 = numerical_jacobian(anchor, Eigen::Matrix4d::Identity());
  near(matrix(new_pub->copies.back().pose.covariance), J0*diagonal()*J0.transpose(), 1e-9, "before zero initialization");
  passed.push_back("zero_start_before_initialization_identity_parent");
  configure(anchor, diagonal(), 2.);
  candidate_zero_pose_update();
  require(zero_pose_initialized, "zero initialized at configured scan");
  candidate_bundle::publish_odometry(new_pub, br);
  near(pub_pos, Eigen::Vector3d::Zero(), 1e-14, "zero anchor position");
  near(pub_quat.toRotationMatrix(), Eigen::Matrix3d::Identity(), 1e-14, "zero anchor rotation");
  Matrix6 anchor_expected = diagonal();
  anchor_expected(0,0) = .01; anchor_expected(1,1) = .09;
  near(matrix(new_pub->copies.back().pose.covariance), anchor_expected, 1e-14, "zero anchor basis, nonzero conditioned P");
  // Prime the legacy message then publish the same epoch/basis twice to expose
  // missing propagation independently of its delay and index permutation.
  old_bundle::publish_odometry(old_pub, br);
  old_bundle::publish_odometry(old_pub, br);
  require(difference(matrix(old_pub->copies.back().pose.covariance), anchor_expected) > .01,
          "legacy zero-start fails conditioned publication contract");
  passed.push_back("zero_start_activation_known_90deg_anchor_nonzero_covariance");
  const auto next = state(R0 * Eigen::AngleAxisd(.4,Eigen::Vector3d::UnitX()).toRotationMatrix(), Eigen::Vector3d(5,-1,.5));
  configure(next, dense(), 3.);
  candidate_zero_pose_update();
  candidate_bundle::publish_odometry(new_pub, br);
  const auto J = numerical_jacobian(next, T_world_to_zero);
  const auto error = difference(matrix(new_pub->copies.back().pose.covariance), J*dense()*J.transpose());
  maximum_fd_covariance_error = std::max(maximum_fd_covariance_error, error);
  require(error < 1e-9, "zero-start subsequent dense covariance");
  passed.push_back("zero_start_consecutive_dense_rotated_body_fd");
  // Disabled flag must dominate an initialized/stale transform.
  zero_start_pose = false;
  configure(next, dense(), 4.);
  candidate_zero_pose_update();
  candidate_bundle::publish_odometry(new_pub, br);
  const auto Ji = numerical_jacobian(next, Eigen::Matrix4d::Identity());
  near(matrix(new_pub->copies.back().pose.covariance), Ji*dense()*Ji.transpose(), 1e-9, "disabled zero-start ignores stored basis");
  passed.push_back("zero_start_disabled_ignores_stored_rotation");
}
#endif

int main() {
  try {
    require(state_ikfom::DOF == 23 && state_point.pos.IDX == 0 && state_point.rot.IDX == 3,
            "native state layout");
    require(kf.get_P().rows() == 23 && kf.get_P().cols() == 23, "native covariance dimension");
    const auto R = Eigen::AngleAxisd(.8,Eigen::Vector3d::UnitY()).toRotationMatrix();
    auto x = state(R, Eigen::Vector3d(1,2,3));
    Eigen::Matrix<double,23,1> d = Eigen::Matrix<double,23,1>::Zero();
    d.head<3>() << .1,.2,.3; d.tail<3>().setZero(); d(3) = .02;
    auto perturbed = x; perturbed.boxplus(d);
    near(perturbed.pos - x.pos, d.head<3>(), 1e-14, "additive world position");
    near(perturbed.rot.toRotationMatrix(), R*Eigen::AngleAxisd(.02,Eigen::Vector3d::UnitX()).toRotationMatrix(), 1e-14, "right body rotation");
    std::cout << std::setprecision(17) << "{\"native_layout\":{\"dof\":23,\"position_index\":0,\"rotation_index\":3,\"covariance_rows\":23,\"chart\":\"additive world dp; right body dphi\"},\"legacy\":[";
    baseline_witness(old_bundle::publish_odometry, old_bundle::odomAftMapped, "bundle"); std::cout << ',';
    baseline_witness(old_adaptive::publish_odometry, old_adaptive::odomAftMapped, "adaptive"); std::cout << ',';
    baseline_witness(old_async::publish_odometry, old_async::odomAftMapped, "async");
    std::cout << ']';
    baseline_zero_witness();
#ifdef TEST_CANDIDATE
    zero_start_pose = false; zero_pose_initialized = false;
    candidate_witness(candidate_bundle::publish_odometry, candidate_bundle::odomAftMapped, "bundle");
    candidate_witness(candidate_adaptive::publish_odometry, candidate_adaptive::odomAftMapped, "adaptive");
    candidate_witness(candidate_async::publish_odometry, candidate_async::odomAftMapped, "async");
    transformation_checks(); zero_start_checks();
    std::cout << ",\"candidate\":{\"maximum_fd_covariance_error\":" << maximum_fd_covariance_error << ",\"passed\":[";
    for (std::size_t i=0;i<passed.size();++i) { if(i) std::cout << ','; std::cout << '\"' << passed[i] << '\"'; }
    std::cout << "]}";
#endif
    std::cout << "}\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    return 1;
  }
}
