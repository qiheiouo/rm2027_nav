// Research-only mirror of bd6cf689 performRegistration's PCL and basic gates.
// No ROS node, TF or command publication. T quality is passive and never gates V.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>
#include <set>
#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/gicp.h>
#include <pcl/search/kdtree.h>
#include "rm_gicp_relocalization/relocalization_math.hpp"
#include "rm_gicp_relocalization/registration_quality.hpp"
#include "rm_relocalization_bridge/global_pose_gate.hpp"

using Point = pcl::PointXYZ;
using Cloud = pcl::PointCloud<Point>;

struct QualityDump {
  std::size_t nearest_found=0, fitness_inliers=0, geometric_inliers=0;
  std::set<int> unique_targets;
  std::vector<double> squared_residuals;
  Eigen::Vector3d centroid=Eigen::Vector3d::Zero();
  Eigen::Vector3d matched_min=Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector3d matched_max=-matched_min;
  Eigen::Matrix<double,6,6> information=Eigen::Matrix<double,6,6>::Zero();
};

void writeDump(const std::string & path, const QualityDump & dump,
    const rm_gicp_relocalization::RegistrationQualityMetrics & quality,
    const double cutoff, const bool attempted, const std::size_t source_count,
    const std::string & reference_reason) {
  std::ofstream out(path);
  if (!out) throw std::runtime_error("Diagnostic output cannot be opened");
  auto number=[&](double v) {if(std::isfinite(v)) out<<v; else out<<"null";};
  auto vector=[&](const Eigen::Vector3d & v) {
    out<<'[';for(int i=0;i<3;++i) {if(i) out<<',';number(v[i]);}out<<']';
  };
  out<<std::setprecision(17)<<"{\n\"registration_attempted\":"<<(attempted?"true":"false")
     <<",\n\"filtered_source_points\":"<<source_count
     <<",\n\"T_evaluated_source_points\":"<<quality.source_points
     <<",\n\"T_information_valid\":"<<(quality.information_valid?"true":"false")
     <<",\n\"T_gate_reason\":"<<std::quoted(reference_reason)
     <<",\n\"nearest_found\":"<<dump.nearest_found
     <<",\n\"fitness_inliers\":"<<dump.fitness_inliers
     <<",\n\"postfit_geometric_inliers_strict\":"<<dump.geometric_inliers
     <<",\n\"T_normal_valid_inliers\":"<<quality.inliers
     <<",\n\"unique_target_points\":"<<dump.unique_targets.size()
     <<",\n\"PCL_fitness_squared_distance_cutoff\":"<<cutoff
     <<",\n\"geometric_distance_cutoff_m\":"<<cutoff
     <<",\n\"T_distance_cutoff_m\":1.0,\n\"T_normal_K\":12,\n\"T_sample_centroid\":";
  vector(dump.centroid);out<<",\n\"matched_target_min\":";vector(dump.matched_min);
  out<<",\n\"matched_target_max\":";vector(dump.matched_max);
  out<<",\n\"T_information_average_JtJ\":[";
  for(int r=0;r<6;++r) {if(r)out<<',';out<<'[';
    for(int c=0;c<6;++c) {if(c)out<<',';number(dump.information(r,c));}out<<']';}
  out<<"],\n\"fitness_inlier_squared_residuals\":[";
  for(std::size_t i=0;i<dump.squared_residuals.size();++i) {
    if(i)out<<',';number(dump.squared_residuals[i]);}
  out<<"]\n}\n";
}

Cloud::Ptr loadFiltered(const std::string & path, const double leaf) {
  auto raw = std::make_shared<Cloud>();
  if (pcl::io::loadPCDFile<Point>(path, *raw) < 0) {
    throw std::runtime_error("PCD load failed");
  }
  auto finite = std::make_shared<Cloud>();
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*raw, *finite, indices);
  pcl::VoxelGrid<Point> filter;
  filter.setLeafSize(leaf, leaf, leaf);
  filter.setInputCloud(finite);
  auto filtered = std::make_shared<Cloud>();
  filter.filter(*filtered);
  return filtered;
}

rm_gicp_relocalization::RegistrationQualityMetrics passiveQuality(
    const Cloud::Ptr & target, const Cloud & aligned, const double cutoff,
    QualityDump & dump) {
  auto search = std::make_shared<pcl::search::KdTree<Point>>();
  search->setInputCloud(target);
  pcl::NormalEstimation<Point, pcl::Normal> estimator;
  estimator.setInputCloud(target);
  estimator.setSearchMethod(search);
  estimator.setKSearch(12);  // T default, historical diagnostics only.
  pcl::PointCloud<pcl::Normal> normals;
  estimator.compute(normals);
  std::vector<rm_gicp_relocalization::RegistrationQualitySample> samples;
  std::vector<int> indices(1);
  std::vector<float> squared(1);
  for (const auto & point : aligned) {
    if (search->nearestKSearch(point, 1, indices, squared) != 1 ||
        !std::isfinite(squared[0])) continue;
    ++dump.nearest_found;
    // PCL 1.14 fitness compares squared distance to max_range directly.
    if (squared[0] <= cutoff) {
      ++dump.fitness_inliers;dump.squared_residuals.push_back(squared[0]);
    }
    // Post-fit geometric eligibility, not the hidden last optimizer iteration.
    if (squared[0] < cutoff*cutoff) ++dump.geometric_inliers;
    if (squared[0] > 1.0f || indices[0] < 0 ||
        static_cast<std::size_t>(indices[0]) >= normals.size()) continue;
    const auto & n = normals[indices[0]];
    const double length = std::hypot(std::hypot(n.normal_x,n.normal_y),n.normal_z);
    if (!std::isfinite(length) || length <= 1e-12) continue;
    rm_gicp_relocalization::RegistrationQualitySample sample;
    sample.aligned_point = Eigen::Vector3d(point.x,point.y,point.z);
    sample.target_normal = Eigen::Vector3d(n.normal_x,n.normal_y,n.normal_z);
    samples.push_back(sample);
    dump.unique_targets.insert(indices[0]);
    const auto & matched=(*target)[indices[0]];
    const Eigen::Vector3d p(matched.x,matched.y,matched.z);
    dump.matched_min=dump.matched_min.cwiseMin(p);
    dump.matched_max=dump.matched_max.cwiseMax(p);
  }
  // Export the same centered point-to-plane approximation as historical T.
  // This does not replace PCL's covariance-weighted GICP objective or gate V.
  if (!samples.empty()) {
    for(const auto & s:samples) dump.centroid+=s.aligned_point;
    dump.centroid/=static_cast<double>(samples.size());
    for(const auto & s:samples) {
      const Eigen::Vector3d n=s.target_normal.normalized();
      Eigen::Matrix<double,1,6> j;
      j<<n.transpose(),(s.aligned_point-dump.centroid).cross(n).transpose();
      dump.information.noalias()+=j.transpose()*j;
    }
    dump.information/=static_cast<double>(samples.size());
    dump.information=(0.5*(dump.information+dump.information.transpose())).eval();
  }
  return rm_gicp_relocalization::calculateRegistrationQuality(samples, aligned.size());
}

int main(int argc, char ** argv) {
  if (argc != 19 && argc != 20 && argc != 21) {
    std::cerr << "target source seed_x seed_y seed_yaw age has_seed map_leaf source_leaf "
                 "corr_distance trans_epsilon fitness_epsilon iterations min_source "
                 "min_target max_score max_translation_jump max_yaw_jump [diagnostic.json [seed_matrix.txt]]\n";
    return 2;
  }
  const auto begin = std::chrono::steady_clock::now();
  const auto target = loadFiltered(argv[1],std::stod(argv[8]));
  const auto source = loadFiltered(argv[2],std::stod(argv[9]));
  const bool has_seed = std::stoi(argv[7]) != 0;
  Eigen::Isometry3d guess=Eigen::Isometry3d::Identity();
  guess.translation()=Eigen::Vector3d(std::stod(argv[3]),std::stod(argv[4]),0);
  guess.linear()=Eigen::AngleAxisd(std::stod(argv[5]),Eigen::Vector3d::UnitZ()).toRotationMatrix();
  // L2 research may provide a coarse full SE(3) seed. Legacy CLI is unchanged.
  if (argc==21) {
    std::ifstream seed(argv[20]);
    for(int r=0;r<4;++r) for(int c=0;c<4;++c) {
      if (!(seed>>guess.matrix()(r,c))) throw std::runtime_error("Invalid seed matrix");
    }
    if (!guess.matrix().allFinite() ||
        !guess.matrix().row(3).isApprox(Eigen::RowVector4d(0,0,0,1),1e-8) ||
        !(guess.linear().transpose()*guess.linear()).isApprox(Eigen::Matrix3d::Identity(),1e-5) ||
        std::abs(guess.linear().determinant()-1)>1e-5)
      throw std::runtime_error("Seed must be a rigid SE(3) transform");
  }
  // argv[6] is case-specific source timestamp age, used by the real pose gate.
  const double age=std::stod(argv[6]);
  Eigen::Isometry3d result=guess;
  Cloud aligned;
  bool converged=false, accept=false, pose_gate=false;
  double score=std::numeric_limits<double>::quiet_NaN();
  double solve_ms=0, translation_jump=0, yaw_jump=0;
  std::string reason="no_seed_no_registration";
  rm_gicp_relocalization::RegistrationQualityMetrics quality;
  QualityDump dump;
  std::string reference_reason;
  bool reference_gate=false;
  if (has_seed && target->size() < static_cast<std::size_t>(std::stoi(argv[15]))) {
    reason="invalid_target";
  } else if (has_seed && source->size() < static_cast<std::size_t>(std::stoi(argv[14]))) {
    reason="too_few_source_points";
  } else if (has_seed) {
    pcl::GeneralizedIterativeClosestPoint<Point,Point> registration;
    registration.setInputTarget(target);
    registration.setInputSource(source);
    registration.setMaxCorrespondenceDistance(std::stod(argv[10]));
    registration.setTransformationEpsilon(std::stod(argv[11]));
    registration.setEuclideanFitnessEpsilon(std::stod(argv[12]));
    registration.setMaximumIterations(std::stoi(argv[13]));
    const auto t0=std::chrono::steady_clock::now();
    registration.align(aligned,guess.matrix().cast<float>());
    solve_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-t0).count();
    score=registration.getFitnessScore(std::stod(argv[10]));
    converged=registration.hasConverged();
    result.matrix()=registration.getFinalTransformation().cast<double>();
    translation_jump=(result.translation()-guess.translation()).norm();
    yaw_jump=std::abs(rm_gicp_relocalization::planarYawDifference(result,guess));
    reason="accepted";
    if (!converged || !std::isfinite(score) || score>std::stod(argv[16])) {
      reason="convergence_or_fitness";
    } else if (!result.matrix().allFinite()) {
      reason="non_finite";
    } else if (translation_jump>std::stod(argv[17]) || yaw_jump>std::stod(argv[18])) {
      reason="correction_jump";
    }
    accept=reason=="accepted";
    quality=passiveQuality(target,aligned,std::stod(argv[10]),dump);
    rm_gicp_relocalization::RegistrationQualityThresholds thresholds;
    thresholds.min_overlap_ratio=0.30;
    thresholds.min_information_eigenvalue=0.001;
    thresholds.max_information_condition_number=1e5;
    reference_gate=rm_gicp_relocalization::passesRegistrationQuality(quality,thresholds,reference_reason);
    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header.frame_id="map";
    pose.header.stamp.sec=100;
    pose.pose.pose.position.x=result.translation().x();
    pose.pose.pose.position.y=result.translation().y();
    pose.pose.pose.position.z=result.translation().z();
    const Eigen::Quaterniond q(result.linear());
    pose.pose.pose.orientation.x=q.x();pose.pose.pose.orientation.y=q.y();
    pose.pose.pose.orientation.z=q.z();pose.pose.pose.orientation.w=q.w();
    const double variance=std::clamp(score,0.01,0.50);
    pose.pose.covariance[0]=variance;pose.pose.covariance[7]=variance;
    pose.pose.covariance[14]=std::min(1.0,2*variance);
    pose.pose.covariance[21]=std::min(1.0,2*variance);
    pose.pose.covariance[28]=std::min(1.0,2*variance);
    pose.pose.covariance[35]=variance;
    rm_relocalization_bridge::GlobalPoseGateLimits limits;
    limits.max_abs_z=0.5;limits.max_abs_roll_pitch=0.2;
    std::string gate_reason;
    pose_gate=rm_relocalization_bridge::validateGlobalPose(
       pose,100000000000LL+static_cast<std::int64_t>(age*1e9),limits,&gate_reason);
    if (accept && !pose_gate) reason="pose_gate_"+gate_reason;
  }
  if (argc>=20) writeDump(argv[19],dump,quality,std::stod(argv[10]),solve_ms>0,
                         source->size(),reference_reason);
  const double total_ms=std::chrono::duration<double,std::milli>(
    std::chrono::steady_clock::now()-begin).count();
  std::cout<<std::setprecision(17)<<source->size()<<','<<target->size()<<','
    <<converged<<','<<score<<','<<solve_ms<<','<<total_ms<<','
    <<accept<<','<<pose_gate<<','<<reason<<','<<translation_jump<<','<<yaw_jump<<','
    <<quality.overlap_ratio<<','<<quality.min_information_eigenvalue<<','
    <<quality.information_condition_number<<','<<reference_gate;
  for (int i=0;i<6;++i) std::cout<<','<<quality.information_eigenvalues[i];
  for (int r=0;r<4;++r) for (int c=0;c<4;++c) std::cout<<','<<result.matrix()(r,c);
  std::cout<<'\n';
}
