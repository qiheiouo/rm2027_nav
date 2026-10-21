// Original offline glue using the existing PCL 1.14 public APIs; no ROS.
// FPFH + SAC prerejective is a basin-capture comparator, not place authority.
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <iomanip>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/sample_consensus_prerejective.h>

using P=pcl::PointXYZ;
using Cloud=pcl::PointCloud<P>;
using Feature=pcl::FPFHSignature33;

Cloud::Ptr load(const char * path,double leaf) {
  auto raw=std::make_shared<Cloud>();
  if(pcl::io::loadPCDFile<P>(path,*raw)<0) throw std::runtime_error("PCD load failed");
  auto filtered=std::make_shared<Cloud>();
  pcl::VoxelGrid<P> voxel;voxel.setLeafSize(leaf,leaf,leaf);
  voxel.setInputCloud(raw);voxel.filter(*filtered);return filtered;
}

pcl::PointCloud<Feature>::Ptr features(const Cloud::Ptr & cloud,double nr,double fr) {
  auto normals=std::make_shared<pcl::PointCloud<pcl::Normal>>();
  auto tree=std::make_shared<pcl::search::KdTree<P>>();
  pcl::NormalEstimation<P,pcl::Normal> normal;
  normal.setInputCloud(cloud);normal.setSearchMethod(tree);normal.setRadiusSearch(nr);
  normal.compute(*normals);
  auto result=std::make_shared<pcl::PointCloud<Feature>>();
  pcl::FPFHEstimation<P,pcl::Normal,Feature> fpfh;
  fpfh.setInputCloud(cloud);fpfh.setInputNormals(normals);
  fpfh.setSearchMethod(tree);fpfh.setRadiusSearch(fr);fpfh.compute(*result);
  return result;
}

int main(int argc,char ** argv) {
  if(argc!=10) {
    std::cerr<<"target source leaf normal_radius feature_radius iterations distance inlier_fraction rng_seed\n";
    return 2;
  }
  const auto start=std::chrono::steady_clock::now();
  std::srand(std::stoi(argv[9]));
  auto target=load(argv[1],std::stod(argv[3]));
  auto source=load(argv[2],std::stod(argv[3]));
  auto tf=features(target,std::stod(argv[4]),std::stod(argv[5]));
  auto sf=features(source,std::stod(argv[4]),std::stod(argv[5]));
  pcl::SampleConsensusPrerejective<P,P,Feature> coarse;
  coarse.setInputSource(source);coarse.setSourceFeatures(sf);
  coarse.setInputTarget(target);coarse.setTargetFeatures(tf);
  coarse.setMaximumIterations(std::stoi(argv[6]));
  coarse.setNumberOfSamples(3);coarse.setCorrespondenceRandomness(5);
  coarse.setSimilarityThreshold(.9f);coarse.setMaxCorrespondenceDistance(std::stod(argv[7]));
  coarse.setInlierFraction(std::stod(argv[8]));
  Cloud output;coarse.align(output);
  auto T=coarse.getFinalTransformation();
  auto ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
  std::cout<<std::setprecision(17)<<coarse.hasConverged()<<','<<coarse.getInliers().size()<<','
           <<source->size()<<','<<target->size()<<','<<ms;
  for(int r=0;r<4;++r)for(int c=0;c<4;++c)std::cout<<','<<T(r,c);
  std::cout<<'\n';
}
