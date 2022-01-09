#include <ros/ros.h>
#include "std_msgs/Int32.h"
#include <sensor_msgs/PointCloud2.h>
//#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/time.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/vfh.h>
#include <pcl/features/cvfh.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/crh.h>
#include <pcl/features/shot.h>
#include <pcl/features/shot_omp.h>
#include <pcl/common/centroid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/board.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>

#include <mutex>

typedef pcl::PointXYZRGB XYZRGB;
typedef pcl::PointCloud<XYZRGB> pclXYZRGB;
typedef pcl::PointCloud<XYZRGB>::Ptr pclXYZRGBptr;

typedef pcl::Normal NORMAL;
typedef pcl::PointCloud<NORMAL> pclNORMAL;
typedef pcl::PointCloud<NORMAL>::Ptr pclNORMALptr;

typedef pcl::VFHSignature308 VFHS308;
typedef pcl::PointCloud<VFHS308> pclVFHS308;
typedef pcl::PointCloud<VFHS308>::Ptr pclVFHS308ptr;

typedef pcl::FPFHSignature33 FPFHS33;
typedef pcl::PointCloud<FPFHS33> pclFPFHS33;
typedef pcl::PointCloud<FPFHS33>::Ptr pclFPFHS33ptr;

typedef pcl::Histogram<90> CRH90;
typedef pcl::PointCloud<CRH90> pclCRH90;
typedef pcl::PointCloud<CRH90>::Ptr pclCRH90ptr;

typedef pcl::SHOT352 SHOTS352;
typedef pcl::PointCloud<SHOTS352> pclSHOTS352;
typedef pcl::PointCloud<SHOTS352>::Ptr pclSHOTS352ptr;

typedef pcl::ReferenceFrame REF;
typedef pcl::PointCloud<REF> pclREF;
typedef pcl::PointCloud<REF>::Ptr pclREFptr;

typedef pcl::CorrespondencesPtr pclCORRSptr;
typedef pcl::Correspondences pclCORRS;

typedef std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> rotransMat;
typedef std::vector<pclCORRS> vecpclCORRS;

std::mutex mutex_lock;
pcl::console::TicToc timerstart;

ros::Publisher pclpub;
ros::Subscriber pclsub;

sensor_msgs::PointCloud2 outputMsg;

pclXYZRGBptr sceneCloud(new pclXYZRGB());
pclXYZRGBptr sceneKeypoint(new pclXYZRGB());
pclNORMALptr sceneNormal(new pclNORMAL());
pclSHOTS352ptr sceneSHOT(new pclSHOTS352());
pclREFptr sceneREF(new pclREF());

pclXYZRGBptr objectCloud(new pclXYZRGB());
pclXYZRGBptr objectKeypoint(new pclXYZRGB());
pclNORMALptr objectNormal(new pclNORMAL());
pclSHOTS352ptr objectSHOT(new pclSHOTS352());
pclREFptr objectREF(new pclREF());

pclCORRSptr objscenCORRS(new pclCORRS());

rotransMat objectPose;
vecpclCORRS clusteredCORRS;

std::string objectfilename;

/**
void VFHMatching(pclVFHS308ptr inputDescriptor, pclVFHS308ptr sceneDescriptor){
  //Find Correspondences
  pcl::CorrespondencesPtr corrs (new pcl::Correspondences());
  pcl::KdTreeFLANN<VFHS308>::Ptr vfhSearch(new pcl::KdTreeFLANN<VFHS308>());
  vfhSearch->setInputCloud(inputDescriptor);

  for(size_t i = 0; i < sceneDescriptor->size(); i++){
    std::vector<int> neighbor(1);
    std::vector<float> squaredDistances(1);

    int neighborCount = vfhSearch->nearestKSearch(sceneDescriptor->at(i), 1, neighbor, squaredDistances);
    if(neighborCount == 1 && squaredDistances[0] < 0.25f){ //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
      pcl::Correspondence corr (neighbor[0], static_cast<int> (i), squaredDistances[0]);
      corrs->push_back (corr);
    }
  }
  std::cout << "VFH Correspondences found: " << corrs->size() << std::endl;
}
**/

void CVFHCalculation(pclXYZRGBptr input, pclNORMALptr in_normal, pclVFHS308ptr out_hist){
  pcl::search::KdTree<XYZRGB>::Ptr tree(new pcl::search::KdTree<XYZRGB>());
  pcl::CVFHEstimation<XYZRGB, NORMAL, VFHS308> cvfh;
  cvfh.setInputCloud(input);
  cvfh.setInputNormals(in_normal);
  cvfh.setSearchMethod(tree);
  cvfh.compute(*out_hist);
}

void VFHCalculation(pclXYZRGBptr input, pclNORMALptr in_normal, pclVFHS308ptr out_hist){
  pcl::search::KdTree<XYZRGB>::Ptr tree(new pcl::search::KdTree<XYZRGB>());
  pcl::VFHEstimation<XYZRGB, NORMAL, VFHS308> vfh;
  vfh.setInputCloud(input);
  vfh.setInputNormals(in_normal);
  vfh.setSearchMethod(tree);
  vfh.compute(*out_hist);
}

void GCGroupingFunc(pclXYZRGBptr objKeyp, pclXYZRGBptr scenKeyp, pclCORRSptr objscenCorrsParam, rotransMat rotrans, vecpclCORRS vecCorrs){
  pcl::GeometricConsistencyGrouping<XYZRGB, XYZRGB> gcg;
  gcg.setGCSize(20);
  gcg.setGCThreshold(5000);

  gcg.setInputCloud(objKeyp);
  gcg.setSceneCloud(scenKeyp);
  gcg.setModelSceneCorrespondences(objscenCorrsParam);

  //gc_clusterer.cluster (clustered_corrs);
  gcg.recognize(rotrans, vecCorrs);
}

void H3DGroupingFunc(pclXYZRGBptr objKeyp, pclREFptr objREF, pclXYZRGBptr scenKeyp, pclREFptr scenREF, pclCORRSptr objscenCorrsParam, rotransMat matRotrans, vecpclCORRS vecCorrs){
  pcl::Hough3DGrouping<XYZRGB, XYZRGB, REF, REF> h3Dg;
  h3Dg.setHoughBinSize(20);
  h3Dg.setHoughThreshold(500);
  h3Dg.setUseInterpolation(true);
  h3Dg.setUseDistanceWeight(false);

  h3Dg.setInputCloud(objKeyp);
  h3Dg.setInputRf(objREF);
  h3Dg.setSceneCloud(scenKeyp);
  h3Dg.setSceneRf(scenREF);
  h3Dg.setModelSceneCorrespondences(objscenCorrsParam);

  //clusterer.cluster (clustered_corrs);
  h3Dg.recognize (matRotrans, vecCorrs);
  std::cout << "Model Instances found: " << matRotrans.size () << std::endl;
}

void BLRFEstimationFunc(pclXYZRGBptr keypoints, pclNORMALptr in_normal, pclXYZRGBptr input, pclREFptr outREF){
  pcl::BOARDLocalReferenceFrameEstimation<XYZRGB, NORMAL, REF> blrfEst;
  blrfEst.setFindHoles(true);
  blrfEst.setRadiusSearch(20);

  blrfEst.setInputCloud(keypoints);
  blrfEst.setInputNormals(in_normal);
  blrfEst.setSearchSurface(input);
  blrfEst.compute(*outREF);
}

void SHOTMatching(pclSHOTS352ptr inputDescriptor, pclSHOTS352ptr sceneDescriptor, pcl::CorrespondencesPtr corrs){
  corrs->clear();

  pcl::KdTreeFLANN<SHOTS352>::Ptr SHOTsearch (new pcl::KdTreeFLANN<SHOTS352>);
  SHOTsearch->setInputCloud (inputDescriptor);

  //  For each scene keypoint descriptor, find nearest neighbor into the objectCloud keypoints descriptor cloud and add it to the correspondences vector.
  for (size_t i = 0; i < sceneDescriptor->size (); ++i){
    std::vector<int> neighbor(1);
    std::vector<float> squaredDistances(1);
    if (!std::isfinite (sceneDescriptor->at(i).descriptor[0])){ //skipping NaNs
      continue;
    }
    int neighborCount = SHOTsearch->nearestKSearch(sceneDescriptor->at(i), 1, neighbor, squaredDistances);
    if(neighborCount == 1 && squaredDistances[0] < 0.25f){ //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
      pcl::Correspondence corr (neighbor[0], static_cast<int>(i), squaredDistances[0]);
      corrs->push_back (corr);
    }
  }
}

void SHOTOMPCalculation(pclXYZRGBptr input, pclXYZRGBptr keypoints, pclNORMALptr in_normal, pclSHOTS352ptr out_hist){
  pcl::SHOTEstimationOMP<XYZRGB, NORMAL, SHOTS352> shotomp;
  shotomp.setRadiusSearch(20);
  shotomp.setInputCloud(keypoints);
  shotomp.setInputNormals(in_normal);
  shotomp.setSearchSurface(input);
  shotomp.compute(*out_hist);
}

void UniformSamplingFunc(pclXYZRGBptr input, pclXYZRGBptr output){
  pcl::UniformSampling<XYZRGB> us;
  us.setInputCloud(input);
  us.setRadiusSearch(20);
  us.filter(*output);
}

void NormalEstimationOMPFunc(pclXYZRGBptr input, pclNORMALptr out_normal){
  pcl::search::KdTree<XYZRGB>::Ptr tree (new pcl::search::KdTree<XYZRGB>);
  pcl::NormalEstimationOMP<XYZRGB, NORMAL> neo;
  neo.setKSearch(20);
  neo.setInputCloud(input);
  neo.compute(*out_normal);
}

void ObjectLoad(std::string name, pclXYZRGBptr input){
  pcl::io::loadPCDFile(name, *input);
}

void cloudReceive(const sensor_msgs::PointCloud2ConstPtr& inputMsg){
  mutex_lock.lock();

  pcl::fromROSMsg(*inputMsg, *sceneCloud);
  ObjectLoad(objectfilename, objectCloud);
  //timerstart.tic();
  NormalEstimationOMPFunc(objectCloud, objectNormal);
  UniformSamplingFunc(objectCloud, objectKeypoint);
  SHOTOMPCalculation(objectCloud, objectKeypoint, objectNormal, objectSHOT);
  BLRFEstimationFunc(objectKeypoint, objectNormal, objectCloud, objectREF);
  pcl::io::savePCDFileASCII("Database/5._BLRF_Obj.pcd", *objectREF);

  NormalEstimationOMPFunc(sceneCloud, sceneNormal);
  UniformSamplingFunc(sceneCloud, sceneKeypoint);
  SHOTOMPCalculation(sceneCloud, sceneKeypoint, sceneNormal, sceneSHOT);
  BLRFEstimationFunc(sceneKeypoint, sceneNormal, sceneCloud, sceneREF);
  pcl::io::savePCDFileASCII("Database/5._BLRF_Scen.pcd", *sceneREF);

  SHOTMatching(objectSHOT, sceneSHOT, objscenCORRS);
  std::cout << "SHOT Correspondences found: " << objscenCORRS->size() << std::endl;

  H3DGroupingFunc(objectKeypoint, objectREF, sceneKeypoint, sceneREF, objscenCORRS, objectPose, clusteredCORRS);

  // std::cout << "Model Instances found: " << objectPose.size () << std::endl;

  pcl::toROSMsg(*sceneCloud, outputMsg);
  pclpub.publish(outputMsg);
  mutex_lock.unlock();

  sceneCloud->clear();
  objectCloud->clear();
}


int main(int argc, char** argv){
  ros::init(argc, argv, "ROSpclRecognition");
  if(argc < 2){
    return -1;
  }
  objectfilename = argv[1];
  ros::NodeHandlePtr nh(new ros::NodeHandle());
  pclsub = nh->subscribe("/pclsegmentation", 1, cloudReceive);
  pclpub = nh->advertise<sensor_msgs::PointCloud2>("/pclrecognition", 1);
  ros::spin();
}


// PointCloud<VFHSignature308> VFHSig;
// VFHSig::Ptr train_vfhs (new VFHSig ());
//
// pcl::KdTreeFLANN<VFHSignature308>::Ptr vfh_training;
// //Load VFH features from disk for models
// std::vector<std::string> filenames;
// //Fill filenames content.
// for (size_t i = 0; filenames.size() < n; ++i){
  //   train_vfhs += *(loadPointCloud<VFHSignature308>(filenames[i], ""));
  // }
  // vfh_training = pcl::KdTreeFLANN<VFHSignature308>::Ptr (new pcl::KdTreeFLANN<VFHSignature308>);
  // vfh_training->setInputCloud (descriptors_);

  //void matchingDB(){}

  // void Feature(pclVFHS308ptr vfhHist, int n){
    //   //loadTraining
    //   pclVFHS308ptr vfh(new pclVFHS308());
    //   pcl::KdTreeFLANN<VFHS308>::Ptr treeFLANN(new pcl::KdTreeFLANN<VFHS308>);
    //   std::vector<std::string> databasenames;
    //   //fill filenames content
    //   std::vector<std::string> filenames;
    //   dpdf = opendir("/data/files");
    //   if (dpdf != NULL) {
      //     while (epdf = readdir(dpdf)) {
        //       filenames.push_back(std::string(epdf->d_name));
        //    }
        // }
        //   for(size_t i = 0; databasenames.size() < n; i++){
          //     //vfhHist += *(pcl::io::loadPCDFile<VFHS308> (databasenames[i], ""));
          //     //(pcl::io::loadPCDFile<VFHS308> ("test_pcd.pcd", *cloud) == -1)
          //     // pcl::io::savePCDFileASCII("Result2/4._SHOTSignature.pcd", *tempshotHist);
          //   }
          //   treeFLANN->setInputCloud(vfh);
          //
          //   //matching Feature
          //   VFHSignature308 query;
          //   //compute VFH for the current object
          //   //Retrieve first nearest neighbour
          //   std::vector<int> nn_index (1);
          //   std::vector<float> nn_sqr_distance (1);
          //   vfh_training->nearestKSearch (query,1,nn_index,nn_sqr_distance);
          //   const int & best_match = nn_index[0];
          // }
