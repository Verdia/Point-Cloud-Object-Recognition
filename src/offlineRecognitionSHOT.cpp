#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/features/shot.h>
#include <pcl/common/centroid.h>
//#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>

typedef pcl::PointXYZRGB XYZRGB;
typedef pcl::PointCloud<XYZRGB> pclXYZRGB;
typedef pcl::PointCloud<XYZRGB>::Ptr pclXYZRGBptr;

typedef pcl::Normal NORMAL;
typedef pcl::PointCloud<NORMAL> pclNORMAL;
typedef pcl::PointCloud<NORMAL>::Ptr pclNORMALptr;

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

typedef pcl::visualization::PCLVisualizer pclVisual;

void NormalEst(pclXYZRGBptr input, pclNORMALptr outputNormal){
  pcl::NormalEstimationOMP<XYZRGB, NORMAL> norm_est;
  norm_est.setKSearch (10);
  norm_est.setInputCloud (input);
  norm_est.compute (*outputNormal);
}

void UniformSample(pclXYZRGBptr input, pclXYZRGBptr outputUniform){
  pcl::UniformSampling<XYZRGB> us;
  us.setInputCloud (input);
  us.setRadiusSearch (0.01f);
  us.filter (*outputUniform);
}

void SHOTEst(pclXYZRGBptr input, pclXYZRGBptr keypoints, pclNORMALptr inNormal, pclSHOTS352ptr SHOTout){
  pcl::SHOTEstimationOMP<XYZRGB, NORMAL, SHOTS352> descr_est;
  descr_est.setRadiusSearch (0.02f);
  descr_est.setInputCloud (keypoints);
  descr_est.setInputNormals (inNormal);
  descr_est.setSearchSurface (input);
  descr_est.compute (*SHOTout);
}

void MatchingSHOT(pclSHOTS352ptr inDescriptor, pclSHOTS352ptr sceneDescriptor, pcl::CorrespondencesPtr corrs){
  corrs->clear();

  pcl::KdTreeFLANN<SHOTS352>::Ptr match_search (new pcl::KdTreeFLANN<SHOTS352>);
  match_search->setInputCloud (inDescriptor);

  //  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
  for (std::size_t i = 0; i < sceneDescriptor->size (); ++i) {
    std::vector<int> neighbor(1);
    std::vector<float> squareDist (1);
    if (!std::isfinite (sceneDescriptor->at(i).descriptor[0])) {
      continue;
    }
    int neighCount = match_search->nearestKSearch (sceneDescriptor->at (i), 1, neighbor, squareDist);
    if(neighCount == 1 && squareDist[0] < 0.25f){ //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
      pcl::Correspondence corr (neighbor[0], static_cast<int> (i), squareDist[0]);
      corrs->push_back (corr);
    }
  }
}

void BLRFEst(pclXYZRGBptr keypoints, pclNORMALptr inNormal, pclXYZRGBptr input, pclREFptr outREF){
  pcl::BOARDLocalReferenceFrameEstimation<XYZRGB, NORMAL, REF> blrfEst;
  blrfEst.setFindHoles (true);
  blrfEst.setRadiusSearch (0.015f);

  blrfEst.setInputCloud (keypoints);
  blrfEst.setInputNormals (inNormal);
  blrfEst.setSearchSurface (input);
  blrfEst.compute (*outREF);
}

void H3DGroup(pclXYZRGBptr objKeypoints, pclREFptr objREF, pclXYZRGBptr sceneKeypoints, pclREFptr sceneREF, pclCORRSptr objsceneCorrsParameters, rotransMat matRotrans, vecpclCORRS vecCorrs){
  pcl::Hough3DGrouping<XYZRGB, XYZRGB, REF, REF> h3Dg;
  h3Dg.setHoughBinSize (0.01f);
  h3Dg.setHoughThreshold (5.0f);
  h3Dg.setUseInterpolation (true);
  h3Dg.setUseDistanceWeight (false);

  h3Dg.setInputCloud (objKeypoints);
  h3Dg.setInputRf (objREF);
  h3Dg.setSceneCloud (sceneKeypoints);
  h3Dg.setSceneRf (sceneREF);
  h3Dg.setModelSceneCorrespondences (objsceneCorrsParameters);

  //clusterer.cluster (clustered_corrs);
  h3Dg.recognize (matRotrans, vecCorrs);
  std::cout << "Model Instances found: " << matRotrans.size () << std::endl;
}

void GCGrouping(pclXYZRGBptr objKeypoints, pclXYZRGBptr sceneKeypoints, pclCORRSptr objsceneCorrsParameters, rotransMat rotrans, vecpclCORRS vecCorrs){
  pcl::GeometricConsistencyGrouping<XYZRGB, XYZRGB> gcg;
  gcg.setGCSize (0.01f);
  gcg.setGCThreshold (5.0f);

  gcg.setInputCloud (objKeypoints);
  gcg.setSceneCloud (sceneKeypoints);
  gcg.setModelSceneCorrespondences (objsceneCorrsParameters);

  //gc_clusterer.cluster (clustered_corrs);
  gcg.recognize (rotrans, vecCorrs);
}

void loadPCDFile(std::string fileName, pclXYZRGBptr cloud){
  if (pcl::io::loadPCDFile (fileName, *cloud) < 0) {
    std::cout << "Error loading cloud." << std::endl;
    exit (-1);
  }
}

int main (int argc, char *argv[]) {
  
  pclXYZRGBptr model (new pclXYZRGB());
  pclXYZRGBptr scene (new pclXYZRGB());

  loadPCDFile(argv[1], model);
  loadPCDFile(argv[2], scene);

  pclXYZRGBptr sceneKeyp(new pclXYZRGB());
  pclNORMALptr sceneNormal(new pclNORMAL());
  pclSHOTS352ptr sceneSHOT(new pclSHOTS352());
  pclREFptr sceneREF(new pclREF());

  pclXYZRGBptr modelKeyp(new pclXYZRGB());
  pclNORMALptr modelNormal(new pclNORMAL());
  pclSHOTS352ptr modelSHOT(new pclSHOTS352());
  pclREFptr modelREF(new pclREF());

  pclCORRSptr objscenCORRS(new pclCORRS());

  rotransMat objectPose;
  vecpclCORRS clusteredCORRS;

  NormalEst(model, modelNormal);
  UniformSample(model, modelKeyp);
  SHOTEst(model, modelKeyp, modelNormal, modelSHOT);
  BLRFEst(modelKeyp, modelNormal, model, modelREF);
  pcl::io::savePCDFileASCII("BLRF_Obj.pcd", *modelREF);

  NormalEst(scene, sceneNormal);
  UniformSample(scene, sceneKeyp);
  SHOTEst(scene, sceneKeyp, sceneNormal, sceneSHOT);
  BLRFEst(sceneKeyp, sceneNormal, scene, sceneREF);
  pcl::io::savePCDFileASCII("BLRF_Scene.pcd", *sceneREF);
  
  MatchingSHOT(modelSHOT, sceneSHOT, objscenCORRS);
  std::cout << "SHOT Correspondences found: " << objscenCORRS->size() << std::endl;

  H3DGroup(modelKeyp, modelREF, sceneKeyp, sceneREF, objscenCORRS, objectPose, clusteredCORRS);


  std::cout << "Model instances found: " << objectPose.size () << std::endl;
  // for (std::size_t i = 0; i < objectPose.size (); ++i) {
  //   std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
  //   std::cout << "        Correspondences belonging to this instance: " << clusteredCORRS[i].size () << std::endl;

  //   // Print the rotation matrix and translation vector
  //   Eigen::Matrix3f rotation = objectPose[i].block<3,3>(0, 0);
  //   Eigen::Vector3f translation = objectPose[i].block<3,1>(0, 3);

  //   printf ("\n");
  //   printf ("            | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
  //   printf ("        R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
  //   printf ("            | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
  //   printf ("\n");
  //   printf ("        t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));
  // }

  pclVisual viewer ("Correspondence Grouping");
  viewer.addPointCloud (scene, "scene_cloud");

  std::cout << "load scene" << std::endl;

  pclXYZRGBptr off_scene_model (new pclXYZRGB());
  pclXYZRGBptr off_scene_model_keyp (new pclXYZRGB());

  //  We are translating the model so that it doesn't end in the middle of the scene representation
  pcl::transformPointCloud (*model, *off_scene_model, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));
  pcl::transformPointCloud (*modelKeyp, *off_scene_model_keyp, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));

  std::cout << "create transform" << std::endl;

  pcl::visualization::PointCloudColorHandlerCustom<XYZRGB> off_scene_model_color_handler (off_scene_model, 255, 255, 128);
  viewer.addPointCloud (off_scene_model, off_scene_model_color_handler, "off_scene_model");

  pcl::visualization::PointCloudColorHandlerCustom<XYZRGB> scene_keypoints_color_handler (sceneKeyp, 0, 0, 255);
  viewer.addPointCloud (sceneKeyp, scene_keypoints_color_handler, "scene_keypoints");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keypoints");

  pcl::visualization::PointCloudColorHandlerCustom<XYZRGB> off_scene_model_keypoints_color_handler (off_scene_model_keyp, 0, 0, 255);
  viewer.addPointCloud (off_scene_model_keyp, off_scene_model_keypoints_color_handler, "off_scene_model_keypoints");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "off_scene_model_keypoints");
  
  for (std::size_t i = 0; i < objectPose.size (); ++i) {
    pclXYZRGBptr rotated_model (new pclXYZRGB());
    pcl::transformPointCloud (*model, *rotated_model, objectPose[i]);

    std::stringstream ss_cloud;
    ss_cloud << "instance" << i;


    pcl::visualization::PointCloudColorHandlerCustom<XYZRGB> rotated_model_color_handler (rotated_model, 255, 0, 0);
    viewer.addPointCloud (rotated_model, rotated_model_color_handler, ss_cloud.str ());

    for (std::size_t j = 0; j < clusteredCORRS[i].size (); ++j) {
      std::stringstream ss_line;
      ss_line << "correspondence_line" << i << "_" << j;
      XYZRGB& model_point = off_scene_model_keyp->at (clusteredCORRS[i][j].index_query);
      XYZRGB& scene_point = sceneKeyp->at (clusteredCORRS[i][j].index_match);

      //  We are drawing a line for each pair of clustered correspondences found between the model and the scene
      viewer.addLine<XYZRGB, XYZRGB> (model_point, scene_point, 0, 255, 0, ss_line.str ());
    }
  }

  while (!viewer.wasStopped ()) {
    viewer.spinOnce ();
  }

  return (0);
}
