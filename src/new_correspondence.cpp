#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_plotter.h>
#include <bits/stdc++.h>
#include <pcl/visualization/histogram_visualizer.h>

typedef pcl::PointXYZRGB PointType;
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::SHOT352 DescriptorType;

std::string model_filename_;
std::string model2_filename_;
std::string scene_filename_;
std::string pose_filename_;

//Algorithm params
bool show_keypoints_ (false);
bool show_correspondences_ (false);
bool use_cloud_resolution_ (false);
bool use_hough_ (true);
float model_ss_ (0.01f);
float scene_ss_ (0.03f);
float rf_rad_ (0.015f);
float descr_rad_ (0.02f);
float cg_size_ (0.01f);
float cg_thresh_ (5.0f);

void
parseCommandLine (int argc, char *argv[])
{
  //Model & scene filenames
  std::vector<int> filenames;
  filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
  if (filenames.size () != 4)
  {
    std::cout << "Filenames missing.\n";
    exit (-1);
  }

  model_filename_ = argv[filenames[0]];
  model2_filename_ = argv[filenames[1]];
  pose_filename_ = argv[filenames[2]];
  scene_filename_ = argv[filenames[3]];
}


int
main (int argc, char *argv[])
{
  parseCommandLine (argc, argv);
	clock_t start, startSHOT, end, endSHOT;

  pcl::PointCloud<PointType>::Ptr model (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<NormalType>::Ptr model_normals (new pcl::PointCloud<NormalType> ());
  pcl::PointCloud<PointType>::Ptr model_keypoints (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<DescriptorType>::Ptr model_descriptors (new pcl::PointCloud<DescriptorType> ());

  pcl::PointCloud<PointType>::Ptr model2 (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<NormalType>::Ptr model2_normals (new pcl::PointCloud<NormalType> ());
  pcl::PointCloud<PointType>::Ptr model2_keypoints (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<DescriptorType>::Ptr model2_descriptors (new pcl::PointCloud<DescriptorType> ());

  pcl::PointCloud<PointType>::Ptr pose (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<NormalType>::Ptr pose_normals (new pcl::PointCloud<NormalType> ());
  pcl::PointCloud<PointType>::Ptr pose_keypoints (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<DescriptorType>::Ptr pose_descriptors (new pcl::PointCloud<DescriptorType> ());

  pcl::PointCloud<PointType>::Ptr scene (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<NormalType>::Ptr scene_normals (new pcl::PointCloud<NormalType> ());
  pcl::PointCloud<PointType>::Ptr scene_keypoints (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<DescriptorType>::Ptr scene_descriptors (new pcl::PointCloud<DescriptorType> ());
	pcl::PointCloud<PointType>::Ptr alignedModel(new pcl::PointCloud<PointType> ());

  //
  //  Load clouds
  //
  if (pcl::io::loadPCDFile (model_filename_, *model) < 0)
  {
    std::cout << "Error loading model cloud." << std::endl;
    return (-1);
  }
  if (pcl::io::loadPCDFile (model2_filename_, *model2) < 0)
  {
    std::cout << "Error loading model2 cloud." << std::endl;
    return (-1);
  }
  if (pcl::io::loadPCDFile (pose_filename_, *pose) < 0)
  {
    std::cout << "Error loading pose cloud." << std::endl;
    return (-1);
  }
  if (pcl::io::loadPCDFile (scene_filename_, *scene) < 0)
  {
    std::cout << "Error loading scene cloud." << std::endl;
    return (-1);
  }

	start = clock();

  //
  //  Compute Normals
  //
  pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
  norm_est.setKSearch (10);
  norm_est.setInputCloud (model);
  norm_est.compute (*model_normals);

  norm_est.setInputCloud (model2);
  norm_est.compute (*model2_normals);

  norm_est.setInputCloud (pose);
  norm_est.compute (*pose_normals);

  norm_est.setInputCloud (scene);
  norm_est.compute (*scene_normals);

  //
  //  Downsample Clouds to Extract keypoints
  //

  pcl::UniformSampling<PointType> uniform_sampling;
  uniform_sampling.setInputCloud (model);
  uniform_sampling.setRadiusSearch (model_ss_);
  uniform_sampling.filter (*model_keypoints);
  std::cout << "Hand total points: " << model->size () << "; Selected Keypoints: " << model_keypoints->size () << std::endl;

  uniform_sampling.setInputCloud (model2);
  uniform_sampling.setRadiusSearch (model_ss_);
  uniform_sampling.filter (*model2_keypoints);
  std::cout << "Object total points: " << model2->size () << "; Selected Keypoints: " << model2_keypoints->size () << std::endl;

  uniform_sampling.setInputCloud (pose);
  uniform_sampling.setRadiusSearch (model_ss_);
  uniform_sampling.filter (*pose_keypoints);
  std::cout << "Pose total points: " << pose->size () << "; Selected Keypoints: " << pose_keypoints->size () << std::endl;

  uniform_sampling.setInputCloud (scene);
  uniform_sampling.setRadiusSearch (scene_ss_);
  uniform_sampling.filter (*scene_keypoints);
  std::cout << "Scene total points: " << scene->size () << "; Selected Keypoints: " << scene_keypoints->size () << std::endl;

	startSHOT = clock();

  //
  //  Compute Descriptor for keypoints
  //
  pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
  descr_est.setRadiusSearch (descr_rad_);

  descr_est.setInputCloud (model_keypoints);
  descr_est.setInputNormals (model_normals);
  descr_est.setSearchSurface (model);
  descr_est.compute (*model_descriptors);

  descr_est.setInputCloud (model2_keypoints);
  descr_est.setInputNormals (model2_normals);
  descr_est.setSearchSurface (model2);
  descr_est.compute (*model2_descriptors);

  descr_est.setInputCloud (pose_keypoints);
  descr_est.setInputNormals (pose_normals);
  descr_est.setSearchSurface (pose);
  descr_est.compute (*pose_descriptors);

  descr_est.setInputCloud (scene_keypoints);
  descr_est.setInputNormals (scene_normals);
  descr_est.setSearchSurface (scene);
  descr_est.compute (*scene_descriptors);

  std::cout << "SHOT Gripper output points.size (): " << model_descriptors->points.size () << std::endl;
  std::cout << "------------------------" << std::endl;
  pcl::SHOT352 descriptor = model_descriptors->points[0];
  std::cout << descriptor << std::endl;
  std::cout << "\n" << std::endl;


  std::cout << "SHOT Object output points.size (): " << model2_descriptors->points.size () << std::endl;
  std::cout << "------------------------" << std::endl;
  pcl::SHOT352 descriptor2 = model2_descriptors->points[0];
  std::cout << descriptor2 << std::endl;
  std::cout << "\n" << std::endl;

  std::cout << "SHOT Pose output points.size (): " << pose_descriptors->points.size () << std::endl;
  std::cout << "------------------------" << std::endl;
  pcl::SHOT352 descriptor3 = pose_descriptors->points[0];
  std::cout << descriptor3 << std::endl;
  std::cout << "\n" << std::endl;

	endSHOT = clock();

  double time_takenSHOT = double(endSHOT - startSHOT) / double(CLOCKS_PER_SEC);
  cout << "Time taken by Feature Extraction process is : " 
       << time_takenSHOT << setprecision(5);
  cout << " sec " << endl;
  cout << " \n " << endl;

  std::stringstream se;
  se << "descriptor_Hand.pcd";
  pcl::io::savePCDFileASCII(se.str(), *model_descriptors);

  std::stringstream si;
  si << "descriptor_Object.pcd";
  pcl::io::savePCDFileASCII(si.str(), *model2_descriptors);

  std::stringstream es;
  es << "descriptor_scene.pcd";
  pcl::io::savePCDFileASCII(es.str(), *pose_descriptors);

  // float shot[352] = {0};
  // int iter = 0;
  
  //
  //  Find Model-Scene Correspondences with KdTree
  //
  pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());

  pcl::KdTreeFLANN<DescriptorType> match_search;
  match_search.setInputCloud (model_descriptors);

  //  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
  for (std::size_t i = 0; i < scene_descriptors->size (); ++i)
  {
    std::vector<int> neigh_indices (1);
    std::vector<float> neigh_sqr_dists (1);
    if (!std::isfinite (scene_descriptors->at (i).descriptor[0])) //skipping NaNs
    {
      continue;
    }
    int found_neighs = match_search.nearestKSearch (scene_descriptors->at (i), 1, neigh_indices, neigh_sqr_dists);
    if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
    {
      pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
      model_scene_corrs->push_back (corr);
    }
  }
  std::cout << "Correspondences Gripper found: " << model_scene_corrs->size () << std::endl;

  pcl::CorrespondencesPtr model2_scene_corrs (new pcl::Correspondences ());

  pcl::KdTreeFLANN<DescriptorType> match_search2;
  match_search2.setInputCloud (model2_descriptors);

  //  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
  for (std::size_t i = 0; i < scene_descriptors->size (); ++i)
  {
    std::vector<int> neigh_indices2 (1);
    std::vector<float> neigh_sqr_dists2 (1);
    if (!std::isfinite (scene_descriptors->at (i).descriptor[0])) //skipping NaNs
    {
      continue;
    }
    int found_neighs2 = match_search2.nearestKSearch (scene_descriptors->at (i), 1, neigh_indices2, neigh_sqr_dists2);
    if(found_neighs2 == 1 && neigh_sqr_dists2[0] < 0.25f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
    {
      pcl::Correspondence corr2 (neigh_indices2[0], static_cast<int> (i), neigh_sqr_dists2[0]);
      model2_scene_corrs->push_back (corr2);
    }
  }
  std::cout << "Correspondences Object found: " << model2_scene_corrs->size () << std::endl;

  pcl::CorrespondencesPtr pose_scene_corrs (new pcl::Correspondences ());

  pcl::KdTreeFLANN<DescriptorType> match_search3;
  match_search3.setInputCloud (pose_descriptors);

  //  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
  for (std::size_t i = 0; i < scene_descriptors->size (); ++i)
  {
    std::vector<int> neigh_indices3 (1);
    std::vector<float> neigh_sqr_dists3 (1);
    if (!std::isfinite (scene_descriptors->at (i).descriptor[0])) //skipping NaNs
    {
      continue;
    }
    int found_neighs3 = match_search3.nearestKSearch (scene_descriptors->at (i), 1, neigh_indices3, neigh_sqr_dists3);
    if(found_neighs3 == 1 && neigh_sqr_dists3[0] < 0.25f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
    {
      pcl::Correspondence corr3 (neigh_indices3[0], static_cast<int> (i), neigh_sqr_dists3[0]);
      pose_scene_corrs->push_back (corr3);
    }
  }
  std::cout << "Correspondences Z Pose Object found: " << pose_scene_corrs->size () << std::endl;

  //
  //  Actual Clustering
  //
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
  std::vector<pcl::Correspondences> clustered_corrs;

  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations2;
  std::vector<pcl::Correspondences> clustered_corrs2;

  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations3;
  std::vector<pcl::Correspondences> clustered_corrs3;

  //  Using Hough3D
    //
    //  Compute (Keypoints) Reference Frames only for Hough
    //
    pcl::PointCloud<RFType>::Ptr model_rf (new pcl::PointCloud<RFType> ());
    pcl::PointCloud<RFType>::Ptr model2_rf (new pcl::PointCloud<RFType> ());
    pcl::PointCloud<RFType>::Ptr pose_rf (new pcl::PointCloud<RFType> ());
    pcl::PointCloud<RFType>::Ptr scene_rf (new pcl::PointCloud<RFType> ());

    pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est;
    rf_est.setFindHoles (true);
    rf_est.setRadiusSearch (rf_rad_);

    rf_est.setInputCloud (model_keypoints);
    rf_est.setInputNormals (model_normals);
    rf_est.setSearchSurface (model);
    rf_est.compute (*model_rf);

    rf_est.setInputCloud (model2_keypoints);
    rf_est.setInputNormals (model2_normals);
    rf_est.setSearchSurface (model2);
    rf_est.compute (*model2_rf);

    rf_est.setInputCloud (pose_keypoints);
    rf_est.setInputNormals (pose_normals);
    rf_est.setSearchSurface (pose);
    rf_est.compute (*pose_rf);

    rf_est.setInputCloud (scene_keypoints);
    rf_est.setInputNormals (scene_normals);
    rf_est.setSearchSurface (scene);
    rf_est.compute (*scene_rf);

    //  Clustering
    pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer;
    clusterer.setHoughBinSize (cg_size_);
    clusterer.setHoughThreshold (cg_thresh_);
    clusterer.setUseInterpolation (true);
    clusterer.setUseDistanceWeight (false);

    clusterer.setInputCloud (model_keypoints);
    clusterer.setInputRf (model_rf);
    clusterer.setSceneCloud (scene_keypoints);
    clusterer.setSceneRf (scene_rf);
    clusterer.setModelSceneCorrespondences (model_scene_corrs);

    //clusterer.cluster (clustered_corrs);
    clusterer.recognize (rototranslations, clustered_corrs);

    pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer2;
    clusterer2.setHoughBinSize (cg_size_);
    clusterer2.setHoughThreshold (cg_thresh_);
    clusterer2.setUseInterpolation (true);
    clusterer2.setUseDistanceWeight (false);

    clusterer2.setInputCloud (model2_keypoints);
    clusterer2.setInputRf (model2_rf);
    clusterer2.setSceneCloud (scene_keypoints);
    clusterer2.setSceneRf (scene_rf);
    clusterer2.setModelSceneCorrespondences (model2_scene_corrs);

    //clusterer.cluster (clustered_corrs);
    clusterer2.recognize (rototranslations2, clustered_corrs2);

    pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer3;
    clusterer3.setHoughBinSize (cg_size_);
    clusterer3.setHoughThreshold (cg_thresh_);
    clusterer3.setUseInterpolation (true);
    clusterer3.setUseDistanceWeight (false);

    clusterer3.setInputCloud (pose_keypoints);
    clusterer3.setInputRf (pose_rf);
    clusterer3.setSceneCloud (scene_keypoints);
    clusterer3.setSceneRf (scene_rf);
    clusterer3.setModelSceneCorrespondences (pose_scene_corrs);

    //clusterer.cluster (clustered_corrs);
    clusterer3.recognize (rototranslations3, clustered_corrs3);
  // }


  // Centroid
  Eigen::Vector4f centroid;
	pcl::compute3DCentroid(*model, centroid);

  Eigen::Vector4f centroid2;
	pcl::compute3DCentroid(*model2, centroid2);

  Eigen::Vector4f centroid3;
	pcl::compute3DCentroid(*pose, centroid3);

  //
  //  Output results
  //
  std::cout << "Hand instances found: " << rototranslations.size () << std::endl;
  for (std::size_t i = 0; i < rototranslations.size (); ++i)
  {
    std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
    std::cout << "        Correspondences belonging to this instance: " << clustered_corrs[i].size () << std::endl;

    // Print the rotation matrix and translation vector
    Eigen::Matrix3f rotation = rototranslations[i].block<3,3>(0, 0);
    Eigen::Vector3f translation = rototranslations[i].block<3,1>(0, 3);
  }

  std::cout << "Object instances found: " << rototranslations2.size () << std::endl;
  for (std::size_t i = 0; i < rototranslations2.size (); ++i)
  {
    std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
    std::cout << "        Correspondences belonging to this instance: " << clustered_corrs2[i].size () << std::endl;

    // Print the rotation matrix and translation vector
    Eigen::Matrix3f rotation2 = rototranslations2[i].block<3,3>(0, 0);
    Eigen::Vector3f translation2 = rototranslations2[i].block<3,1>(0, 3);
  }

  std::cout << "Object pose found: " << rototranslations3.size () << std::endl;
  for (std::size_t i = 0; i < rototranslations3.size (); ++i)
  {
    std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
    std::cout << "        Correspondences belonging to this instance: " << clustered_corrs3[i].size () << std::endl;

    // Print the rotation matrix and translation vector
    Eigen::Matrix3f rotation3 = rototranslations3[i].block<3,3>(0, 0);
    Eigen::Vector3f translation3 = rototranslations3[i].block<3,1>(0, 3);
  }

  printf ("\n");
  std::cout << "Centroid of Hand : " << std::endl;
  std::cout << "        X = " << centroid(0) << std::endl;
	std::cout << "        Y = " << centroid(1) << std::endl;
	std::cout << "        Z = " << centroid(2) << std::endl;
  printf ("\n");

  printf ("\n");
  std::cout << "Centroid of Object : " << std::endl;
  std::cout << "        X = " << centroid2(0) << std::endl;
	std::cout << "        Y = " << centroid2(1) << std::endl;
	std::cout << "        Z = " << centroid2(2) << std::endl;
  printf ("\n");

  printf ("\n");
  std::cout << "Centroid Object Pose : " << std::endl;
  std::cout << "        X = " << centroid3(0) << std::endl;
	std::cout << "        Y = " << centroid3(1) << std::endl;
	std::cout << "        Z = " << centroid3(2) << std::endl;
  printf ("\n");

  // int idx = 1000;

  //
  //  Visualization
  //
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Hand Detection"));
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2(new pcl::visualization::PCLVisualizer("Object Detection"));
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer3(new pcl::visualization::PCLVisualizer("Pose Estimation Object Surface"));
  // pcl::visualization::PCLHistogramVisualizer plotter;
  viewer->addPointCloud (scene, "scene_cloud");
  viewer2->addPointCloud (scene, "scene_cloud");
  viewer3->addPointCloud (scene, "scene_cloud");

  // plotter.addFeatureHistogram(*model_descriptors, 352);
  // plotter.spin();

  pcl::PointCloud<PointType>::Ptr off_scene_model (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<PointType>::Ptr off_scene_model_keypoints (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<PointType>::Ptr off_scene_model2 (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<PointType>::Ptr off_scene_model2_keypoints (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<PointType>::Ptr off_scene_pose (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<PointType>::Ptr off_scene_pose_keypoints (new pcl::PointCloud<PointType> ());

 //  We are translating the model so that it doesn't end in the middle of the scene representation
    pcl::transformPointCloud (*model, *off_scene_model, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));
    pcl::transformPointCloud (*model_keypoints, *off_scene_model_keypoints, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));

    pcl::transformPointCloud (*model2, *off_scene_model2, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));
    pcl::transformPointCloud (*model2_keypoints, *off_scene_model2_keypoints, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));

    pcl::transformPointCloud (*pose, *off_scene_pose, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));
    pcl::transformPointCloud (*pose_keypoints, *off_scene_pose_keypoints, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));

    // pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_color_handler (off_scene_model, 255, 255, 128);
    // viewer->addPointCloud (off_scene_model, off_scene_model_color_handler, "off_scene_model");

    // pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model2_color_handler (off_scene_model2, 255, 255, 128);
    // viewer2->addPointCloud (off_scene_model2, off_scene_model2_color_handler, "off_scene_model2");

    // pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_pose_color_handler (off_scene_pose, 255, 255, 128);
    // viewer3->addPointCloud (off_scene_pose, off_scene_pose_color_handler, "off_scene_pose");

    // pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_keypoints_color_handler (scene_keypoints, 0, 0, 255);
    // viewer->addPointCloud (scene_keypoints, scene_keypoints_color_handler, "scene_keypoints");
    // viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keypoints");
    // viewer2->addPointCloud (scene_keypoints, scene_keypoints_color_handler, "scene_keypoints");
    // viewer2->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keypoints");
    // viewer3->addPointCloud (scene_keypoints, scene_keypoints_color_handler, "scene_keypoints");
    // viewer3->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keypoints");

  	// viewer->addPointCloudNormals<PointType, NormalType>(model, model_normals, 20, 0.01, "normals");
    // viewer2->addPointCloudNormals<PointType, NormalType>(model2, model2_normals, 20, 0.01, "normals2");

  for (std::size_t i = 0; i < rototranslations.size (); ++i)
  {
    pcl::PointCloud<PointType>::Ptr rotated_model (new pcl::PointCloud<PointType> ());
    pcl::transformPointCloud (*model, *rotated_model, rototranslations[i]);

    std::stringstream ss_cloud;
    ss_cloud << "instance" << i;

    pcl::visualization::PointCloudColorHandlerCustom<PointType> rotated_model_color_handler (rotated_model, 0, 255, 0);
    viewer->addPointCloud (rotated_model, rotated_model_color_handler, ss_cloud.str ());

    if (show_correspondences_)
    {
      for (std::size_t j = 0; j < clustered_corrs[i].size (); ++j)
      {
        std::stringstream ss_line;
        ss_line << "correspondence_line" << i << "_" << j;
        PointType& model_point = off_scene_model_keypoints->at (clustered_corrs[i][j].index_query);
        PointType& scene_point = scene_keypoints->at (clustered_corrs[i][j].index_match);

        //  We are drawing a line for each pair of clustered correspondences found between the model and the scene
        viewer->addLine<PointType, PointType> (model_point, scene_point, 0, 255, 0, ss_line.str ());
      }
    }

  for (std::size_t i = 0; i < rototranslations2.size (); ++i)
  {
    pcl::PointCloud<PointType>::Ptr rotated_model2 (new pcl::PointCloud<PointType> ());
    pcl::transformPointCloud (*model2, *rotated_model2, rototranslations2[i]);

    std::stringstream ss_cloud;
    ss_cloud << "instance" << i;

    pcl::visualization::PointCloudColorHandlerCustom<PointType> rotated_model2_color_handler (rotated_model2, 255, 0, 0);
    viewer2->addPointCloud (rotated_model2, rotated_model2_color_handler, ss_cloud.str ());

    if (show_correspondences_)
    {
      for (std::size_t j = 0; j < clustered_corrs2[i].size (); ++j)
      {
        std::stringstream ss_line;
        ss_line << "correspondence_line" << i << "_" << j;
        PointType& model2_point = off_scene_model2_keypoints->at (clustered_corrs2[i][j].index_query);        
        PointType& scene_point = scene_keypoints->at (clustered_corrs2[i][j].index_match);

        //  We are drawing a line for each pair of clustered correspondences found between the model and the scene
        viewer2->addLine<PointType, PointType> (model2_point, scene_point, 255, 0, 0, ss_line.str ());        
      }
    }
  }

  for (std::size_t i = 0; i < rototranslations3.size (); ++i)
  {
    pcl::PointCloud<PointType>::Ptr rotated_pose (new pcl::PointCloud<PointType> ());
    pcl::transformPointCloud (*pose, *rotated_pose, rototranslations3[i]);

    std::stringstream ss_cloud;
    ss_cloud << "instance" << i;

    pcl::visualization::PointCloudColorHandlerCustom<PointType> rotated_pose_color_handler (rotated_pose, 0, 0, 255);
    viewer3->addPointCloud (rotated_pose, rotated_pose_color_handler, ss_cloud.str ());

    if (show_correspondences_)
    {
      for (std::size_t j = 0; j < clustered_corrs3[i].size (); ++j)
      {
        std::stringstream ss_line;
        ss_line << "correspondence_line" << i << "_" << j;
        PointType& pose_point = off_scene_pose_keypoints->at (clustered_corrs3[i][j].index_query);        
        PointType& scene_point = scene_keypoints->at (clustered_corrs3[i][j].index_match);

        //  We are drawing a line for each pair of clustered correspondences found between the model and the scene
        viewer3->addLine<PointType, PointType> (pose_point, scene_point, 255, 0, 0, ss_line.str ());        
      }
    }
  }

	end = clock();

  double time_taken = double(end - start) / double(CLOCKS_PER_SEC);
  cout << "Time taken by all process is : " 
       << time_taken << setprecision(5);
  cout << " sec " << endl;
  cout << " \n " << endl;

  while (!viewer->wasStopped ())
  {
    viewer->spinOnce ();
  }
  while (!viewer2->wasStopped ())
  {
    viewer2->spinOnce ();
  }
  while (!viewer3->wasStopped ())
  {
    viewer3->spinOnce ();
  }
  return (0);
}
}