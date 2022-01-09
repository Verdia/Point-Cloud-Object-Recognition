#include <ros/ros.h>
#include <iostream>
#include <pcl/common/common_headers.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/conditional_removal.h>
//#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/vfh.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h>
#include <bits/stdc++.h>


typedef pcl::PointXYZRGB XYZRGB;
typedef pcl::PointXYZ XYZ;
typedef pcl::PointCloud<XYZRGB> pclXYZRGB;
typedef pcl::PointCloud<XYZRGB>::Ptr pclXYZRGBptr;
typedef pcl::PointCloud<XYZ>::Ptr pclXYZptr;

pclXYZRGBptr sceneCloud (new pclXYZRGB());
pcl::PCDWriter writer;

//Crop Config
float minX = -5.1, minY = -25.5, minZ = +90;
float maxX = +5.1, maxY = +10, maxZ = +180.5;

bool has_save = false;

void convert(pclXYZRGBptr input, pclXYZRGBptr output){
  int ID = 0;
  std::stringstream ss;
  ss << "Object_ASCII_"<< ID <<".pcd";
  pcl::io::savePCDFileASCII(ss.str(), *output);

  ID++;
}

void removeFilterNaN(pclXYZRGBptr input, pclXYZRGBptr output){
  std::vector<int> indeks;
  pcl::removeNaNFromPointCloud(*input,*output,indeks); //Menghilangkan nilai NaN 

  int ID = 0;
  std::stringstream ss;
  ss << "Object_ASCII_ClearNaN_"<< ID <<".pcd";
  pcl::io::savePCDFileASCII(ss.str(), *output);

  ID++;
}

void setMinMaxXYZ(pclXYZRGBptr input){
  pcl::PointXYZRGB minPt, maxPt;
  pcl::getMinMax3D (*input, minPt, maxPt);
  std::cout << "Max x: " << maxPt.x << std::endl;
  std::cout << "Max y: " << maxPt.y << std::endl;
  std::cout << "Max z: " << maxPt.z << std::endl;
  std::cout << "Min x: " << minPt.x << std::endl;
  std::cout << "Min y: " << minPt.y << std::endl;
  std::cout << "Min z: " << minPt.z << std::endl;
}

void CropBoxFilter(pclXYZRGBptr input, pclXYZRGBptr output){
  pcl::CropBox<XYZRGB> cb;
  cb.setInputCloud(input);
  cb.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0f));
  cb.setMax(Eigen::Vector4f(minX, minY, minZ, 1.0f));
  cb.filter(*output);

  int ID = 0;
  std::stringstream ss;
  ss << "Hand_Cropped_"<< ID <<".pcd";
  pcl::io::savePCDFileASCII(ss.str(), *output);

  ID++;
}

void PassThrough(pclXYZRGBptr input, pclXYZRGBptr output, Eigen::Vector4f mindis, Eigen::Vector4f maxdis){
  pclXYZRGBptr temp (new pclXYZRGB());
  pclXYZRGBptr temp2 (new pclXYZRGB());
  pcl::PassThrough<XYZRGB> pt;
  pt.setInputCloud (input);
  pt.setFilterFieldName ("x");
  pt.setFilterLimits (mindis(0), maxdis(0));
  pt.filter(*temp);
  pt.setInputCloud (temp);
  pt.setFilterFieldName ("y");
  pt.setFilterLimits (mindis(1), maxdis(1));
  pt.filter(*temp2);
  pt.setInputCloud (temp2);
  pt.setFilterFieldName ("z");
  pt.setFilterLimits (mindis(2), maxdis(2));
  pt.filter(*output);
}

void ConditionalRemovalColor(pclXYZRGBptr input, pclXYZRGBptr output){
  pcl::ConditionAnd<XYZRGB>::Ptr ca (new pcl::ConditionAnd<XYZRGB>());
  ca->addComparison(pcl::PackedRGBComparison<XYZRGB>::ConstPtr (new pcl::PackedRGBComparison<XYZRGB>("r", pcl::ComparisonOps::GT, 10)));
  ca->addComparison(pcl::PackedRGBComparison<XYZRGB>::ConstPtr (new pcl::PackedRGBComparison<XYZRGB>("r", pcl::ComparisonOps::LT, 90)));

  pcl::ConditionalRemoval<XYZRGB> cr;
  cr.setCondition (ca);
  cr.setInputCloud(input);
  cr.filter(*output);
}

void ConditionalRemovalFieldPOS(pclXYZRGBptr input, pclXYZRGBptr output, Eigen::Vector4f mindis, Eigen::Vector4f maxdis){
  
  pcl::ConditionAnd<XYZRGB>::Ptr ca (new pcl::ConditionAnd<XYZRGB>());
  //Set Positive
  ca->addComparison(pcl::FieldComparison<XYZRGB>::ConstPtr (new pcl::FieldComparison<XYZRGB>("x", pcl::ComparisonOps::GT, mindis(0))));
  ca->addComparison(pcl::FieldComparison<XYZRGB>::ConstPtr (new pcl::FieldComparison<XYZRGB>("x", pcl::ComparisonOps::LT, maxdis(0))));

  pcl::ConditionalRemoval<XYZRGB> cr;
  cr.setCondition (ca);
  cr.setInputCloud (input);
  cr.filter (*output);

  int ID = 0;
  std::stringstream ss;
  ss << "Hand_Cropped_Positive_"<< ID <<".pcd";
  pcl::io::savePCDFileASCII(ss.str(), *output);

  ID++;
}

void ConditionalRemovalFieldNEG(pclXYZRGBptr input, pclXYZRGBptr output, Eigen::Vector4f mindis, Eigen::Vector4f maxdis){
  pcl::ConditionOr<XYZRGB>::Ptr co (new pcl::ConditionOr<XYZRGB>());
  co->addComparison(pcl::FieldComparison<XYZRGB>::ConstPtr (new pcl::FieldComparison<XYZRGB>("x", pcl::ComparisonOps::GT, maxdis(0))));
  co->addComparison(pcl::FieldComparison<XYZRGB>::ConstPtr (new pcl::FieldComparison<XYZRGB>("x", pcl::ComparisonOps::LT, mindis(0))));

  pcl::ConditionalRemoval<XYZRGB> cr;
  cr.setCondition (co);
  cr.setInputCloud (input);
  cr.filter (*output);

  int ID = 0;
  std::stringstream ss;
  ss << "Hand_Cropped_setNegative_"<< ID <<".pcd";
  pcl::io::savePCDFileASCII(ss.str(), *output);

  ID++;
}

void outlierRemoval(pclXYZRGBptr input, pclXYZRGBptr output){
  pcl::StatisticalOutlierRemoval<XYZRGB> sor;
  sor.setInputCloud(input);
  sor.setMeanK(800);
  sor.setStddevMulThresh(1.5);
  sor.filter(*output);
  
  int ID = 0;
  std::stringstream ss;
  ss << "Hand_OutlierRemove_"<< ID <<".pcd";
  pcl::io::savePCDFileASCII(ss.str(), *output);

  ID++;
}

void SACSegmentation(pclXYZRGBptr input, pclXYZRGBptr output){

  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

  pcl::SACSegmentation<XYZRGB> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(5);
  seg.setDistanceThreshold(0.005);

  // Eigen::Vector3f axis = Eigen::Vector3f(0.0,1.0,0.0); //y axis
  // seg.setAxis(Eigen::Vector3f(1.0,0.0,0.0));
  // seg.setAxis(Eigen::Vector3f(0.0,1.0,0.0));
  // seg.setAxis(Eigen::Vector3f(0.0,0.0,1.0));
  // seg.setEpsAngle(pcl::deg2rad(0.0f));
  seg.setInputCloud(input);
  seg.segment(*inliers, *coefficients);

  if(inliers->indices.size() == 0){
    std::cout << "no planar found" << std::endl;
  }

  int ID = 1;

  pcl::ExtractIndices<XYZRGB> ei;
  ei.setInputCloud(input);
  ei.setIndices(inliers);
  // ei.setNegative(false);
  // ei.filter(*output);
  ei.setNegative(true);
  ei.filter(*output);
  //  pcl::io::savePCDFileASCII("setNegative_RANSAC_false.pcd", *output);
  pcl::io::savePCDFileASCII("setNegative_RANSAC_true.pcd", *output);

  std::stringstream ss;
  ss << "Hand_segmentation_"<< ID <<".pcd";
  pcl::io::savePCDFileASCII(ss.str(), *output);

  ID++;
}

inline float convertColor(int r, int g, int b) {
	uint32_t color_uint = ((uint32_t) r << 16 | (uint32_t) g << 8 | (uint32_t) b);
  return *reinterpret_cast<float *> (&color_uint);
}

void cvtColor(pclXYZRGBptr input, pclXYZRGBptr output){
  for (auto &points : input->points) {
		points.rgb = convertColor(255, 255, 255);
	}

    std::stringstream ss;
    // ss << "cvt_Color_setNegative_RANSAC_false.pcd";
    ss << "cvt_Color_Cropped_setNegative.pcd";
    pcl::io::savePCDFileASCII(ss.str(), *output);

}

void EuclideanCluster(pclXYZRGBptr input, pclXYZRGBptr output){
  sceneCloud->clear();
  pcl::search::KdTree<XYZRGB>::Ptr kdtree (new pcl::search::KdTree<XYZRGB>);
  kdtree->setInputCloud (input);

  std::vector<pcl::PointIndices> vectorPointIndices;
  pcl::EuclideanClusterExtraction<XYZRGB> ece;
  ece.setClusterTolerance (0.05); // 2cm
  ece.setMinClusterSize (500);
  ece.setMaxClusterSize (50000);
  ece.setSearchMethod (kdtree);
  ece.setInputCloud (input);
  ece.extract (vectorPointIndices);

  int objectID = 1;
  for (std::vector<pcl::PointIndices>::const_iterator vpi = vectorPointIndices.begin(); vpi != vectorPointIndices.end (); ++vpi){
    pclXYZRGBptr objectCloud (new pclXYZRGB());
    for (std::vector<int>::const_iterator vi = vpi->indices.begin(); vi != vpi->indices.end(); ++vi){
      objectCloud->points.push_back(input->points[*vi]);
      //sceneCloud->points.push_back(input->points[*vi]);
    }
    // sceneCloud->width = sceneCloud->points.size();
    // sceneCloud->height = 1;
    // sceneCloud->is_dense = true;
    objectCloud->width = objectCloud->points.size();
    objectCloud->height = 1;
    objectCloud->is_dense = true;

    std::cout << "PointCloud representing the Cluster: " << objectCloud->points.size () << " data points." << std::endl;
    std::stringstream ss;
    ss << "cloud_cluster_" << objectID << ".pcd";
    writer.write<XYZRGB> (ss.str (), *objectCloud, false); //*
    objectID++;
  }
  
}

void compute3DCentroid(pclXYZRGBptr input){
  Eigen::Vector4f centroid;
	pcl::compute3DCentroid(*input, centroid);
	
	std::cout << " X = " << centroid(0) << std::endl;
	std::cout << " Y = " << centroid(1) << std::endl;
	std::cout << " Z = " << centroid(2) << std::endl;
}

std::string getExtension(std::string inputString){
    return inputString.substr(inputString.find_last_of("."), inputString.length());
}

std::string getName(std::string inputString){
    return inputString.substr(0, inputString.find_last_of("."));
}

int main(int argc, char *argv[]){

  pclXYZRGBptr acquiredCloud (new pclXYZRGB());
  
  pcl::io::loadPCDFile(argv[1], *acquiredCloud);
  std::string fileName(getName(argv[1]));

  clock_t start, end;

  start = clock();
  // convert(acquiredCloud,acquiredCloud);
  // removeFilterNaN(acquiredCloud,acquiredCloud);
  // PassThrough(acquiredCloud,acquiredCloud,Eigen::Vector4f(-0,-0, 0),Eigen::Vector4f(0,0, 0));
  // setMinMaxXYZ(acquiredCloud);
  // CropBoxFilter(acquiredCloud,acquiredCloud);
  ConditionalRemovalFieldPOS(acquiredCloud, acquiredCloud, Eigen::Vector4f(0.013,0,0,1), Eigen::Vector4f(0.082,0,0,1));
  // ConditionalRemovalFieldNEG(acquiredCloud, acquiredCloud, Eigen::Vector4f(-0.1,0,0,1), Eigen::Vector4f(0.3,0,0,1));
  // SACSegmentation(acquiredCloud,acquiredCloud);
  // outlierRemoval(acquiredCloud,acquiredCloud);
  // cvtColor(acquiredCloud,acquiredCloud);
  // EuclideanCluster(acquiredCloud,acquiredCloud);
  // compute3DCentroid(acquiredCloud);
  
  pcl::visualization::CloudViewer viewer ("hasil");
  viewer.showCloud(acquiredCloud);

  end = clock();

  double time_taken = double(end - start) / double(CLOCKS_PER_SEC);
  cout << "Time taken by program is : "  
       << time_taken << setprecision(5);
  cout << " sec " << endl;

  while (!viewer.wasStopped())
  {
    /* code */
  }
  

  return 0;
}
