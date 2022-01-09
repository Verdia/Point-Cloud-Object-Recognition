#include <ros/ros.h>
#include <time.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>

#include <pcl/common/common_headers.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/conditional_removal.h>
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
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>

#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

typedef pcl::PointXYZRGB XYZRGB;
typedef pcl::PointCloud<XYZRGB> pclXYZRGB;
typedef pcl::PointCloud<XYZRGB>::Ptr pclXYZRGBptr;
typedef pcl::PointCloud<XYZRGB>::ConstPtr pclXYZRGBcptr;

std::string fileName;

bool has_name = false;
bool has_save;

pclXYZRGBptr acquiredCloud(new pclXYZRGB());
pclXYZRGBptr preprocessingCloud(new pclXYZRGB());

cv::Mat acquiredImage, acquiredImageRotate;
int count = 0;
bool button = true;

float param[6] = {0};

//---------------------------------------------------------------------------------------------------------------------------
inline float PackRGB(uint8_t r, uint8_t g, uint8_t b) {
  uint32_t color_uint = ((uint32_t)r << 16 | (uint32_t) g << 8 | (uint32_t)b);
  return *reinterpret_cast<float *>(&color_uint);
}

std::string setfolderName(std::string inputString) {
    std::stringstream ss;
    time_t now = time(0);
    tm *ltm = localtime(&now);
    ss << 1900 + ltm->tm_year << "-" << 1 + ltm->tm_mon << "-" << ltm->tm_mday << "/" << ltm->tm_hour << ":" << "00";

    return inputString + "/" + ss.str();
}

std::string setfileName(std::string inputString) {
    std::stringstream ss;
    time_t now = time(0);
    tm *ltm = localtime(&now);
    ss << 1900 + ltm->tm_year << "-" << 1 + ltm->tm_mon << "-" << ltm->tm_mday << "_" << ltm->tm_hour << ":" << ltm->tm_min << ":" << ltm->tm_sec;

    return inputString + "_" + ss.str();
}

//---------------------------------------------------------------------------------------------------------------------------

//===========================================================================================================================
std::shared_ptr<pcl::visualization::PCLVisualizer> createXYZRGBVisualizer(pclXYZRGBcptr cloud, std::string nameWindow) {
  std::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer(nameWindow));
  viewer->setBackgroundColor(double(127)/double(255), double(127)/double(255), double(127)/double(255));
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb);
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.0);
  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();
  return (viewer);
}

std::shared_ptr<pcl::visualization::PCLVisualizer> viewer = createXYZRGBVisualizer(acquiredCloud, "Offline Acquired Cloud");

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer_void) {
  if (event.keyDown()) {
    //std::cout << event.getKeySym() << std::endl;
    if (event.getKeySym() == "KP_1" && button == true) {
      if(has_name == true){
        //pcl::io::savePCDFileASCII(fileName+"RTT.pcd", *acquiredCloud);
        pcl::io::savePCDFileBinaryCompressed(fileName+"RAW.pcd", *acquiredCloud);
      }
      else {
        system(("mkdir -p " + setfolderName("Pengujian")).c_str());
        pcl::io::savePCDFileBinaryCompressed(setfolderName("Pengujian") + "/" + setfileName("Tes") +".pcd", *acquiredCloud);
      }
      button = false;
    }
  }
  else {
    button = true;
  }
}
//===========================================================================================================================

//***************************************************************************************************************************
void NonMeasuredFilter(pclXYZRGBptr input, pclXYZRGBptr output){
  //Menghilangkan nilai NaN 
  std::vector<int> index;
  pcl::removeNaNFromPointCloud(*input,*output,index); 
}

void ConditionalRemovalField(pclXYZRGBptr input, pclXYZRGBptr output, Eigen::Vector4f mindis, Eigen::Vector4f maxdis){
  pcl::ConditionAnd<XYZRGB>::Ptr ca (new pcl::ConditionAnd<XYZRGB>());
  //Set Positive
  ca->addComparison(pcl::FieldComparison<XYZRGB>::ConstPtr (new pcl::FieldComparison<XYZRGB>("x", pcl::ComparisonOps::GT, mindis(0))));
  ca->addComparison(pcl::FieldComparison<XYZRGB>::ConstPtr (new pcl::FieldComparison<XYZRGB>("x", pcl::ComparisonOps::LT, maxdis(0))));
  ca->addComparison(pcl::FieldComparison<XYZRGB>::ConstPtr (new pcl::FieldComparison<XYZRGB>("y", pcl::ComparisonOps::GT, mindis(1))));
  ca->addComparison(pcl::FieldComparison<XYZRGB>::ConstPtr (new pcl::FieldComparison<XYZRGB>("y", pcl::ComparisonOps::LT, maxdis(1))));
  ca->addComparison(pcl::FieldComparison<XYZRGB>::ConstPtr (new pcl::FieldComparison<XYZRGB>("z", pcl::ComparisonOps::GT, mindis(2))));
  ca->addComparison(pcl::FieldComparison<XYZRGB>::ConstPtr (new pcl::FieldComparison<XYZRGB>("z", pcl::ComparisonOps::LT, maxdis(2))));

  pcl::ConditionalRemoval<XYZRGB> cr;
  cr.setCondition (ca);
  cr.setInputCloud (input);
  cr.filter (*output);
}

void PlaneSegmentation(pclXYZRGBptr input, pclXYZRGBptr output){
  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<XYZRGB> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<XYZRGB>::Ptr cloud_plane (new pcl::PointCloud<XYZRGB> ());
  pcl::PCDWriter writer;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.02);

  int nr_points = input->size ();
  while (input->size () > 0.3 * nr_points) {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (input);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0) {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }
    
    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<XYZRGB> extract;
    extract.setInputCloud (input);
    extract.setIndices (inliers);
    extract.setNegative (false);
    
    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);
    std::cout << "PointCloud representing the planar component: " << cloud_plane->size () << " data points." << std::endl;

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*output);
    // *cloud_filtered = *cloud_f;
  }



}

void Clustering(pclXYZRGBptr input, pclXYZRGBptr output){
  // sceneCloud->clear();
  pcl::search::KdTree<XYZRGB>::Ptr kdtree (new pcl::search::KdTree<XYZRGB>);
  kdtree->setInputCloud (input);

  std::vector<pcl::PointIndices> vectorPointIndices;
  pcl::EuclideanClusterExtraction<XYZRGB> ece;
  ece.setClusterTolerance (0.1); // 2cm
  ece.setMinClusterSize (100);
  ece.setMaxClusterSize (50000);
  ece.setSearchMethod (kdtree);
  ece.setInputCloud (input);
  ece.extract (vectorPointIndices);

  for (std::vector<pcl::PointIndices>::const_iterator vpi = vectorPointIndices.begin(); vpi != vectorPointIndices.end (); ++vpi){
    pclXYZRGBptr objectCloud (new pclXYZRGB());
    for (std::vector<int>::const_iterator vi = vpi->indices.begin(); vi != vpi->indices.end(); ++vi){
      objectCloud->points.push_back(input->points[*vi]);
      // ->points.push_back(input->points[*vi]);
    }
    // sceneCloud->width = sceneCloud->points.size();
    // sceneCloud->height = 1;
    // sceneCloud->is_dense = true;
    objectCloud->width = objectCloud->points.size();
    objectCloud->height = 1;
    objectCloud->is_dense = true;

  std::cout << "PointCloud representing the Cluster: " << objectCloud->points.size () << " data points." << std::endl;
  }
}




//***************************************************************************************************************************



void cloudOfflineAcquirerReceive(const sensor_msgs::PointCloud2ConstPtr& cloudInMsg){
  pcl::fromROSMsg(*cloudInMsg, *acquiredCloud);
  NonMeasuredFilter(acquiredCloud, acquiredCloud);
  ConditionalRemovalField(acquiredCloud, acquiredCloud, Eigen::Vector4f(param[0],param[2],param[4], 1.0f), Eigen::Vector4f(param[1],param[3],param[5], 1.0f));




  viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)&viewer);
  viewer->updatePointCloud(acquiredCloud);
  viewer->spinOnce();
}

void fileNameReceive(const std_msgs::String::ConstPtr& strInMsg){
  if (has_name != true){
    fileName = (strInMsg->data.c_str());
    has_name = true;
  }
}

void imageAcquirerReceive(const sensor_msgs::ImageConstPtr& msg){
	acquiredImage = cv::Mat(cv_bridge::toCvShare(msg, "bgr8")->image);
  cv::rotate(acquiredImage, acquiredImageRotate, cv::ROTATE_180);
}

void cloudAcquirerReceive(const sensor_msgs::PointCloud2ConstPtr &cloudInMsg) {
  pcl::fromROSMsg(*cloudInMsg, *acquiredCloud);
  int i = 640, j = 480, k;
  
  for (auto& it : acquiredCloud->points) {
    it.rgb = PackRGB(acquiredImageRotate.at<cv::Vec3b>(j,i)[2], acquiredImageRotate.at<cv::Vec3b>(j,i)[1], acquiredImageRotate.at<cv::Vec3b>(j,i)[0]);
    i--;
    if(i <= 0) {
      i = 640;
      j--;
    }
    if(j < 0) {
      break;
    }
  }
  viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)&viewer);
  
  viewer->updatePointCloud(acquiredCloud);
  viewer->spinOnce();
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "pclVisualizer");
  ros::NodeHandlePtr nhp(new ros::NodeHandle());
  ros::Subscriber pclsub;
  ros::Subscriber strsub;
  image_transport::ImageTransport it(*nhp);
  image_transport::Subscriber imgsub;
  
  pclsub = nhp->subscribe("/RSpclAcquisition", 1, cloudOfflineAcquirerReceive);
  imgsub = it.subscribe("/RSimgAcquisition", 1, imageAcquirerReceive);
  strsub = nhp->subscribe("/strFilename", 1, fileNameReceive);
  
  ros::spin();
}
