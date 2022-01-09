#include <ros/ros.h>
#include <time.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>

#include <pcl/common/common_headers.h>
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
cv::Mat acquiredImage, acquiredImageRotate;
int count = 0;
bool button = true;

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

void cloudOfflineAcquirerReceive(const sensor_msgs::PointCloud2ConstPtr& cloudInMsg){
  pcl::fromROSMsg(*cloudInMsg, *acquiredCloud);
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
