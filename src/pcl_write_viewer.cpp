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

using namespace std;

typedef pcl::PointXYZRGB XYZRGB;
typedef pcl::PointCloud<XYZRGB> pclXYZRGB;
typedef pcl::PointCloud<XYZRGB>::Ptr pclXYZRGBptr;
bool save_cloud_ = false;
int counter_ = 0;
std::string pcd_filename_;

ros::Publisher pclpub;
ros::Subscriber pclsub;
ros::Subscriber strsub;

pclXYZRGBptr acquiredCloud (new pclXYZRGB());
cv::Mat acquiredImage, acquiredImageRotate;


inline float PackRGB(uint8_t r, uint8_t g, uint8_t b) {
  uint32_t color_uint = ((uint32_t)r << 16 | (uint32_t) g << 8 | (uint32_t)b);
  return *reinterpret_cast<float *>(&color_uint);
}

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer_void){
  if (event.getKeySym() == "m" && event.keyDown()){
    save_cloud_ = true;
  }
}

std::shared_ptr<pcl::visualization::PCLVisualizer> createXYZRGBVisualizer(pclXYZRGBptr cloud) {
	std::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("PCL RS 3D Viewer"));
	viewer->setBackgroundColor(0.12, 0.12, 0.12);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.5);
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	return (viewer);
}

void imageShow(const cv::Mat image){
	cv::imshow("OCV RS 2D Viewer", image);
	cv::waitKey(30);
}

std::shared_ptr<pcl::visualization::PCLVisualizer> viewer = createXYZRGBVisualizer(acquiredCloud);

void cloudAcquirerReceive(const sensor_msgs::PointCloud2ConstPtr& cloudInMsg){
	pcl::fromROSMsg(*cloudInMsg, *acquiredCloud);
  int i = 640, j = 480, k;

  for (auto& it : acquiredCloud->points){
        it.x = it.x;
        it.y = it.y;
        it.z = it.z;
        it.rgb = PackRGB(
            acquiredImageRotate.at<cv::Vec3b>(j,i)[2],  //r
            acquiredImageRotate.at<cv::Vec3b>(j,i)[1], // g
            acquiredImageRotate.at<cv::Vec3b>(j,i)[0]  //b
        ); //acquiredImage explode
        i--;
        if(i <= 0)
        {
            i=640;
            j--;
        }
        if(j < 0)
        {
            break;
        }
	}
  viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)&viewer);
  if(save_cloud_ == true){
    save_cloud_ = false;
    pcd_filename_ = std::to_string(counter_) + "_cloud_file.pcd";
    pcl::io::savePCDFileASCII(pcd_filename_, *acquiredCloud);
  }
	viewer->updatePointCloud(acquiredCloud);
	viewer->spinOnce();

  counter_++;
}

void imageAcquirerReceive(const sensor_msgs::ImageConstPtr& msg){
	acquiredImage = cv::Mat(cv_bridge::toCvShare(msg, "bgr8")->image);
  cv::rotate(acquiredImage, acquiredImageRotate, cv::ROTATE_180);
	imageShow(acquiredImage);
}


int main(int argc, char **argv) {
	ros::init(argc, argv, "ROSpclVisualizer");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);

	cv::namedWindow("OCV RS 2D Viewer", cv::WINDOW_AUTOSIZE);
	// cv::startWindowThread();

	image_transport::Subscriber sub = it.subscribe("/RSimgAcquisition", 1, imageAcquirerReceive);
  ros::Subscriber pclsubAcquirer = nh.subscribe("/RSpclAcquisition", 1, cloudAcquirerReceive);

	ros::spin();
	cv::destroyWindow("OCV RS 2D Viewer");
}