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
typedef pcl::PointCloud<XYZRGB>::ConstPtr pclXYZRGBcptr;

ros::Publisher pclpub;
ros::Subscriber pclsub;
ros::Subscriber strsub;

pclXYZRGBptr acquiredCloud(new pclXYZRGB());
pclXYZRGBptr croppedCloud(new pclXYZRGB());

// cv::Mat acquiredImage, acquiredImageRotate, acquiredCropped, acquiredCroppedRotate;

std::shared_ptr<pcl::visualization::PCLVisualizer> createXYZRGBVisualizer(pclXYZRGBcptr cloud)
{
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("PCL ZED 3D Viewer"));
    viewer->setBackgroundColor(0.12, 0.12, 0.12);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.5);
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    return (viewer);
}

std::shared_ptr<pcl::visualization::PCLVisualizer> viewer = createXYZRGBVisualizer(acquiredCloud);
std::shared_ptr<pcl::visualization::PCLVisualizer> viewerCropped = createXYZRGBVisualizer(acquiredCloud);

inline float PackRGB(uint8_t r, uint8_t g, uint8_t b) {
  uint32_t color_uint = ((uint32_t)r << 16 | (uint32_t) g << 8 | (uint32_t)b);
  return *reinterpret_cast<float *>(&color_uint);
}

void cloudAcquirerReceive(const sensor_msgs::PointCloud2ConstPtr &cloudInMsg)
{
    pcl::fromROSMsg(*cloudInMsg, *acquiredCloud);
    // int i = 640, j = 480, k;

    // for (auto& it : acquiredCloud->points){
    //     it.x = it.x;
    //     it.y = it.y;
    //     it.z = it.z;
    //     it.rgb = PackRGB(
    //         acquiredImageRotate.at<cv::Vec3b>(j,i)[2],  //r
    //         acquiredImageRotate.at<cv::Vec3b>(j,i)[1], // g
    //         acquiredImageRotate.at<cv::Vec3b>(j,i)[0]  //b
    //     ); //acquiredImage explode
    //     i--;
    //     if(i <= 0)
    //     {
    //         i=640;
    //         j--;
    //     }
    //     if(j < 0)
    //     {
    //         break;
    //     }
	// }
    viewer->updatePointCloud(acquiredCloud);
    viewer->spinOnce();
}

void cloudCropped(const sensor_msgs::PointCloud2ConstPtr &cloudInMsg)
{
    pcl::fromROSMsg(*cloudInMsg, *croppedCloud);
    // int i = 640, j = 480, k;

    // for (auto& it : croppedCloud->points){
    //     it.x = it.x;
    //     it.y = it.y;
    //     it.z = it.z;
    //     it.rgb = PackRGB(
    //         acquiredCroppedRotate.at<cv::Vec3b>(j,i)[2],  //r
    //         acquiredCroppedRotate.at<cv::Vec3b>(j,i)[1], // g
    //         acquiredCroppedRotate.at<cv::Vec3b>(j,i)[0]  //b
    //     ); //acquiredImage explode
    //     i--;
    //     if(i <= 0)
    //     {
    //         i=640;
    //         j--;
    //     }
    //     if(j < 0)
    //     {
    //         break;
    //     }
	// }
    viewerCropped->updatePointCloud(croppedCloud);
    viewerCropped->spinOnce();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ROSpclVisualizer");
    ros::NodeHandlePtr nhp(new ros::NodeHandle());

    ros::Subscriber pclsubAcquirer = nhp->subscribe("/RSpclAcquisition", 1, cloudAcquirerReceive);
    ros::Subscriber pclsubCropped = nhp->subscribe("/pclCropping", 1, cloudCropped);

    ros::spin();
}