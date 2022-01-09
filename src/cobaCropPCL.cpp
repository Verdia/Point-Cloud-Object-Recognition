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

#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/String.h"

using namespace std;

typedef pcl::PointXYZRGB XYZRGB;
typedef pcl::PointCloud<XYZRGB> pclXYZRGB;
typedef pcl::PointCloud<XYZRGB>::Ptr pclXYZRGBptr;
typedef pcl::PointCloud<XYZRGB>::ConstPtr pclXYZRGBcptr;

ros::Publisher pclpub;
ros::Subscriber pclsub;
ros::Subscriber strSub;
ros::Subscriber crpSub;
std::string fileName;

bool has_name = false;
bool has_save;
bool save_cloud = false;

float param[6] = {0};

sensor_msgs::PointCloud2Ptr cloudOutMsg (new sensor_msgs::PointCloud2());
pclXYZRGBptr acquiredCloud(new pclXYZRGB());
cv::Mat acquiredImage, acquiredImageRotate;

inline float PackRGB(uint8_t r, uint8_t g, uint8_t b) {
  uint32_t color_uint = ((uint32_t)r << 16 | (uint32_t) g << 8 | (uint32_t)b);
  return *reinterpret_cast<float *>(&color_uint);
}

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

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer_void){
  if (event.getKeySym() == "m" && event.keyDown()){
    save_cloud = true;
  }
}

void setMinMaxXYZ(pclXYZRGBptr input){
  pcl::PointXYZRGB minPt, maxPt;
  pcl::getMinMax3D (*input, minPt, maxPt);
}

void removeFilterNaN(pclXYZRGBptr input, pclXYZRGBptr output){
  std::vector<int> indeks;
  pcl::removeNaNFromPointCloud(*input,*output,indeks); //Menghilangkan nilai NaN 
}

void outlierRemoval(pclXYZRGBptr input, pclXYZRGBptr output){
  pcl::StatisticalOutlierRemoval<XYZRGB> sor;
  sor.setInputCloud(input);
  sor.setMeanK(3000);
  sor.setStddevMulThresh(1.5);
  sor.filter(*output);
}

void ConditionalRemovalFieldPOS(pclXYZRGBptr input, pclXYZRGBptr output, Eigen::Vector4f mindis, Eigen::Vector4f maxdis){
  
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

void SACSegmentation(pclXYZRGBptr input, pclXYZRGBptr output){

  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

  pcl::SACSegmentation<XYZRGB> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(10);
  seg.setDistanceThreshold(0.01);

  //seg.setAxis(1);
  //seg.setEpsAngle(pcl::deg2rad(1));
  seg.setInputCloud(input);
  seg.segment(*inliers, *coefficients);

  if(inliers->indices.size() == 0){
    std::cout << "no planar found" << std::endl;
  }

  pcl::ExtractIndices<XYZRGB> ei;
  ei.setInputCloud(input);
  ei.setIndices(inliers);
  // ei.setNegative(false);
  //ei.filter(*BackCloud);
  ei.setNegative(true);
  ei.filter(*output);
  //  pcl::io::savePCDFileASCII("setNegative_RANSAC_false.pcd", *output);
  // pcl::io::savePCDFileASCII("setNegative_RANSAC_true.pcd", *output);
}

void cloudAcquirerReceive(const sensor_msgs::PointCloud2ConstPtr &cloudInMsg)
{
    pcl::fromROSMsg(*cloudInMsg, *acquiredCloud);
    ConditionalRemovalFieldPOS(acquiredCloud,acquiredCloud,Eigen::Vector4f(param[0],param[2],param[4], 1.0f), Eigen::Vector4f(param[1],param[3],param[5], 1.0f));
    removeFilterNaN(acquiredCloud,acquiredCloud);
    // outlierRemoval(acquiredCloud,acquiredCloud);
    // SACSegmentation(acquiredCloud,acquiredCloud);

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

    if(save_cloud == true){
        save_cloud = false;
        pcl::io::savePCDFileASCII( fileName + "_Cropped-Data.pcd", *acquiredCloud);
    }

    std::cout << fileName << std::endl;

    viewer->updatePointCloud(acquiredCloud);
    viewer->spinOnce();

    pcl::fromROSMsg(*cloudInMsg, *acquiredCloud);
    pclpub.publish(*cloudOutMsg);
}

void nameReceive(const std_msgs::String::ConstPtr& strInMsg){
  if (has_name != true){
    has_name == true;
    fileName = (strInMsg->data.c_str());
  }
}

void parsingParameter6(const std_msgs::String::ConstPtr& strInMsg){
    // fileName = (strInMsg->data.c_str());
    size_t i = 0;
    std::string parse;
    std::stringstream ss((strInMsg->data.c_str()));
    while (ss >> parse)
    {
        param[i] = std::stof(parse);
        i++;
    }
    std::cout << param[0] << " " << param[1] << " " << param[2] << " " << param[3] << " " << param[4] << " " << param[5] << std::endl;

}

void imageAcquirerReceive(const sensor_msgs::ImageConstPtr& msg){
	acquiredImage = cv::Mat(cv_bridge::toCvShare(msg, "bgr8")->image);
    cv::rotate(acquiredImage, acquiredImageRotate, cv::ROTATE_180);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "CaptureViewerRGBD");
    ros::NodeHandlePtr nhp(new ros::NodeHandle());
	image_transport::ImageTransport it(*nhp);
    ros::Subscriber pclsubAcquirer;
    image_transport::Subscriber sub;

    pclsubAcquirer = nhp->subscribe("/RSpclAcquisition", 1, cloudAcquirerReceive);
    sub = it.subscribe("/RSimgAcquisition", 1, imageAcquirerReceive);
    strSub = nhp->subscribe("/strFilename",1,nameReceive);
    crpSub = nhp->subscribe("/chatter",1,parsingParameter6);
    pclpub = nhp->advertise<sensor_msgs::PointCloud2>("/pclCropping",1);

    ros::spin();
}

