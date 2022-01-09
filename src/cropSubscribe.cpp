#include <ros/ros.h>
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
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/String.h"

int temp[90];

void chatterCallback(const std_msgs::String::ConstPtr& msg) {
	ROS_INFO("%s", msg->data.c_str());
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "Array_sub");
	ros::NodeHandlePtr nh;
	ros::Subscriber sub = nh.subscribe("chatter", 1000, chatterCallback);
	ros::spin();
    return 0;
}