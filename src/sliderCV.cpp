#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/String.h"

int X = 50;
int Y = 50;
int Z = 50;

int main(int argc, char** argv) {
    cv::Mat src = cv::imread("2.jpeg");
	
    ros::init(argc, argv, "Array_pub");
    ros::NodeHandle nh;

    ros::Publisher chatter_pub = nh.advertise<std_msgs::Int32MultiArray>("/chatter",1);
    ros::Rate loop_rate(10);

	if (!src.data) {
		std::cout << "Error loading the image" << std::endl;
		return -1; 
	}
    cv::namedWindow("My Window", 1);
    
    cv::createTrackbar("X: ", "My Window", &X, 100);
    cv::createTrackbar("Y: ", "My Window", &Y, 100);
    cv::createTrackbar("Z: ", "My Window", &Z, 100);
    
    while (true) {
        cv::imshow("My Window", src);
        std::stringstream ss;
        ss << "X" << X << "Y" << Y << "Z" << Z;
        std::cout << ss.str() << std::endl; 
        //writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*

        std_msgs::Int32MultiArray msg;
        
        msg.data.push_back(X);
        msg.data.push_back(Y);
        msg.data.push_back(Z);

        chatter_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();

        // std::cout << X << " - " << Y << " - " << Z << " " << std::endl; 
        int iKey = cv::waitKey(50);
		//if user press 'ESC' key
		if (iKey == 27) {
			break;
		}
    }
     return 0;
}