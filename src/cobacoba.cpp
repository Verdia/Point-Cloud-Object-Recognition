#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/String.h"

int param[6] = {0};
int Xmin = 50;
int Xmax = 50;
int Ymin = 50;
int Ymax = 50;
int Zmin = 50;
int Zmax = 50;

std_msgs::String::Ptr msg(new std_msgs::String());

void parsingParameter6(std::string inputString)
{
    size_t i = 0;
    std::string parse;
    std::stringstream ss(inputString);
    while (ss >> parse)
    {
        param[i] = std::stof(parse);
        i++;
    }
    std::cout << param[0] << " " << param[1] << " " << param[2] << " " << param[3] << " " << param[4] << " " << param[5] << std::endl;
}

int main(int argc, char **argv)
{
    cv::Mat src = cv::imread("2.png");

    ros::init(argc, argv, "Array_pub");
    ros::NodeHandle nh;

    ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("/chatter", 1);
    ros::Rate loop_rate(10);

    // std::string inputString = "52 51 55";

    if (!src.data)
    {
        std::cout << "Error loading the image" << std::endl;
        return -1;
    }
    cv::namedWindow("My Window", 1);

    cv::createTrackbar("Xmin: ", "My Window", &Xmin, 100);
    cv::createTrackbar("Xmax: ", "My Window", &Xmax, 100);
    cv::createTrackbar("Ymin: ", "My Window", &Ymin, 100);
    cv::createTrackbar("Ymax: ", "My Window", &Ymax, 100);
    cv::createTrackbar("Zmin: ", "My Window", &Zmin, 100);
    cv::createTrackbar("Zmax: ", "My Window", &Zmax, 100);

    while (true)
    {
        cv::imshow("My Window", src);
        std::stringstream ss;
        ss << (float)Xmin/(float)100 << " " << (float)Xmax/(float)100 << " " << (float)Ymin/(float)100 << " " << (float)Ymax/(float)100 << " " << (float)Zmin/(float)100 << " " << (float)Zmax/(float)100;
        // std::cout << ss.str() << std::endl;
        msg->data = (ss.str());
        //writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*

        chatter_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();

        // std::cout << X << " - " << Y << " - " << Z << " " << std::endl;
        int iKey = cv::waitKey(50);
        //if user press 'ESC' key
        if (iKey == 27)
        {
            break;
        }
    }
    return 0;
}