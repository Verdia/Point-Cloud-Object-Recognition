#include <iostream>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

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

int main(int argc, char **argv) {
    cv::Mat src = cv::imread("logo-er2c.png");
    cv::resize(src, src, cv::Size(src.cols*0.5,src.rows*0.5),0,0,cv::INTER_LINEAR);

    ros::init(argc, argv, "sliderParameter");
    ros::NodeHandlePtr nhp(new ros::NodeHandle());

    ros::Publisher parampub = nhp->advertise<std_msgs::String>("/cropParam", 1);

    if (!src.data) {
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

    while (true) {
        cv::imshow("My Window", src);
        std::stringstream ss;
        ss << (float)Xmin/(float)100 << " " << (float)Xmax/(float)100 << " " << (float)Ymin/(float)100 << " " << (float)Ymax/(float)100 << " " << (float)Zmin/(float)100 << " " << (float)Zmax/(float)100;
        msg->data = (ss.str());

        parampub.publish(msg);
        ros::spinOnce();

        int iKey = cv::waitKey(50);
        if (iKey == 27) {
            break;
        }
    }
    return 0;
}