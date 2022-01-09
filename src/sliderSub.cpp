#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/String.h"

float param[6] = {0};

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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sliderSub");
    ros::NodeHandlePtr nhp(new ros::NodeHandle());
    ros::Subscriber sliderSubs;

    sliderSubs = nhp->subscribe("/chatter", 1, parsingParameter6);

    ros::spin();
}