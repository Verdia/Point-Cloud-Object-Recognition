#include <librealsense2/rs.hpp>
#include <iostream>
#include <ctime>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common_headers.h>
#include <pcl_conversions/pcl_conversions.h>

#include "std_msgs/String.h"

using namespace std;

typedef pcl::PointXYZRGB XYZRGB;
typedef pcl::PointCloud<XYZRGB> pclXYZRGB;
typedef pcl::PointCloud<XYZRGB>::Ptr pclXYZRGBptr;

std::string getName(std::string inputString) {
    return inputString.substr(0, inputString.find_last_of("."));
}

inline float PackRGB(uint8_t r, uint8_t g, uint8_t b) {
    uint32_t color_uint = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
    return *reinterpret_cast<float *>(&color_uint);
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "pclCapturer");
    ros::NodeHandlePtr nhp(new ros::NodeHandle());

    image_transport::ImageTransport it(*nhp);
    image_transport::Publisher imgpub = it.advertise("/RSimgAcquisition", 1);
    ros::Publisher pclpub = nhp->advertise<sensor_msgs::PointCloud2>("/RSpclAcquisition", 1);
    ros::Publisher strpub = nhp->advertise<std_msgs::String>("/strFilename", 1);

    sensor_msgs::PointCloud2Ptr cloudMsg(new sensor_msgs::PointCloud2());
    sensor_msgs::ImagePtr imageMsg;
    std_msgs::String::Ptr strMsg(new std_msgs::String());

    clock_t clockstart;

    pclXYZRGBptr acquiredCloud(new pclXYZRGB);
    cv::Mat acquiredImage;

    if (argc > 1) {
        pcl::io::loadPCDFile(argv[1], *acquiredCloud);
        strMsg->data = (getName(argv[1]));

        while (ros::ok()) {
            clockstart = clock();
            pcl::toROSMsg(*acquiredCloud, *cloudMsg);
            strpub.publish(strMsg);
            pclpub.publish(cloudMsg);
            std::cout << "Acquire Computation Time: " << (double)(clock()-clockstart)/(double)CLOCKS_PER_SEC << " ms" << std::endl;
            ros::spinOnce();
        }
    }
    
    else {
        // configure and start the camera
        rs2::pipeline rsCamera;
        rs2::config RSConfig;
        rs2::frameset rs_Frameset;
        rs2::pointcloud rs_MatCloud;

        // RSConfig.enable_stream(RS2_STREAM_DEPTH);
        RSConfig.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
        // RSConfig.enable_stream(RS2_STREAM_COLOR);
        RSConfig.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);

        rsCamera.start(RSConfig);
        acquiredCloud->width = 640;
        acquiredCloud->height = 480;
        acquiredCloud->points.resize(acquiredCloud->width * acquiredCloud->height);
        acquiredCloud->is_dense = false;

        rs2::align align_to_depth(RS2_STREAM_DEPTH);
        rs2::align align_to_color(RS2_STREAM_COLOR);

        int dir = 1;
        
        while (ros::ok()) {
            // Using the align object, we block the application until a frameset is available
            rs_Frameset = rsCamera.wait_for_frames();

            strMsg->data = (setName("DatabaseOut"));


            if (dir == 0) {
                // Align all frames to depth viewport
                rs_Frameset = align_to_depth.process(rs_Frameset);
            }
            else {
                // Align all frames to color viewport
                rs_Frameset = align_to_color.process(rs_Frameset);
            }

            // With the aligned frameset we proceed as usual
            // auto colorized_depth = rs_Colorizer.colorize(depth);
            auto ptr = rs_MatCloud.calculate(rs_Frameset.get_depth_frame()).get_vertices();
            for (auto &it : acquiredCloud->points) {
                it.x = ptr->x;
                it.y = ptr->y;
                it.z = ptr->z;
                it.rgb = PackRGB(127, 127, 127);
                ptr++;
            }
            acquiredImage = cv::Mat(cv::Size(640, 480), CV_8UC3, (void *)rs_Frameset.get_color_frame().get_data(), cv::Mat::AUTO_STEP);
            imageMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", acquiredImage).toImageMsg();
            imgpub.publish(imageMsg);

            pcl::toROSMsg(*acquiredCloud, *cloudMsg);
            pclpub.publish(cloudMsg);
            strpub.publish(strMsg);
            // std::cout << strMsg->data << std::endl;
        }
    }
    
    return EXIT_SUCCESS;
}