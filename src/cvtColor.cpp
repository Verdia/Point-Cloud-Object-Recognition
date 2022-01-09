#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>


typedef pcl::PointXYZRGB XYZRGB;
typedef pcl::PointCloud<XYZRGB> pclXYZRGB;
typedef pcl::PointCloud<XYZRGB>::Ptr pclXYZRGBptr;

inline float convertColor(int r, int g, int b) {
	uint32_t color_uint = ((uint32_t) r << 16 | (uint32_t) g << 8 | (uint32_t) b);
  return *reinterpret_cast<float *> (&color_uint);
}

std::string getName(std::string inputString){
    return inputString.substr(0, inputString.find_last_of("."));
}

int main(int argc, char *argv[]){
    pclXYZRGBptr acquiredCloud (new pclXYZRGB());
    
	pcl::io::loadPCDFile(argv[1], *acquiredCloud);
    std::string fileName(getName(argv[1]));
	
	for (auto &points : acquiredCloud->points) {
		points.rgb = convertColor(0, 0, 0);
	}

    std::stringstream ss;
    // ss << "cvt_Color_setNegative_RANSAC_false.pcd";
    ss << "cvt_Color_Cropped_setNegative.pcd";
    pcl::io::savePCDFileASCII(ss.str(), *acquiredCloud);

    pcl::visualization::CloudViewer viewer ("hasil");
    viewer.showCloud(acquiredCloud);

    while (!viewer.wasStopped())
    {
        /* code */
    }
  
    return 0;
}