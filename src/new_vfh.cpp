#include <iostream>
#include <pcl/io/pcd_io.h>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <flann/flann.h>
#include <pcl/features/vfh.h>
#include <pcl/visualization/pcl_visualizer.h>

std::string getName(std::string inputString){
    return inputString.substr(0, inputString.find_last_of("."));
}

int main(int argc, char *argv[]){
    // pcl::visualization::PCLVisualizer viewer("PCL VIEWER");

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>());
    pcl::PointCloud<pcl::VFHSignature308>::Ptr descriptor (new pcl::PointCloud<pcl::VFHSignature308>);
    
    pcl::io::loadPCDFile(argv[1], *in_cloud);
    std::string fileName(getName(argv[1]));

    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normalEstimation;
    normalEstimation.setInputCloud(in_cloud);
    normalEstimation.setRadiusSearch(0.03);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    normalEstimation.setSearchMethod(kdtree);
    normalEstimation.compute(*normals);

    pcl::VFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::VFHSignature308> VFH;
    VFH.setInputCloud(in_cloud);
    VFH.setInputNormals(normals);
    VFH.setSearchMethod(kdtree);
    VFH.setNormalizeBins(true);
    VFH.setNormalizeDistance(false);
    VFH.compute(*descriptor);

    // std::cout << "output points.size (): " << descriptor->points.size () << std::endl;
    // pcl::VFHSignature308 vfhFeatures = descriptor->points[0];

    // std::cout << vfhFeatures << std::endl;
    float vfh[308] = {0};
    int iter = 0;

    for(auto it : descriptor->points[0].histogram){
        vfh[iter] = it;
        iter++;
    }

    for(int i=0; i<308; i++){
        std::cout << vfh[i] << " ";
    }
    
    // std::cout << descriptor->points[0] << std::endl;
    // std::cout << "------------------------" << std::endl;
    // std::cout << descriptor->points[0].histogram << std::endl;

    std::stringstream ss;
    ss << "vfh_object_"<< argv[2] <<".pcd";
    pcl::io::savePCDFile(ss.str(), *descriptor);
    
}