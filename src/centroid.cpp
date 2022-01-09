#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>

int main (int argc, char *argv[])
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *cloud);

  float xCentroid, yCentroid, zCentroid;
  float xPoints, yPoints, zPoints;

  for (auto &points : cloud->points) {
		xCentroid = points.x / cloud->size();
    yCentroid = points.y / cloud->size();
    zCentroid = points.z / cloud->size();
    
    xPoints = points.x;
    yPoints = points.y;
    zPoints = points.z;

    }

  std::cout << "nilai point x: " << xPoints << std::endl;
  std::cout << "nilai point y: " << yPoints << std::endl;
  std::cout << "nilai point z: " << zPoints << std::endl;
	  
  std::cout << "nilai Centroid x: " << xCentroid << std::endl;
  std::cout << "nilai Centroid y: " << yCentroid << std::endl;
  std::cout << "nilai Centroid z: " << zCentroid << std::endl;

  std::cout << "Total points: " << cloud->size() << std::endl;

  return (0);
}