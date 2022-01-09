#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <bits/stdc++.h>


int
main(int argc, char **argv)
{
	clock_t start, end;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

	if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(argv[1], *cloud) != 0)
	{
		return -1;
	}

	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normalEstimation;
	normalEstimation.setInputCloud(cloud);

	start = clock();

	// For every point, use all neighbors in a radius of 3cm.
	normalEstimation.setRadiusSearch(0.03);
	// A kd-tree is a data structure that makes searches efficient. More about it later.
	// The normal estimation object will use it to find nearest neighbors.
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>);
	normalEstimation.setSearchMethod(kdtree);
	normalEstimation.compute(*normals);

	std::stringstream mnormals;
  	mnormals << "Normals.pcd";
  	pcl::io::savePCDFileASCII(mnormals.str(), *normals);
	
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Normals"));
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "cloud");
	// Display one normal out of 20, as a line of length 3cm.
	viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud, normals, 20, 0.01, "normals");

	end = clock();

 	double time_taken = double(end - start) / double(CLOCKS_PER_SEC);
    cout << "Time taken by program is : " 
         << time_taken << setprecision(5);
    cout << " sec " << endl;

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	
	return 0;

}