#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>
//variabel upload pcd dan save file pcd
std::string model_filename_;
std::string save_filename_;

typedef pcl::PointXYZRGB PointType;

int main (int argc, char** argv)
{
  pcl::PointCloud<PointType>::Ptr cloud (new pcl::PointCloud<PointType>);
  pcl::visualization::PCLVisualizer viewer("PCD Viewer");
  pcl::PointCloud<PointType>::Ptr cloudASCII (new pcl::PointCloud<PointType>);


   //Model & scene filenames
  std::vector<int> filenames;
  filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
  if (filenames.size () != 2)
  {
    std::cout << "Filenames missing.\n";
    exit (-1);
  }

  model_filename_ = argv[filenames[0]];
  save_filename_ = argv[filenames[1]];

  if (pcl::io::loadPCDFile<PointType> (model_filename_, *cloud) == -1) // load the file
  {
    PCL_ERROR ("Couldn't read file demo1.pcd \n");
    return (-1);
  }
 std::vector<int> indeks;

 // pcl::removeNaNFromPointCloud(*cloud,*cloudASCII,indeks); //Menghilangkan nilai NaN 

  for(auto& range: *cloud){
      range.x = range.x *100;  //mengubah ascii(meter) ke centimeter
      range.y = range.y *100; 
      range.z = range.z *100; 
  }

 pcl::io::savePCDFileASCII (save_filename_, *cloud);//load the
 std::cerr << "Saved " << cloud->size () << " data points to "<< save_filename_ << std::endl;
viewer.addPointCloud(cloud, "CloudCropping");
viewer.spin();
return (0);
}