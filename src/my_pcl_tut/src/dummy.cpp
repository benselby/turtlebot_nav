#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

int
main (int argc, char** argv)
{
  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());

  // Fill in the cloud data
  pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  reader.read ("/home/turtlebot/Downloads/table_scene_lms400.pcd", *cloud); // Remember to download the file first!

  std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
	    << " data points (" << pcl::getFieldsList (*cloud) << ").";

  pcl::PCDWriter writer;
  writer.write ("/home/turtlebot/Downloads/table_scene_lms400_downsampled.pcd", *cloud, 
		Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);
  std::cerr << "wrote...\n";
  return (0);
}
