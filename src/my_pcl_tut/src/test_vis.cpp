#include <ros/ros.h>
#include <iostream>
#include <cstdio>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/people/ground_based_people_detection_app.h>
#include <pcl/common/time.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/PointCloud2.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;


// PCL viewer //                                                                   
//pcl::visualization::PCLVisualizer viewer("PCL Viewer");

// Mutex: //                                                                       
//boost::mutex cloud_mutex;

//enum { COLS = 640, ROWS = 480 };

// void cloud_cb_ (const PointCloudT::ConstPtr &callback_cloud, PointCloudT::Ptr& cloud, bool* new_cloud_available_flag)
// {
//   cloud_mutex.lock ();    // for not overwriting the point cloud from another thread
//   *cloud = *callback_cloud;
//   *new_cloud_available_flag = true;
//   cloud_mutex.unlock ();
// }

void callback(const sensor_msgs::PointCloud2ConstPtr& cloud){
  std::cout<<"Starting cb function\n";
  sensor_msgs::PointCloud2 msg = *cloud;
  pcl::PCLPointCloud2 pcl_cloud;
  pcl_conversions::toPCL(msg, pcl_cloud);
  std::cout<<"Converted\n";

  pcl::PCDWriter writer;
  writer.write("/home/turtlebot/oswin_stuff/pcl_tut/random.pcd", pcl_cloud, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), false);
 
  //pcl::io::savePCDFileASCII("random_kieran.pcd", pcl_cloud);
  //std::cout << "Wrote PCD file";
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_vis");
  std::cout << "Initialised\n";
  ros::NodeHandle n;
  std::cout << "Node-ified\n";
 /* float sampling_factor = 1;
  Eigen::Matrix3f rgb_intrinsics_matrix;
  rgb_intrinsics_matrix << 525, 0.0, 319.5, 0.0, 525, 239.5, 0.0, 0.0, 1.0;
  
  pcl::console::parse_argument (argc, argv, "--sample", sampling_factor);

  pcl::PCLPointCloud2::Ptr input_pcd_1 (new pcl::PCLPointCloud2());
  PointCloudT::Ptr input_pcd (new PointCloudT());
  sensor_msgs::PointCloud2::Ptr middle (new sensor_msgs::PointCloud2());
*/
  
  ros::Subscriber cloud_sub = n.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 1, callback);
   
  std::cout<<"Liked and subscribed\n";

  ros::spin();

 

/*
  pcl::toROSMsg (*input_pcd_1, *middle);
  pcl::fromROSMsg (*middle, *input_pcd);

  pcl::visualization::PointCloudColorHandlerRGBField<PointCloudT> rgb(input_pcd);
  viewer.addPointCloud<PointCloudT> (input_pcd, rgb, "input_cloud");
  viewer.setCameraPosition(0,0,-2,0,-1,0,0);

  pcl::people::GroundBasedPeopleDetectionApp<PointCloudT> people_detector;
  people_detector.setSamplingFactor(sampling_factor);

  static unsigned count = 0;
  static double last = pcl::getTime ();

  while (!viewer.wasStopped())
    {
      if (cloud_mutex.try_lock())
	{
	  viewer.removeAllPointClouds();
	  viewer.removeAllShapes();
	  pcl::visualization::PointCloudColorHandlerRGBField<PointCloudT> rgb(input_pcd);
	  viewer.addPointCloud<PointCloudT> (input_pcd, rgb, "input_cloud");
	  viewer.spinOnce();

	  // Display average framerate:                                                
	  if (++count == 30)
	    {
	      double now = pcl::getTime ();
	      std::cout << "Average framerate: " << double(count)/double(now - last) <<" Hz" <<  std::endl;
	      count = 0;
	      last = now;
	    }
	  cloud_mutex.unlock ();
	}
    }
*/
  return 0;
}

