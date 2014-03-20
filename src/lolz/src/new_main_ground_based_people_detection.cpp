#include <iostream>
#include <cstdio>
#include <cstdlib>  
#include <ros/ros.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>    
#include <pcl/io/openni_grabber.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/people/ground_based_people_detection_app.h>
#include <pcl/common/time.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

// PCL viewer //
pcl::visualization::PCLVisualizer viewer("PCL Viewer");

// Mutex: //
boost::mutex cloud_mutex;

bool new_cloud_available_flag;

enum { COLS = 640, ROWS = 480 };

int print_help()
{
  cout << "*******************************************************" << std::endl;
  cout << "Ground based people detection app options:" << std::endl;
  cout << "   --help    <show_this_help>" << std::endl;
  cout << "   --svm     <path_to_svm_file>" << std::endl;
  cout << "   --conf    <minimum_HOG_confidence (default = -1.5)>" << std::endl;
  cout << "   --min_h   <minimum_person_height (default = 1.3)>" << std::endl;
  cout << "   --max_h   <maximum_person_height (default = 2.3)>" << std::endl;
  cout << "   --sample  <sampling_factor (default = 1)>" << std::endl;
  cout << "*******************************************************" <<std::endl;
}

 /*void cloud_cb_ (const PointCloudT::ConstPtr &callback_cloud, PointCloudT::Ptr& cloud,
    bool* new_cloud_available_flag)
{
  cloud_mutex.lock ();    // for not overwriting the point cloud from another thread
  *cloud = *callback_cloud;
  *new_cloud_available_flag = true;
  cloud_mutex.unlock ();
  }*/
 PointCloudT::Ptr cloud(new PointCloudT);

void callback(const sensor_msgs::PointCloud2ConstPtr& msg){
   cloud_mutex.lock();
   std::cout << "Entering callback\n";
   sensor_msgs::PointCloud2 msg0 = *msg;
   PointCloudT cloud0;
   pcl::fromROSMsg(msg0, cloud0);
   *cloud = cloud0;
   new_cloud_available_flag = true;
   std::cout << "Converted" << endl;
   cloud_mutex.unlock();
 }

struct callback_args{
  // structure used to pass arguments to the callback function
  PointCloudT::Ptr clicked_points_3d;
  pcl::visualization::PCLVisualizer::Ptr viewerPtr;
};
  
void
pp_callback (const pcl::visualization::PointPickingEvent& event, void* args)
{
  struct callback_args* data = (struct callback_args *)args;
  if (event.getPointIndex () == -1)
    return;
  PointT current_point;
  event.getPoint(current_point.x, current_point.y, current_point.z);
  data->clicked_points_3d->points.push_back(current_point);
  // Draw clicked points in red:
  pcl::visualization::PointCloudColorHandlerCustom<PointT> red (data->clicked_points_3d, 255, 0, 0);
  data->viewerPtr->removePointCloud("clicked_points");
  data->viewerPtr->addPointCloud(data->clicked_points_3d, red, "clicked_points");
  data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");
  std::cout << current_point.x << " " << current_point.y << " " << current_point.z << std::endl;
}

int main (int argc, char** argv)
{
  ros::init(argc, argv, "viewer_no_way");
  ros::NodeHandle n;
  new_cloud_available_flag = false;
  ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 1000, callback);
  double model_resolution;
  if(pcl::console::find_switch (argc, argv, "--help") || pcl::console::find_switch (argc, argv, "-h"))
        return print_help();

  // Algorithm parameters:
  std::string svm_filename = "/usr/share/doc/libpcl-1.7-doc/doc/pcl-1.7/tutorials/sources/ground_based_rgbd_people_detection/data/trainedLinearSVMForPeopleDetectionWithHOG.yaml";
  float min_confidence = -1.5;
  float min_height = 1.3;
  float max_height = 2.3;
  float voxel_size = 0.06;
  float sampling_factor = 1;
  Eigen::Matrix3f rgb_intrinsics_matrix;
  rgb_intrinsics_matrix << 525, 0.0, 319.5, 0.0, 525, 239.5, 0.0, 0.0, 1.0; // Kinect RGB camera intrinsics

  // Read if some parameters are passed from command line:
  pcl::console::parse_argument (argc, argv, "--svm", svm_filename);
  pcl::console::parse_argument (argc, argv, "--conf", min_confidence);
  pcl::console::parse_argument (argc, argv, "--min_h", min_height);
  pcl::console::parse_argument (argc, argv, "--max_h", max_height);
  pcl::console::parse_argument (argc, argv, "--sample", sampling_factor);

  // Read Kinect live stream:
  std::cout<<"initiate reading\n";
  //ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 50, callback);
 
  // boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
    // boost::bind (&cloud_cb_, _1, cloud, &new_cloud_available_flag);
  // std::cout<<"did boost thingy\n";
  /*interface->registerCallback (f);
  std::cout<<"did registerCallback(f)\n";
  interface->start ();
  std::cout<<"did start()\n";*/
  unsigned int x=0;
  // Wait for the first frame:
  while(!new_cloud_available_flag) 
    {
        if(x%1000==0||x==0)
	  std::cout<<"waiting for first frame\n";
	x++;
	ros::Duration(0.001).sleep();
	ros::spinOnce();
    }
 // if(!n.ok())
 //   return 0;
  std::cout<<"got first frame\n";
  new_cloud_available_flag = false;

  cloud_mutex.lock ();    // for not overwriting the point cloud
  std::cout<<"locked mutex\n";

  // Display pointcloud:
  pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
  viewer.addPointCloud<PointT> (cloud, rgb, "input_cloud");
  std::cout<<"did viewer.addPointCloud\n";
  viewer.setCameraPosition(0,0,-2,0,-1,0,0);
  std::cout<<"did viewer.setCamPos\n";

  // Add point picking callback to viewer:
  struct callback_args cb_args;
  PointCloudT::Ptr clicked_points_3d (new PointCloudT);
  cb_args.clicked_points_3d = clicked_points_3d;
  cb_args.viewerPtr = pcl::visualization::PCLVisualizer::Ptr(&viewer);
  viewer.registerPointPickingCallback (pp_callback, (void*)&cb_args);
  std::cout << "did viewer.regPPCB\n"; 
  std::cout << "Shift+click on three floor points, then press 'Q'..." << std::endl;

  // Spin until 'Q' is pressed:
  viewer.spin();
  std::cout << "done." << std::endl;
  
  cloud_mutex.unlock ();
  std::cout<<"unlocked mutex\n";

  // Ground plane estimation:
  Eigen::VectorXf ground_coeffs;
  ground_coeffs.resize(4);
  std::vector<int> clicked_points_indices;
  for (unsigned int i = 0; i < clicked_points_3d->points.size(); i++)
    {
      clicked_points_indices.push_back(i);
      std::cout<<"did indices.push_back\n"; //Does not show :S
    }
  pcl::SampleConsensusModelPlane<PointT> model_plane(clicked_points_3d);
  model_plane.computeModelCoefficients(clicked_points_indices,ground_coeffs);
  std::cout << "computed plane\n";
  std::cout << "Ground plane: " << ground_coeffs(0) << " " << ground_coeffs(1) << " " << ground_coeffs(2) << " " << ground_coeffs(3) << std::endl;

  // Initialize new viewer:
  pcl::visualization::PCLVisualizer viewer("PCL Viewer");          // viewer initialization
  std::cout << "Initialized new viewer" << std::endl;
  viewer.setCameraPosition(0,0,-2,0,-1,0,0);
  std::cout << "Set camera position" << std::endl;
  // Create classifier for people detection:  
  pcl::people::PersonClassifier<pcl::RGB> person_classifier;
  person_classifier.loadSVMFromFile(svm_filename);   // load trained SVM
  std::cout << "Initialized and loaded person classifier" << std::endl;

  // People detection app initialization:
  pcl::people::GroundBasedPeopleDetectionApp<PointT> people_detector;    // people detection object
  people_detector.setVoxelSize(voxel_size);                        // set the voxel size
  people_detector.setIntrinsics(rgb_intrinsics_matrix);            // set RGB camera intrinsic parameters
  people_detector.setClassifier(person_classifier);                // set person classifier
  people_detector.setHeightLimits(min_height, max_height);         // set person classifier
  people_detector.setSamplingFactor(sampling_factor);              // set a downsampling factor to the point cloud (for increasing speed)
//  people_detector.setSensorPortraitOrientation(true);              // set sensor orientation to vertical
  std::cout << "set some parameters.." << std::endl;
  // For timing:
  static unsigned count = 0;
  static double last = pcl::getTime ();
  
  unsigned int a = 0;

  // Main loop:
  while (!viewer.wasStopped() && ros::ok())
  {
    if (a%1000 == 0 || a==0)
      std::cout << "Entered loop\n"
		<< new_cloud_available_flag << std::endl;
    a++;
    if (new_cloud_available_flag && cloud_mutex.try_lock ())    // if a new cloud is available
    {
      std::cout << "Met condition" << std::endl;
      new_cloud_available_flag = false;

      // Perform people detection on the new cloud:
      std::vector<pcl::people::PersonCluster<PointT> > clusters;   // vector containing persons clusters
      people_detector.setInputCloud(cloud);
      std::cout << "set input cloud" << std::endl;
      people_detector.setGround(ground_coeffs);   
      std::cout <<"setGround"<< std::endl;               // set floor coefficients
      people_detector.compute(clusters);                           // perform peopxle detection
      std::cout <<"computed clusters"<< std::endl;
      ground_coeffs = people_detector.getGround();                 // get updated floor coefficients
      std::cout << "get ground" << std::endl;
      // Draw cloud and people bounding boxes in the viewer:
      viewer.removeAllPointClouds();
      std::cout <<"removed PCs"<< std::endl;
      viewer.removeAllShapes();
      std::cout << "removed shapes" << std::endl;
      pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
      viewer.addPointCloud<PointT> (cloud, rgb, "input_cloud");
      std::cout << "Added point cloud to viewer" << std::endl;
 //ros::spinOnce();
      unsigned int k = 0;
      for(std::vector<pcl::people::PersonCluster<PointT> >::iterator it = clusters.begin(); it != clusters.end(); ++it)
      {
        if(it->getPersonConfidence() > min_confidence)             // draw only people with confidence above a threshold
        {
          // draw theoretical person bounding box in the PCL viewer:
          it->drawTBoundingBox(viewer, k);
          k++;
        }
      }
      std::cout << "Did iterator things" << std::endl;

      if(k==1)
	std::cout<<k<<" person found" << std::endl;
      else
        std::cout << k << " people found" << std::endl;
      viewer.spinOnce();
      // Display average framerate:
      if (++count == 30)
      {
        double now = pcl::getTime ();
        std::cout << "Average framerate: " << double(count)/double(now - last) << " Hz" <<  std::endl;
        count = 0;
        last = now;
      }
      cloud_mutex.unlock ();
    }
    ros::spinOnce();
  }
  ros::spin();
  return 0;
}
