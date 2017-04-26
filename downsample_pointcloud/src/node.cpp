/*!
 *  \brief This file is based on an example found on: http://www.ros.org/wiki/pcl/Tutorials
 *  It downsamples a pointcloud2 message and republishes it.  This is the node version of this code.
 *  \date March 2013
 *  \author Robert Leishman
*/

#include <opencv2/core/core.hpp>
#include <ros/ros.h>
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

ros::Publisher pub;

double x_,y_,z_;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud)
{
//  double start = cv::getTickCount();
  sensor_msgs::PointCloud2 cloud_filtered;

  // Perform the actual filtering
  pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (x_,y_,z_); // (0.01, 0.01, 0.01);
  sor.filter (cloud_filtered);

  // Publish the data
  pub.publish (cloud_filtered);
//  double time = (cv::getTickCount() - start)/cv::getTickFrequency();
//  std::cout << "Processing Time: " << time << std::endl;
//  std::cout << "Points before: " << cloud->width * cloud->height << " data points. " << std::endl;
//  std::cout << "Points after: " << cloud_filtered.width * cloud_filtered.height << " data points. " << std::endl;
}


int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pc_downsampler");
  ros::NodeHandle nh;
  std::string cloud_name, output;
  nh.param<double>("x_leaf_size", x_,0.05);
  nh.param<double>("y_leaf_size", y_,0.05);
  nh.param<double>("z_leaf_size", z_,0.10);
  nh.param<std::string>("input_cloud_name",cloud_name,"/cloud_throttled");
  nh.param<std::string>("output_cloud_name",output,"/downsampled_cloud");

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe (cloud_name, 1, cloud_cb); ///camera/depth_registered/points

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> (output, 1);

  // Spin
  ros::spin ();
}
