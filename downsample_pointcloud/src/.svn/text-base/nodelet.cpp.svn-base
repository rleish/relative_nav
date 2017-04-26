/*!
 *  \brief This is a nodelet for downsampling point clouds
 *
*/

#include <opencv2/core/core.hpp>
#include "ros/ros.h"
#include "pluginlib/class_list_macros.h"
#include "nodelet/nodelet.h"
#include "sensor_msgs/LaserScan.h"
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>


namespace downsample_pointcloud
{
class CloudThrottleNodelet : public nodelet::Nodelet
{
public:
  //Constructor
  CloudThrottleNodelet()
  {
  }

private:

  virtual void onInit()
  {
    ros::NodeHandle& nh = getNodeHandle();
    ros::NodeHandle& private_nh = getPrivateNodeHandle();

    std::string cloud_name, output;
    x_=0.05;
    y_=0.05;
    z_=0.1;

    private_nh.param<double>("x_leaf_size", x_,0.05);
    private_nh.param<double>("y_leaf_size", y_,0.05);
    private_nh.param<double>("z_leaf_size", z_,0.10);
    private_nh.param<std::string>("input_cloud_name",cloud_name,"/cloud_throttled");
    private_nh.param<std::string>("output_cloud_name",output,"/downsampled_cloud");

    pub_ = nh.advertise<sensor_msgs::PointCloud2>(output, 10);
    sub_ = nh.subscribe(cloud_name, 10, &CloudThrottleNodelet::callback, this);
  }


  void callback(const sensor_msgs::PointCloud2ConstPtr& cloud)
  {
//    double start = cv::getTickCount();

    sensor_msgs::PointCloud2 cloud_filtered;

    // Perform the actual filtering
    pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (x_, y_, z_); // (0.01, 0.01, 0.01);
    sor.filter (cloud_filtered);

    // Publish the data
    pub_.publish (cloud_filtered);

//    double time = (cv::getTickCount() - start)/cv::getTickFrequency();
//    std::cout << "Processing Time: " << time << std::endl;
//    std::cout << "Points before: " << cloud->width * cloud->height << " data points. " << std::endl;
//    std::cout << "Points after: " << cloud_filtered.width * cloud_filtered.height << " data points. " << std::endl;
  }

  double x_,y_,z_;
  ros::Publisher pub_;
  ros::Subscriber sub_;

};

//This code is essential for making the nodelet
PLUGINLIB_DECLARE_CLASS(downsample_pointcloud, CloudThrottleNodelet, downsample_pointcloud::CloudThrottleNodelet, nodelet::Nodelet);
}
