 /* \copyright This work was completed by Robert Leishman while performing official duties as 
  * a federal government employee with the Air Force Research Laboratory and is therefore in the 
  * public domain (see 17 USC § 105). Public domain software can be used by anyone for any purpose,
  * and cannot be released under a copyright license
  */

/*!
 *  \file set_calibration.cpp
 *  \author Robert Leishman
 *
 *  \brief Simply implements the method to call the services to set the camera calibration files.
*/

#include "set_calibration.h"



SetCalibration::SetCalibration(ros::NodeHandle n)
{
  //initialize the strings for the URLs: (should probably be on the param server)
  url_rgb = "file://${HOME}/ros_workspace/kinect_visual_odometry/rgb_calibration/rgb_B00364612147051B.yaml";
  url_depth = "file://${HOME}/ros_workspace/kinect_visual_odometry/depth_calibration/depth_B00364612147051B.yaml";

  ros::ServiceClient rgb_client = n.serviceClient<sensor_msgs::SetCameraInfo>("rgb/set_camera_info");
  ros::ServiceClient depth_client = n.serviceClient<sensor_msgs::SetCameraInfo>("ir/set_camera_info");

  sensor_msgs::SetCameraInfo srv_rgb;
  sensor_msgs::SetCameraInfo srv_depth;

  //srv_rgb.request.camera_info.

}
