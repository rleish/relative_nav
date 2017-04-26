 /* \copyright This work was completed by Robert Leishman while performing official duties as 
  * a federal government employee with the Air Force Research Laboratory and is therefore in the 
  * public domain (see 17 USC § 105). Public domain software can be used by anyone for any purpose,
  * and cannot be released under a copyright license
  */

/*! \file main.cpp
  * \author Robert Leishman
  * \date March 2012
  * \brief main.cpp is for the project kinect_visual_odometry.  It does not provide much functionality,
  * and really only starts ROS and runs the ros_relay class.
  *
*/

#include <sys/resource.h>

#include <ros/ros.h>
#include "ros_relay.h"





/*!
 *  \brief The main starts ROS and an instance of ROSRelay to handle the communciation of images and 6 DOF pose changes.
 *
*/
int main(int argc, char **argv)
{   


  //start the ROS node
  ros::init( argc, argv, "kinect_visual_odometry"); //!< start the node
  ros::NodeHandle nh("kinect_visual_odometry");

  ROSRelay rly(nh);

  ros::Rate r(60); // (x) is the run-rate in Hz

  //main loop
  while(ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }

  pthread_mutex_destroy(&mutex_);
  rly.~ROSRelay();

}
