 /* \copyright This work was completed by Robert Leishman while performing official duties as 
  * a federal government employee with the Air Force Research Laboratory and is therefore in the 
  * public domain (see 17 USC § 105). Public domain software can be used by anyone for any purpose,
  * and cannot be released under a copyright license
  */

/*!
 *  \title Relative Estimator
 *  \author Robert Leishman
 *  \date Sept 2012
 *  \brief This project provides control commands given either truth or relative estimates.
 *
 *  This code was origninally written in C# and on Windows.  This is a newer, improved version of that code to interface
 *  with ROS.
 */

#include <pthread.h>
#include <ros/ros.h>
#include "controller/ros_server.h"


/*!
 *  \brief The main function starts ROS and hands it off to ros_server.
 *
*/
int main(int argc, char **argv)
{

  ros::init(argc,argv,"control");

  ros::NodeHandle nh("control");

  ROSServer server(nh);

  //We want the control called at a specific rate - if it comes too fast, the hex cannot process all the commands
  ros::Rate r(25);

  while(ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }

  pthread_mutex_destroy(&mutex_);
  server.~ROSServer();
}
