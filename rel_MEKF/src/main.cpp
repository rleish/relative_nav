 /* \copyright This work was completed by Robert Leishman while performing official duties as 
  * a federal government employee with the Air Force Research Laboratory and is therefore in the 
  * public domain (see 17 USC § 105). Public domain software can be used by anyone for any purpose,
  * and cannot be released under a copyright license
  */

/*!
 *  \title Relative Estimator
 *  \author Robert Leishman
 *  \date June 2012
 *  \brief This project estimates the relative states of hexacopter rotorcraft in GPS-denied environments.
 *
 *
 */

#include <ros/ros.h>
#include "rel_estimator/ros_server.h"
#include "rel_estimator/constants.h"
#include <boost/thread.hpp>



/*!
 *  \brief The main function starts ROS and hands it off to ros_server.
 *
 *
*/
int main(int argc, char **argv)
{

  ros::init(argc,argv,"relative_estimation");

  /// \note Any string passed into the node handle like this will be prepended to any topics published by this node
  ros::NodeHandle nh("relative");

  Constants consts;

  ROSServer server(nh, &consts);

  //Start another thread to run the main loop:
  boost::thread* process_thread = new boost::thread(&ROSServer::Run,&server);

  ros::Rate r(150);

  while(ros::ok())
  {
    ros::spinOnce();
    r.sleep();
    //make sure the other thread is still running, if not, finish up and exit
    if(server.accessWhileTrue() != 1)
      break;
  }

  //send signal to other thread to exit
  server.accessWhileTrue(0);

  process_thread->timed_join(boost::posix_time::seconds(2)); //wait a couple seconds for it to finish.

  delete process_thread;

  pthread_mutex_destroy(&w_mutex_);
  pthread_mutex_destroy(&i_mutex_);
  pthread_mutex_destroy(&v_mutex_);
  pthread_mutex_destroy(&a_mutex_);
  pthread_mutex_destroy(&t_mutex_);
  pthread_mutex_destroy(&h_mutex_);

}
