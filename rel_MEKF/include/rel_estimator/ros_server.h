 /* \copyright This work was completed by Robert Leishman while performing official duties as 
  * a federal government employee with the Air Force Research Laboratory and is therefore in the 
  * public domain (see 17 USC § 105). Public domain software can be used by anyone for any purpose,
  * and cannot be released under a copyright license
  */

/*! \file ros_server.h
  * \author Robert Leishman
  * \date June 2012
  *
  * \brief The ros_server.h file is the header for the ros_server class.
  *
  *
*/

#ifndef ROS_SERVER_H
#define ROS_SERVER_H

#include <queue>
#include <deque>
#include <pthread.h>
#include <boost/thread.hpp>
#include <Eigen/Core>
#include <Eigen/StdVector>
#include <Eigen/StdDeque>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include "mikro_serial/mikoImu.h"
#include "rel_estimator/estimator.h"
#include "rel_estimator/constants.h"
#include "rel_estimator/vodata.h"
#include "rel_MEKF/relative_state.h"
#include "rel_MEKF/edge.h"
#include <visualization_msgs/Marker.h>

/*! These mutex's allow the different measurements to be placed into the queues asynchronously.  I maybe could have done
     it all with only one, but I thought it would be better to cover each queue with a different mutex allowing more
     flexibility.
*/
extern pthread_mutex_t w_mutex_; //!< the mutex for the while_true_ access
extern pthread_mutex_t i_mutex_; //!< mutex for IMU data
extern pthread_mutex_t v_mutex_; //!< mutex for vision data
extern pthread_mutex_t a_mutex_; //!< mutex for altitude data
extern pthread_mutex_t t_mutex_; //!< mutex for truth data
extern pthread_mutex_t h_mutex_; //!< mutex for hex data


/*!
 *  \class ROSServer ros_server.h "include/rel_estimator/ros_server.h"
 *  \brief The ROSServer class subscribes and publishes to ROS messages and provides the main loop for the project.
 *
 *  This class listens for visual odometry, IMU, altitude, and motion capture system messages and publishes the current
 *  vehicle pose and covariance, the keyframe pose and covariance, and when available, the truth expressed in the current
 *  relative frame.  This code is the embodiment of the publication: \todo fill in the publication for the rel_MEKF!
*/
class ROSServer
{
public:


  /*!
   *  \brief The constructor for the ROSServer class.
   *
   *  \param nh is the node handle for the current ROS node
   *  \param constants is a pointer to the Constants class.
  */
  ROSServer(ros::NodeHandle &nh, Constants *mk_const);


  /*!
   *  \brief The destructor, it destroys things...
   *
  */
  ~ROSServer();


  /*!
   *  \brief The Run function contains the main loop for the software.
   *
   *  At each loop, the queues that contain information are checked, when information is available, one loop is processed.
   *  This function is run on it's own thread, the original thread is used to check ROS for messages from the sensors
   *  and place those into queues.  This function then pulls the data out of the queues as needed.  Ideally, the Run
   *  function should be fast enough to keep the queue sizes low.
  */
  void Run();


  /*!
   *  \brief this function provides thread-safe read/write access to the variable while_true_, which is the condition
   *  that the while loop uses in the Run method.
   *
   *  \param while_true_value should be set to 0 to stop the loop and exit the thread, the default is 1 (when used, the
   *  function returns the current value of while_true_value_.
  */
  int accessWhileTrue(int while_true_value = 1);


protected:

  /*!
   *  \brief This function is called when ROS recieves an IMU message.
   *
   *  The IMU is the drumbeat, if it were, of the estimation process.  The estimator runs through one iteration only
   *  when IMU info is available.  Visual and altitude data are
   *  incorporated into the estimator only on these drumbeats (but at the times designated by their timestamps).
   *
   *  \param imu_message is the message sent by a strap-down IMU.  It comes at about 100Hz.
  */
  void imuCallback(const IMU_message &imu_message);


  /*!
   *  \brief This function is called when ROS recieves a visual odometry (view matching) message.
   *
   *  This function saves the view matching information to a queue that is then processed in the main loop of the program,
   *  which resides in the IMU callback.
   *
   *  \param vo_message is the ROS message returned by the kinect visual odometry message. It comes at either 15 or 30 Hz.
  */
  void visualCallback(const k_message &vo_message);


  /*!
   *  \brief This function processes that altitude measurements recieved through ROS.
   *
   *  The altimeter server already brings the data into the appropriate units and adds on the necessary bias to get
   *  the measurement up to the center of mass of the aircraft. However, the laser server does not do this.
   *
   *  \param alt_message is a range message with the altimeter or laser information inside.  Altimeter comes at about 40Hz and laser comes at about 10Hz
  */
  void altCallback(const sensor_msgs::Range &alt_message);


  /*!
   *  \brief The function that is called to process truth information published from a motion capture system
   *
   *  \note This function assumes that the motion capture truth is expressed in a north-east-down coordinate system.
   *  For a motion analysis system, it also assumes that the Euler Sequence is 321 (then we further change it to the NED
   *  system).
   *
   *  \param truth is the 6 DoF pose, with the rotation expressed as a quaternion.
  */
  void truthCallback(const TRUTH_message &truth);


  /*!
   *  \brief This callback recieves the debug data from the Hexacopter
   *  \param hex_message comes from the mikro_serial node at about 20Hz
  */
  void hexCallback(const Hex_message &hex_message);



  //variables:
  ros::Publisher rel_state_publisher_; //!< the publisher for the relative 6DoF state and covariance info
  ros::Publisher global_pose_publisher_; //!< publishes the global pose estimate (TEMPORARY!!)
  ros::Publisher node_global_pub_; //!< publishes the current node's global pose
  ros::Publisher edge_pub_; //!< publishes the edge when a new node is created

#ifdef LASER
#ifdef DETECT
  ros::Publisher status_pub_; //!< publishes the laser's operational status
#endif
#endif


  ros::Subscriber imu_subscriber_; //!< the subscriber for IMU data
  ros::Subscriber vo_subscriber_; //!< the subscriber for VO data
  ros::Subscriber alt_subscriber_; //!< the altitude subscriber
  ros::Subscriber truth_subscriber_; //!< the truth (from motion capture) subscriber
  ros::Subscriber hex_subscriber_; //!< for the debug data out of the hexacopter

  tf::TransformBroadcaster relative_tf_; //!< for broadcasting the relative state for mapping in the node frame
  std::string node_frame_name_; //!< name for the node frame for publishing a tf for relative states
  std::string body_frame_name_; //!< name for the body-fixed frame
  std::string global_frame_name_; //!< the name for the global coordinate frame (for mapping global information)
  std::string global_body_frame_name_; //!< the name for the current global position
  std::string base_node_name_; //!< the base name for each of the node global coordinates (to visualize the only the nodes)

  /// Queues for collecting IMU, VO, Altitude, and Truth data: (These queues are accessed by two threads)
  std::queue<IMU_message> imu_queue_; //!< the IMU queue
  /// \note When a class has fixed-size Eigen members, you must use an aligned allocator for standard containers:
  /// See: http://eigen.tuxfamily.org/dox/TopicStlContainers.html
  std::deque<VO_message, Eigen::aligned_allocator<VO_message> > vo_queue_; //!< the queue holding all the vo messages
  std::queue<sensor_msgs::Range> alt_queue_; //!< the queue for altitude
  std::queue<TRUTH_message> truth_queue_; //!< the truth queue
  std::queue<Hex_message> hex_queue_; //!< the queue for hexacopter messages

  static const double ACCZ_LANDED_ = -20.0; //!< if two consecutive accz measurements are below this, we've touched ground.
  double past_accz_; //!< for use with determining if we've landed.
  ros::Time landed_time_; //time we landed

  double old_time_; //!< holder for the previous IMU packet time
  double dt_; //!< the delta between the past IMU timestep and the current

  Estimator *estimator_; //!< the instance of the estimator class that implements the EKF
  Constants *mk_consts_; //!< the instance of the constants class that provides, you guessed it, constants

  volatile int while_true_; //!< the value to put low when the thread for the Run method should exit

};



















#endif
