 /* \copyright This work was completed by Robert Leishman while performing official duties as 
  * a federal government employee with the Air Force Research Laboratory and is therefore in the 
  * public domain (see 17 USC § 105). Public domain software can be used by anyone for any purpose,
  * and cannot be released under a copyright license
  */

/*! \file ros_server.h
  * \author Robert Leishman
  * \date Sept 2012
  *
  * \brief The ros_server.h file is the header for the ros_server class.
  *
  *
*/

#ifndef ROS_SERVER_H
#define ROS_SERVER_H

#include <iostream>
#include <fstream>
#include <queue>
#include <deque>
#include <pthread.h>
//#include <boost/thread.hpp>
#include <Eigen/Core>
#include <Eigen/StdVector>
#include <Eigen/StdDeque>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <mikro_serial/mikoCmd.h>
#include <mikro_serial/mikoImu.h>
#include <evart_bridge/transform_plus.h>
#include "controller/diff_flat.h"
#include <rel_MEKF/relative_state.h>
#include <rel_MEKF/edge.h>
#include <diff_flat_control/request_turn.h>




extern pthread_mutex_t mutex_; //!< mutex just in case for the waypoint_list_ vector (used by multiple callbacks)


/*!
 *  \class ROSServer ros_server.h "include/controller/ros_server.h"
 *  \brief The ROSServer class subscribes and publishes to ROS messages and provides the main loop for the project.
 *
 *  This class listens for sensor fusion (MEKF) and motion capture system messages and publishes the control commands
 *  to the mikro_serial node (which then pushes the commands to the autopilot using serial)
*/
class ROSServer
{
public:
  /// Eigen macro used when there are fixed-sized class member variables and you dynamically create an instance of the
  /// class.  (See: http://eigen.tuxfamily.org/dox/TopicStructHavingEigenMembers.html)
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /*!
   *  \brief The constructor for the ROSServer class.
   *
   *  \param nh is the node handle for the current ROS node
  */
  ROSServer(ros::NodeHandle &nh);


  /*!
   *  \brief The destructor, it destroys things...
   *
  */
  ~ROSServer();


  /*!
   *  \brief This service allows you to request that the hex rotate in place to right or left by degrees.
   *  Right and left are determined from the hex perspective, looking out the front (as if you were the hex)
   *  \attention This function switches the flag ROSServer::goal_achieved_ so that progress will stop along the specified
   *  path.  The planner will require a new goal location to again follow a waypoint path.
   *  \param turn_name contains the string "direction" that specifies right or left (from the hex point of view)
   *  \param flag is where the response "true" or "false" is specified.
  */
  inline bool requestTurnCallback(diff_flat_control::request_turn::Request &turn_name,
                                  diff_flat_control::request_turn::Response &flag)
  {
    if(turn_name.direction == "left" || turn_name.direction == "right")
    {
      /// We change the flag goal_achieved here to stop progress on the specified path - then we hover and turn.
      /// After the turn the opterator must send in a new goal, to compute a new path.
      goal_achieved_ = true;
      new_yaw_goal_ = true;
      yaw_goal_ = turn_name.angle*3.14159265359/180.0;
      if(turn_name.direction == "left")
      {
        yaw_goal_*= -1.0;
      }
      flag.suceed = true;
    }
    else
    {
      flag.suceed = false;
    }
    return true;
  }






protected:

  /*!
   *  \brief This function is called when ROS recieves a relative MEKF message with the estimated states of the filter.
   *
   *  This function computes the control based on the recieved estimated states and publishes a control message
   *  to be passed into the mikrokopter autopilot
   *
   *  \param ekf_message is the ROS message returned by the relative MEKF.
  */
  void ekfCallback(const rel_MEKF::relative_state &ekf_message);


  /*!
   *  \brief The function that is called calculate control based on truth from a motion capture system
   *
   *  \note This function assumes that the motion capture truth is expressed in a north-east-down coordinate system.
   *  For a motion analysis system, it also assumes that the Euler Sequence is 321 (then we further change it to the NED
   *  system).
   *
   *  \param truth is the 6 DoF pose, with the rotation expressed as a quaternion.
  */
  void truthCallback(const evart_bridge::transform_plus &truth);


  /*!
   *  \brief This callback brings in the navigation path and puts it into the waypoint form for the control to follow
   *  \param path is documented in http://www.ros.org/doc/api/nav_msgs/html/msg/Path.html
  */
  void pathCallback(const nav_msgs::Path &path);


  /*!
   *  \brief When a new node is made, this callback captures the edge information, so that the current waypoints
   *  (and hoverpoint) can be expressed in the new node frame.
  */
  void edgeCallback(const rel_MEKF::edge &new_edge);


  /*!
   *  \brief Subscribe to the debug out stuff from the hexacopter to get battery voltage
   *  \todo Make batt_voltage_ threadsafe
  */
  void hexCallback(const mikro_serial::mikoImu &imu_packet)
  {
    batt_voltage_ = (double)imu_packet.batt;
  }


  /*!
   *  \brief Subscribe to the goal location here - if we reach it, don't do path control until a new goal location is
   *  sent.
   *
  */
  void goalLocationCallback(const geometry_msgs::PoseStamped goal)
  {
    pthread_mutex_lock(&mutex_);
      //The goal location is really a 2D point in NWU coordinates, convert to NED and add a down component:      
      Eigen::Vector3d goal_vec(goal.pose.position.x,-goal.pose.position.y,desired_global_z_);
      if(!goal_vec.isApprox(goal_location_,0.01))
      {
        goal_location_ = goal_vec;
        new_goal_received_ = true;
      }

      /// \todo Use the quaternion contained in the goal location to specifiy the final yaw angle of the hex.  Meaning
      /// that when it reaches hover at the end, set this yaw (expressed in the right node frame) as the final yaw
      /// angle for hover.

    pthread_mutex_unlock(&mutex_);
  }


  /*!
   *  \brief Get the current node global position to determine desired relative height
   *  \attention THE COORDINATES FOR THE GLOBAL POSITION ARE IN NWU, FOLLOWING ROS CONVENTION!
   *  \param node_g is the global transform sent by the estimator - it is in NWU coordinates
  */
  void nodeGlobalPoseCallback(const geometry_msgs::TransformStamped node_g)
  {
    pthread_mutex_lock(&mutex_);
      node_global_z_ = -node_g.transform.translation.z;
    pthread_mutex_unlock(&mutex_);
  }

  /*!
   *  \brief This function uses a linear interpolation to reduce the number of waypoints in the path.  The output of the
   *  navfnROS package places nodes about every inch along the path, we do not need it that dense.  This function quickly
   *  trims it down by about an order of magnitude.
   *  \param[in,out] waypoint_list is the converted nav_msgs::Path from the planner
  */
  inline void sparsifyWaypointPath(std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > *waypoint_list)
  {
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > new_waypt_list;
    new_waypt_list.push_back(waypoint_list->at(0));
    double m,b; //slope and intercept
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >::iterator i; //access to new_waypt_list
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >::iterator j,k; //access to *waypoint_list
    i = new_waypt_list.begin();
    j = waypoint_list->begin();
    j++; //start at the 2nd element;
    k = j+1;
    Eigen::Vector3d kvec, ivec, jvec;
    kvec = *k;
    ivec = *i;

    m = (kvec(1) - ivec(1))/(kvec(0) - ivec(0)); //rise over run
    b = ivec(1) - m*(ivec(0)); //use the first point in new_waypt_list combined with slope to find intercept

    while( j != waypoint_list->end())
    {
      double y_predict, error_y;
      jvec = *j;
      y_predict = m*(jvec(0)) + b;
      error_y = jvec(1) - y_predict;

      if(fabs(error_y) > Y_LIMIT_)
      {
        new_waypt_list.push_back(*j);
        i++;

        if(j + 2 == waypoint_list->end())
        {
          //we've added the 2nd to last waypoint, that is good enough, just break
          break;
        }
        else
        {
          k = j + 1;
          kvec = *k;
          ivec = *i;

          m = (kvec(1) - ivec(1))/(kvec(0) - ivec(0)); //rise over run
          b = ivec(1) - m*(ivec(0)); //use the first point in new_waypt_list combined with slope to find interceptt
        }
      }
      else if(j + 2 == waypoint_list->end())
      {
        //End of list and we didn't finish with adding a waypoint - add the last one just in case:
        k = j + 1;
        new_waypt_list.push_back(*k);
        break;
      }
      j++;
    }
    waypoint_list->swap(new_waypt_list);
  }


  //variables:
  ros::Publisher dilluted_path_pub_; //!< this is for debug - it publishes the reduced form of the nav path for display
  ros::Publisher command_publisher_; //!< the publisher for the 6DoF pose and covariance info  
  ros::Subscriber ekf_subscriber_; //!< the subscriber for EKF data
  ros::Subscriber truth_subscriber_; //!< the truth (from motion capture) subscriber
  ros::Subscriber plan_subscriber_; //!< for getting the navigation planned path
  ros::Subscriber edge_sub_;  //!< recieves the new edges and applies the change in pose to the waypoints/hoverpoints  
  ros::Subscriber goal_subscriber_; //!< this subscribes to a goal location
  ros::Subscriber hex_subscriber_; //!< subscribes to hexacopter debug stuff (mikoImu) to get battery voltage.
  ros::Subscriber node_sub_; //!< gets the node global position, to determine the desired height
  ros::ServiceServer yaw_server_; //!< allows requests to yaw in place, either to right or left by 45 degrees.

  Eigen::Vector3d goal_location_; //!< current goal location (in global coordinates)!
  bool goal_achieved_; //!< flag that is returned from control if the goal has been achieved.
  bool new_goal_received_; //!< flag for when we get a new goal.
  double sec_to_hover_; //!< the # of sec to hover before starting to follow any waypoint paths.
  double yaw_goal_; //!< the value for the hover_yawpoint in diff_flat:: class
  bool new_yaw_goal_; //!< specifies if the yaw goal is new or not.

  double batt_voltage_; //!< most recent battery voltage from the debug out from the hex
  bool new_batt_volt_;  //!< flag for battery voltage

  ros::Time old_time_; //!< to save the last timestamp that control was called
  ros::Time flying_; //!< time that the vehicle started flying
  bool flying_flag_; //!< flag for setting flying_

  std::string ekf_topic_; //!< the EKF topic name
  std::string truth_topic_; //!< the topic for the motion capture truth
  std::string command_topic_; //!< the control command topic that is published by this class
  double desired_global_z_; //!< the global z value to place all the waypoints at (for 2.5 D flight)
  double node_global_z_; //!< the node global z to determine the desired height in relative frame.
  static const double Y_LIMIT_ = 0.05; //!< the limit on difference between line fit and actual y in smoothWaypointPath (meters)

  DiffFlat *controller_; //!< Instance of the controller

  bool new_waypoint_list_; //!< flag for determining when to pass in a new waypoint list
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > waypoint_list_; //!< used for passing a new list to the control
  /// \note The waypoints should be in global coordinates.  They are converted into node coordinates by the functions
  /// in the diff_flat class (NEED TO FIGURE THIS OUT FOR RELATIVE COORDINATES)

  //std::ofstream log_file_;
  //std::ofstream log_file_out_; //!< for writing the log file for sending out the data

  rel_MEKF::relative_state old_state_; //!< used for checking when the relative state has changed before we recieve
  /// the new edge information.
};

#endif
