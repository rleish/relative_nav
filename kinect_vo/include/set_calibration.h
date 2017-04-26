 /* \copyright This work was completed by Robert Leishman while performing official duties as 
  * a federal government employee with the Air Force Research Laboratory and is therefore in the 
  * public domain (see 17 USC § 105). Public domain software can be used by anyone for any purpose,
  * and cannot be released under a copyright license
  */

/*!
 *  \file set_calibration.h
 *  \author Robert Leishman
 *  \date March 2012
 *
 *  \brief Has a class that sets the calibration for the kinect that is being used.
*/

#ifndef SET_CALIBRATION_H
#define SET_CALIBRATION_H

#include <ros/ros.h>
#include <sensor_msgs/SetCameraInfo.h>




/*!
 *  \class SetCalibration set_calibration.h "\include\set_calibration.h"
 *
 *  \brief This class defines a client node for setting the calibration info for the openni_camera drivers.
 *
 *  It will block while the service calls are being made.  Should only happen once, at the beginning of the program.
*/
class SetCalibration
{

protected:
  std::string url_rgb;  //!< the url that points to the camera calibration file for the rgb camera
  std::string url_depth; //!< the url for the depth camera calibration file


public:
  /*!
   *  \brief The constructor requires the ROS node handle and it sets everything up for the service call.
   *
   *  \param n is the ROS node handle
  */
  SetCalibration(ros::NodeHandle n);


  /*!
   *  \brief This method actually performs the service calls and recieves the responses.
   *
   *  Both the RGB and Depth camera parameters are set using this method
   *
   *  \exception An exception is thrown if the response fails
  */
  void Submit();

};



#endif
