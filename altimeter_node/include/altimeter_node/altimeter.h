 /* \copyright This work was completed by Robert Leishman while performing official duties as 
  * a federal government employee with the Air Force Research Laboratory and is therefore in the 
  * public domain (see 17 USC § 105). Public domain software can be used by anyone for any purpose,
  * and cannot be released under a copyright license
  */

/*!
 *  \author Robert Leishman
 *  \date October 2012
 *
 *  \brief This class handles the ROS communication of the Altitude information
*/


#ifndef ALTITUDESERVER_H
#define ALTITUDESERVER_H

#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <cereal_port/CerealPort.h>
#include "control_toolbox/pid.h"
#include "mikro_serial/mikoCmd.h"


/// Number of bytes to read:
#define NUM_BYTES 4 //number of bytes to read at a time
#define TIMEOUT 25 //timeout in milisecs


/*!
 *  \class Altimeter altimeter.h "src/altimeter_node/altimeter.h"
 *  \brief Publishes the altitude information recieved from a sonar altimeter to a ROS topic
 *  \note I've expanded it to send thrust control commands to a mikrokopter hexacopter also
*/
class Altimeter
{

public:
  /*!
   *  \brief Constructor for the Altitude server class
   *
   *  \param nh the ROS node handle
  */
  Altimeter(ros::NodeHandle nh);


private:

  /*!
   *  \brief This function is a timer callback.  It brings in the raw data, extracts the height, and publishes it over ROS
   *
   *  The height comes in a string with R###.  The height in inches are the three numbers after the R. These numbers are
   *  converted to meters and the data is published.  An estimate of the down velocity is made using the position
   *  measurements.
  */
  void processSerial(const ros::TimerEvent &e);


  /*!
   *  \brief Quick function to saturate a double value
   *
  */
  inline double saturate(double val, double min, double max)
  {
    if(val > max)
    {
      val = max;
    }
    if(val < min)
    {
      val = min;
    }
    return val;
  }


  ros::Publisher alt_publisher_; //!< ROS publisher for publishing the altitude
  ros::Publisher cmd_publisher_; //!< publisher for sending out commands
  ros::Timer timer_; //!< timer for calling the callback
  cereal::CerealPort serial_; //!< the serial port class used to communicate

  bool compute_commands_; //!< flag for computing thrust control commands
  control_toolbox::Pid controller_; //!< PID controller for the height
  double desired_height_; //!< desired height for the control

  double altitude_bias_; //!< the bias in the altimeter to get the measurement up to the center of mass
  double down_limit_; //!< a heuristic to eliminate bad measurements
  double down_old_; //!< the most recent measurement (for calculating velocity and assessing the hueristic)
  double error_old_; //!< for control (if enabled)
  ros::Time time_old_; //!< for calculating the velocity
  double tau_; //!< for the dirty-derivative
  double differentiator_; //!< used to calculate a dirty-derivative of the height for a down velocity estimate



};

#endif // ALTITUDESERVER_H
