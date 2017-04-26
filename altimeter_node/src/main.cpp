 /* \copyright This work was completed by Robert Leishman while performing official duties as 
  * a federal government employee with the Air Force Research Laboratory and is therefore in the 
  * public domain (see 17 USC § 105). Public domain software can be used by anyone for any purpose,
  * and cannot be released under a copyright license
  */

/*!
 *  \file main.cpp
 *  \author Robert Leishman
 *  \date October 2012
 *  \brief This file runs a serial port to access range data from a a sonar rangefinder.
*/

#include <ros/ros.h>
#include <cereal_port/CerealPort.h>
#include "control_toolbox/pid.h"
#include "altimeter_node/altimeter.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "alt_node");
    ros::NodeHandle n;

    Altimeter alt(n);
    ros::spin();

    return 1;
}
