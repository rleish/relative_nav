/*
 *  Mikrokopter Flight control Serial Interface
 *  Copyright (C) 2010, CYPHY lab
 *  Inkyu Sa <i.sa@qut.edu.au>
 *
 *  http://wiki.qut.edu.au/display/cyphy
 *
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef MIKO_FLIGHTCONTROL_FLIGHTCONTROL_H
#define MIKO_FLIGHTCONTROL_FLIGHTCONTROL_H

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <sys/termios.h>
#include <sys/ioctl.h>
#include <cstring>
#include <unistd.h>
#include <cstdlib>
#include <time.h>
#include <errno.h>
#include <bitset>
#include <stdarg.h>

#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8MultiArray.h>
#include <sys/time.h>
#include "serialinterface.h"
#include "sensor_msgs/Imu.h"  //For height
#include "mikro_serial/mikoImu.h"
#include "mikro_serial/mikoCmd.h"


#if 1
#define TRUE 1
#define FALSE 0
#define YES 1
#define NO 0
#endif

#define FC_ADDRESS 1
//#define CHECK_DURA
#define MAX_THROTTLE 254//150

// MAX when hovering
#define MAX_PITCH 127
#define MAX_ROLL 127
#define MAX_YAW 127

#define YAW_SCALE_FACTOR 10.
#define ACC_X_K 35
#define ACC_Y_K 35
#define BATT_MAX 200

//Define indices of variables as they are coming from the MK Firmware
//These are now based on the regular Hexacopter firmware, from mikrokopter
#define ANGLE_ROLL 1
#define ANGLE_PITCH 0
//#define GYRO_ROLL 2
//#define GYRO_PITCH 3
#define GYRO_YAW 4
#define ACCEL_X 2
#define ACCEL_Y 3
#define ACCEL_Z 8
#define MOTOR1 12
#define MOTOR2 13
#define MOTOR3 14
#define MOTOR4 15
//#define MOTOR5 13
//#define MOTOR6 14
#define BATT 9
#define BAROMHEIGHT 5
#define ANGLE_YAW 11

//#define CHECK_DURA
#define PI 3.141592653589793238462643383279
#define DEG_TO_RAD(x) (x*0.0174532925f)
#define RAD_TO_DEG(x) (x*57.2957795f)
#define POINT_ONE_G_RADS_TO_RADS(x) (x*0.00157079633)
#define CONVERT_YAW(x) (x*0.00174532925)
#define ACCEL_X_FAKTOR 41.808365 //converts accel_x from magic MK units to m/s^2 (-410/9.80665)
#define ACCEL_Y_FAKTOR 41.808365 //converts accel_y from magic MK units to m/s^2 (410/9.80665)
#define ACCEL_Z_FAKTOR 73.4196 //converts accel_z from magic MK units to m/s^2 (720/9.80665)
#define BOUNDARY 50   // in mm
#define g 9.81   //9.8 m/s^2
#define K 0.5   // Complementary filter gain
#define Kp_Pitch 12.
#define Kp_Roll 12.

#define Kp_vel_X 3.
#define Kp_vel_Y 12.
#define Kp_pos_X 10.
#define Kp_pos_Y 10.

#define GAS_PARAM 3.32

typedef struct
{
  double x;
  double y;
  double z;
  double theta;
} __attribute__((packed)) Position_t;

typedef struct
{
  int pitch;
  int roll;
  int yaw;
} Control_t;

typedef struct
{
  double x;
  double y;
  double z;
} Velocity_t;


struct str_Data3D
{
   signed int  angle[3]; // pitch, roll, yaw in 0,1�
   signed char Centroid[3];
   signed char reserve[5];
};

typedef struct
{
	uint8_t Digital[2];
	int16_t Analog[32];    // Debugvalues
} __attribute__((packed)) DebugOut_t;

namespace miko
{
	class FlightControl
	{
		private:

		ros::Timer timer_;

		double freq_;
		std::string port_;
		int speed_;
		bool Throttle_Direction;
		typedef struct
		{
			uint8_t	Digital[2];
			uint8_t	RemoteButtons;
			int8_t	Pitch;
			int8_t	Roll;
			int8_t	Yaw;
			uint8_t  Throttle;
			int8_t	Height;
			uint8_t	free;
			uint8_t	Frame;
			uint8_t	Config;
		} ExternControl_t;


		typedef struct
		{
			double AnglePitch;
			double AngleRoll;
			double AngleYaw;
			double ACCX;
			double ACCY;
			double ACCZ;
		} __attribute__((packed)) Attitude_t;




		typedef struct
		{
			double x;
			double y;	 
			double z;	 
			double yaw;	
		}__attribute__((packed)) DesiredPosition_t;

		ros::Publisher pub;
		ros::Publisher pub_pose2D;

		uint64_t time;

		DesiredPosition_t DesiredPosition;
		Attitude_t MyAttitude;

		public:
		SerialInterface* serialInterface_;
    //FILE *fd,*fd_h,*fd_debug;
    std::ofstream log_file_; //!< for writing the log file
    mikro_serial::mikoImu mikoImu;

    //ExternControl_t	ExternControl;
		ros::Subscriber mikoCmdSubscriber;

		FlightControl ();
		virtual ~FlightControl();
    void AddCRC(unsigned char *bffer, uint16_t datelen);
		void SendOutData(uint8_t cmd, uint8_t addr, uint8_t numofbuffers, ...);
		void enablePolling (uint16_t request, uint16_t interval);
		void spin (const ros::TimerEvent & e);
    void mikoCmdCallback (const mikro_serial::mikoCmd& msg);

		unsigned long long time_helper(void);

    int data_rate_; //!< allows selectable data rate for debug data on parameter server
	}; // end class FlightControl
} //end namespace miko

#endif
