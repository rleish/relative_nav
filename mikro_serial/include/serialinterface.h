/*  
 *  Mikrokopter FlightControl Serial Interface
 *  Copyright (C) 2010, Cyphy Lab.
 *  Inkyu Sa <i.sa@qut.edu.au>
 *
 *  https://wiki.qut.edu.au/display/cyphy
 *
 *
 *  
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

#ifndef MIKO_FLIGHTCONTROL_SERIALINTERFACE_H
#define MIKO_FLIGHTCONTROL_SERIALINTERFACE_H

#include <stdio.h>
#include <sys/termios.h>
#include <sys/ioctl.h>
#include <cstring>
#include <unistd.h>
#include <cstdlib>
#include <time.h>
#include <errno.h>
#include <bitset>
#include <math.h>

#include <ros/ros.h>
#include "flightcontrol.h"

#define TXD_BUFFER_LEN  300
#define RXD_BUFFER_LEN  300
#define MAX_PITCH_THRESHOLD 360
#define MAX_ROLL_THRESHOLD 360
#define MAX_YAW_THRESHOLD 360
#define MAX_HEIGHT_THRESHOLD 200
#define MAX_ACC_THRESHOLD 400

extern pthread_mutex_t mutex_; //!< mutex to change "set_next_as_ref_"

namespace miko
{
	class SerialInterface
	{
		public:
		SerialInterface (std::string port, uint32_t speed);
		~SerialInterface ();

		void output (char *output, int len);
		void output (unsigned char *output, int len);
		void Decode64(void);
		void ParsingData(void);
		void dumpDebug (void);
		int getdata (unsigned char *buf, int len);
		uint32_t serialport_bytes_rx_;
		uint32_t serialport_bytes_tx_;
		bool status;
		int pt[800];
		int counter;


		bool Initialized;
		int count;

		private:
		speed_t bitrate (int);
		void flush ();
		void drain ();
		void stall (bool);
		int wait (int);

		int dev_;
		std::string serialport_name_;
		uint32_t serialport_speed_;
		speed_t serialport_baud_;
	};
}
#endif
