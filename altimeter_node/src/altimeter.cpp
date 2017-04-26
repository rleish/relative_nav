 /* \copyright This work was completed by Robert Leishman while performing official duties as 
  * a federal government employee with the Air Force Research Laboratory and is therefore in the 
  * public domain (see 17 USC § 105). Public domain software can be used by anyone for any purpose,
  * and cannot be released under a copyright license
  */

/*!
 *  /file altimeter.cpp
 *  \brief Contains the methods in the Altimeter Class
 *
*/

#include "altimeter_node/altimeter.h"

Altimeter::Altimeter(ros::NodeHandle nh)
{
  std::string port_name,topic_name;
  //Get some values off param:
  ros::param::param<std::string>("~portname", port_name, "/dev/ttyS1");
  ros::param::param<bool>("~computeControl", compute_commands_, "false");
  ros::param::param<double>("~desired_height", desired_height_, 1.0);
  ros::param::param<std::string>("~output_topic_name", topic_name, "/alt_msgs");

  //set up ROS publisher:
  alt_publisher_ = nh.advertise<sensor_msgs::Range>(topic_name,10); //publish to this topic, queue 10 messages

  if(compute_commands_)
  {
    cmd_publisher_ = nh.advertise<mikro_serial::mikoCmd>("\mikroCmd",1);
    controller_.initPid(-7.25, -0.5, -7.75, -2.0, 2.0);
  }

  //connect serial port and signal & slot:
  try
  {
    serial_.open(port_name.c_str(), 9600);
  }
  catch(cereal::Exception &exc)
  {
    ROS_FATAL("Failed to open serial port!!");
    ROS_BREAK();
  }
  ROS_INFO_ONCE("Serial port for the altimeter is open!!");    

  differentiator_ = 0.d;
  down_old_ = 0.d;
  time_old_ = ros::Time::now();
  error_old_ = 0.d;
  down_limit_ = 0.15;  //This value could be changed, but we have found it to work fairly well so far...
  tau_ = 0.04; //This value could also be changed.  Might get a smoother velocity if you did...
  altitude_bias_ = -0.113; //(meters), the bias (negative because down is positive)

  //Start the timer to process the serial data:
  timer_ = nh.createTimer(ros::Duration(1./40.0),&Altimeter::processSerial, this);
}


//
//  Function called when serial data recieved
//
void Altimeter::processSerial(const ros::TimerEvent &e)
{
  //timestamp
  ros::Time timestamp = ros::Time::now();
  double time = timestamp.toSec();

  //get the data:
  char reply[NUM_BYTES];
  try
  {
    serial_.readBytes(reply,NUM_BYTES,TIMEOUT);
  }
  catch(cereal::TimeoutException &e)
  {
    ROS_INFO_THROTTLE(10,"Timeout of the Serial - not necessarily broke");
    //ROS_ERROR("Timeout on Altimeter serial!!");
    return; //no valid data read, just skip the rest of the function
  }

  //process data
  char inch_char[NUM_BYTES-1];
  inch_char[0] = reply[1];
  inch_char[1] = reply[2];
  inch_char[2] = reply[3];

  double inches = atof(inch_char);

  //std::cout << "height = " << inches << std::endl;  
  double meters = -inches*0.0254 + altitude_bias_; //put in meters and put in the bias (negative because down is
                                                   //positive and we want the down position

  ros::Duration dt = timestamp - time_old_;
  if (dt > ros::Duration(1.0))
  {
    dt = ros::Duration(0.0);
  }

  if(abs(meters - down_old_) >= down_limit_)
  {
    //Bad measurement, estimate it as the old one:
    meters = down_old_;
  }

  //compose the message and send it out
  sensor_msgs::Range range_msg;
  range_msg.header.stamp = timestamp;
  range_msg.radiation_type = range_msg.ULTRASOUND;
  range_msg.min_range = 0.f;
  range_msg.max_range = 6.1;
  range_msg.range = (float)meters;

  alt_publisher_.publish(range_msg);

  if(compute_commands_)
  {
    double error = -desired_height_ - meters;
    double pid_out;

    differentiator_ = (2. * tau_ - dt.toSec()) / (2. * tau_ + dt.toSec()) * differentiator_ + 2. / (2. * tau_ + dt.toSec()) * (error - error_old_);

    pid_out = controller_.updatePid(error, differentiator_, dt);

    /// \todo Estimate what the constant value should be for the controller (the gravity offset).
    pid_out += 140.0; //

    double control;
    control = saturate(pid_out,0,255);

    mikro_serial::mikoCmd cmd;
    cmd.pitch = 0.0;
    cmd.roll = 0.0;
    cmd.throttle = control;
    cmd.yaw = 0.0;
    cmd.header.stamp = timestamp;

    cmd_publisher_.publish(cmd);

    error_old_ = error;
  }

  down_old_ = meters;
  time_old_ = timestamp;
}
