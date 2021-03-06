 /* \copyright This work was completed by Robert Leishman while performing official duties as 
  * a federal government employee with the Air Force Research Laboratory and is therefore in the 
  * public domain (see 17 USC § 105). Public domain software can be used by anyone for any purpose,
  * and cannot be released under a copyright license
  */

/*!
 *  \file diff_flat.cpp
 *  \author Robert Leishman
 *  \date September 2012
 *
*/

#include "controller/diff_flat.h"

using namespace Eigen;


pthread_mutex_t list_mutex_ = PTHREAD_MUTEX_INITIALIZER; //!< mutex just in case for the waypoint_list_ vector
pthread_mutex_t hpt_mutex_ = PTHREAD_MUTEX_INITIALIZER;//!< and the hoverpoint

//
//  Constructor
//
DiffFlat::DiffFlat(bool &allow_integrator, std::string &gain_file_location,
                   std::vector<Vector3d, aligned_allocator<Vector3d> > *wypt_list,
                   double *global_z)
{
  use_Integrator_ = allow_integrator;
  flying_ = false;

  pthread_mutex_lock(&list_mutex_);
    if(wypt_list != NULL)
    {
      waypoint_list_ = *wypt_list;
    }
    current_waypoint_ = waypoint_list_.begin();
  pthread_mutex_unlock(&list_mutex_);
  pthread_mutex_lock(&hpt_mutex_);
    hover_flag_ = false;
    hover_setpoint_.setZero();    
  pthread_mutex_unlock(&hpt_mutex_);

  //initialize the PID controllers for phi & theta
  pid_phi_.initPid(-100.0, 0.0, -3.0, 2.0, -2.0);
  pid_theta_.initPid(-100.0, 0.0, -3.0, 2.0, -2.0);

  LQR_K_.setZero();
  LQR_KI_.setZero();
  //Read in the gains:
  //std::string filename = "../Matlab/HeavyGainMatricies.txt";
  readKFromFile(gain_file_location);

  int_error_.setZero();
  old_error_.setZero();

  if(global_z != NULL)
  {
      base_height_ = *global_z*-1.0;
  }
  else
  {
      base_height_ = 0.85;//1.0; //default of 1 meter
  }

  //circle path stuff
  circle_started_ = false;

  //dirty derivative stuff
  dphi_old_ = 0.d;
  der_dphi_ = 0.d;
  dtheta_old_ = 0.d;
  der_dtheta_ = 0.d;

  first_time_ = false; //switch for inserting a waypoint
  waypt_inserted_ = false;  //true when a waypoint has been inserted into the path
  batt_voltage_ = 155; // a quick guess on the initial battery voltage

  //Setup the Logs:
  time_t rawtime;
  struct tm *utc_time;
  rawtime = time(NULL);
  utc_time = localtime(&rawtime);

  std::string file_name = "error";
  file_name += timeStructFilename(utc_time);
  file_name.append(".txt");
  log_file_.open(file_name.c_str(), std::ios::out | std::ios::trunc);

  file_name.clear();
  file_name = "commands";
  file_name += timeStructFilename(utc_time);
  file_name.append(".txt");
  cmd_file_.open(file_name.c_str(), std::ios::out | std::ios::trunc);
}



//
// Destructor
//
DiffFlat::~DiffFlat()
{

}



//
// Apply the new node to the current waypoints and the hover setpoint.
//
void DiffFlat::applyNewNodeToWaypoints(Vector3d &translation, Eigen::Quaterniond &rotation)
{
  Vector3d temp;
  pthread_mutex_lock(&list_mutex_);
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >::iterator i;
    for(i = waypoint_list_.begin(); i < waypoint_list_.end(); i++)
    {
      temp = rotation*(-translation + (*i)); //translation and rotation are in the old coordinate frame
      *i = temp;
    }
  pthread_mutex_unlock(&list_mutex_);
  pthread_mutex_lock(&hpt_mutex_);
    //change the hoverpoint:
    if(hover_flag_)
    {
      temp = rotation.conjugate()*(-translation + hover_setpoint_);
      hover_setpoint_ = temp;
      hover_yawpoint_ = hover_yawpoint_ -
          atan2(2.*rotation.x()*rotation.y() + 2.*rotation.w()*rotation.z(), 2.*rotation.w()*rotation.w() + 2.*rotation.x()*rotation.x() - 1.0);
    }
  pthread_mutex_unlock(&hpt_mutex_);
}



//
//  Hover Path: hover around some setpoint
//
Vector4d DiffFlat::hoverPath(Vector3d &position, Quaterniond &quat, Vector3d &velocity,
                             ros::Time &time, ros::Duration &dt, Vector3d *position_hold, double *yaw_hold, double *voltage)
{
  pthread_mutex_lock(&hpt_mutex_);
    if(hover_flag_ && position_hold != NULL && !position_hold->isApprox(hover_setpoint_,0.01))
    {
      //sent a different hover setpoint, change to that one:
      hover_setpoint_ = *position_hold;
      if(yaw_hold == NULL)
      {
        hover_yawpoint_ =
            atan2(2.*quat.x()*quat.y() + 2.*quat.w()*quat.z(), 2.*quat.w()*quat.w() + 2.*quat.x()*quat.x() - 1.0);
      }
      else
      {
        hover_yawpoint_ = *yaw_hold;
      }
    }
    else if(hover_flag_ && yaw_hold != NULL && fabs(*yaw_hold - hover_yawpoint_) > 0.017 )
    {
      //sent a different yaw hoverpoint
      hover_yawpoint_ = *yaw_hold;

      if(position_hold != NULL)
      {
        hover_setpoint_ = *position_hold;
      }
    }
    else if(position_hold == NULL && !hover_flag_)
    {
      /// \attention:
      /// Right now this function sets the down portion of the setpoint to -1.0*base_height_.  This is a problem for
      /// relative states when the hex is flying.  It will then want to hover "base_height_" above the current node
      /// height.  If you want to set a hover point using relative states, send it in (except on the first run through)
      //Set point about which to hover using current state
      hover_setpoint_ = position;
      hover_setpoint_(2,0) = -1.0*base_height_;
      //the current yaw angle
      if(yaw_hold == NULL)
      {
        hover_yawpoint_ =
            atan2(2.*quat.x()*quat.y() + 2.*quat.w()*quat.z(), 2.*quat.w()*quat.w() + 2.*quat.x()*quat.x() - 1.0);
      }
      else
      {
        hover_yawpoint_ = *yaw_hold;
      }
      hover_flag_ = true;

      std::cout << "Hover set at: " << std::endl;
      std::cout << hover_setpoint_ << std::endl;
      ROS_WARN("New hoverpoint set in the control!");
    }
    else if(position_hold != NULL && !hover_flag_)
    {
      //Use the sent in point as the hover_setpoint_
      hover_setpoint_ = *position_hold;
      if(yaw_hold == NULL)
      {
        hover_yawpoint_ =
            atan2(2.*quat.x()*quat.y() + 2.*quat.w()*quat.z(), 2.*quat.w()*quat.w() + 2.*quat.x()*quat.x() - 1.0);
      }
      else
      {
        hover_yawpoint_ = *yaw_hold;
      }
      hover_flag_ = true;
    }
  pthread_mutex_unlock(&hpt_mutex_);  

  Matrix<double,7,1> error;
  Vector3d vel_node; //velocity in the node frame
  vel_node = quat*velocity; //double conjugated (because of Eigen's way of representing rotations)

  error.block<3,1>(0,0) = hover_setpoint_ - position;
  error.block<3,1>(3,0) = -vel_node;
  error(6,0) = hover_yawpoint_ -
      atan2(2.*quat.x()*quat.y() + 2.*quat.w()*quat.z(), 2.*quat.w()*quat.w() + 2.*quat.x()*quat.x() - 1.0);


  if(fabs(error(6,0)) >= 0.17)
  {
    //A bit of gain scheduling during large yaws:
    error(3,0) *= 0.5;
    error(4,0) *= 0.5;
  }

  //Call Diff_Flatness to compute control:
  Vector4d control_commands;
  control_commands = applyDiffFlatness(error,quat,dt,NULL,voltage);

  return control_commands;
}


//
//  Waypoint Path:  Follow a set of waypoints
//
Vector4d DiffFlat::waypointPath(Vector3d &position, Quaterniond &quat, Vector3d &velocity, ros::Time &time,
                                ros::Duration &dt,  bool *goal_achieved,
                                std::vector<Vector3d, aligned_allocator<Vector3d> > *wypt_list, double *voltage)
{
  Vector3d startpt, direction, *hoverpt;
  Matrix<double,7,1> error;
  Vector4d control_commands;
  bool hover_flag;

  pthread_mutex_lock(&list_mutex_);/// Just lock all of this, I don't want the waypoints changing midway through. (just in case)
    if(wypt_list != NULL)
    {
      ROS_INFO_THROTTLE(0.5,"New Waypoint List Recieved: Following the new path now!");
      waypoint_list_.clear();
      waypoint_list_ = *wypt_list;
      current_waypoint_ = waypoint_list_.begin(); //reset the iterator
    }

    //Without waypoints, we cannot proceed!
    if(waypoint_list_.empty() || current_waypoint_ == waypoint_list_.end())
    {
      ROS_WARN_THROTTLE(1.0,"Either No waypoint list recieved or reached the end of the current list - Hovering will continue!  Send new waypoints to proceed!");
      Vector4d commands;
      commands = hoverPath(position,quat,velocity,time,dt,NULL,NULL,voltage);
      pthread_mutex_unlock(&list_mutex_);
      return commands;
    }
    else
    {
      //not hovering, set the flag to false:
      if(hover_flag_)
        hover_flag_ = false;
    }

  //  /// Necessary????
  //  if(first_time_)
  //  {
  //    first_time_ = false;
  //    //Insert a waypoint at starting position
  //    Vector3d temp(position(0),position(1),-1.0*base_height_);
  //    waypoint_list_.insert(waypoint_list_.begin(), temp);
  //    current_waypoint_ = waypoint_list_.begin(); //reset the iterator
  //    waypt_inserted_ = true;
  //  }
    startpt.setZero();
    direction.setZero();
    hoverpt = new Vector3d();
    hoverpt->setZero();
    hover_flag = pathManager(position, &startpt, &direction, hoverpt); // startpt, direction, and hoverpt are returned
    if(hover_flag)
    {
      //Really means that the goal has been reached:
      *goal_achieved = true;
      if(hoverpt->norm() < 0.01)
      {
        //Returned [0,0,0] as the desired hover position: just hover where you are.
        control_commands = hoverPath(position, quat, velocity, time, dt,NULL,NULL,voltage);
      }
      else
      {
        control_commands = hoverPath(position, quat, velocity, time, dt, hoverpt,NULL,voltage);
      }
    }
    else
    {
      *goal_achieved = false;
      error.setZero();
      pathFollower(startpt,direction, position, quat, velocity, &error); //error is returned

      control_commands = applyDiffFlatness(error, quat, dt,NULL,voltage);
    }
  pthread_mutex_unlock(&list_mutex_);

  return control_commands;
}



//
//  Path Manager: Given a waypoint list, determine the starting point and direction of travel
//  This is done according to the path manager outlined in the book by Beard & McLain: "Small Unmanned Aircraft"
//
bool DiffFlat::pathManager(Vector3d &position, Vector3d *start_point, Vector3d *direction, Vector3d *hoverpoint)
{
  std::vector<Vector3d, aligned_allocator<Vector3d> >::iterator next_waypoint, following_waypoint;

  if(current_waypoint_ + 1 == waypoint_list_.end())
  {
    //At the last waypoint, command hover (the point to hover about was set when we switched to this last waypoint)
    hoverpoint->setZero();
    return true;
  }
  else if(current_waypoint_ + 2 == waypoint_list_.end())
  {
    //2 waypoints until end, wrap following_waypoint, so we can get to the last waypoint (the goal)
    next_waypoint = current_waypoint_ + 1;
    following_waypoint = waypoint_list_.begin();
  }
  else
  {
    next_waypoint = current_waypoint_ + 1;
    following_waypoint = next_waypoint + 1;
  }

  Vector3d w_current, w_next, w_follow, r, q, q_next, n;
  w_current = *current_waypoint_;
  w_next = *next_waypoint;
  w_follow = *following_waypoint;

  r = w_current;
  q = w_next - w_current;
  q = (q*(1.0/q.norm())).eval();   //use eval to prohibit aliasing
  q_next = w_follow - w_next;
  q_next = (q_next*(1.0/q_next.norm())).eval();
  n = (q + q_next)*0.5;

  //use only the f & r directions for checking to see if we are within BOUNDS_
  Vector3d planer_pos(position(0),position(1),0.0);
  Vector3d planer_next(w_next(0),w_next(1),0.0);

  //See if we should switch waypoints:
  /// \todo Tune the error sphere radius that is required to switch waypoints
  //So, if we have passed the plane of the waypoint and are within a bounds, we have "arrived" OR if we are within
  // MIN_BOUNDS_, meaning we are very close, count it as arrived anyway.
  double norm_bounds = (planer_pos - planer_next).norm();
  if(((position - w_next).transpose()*n >= 0.0 && norm_bounds <= BOUNDS_) ||
     norm_bounds <= MIN_BOUNDS_)
  {
    if(waypt_inserted_)
    {
      waypoint_list_.erase(waypoint_list_.begin());
      current_waypoint_ = waypoint_list_.begin();
      waypt_inserted_ = false;
    }
    else
    {
      current_waypoint_++;
    }

    if(current_waypoint_ + 1 == waypoint_list_.end())
    {
      //Hover at the last waypoint:
      *hoverpoint = *current_waypoint_;
      ROS_WARN("End of Waypoint List Reached: Hovering, need further guidance to proceed!!!");
      return true;
    }
    else
    {
      hoverpoint->setZero();
    }

    ROS_INFO("Waypoint Switch Occured!");
  }
  else
  {
    hoverpoint->setZero();
  }

  *start_point = r;
  *direction = q;
  return false;
}



//
//  Path Follower: Given start point and direction, calculate the error.
//
void DiffFlat::pathFollower(Vector3d &start_point, Vector3d &direction, Vector3d &position, Quaterniond &quat,
                            Vector3d &velocity, Matrix<double, 7, 1> *error_state)
{
  Vector3d pos_error, path_error;
  Matrix3d yaw_rotation;

  //the path lies along the desired yaw, not the actual yaw:
  yaw_rotation = createYawRotation(atan2(direction(1),direction(0)));

  pos_error = start_point - position;
  path_error = yaw_rotation*pos_error;
  path_error(0) = 0.d; //don't include error along the path
  pos_error = yaw_rotation.transpose()*path_error;
  Matrix<double,7,1> err_temp;
  err_temp.block<3,1>(0,0) = pos_error;
  err_temp.block<3,1>(3,0) = 1.0*WAYPOINT_SPEED_*direction - quat*velocity; //conjugat*conjugate
  err_temp(6,0) = atan2(direction(1),direction(0)) -
      atan2(2.*quat.x()*quat.y() + 2.*quat.w()*quat.z(), 2.*quat.w()*quat.w() + 2.*quat.x()*quat.x() - 1.0);

  if(fabs(err_temp(6,0)) > YAW_MAX_)
  {
    ROS_INFO_THROTTLE(0.5,"CONTROL: Too much yaw error, hovering at the start point and then following the path.");
    // We have a lot of yaw error, change the err_temp to be based on a hover position at the start point until
    // the yaw error is reduced:
    err_temp.block<3,1>(0,0) = start_point - position;
    err_temp.block<3,1>(3,0) = quat*velocity;
    //Gain scheduling during large yaws: (just for fdot and rdot)
    err_temp(3,0) *= 0.5;
    err_temp(4,0) *= 0.5;
    err_temp.block<3,1>(3,0)*=-1.0; //make the velocity error negative
  }      

  *error_state = err_temp;
}



//
//  Apply Differential Flatness: Given the error, current orientation, the dt - compute the control commands in
//  Mikrokopter units
//
Vector4d DiffFlat::applyDiffFlatness(Matrix<double, 7, 1> &error, Quaterniond &quat, ros::Duration &dt,
                                     Vector4d *feed_forward_accels, double *voltage)
{
  // order for u's and v's is [north; east; down; psi] - different than the order for the Windows version
  Vector4d u_ref, u_control, u, v, control_inputs;
  Vector3d w;
  u_ref.setZero();
  u_control.setZero();
  u.setZero();
  v.setZero();
  control_inputs.setZero();
  w.setZero();

  if(feed_forward_accels != NULL)
  {
    u_ref += *feed_forward_accels;
  }

  //counteract gravity:
  u_ref(2) -= GRAVITY_;
//  double gravity_offset;
//  if(voltage != NULL)
//  {
//    batt_voltage_ = *voltage;
//  }
//  computeGravityOffset(batt_voltage_,&gravity_offset);
//  u_ref(2) -= gravity_offset;

  log_file_.precision(15);
  log_file_ << ros::Time::now().toSec() << " " << error(0,0) << " " << error(1,0) << " " << error(2,0) << " " << error(3,0)
            << " " << error(4,0) << " " << error(5,0) << " " << error(6,0) << std::endl;

  if(!open_Loop_)
  {
    u_control = computeLQRControl(error, dt);
  }

  //add feedforward and feedback terms:
  u = u_ref + u_control;

//  log_file_.precision(10);
//  log_file_ << u(0) << " " << u(1) << " " << u(2) << " " << u(3) << std::endl;

  //
  /// Now use the model inversion to take u (in acceleration/velocity) to v (force/torque):

  //Invert the model for thrust:
  v(2) = sqrt(u(0)*u(0) + u(1)*u(1) + u(2)*u(2))*MASS_;   //thrust

  Matrix3d yaw_rotation;
  yaw_rotation = createYawRotation(
        atan2(2.*quat.x()*quat.y() + 2.*quat.w()*quat.z(), 2.*quat.w()*quat.w() + 2.*quat.x()*quat.x() - 1.0));

  //Now invert model for phi/theta (psi doesn't need model inversion)
  w = yaw_rotation*u.block<3,1>(0,0) * (-1.0*MASS_/v(2));

  v(0) = atan(w(0)/w(2));   //pitch (theta) desired
  v(1) = asin(-w(1));   //roll (phi) desired

  //std::cout << v.transpose() << std::endl;

  //
  /// Convert to Mikrokopter units:
  //Use successive loop closure around phi and theta since we don't know the exact conversion to angle for Mikrokopter)
  double dphi, dtheta; //error in phi and theta:
  dphi = v(1) - atan2(2.*quat.y()*quat.z() + 2.*quat.w()*quat.x(), 2.*quat.w()*quat.w() + 2.*quat.z()*quat.z() - 1);
  dtheta = v(0) - asin(-2.0*quat.x()*quat.z() + 2.0*quat.w()*quat.y());

//  std::cout << "Before v (phi and theta): " << v(1) << " " << v(0) << std::endl;

  //compute derivative of error to feed to PID loops:
  double der_dphi, der_dtheta;
  computeDirtyDerivativeError(dphi,dtheta, dt.toSec(), &der_dphi, &der_dtheta);

  v(0) = -pid_theta_.updatePid( dtheta,der_dtheta, dt);  //The negative is necessary for Mikrokopter's weird coordinate frame
  v(1) = -pid_phi_.updatePid( dphi, der_dphi, dt); //The negative is necessary for Mikrokopter's weird coordinate frame
  v(2) = sqrt(v(2)/(NUM_ROTORS_ * KF_)); // convert the thrust force to the speed of motors (what Mikrokopter expects)
  v(3) = u(3)*YAW_RATE_GAIN_; //convert yaw rate (rad/s) to Mikrokopter units

//  log_file_.precision(10);
//  log_file_ << v(0) << " " << v(1) << " " << v(2) << " " << v(3) << std::endl;

  //saturate and floor the inputs:
  control_inputs(0) = std::floor(saturate(v(0),-15,15)); //pitch
  control_inputs(1) = std::floor(saturate(v(1),-15,15)); //roll
  /// \attention: I HAVE LIMITED HOW LOW THE CONTROLLER CAN PUT THE THRUST!!
  control_inputs(2) = std::floor(saturate(v(2),125,255));  //thrust (LOOK OUT: IT IS SATURATED ON THE LOW END!)
  control_inputs(3) = std::floor(saturate(v(3),-30,30)); //psi_dot

  cmd_file_.precision(20);
  cmd_file_ << ros::Time::now().toSec() << " " << control_inputs(0) << " " << control_inputs(1) << " "
            << control_inputs(2) << " " << control_inputs(3) << std::endl;

//  std::cout << "dPhi: " << dphi << " dTheta: " << dtheta << " roll: " << control_inputs(1) <<
//               " pitch: " << control_inputs(0) << " dt: " << dt.toSec() << std::endl;

  return control_inputs;
}



//
//  Calculate the LQR Feedback
//
Vector4d DiffFlat::computeLQRControl(Eigen::Matrix<double, 7, 1> &error, ros::Duration &dt)
{
  //wrap the error in yaw:
  while(error(6,0) > PI_)
  {
    error(6,0) -= 2.0*PI_;
  }
  while(error(6,0) < -1.0*PI_)
  {
    error(6,0) += 2.0*PI_;
  }

  Vector4d LQR_output, LQRI_output;

  LQR_output = LQR_K_*error;  

  if(use_Integrator_ && flying_ && error.transpose()*error < 0.8)
  {
    int_error_.block<3,1>(0,0) += dt.toSec()/2.0 * (error.block<3,1>(0,0) + old_error_.block<3,1>(0,0));
    int_error_(3) += dt.toSec()/2.0 * (error(6,0) + old_error_(6,0));

    LQRI_output = LQR_KI_ * int_error_;

    LQRI_output(0) = saturate(LQRI_output(0),-3,3); //pitch
    LQRI_output(1) = saturate(LQRI_output(1),-3,3); //roll
    LQRI_output(2) = saturate(LQRI_output(2),-3,3); //thrust
    LQRI_output(3) = saturate(LQRI_output(3),-2,2); //yaw

    LQR_output += LQRI_output;
  }

  LQR_output(0) = saturate(LQR_output(0), -30, 30); //pitch
  LQR_output(1) = saturate(LQR_output(1), -30, 30); //roll
  LQR_output(2) = saturate(LQR_output(2), -80, 80); //thrust
  LQR_output(3) = saturate(LQR_output(3), -60, 60); //yaw

  old_error_ = error;

  return LQR_output;
}


//
//  Read in the gain matrices from a file:
//
void DiffFlat::readKFromFile(std::string &filename)
{
  gain_reader_.open(filename.c_str());
  std::string line;
  std::vector<double> numbers;

  if(gain_reader_.is_open())
  {
    char delim = '\n';
    while(std::getline(gain_reader_, line, delim))
    {
      numbers.push_back(atof(line.c_str()));
    }

    int k = 0;
    for(int i = 0; i<4; i++)
    {
      for(int j = 0; j< 7; j++)
      {
        LQR_K_(i,j) = numbers[k];
        k++;
      }
    }
    for(int i = 0; i<4; i++)
    {
      for(int j = 0; j< 7; j++)
      {
        if(j < 4)
        {
          LQR_KI_(i,j) = numbers[k];
        }
        k++;
      }
    }

    std::cout << "LQR K: = " << std::endl;
    std::cout << LQR_K_ << std::endl;
    std::cout << std::endl;
    std::cout << "LQR KI: = " << std::endl;
    std::cout << LQR_KI_ << std::endl;
  }
  else
  {
    ROS_ERROR("The Gain Matrix file could not be opened!");
    ROS_BREAK();
  }

  gain_reader_.close();
}



//
//  Circle Path: Follow a predefined circle around at a constant velocity (with optional height change)
//
Vector4d DiffFlat::circlePath(Vector3d &position, Quaterniond &quat, Vector3d &velocity, ros::Time &time,
                              ros::Duration &dt, double *voltage)
{
  //not hovering, set the flag to false:
  if(hover_flag_)
    hover_flag_ = false;

  if(!circle_started_)
  {
    circle_started_ = true;
    start_time_ = time;
  }

  Matrix<double,7,1> predicted,actual,error;
  double omega,path_time;
  omega = VELOCITY_/RADIUS_;
  path_time = time.toSec() - start_time_.toSec();

  //Path as a function of time (position, velocity, and yaw)
  predicted(0,0) = RADIUS_*cos(omega*path_time) + N_OFFSET_; //north (or node f)
  predicted(1,0) = RADIUS_*sin(omega*path_time) + E_OFFSET_; //east (or node r) position as function of time
  predicted(2,0) = -1.0*HEIGHT_GAIN_*sin(omega*path_time) - base_height_; //down (or node d) position as function of time
  predicted(3,0) = -1.0*RADIUS_*sin(omega*path_time)*omega;   //d(predicted(0,0))/dt
  predicted(4,0) = -1.0*RADIUS_*cos(omega*path_time)*omega;   //d(predicted(1,0))/dt
  predicted(5,0) = -1.0*HEIGHT_GAIN_*cos(omega*path_time)*omega;   //d(predicted(2,0))/dt
  predicted(6,0) = YAW_RATE_*path_time;

  actual.block<3,1>(0,0) = position;
  actual.block<3,1>(3,0) = quat*velocity; //double conjugate (because of Eigen's way of representing rotations)
  actual(6,0) =  atan2(2.*quat.x()*quat.y() + 2.*quat.w()*quat.z(), 2.*quat.w()*quat.w() + 2.*quat.x()*quat.x() - 1.0);

  error = predicted - actual;

  //Calc the feedforward accelerations/velocity:
  Vector4d feed_forward;

  feed_forward(0,0) = -1.0*RADIUS_*cos(omega*path_time)*omega*omega; //d(predicted(3,0))/dt
  feed_forward(1,0) = -1.0*RADIUS_*sin(omega*path_time)*omega*omega; //d(predicted(4,0))/dt
  feed_forward(2,0) = HEIGHT_GAIN_*sin(omega*path_time)*omega*omega; //d(predicted(5,0))/dt
  feed_forward(3,0) = YAW_RATE_; //d(predicted(6,0))/dt

  Vector4d control_commands;
  control_commands = applyDiffFlatness(error,quat,dt,&feed_forward,voltage);

  return control_commands;
}


std::string DiffFlat::timeStructFilename(tm *time_struct)
{
  std::string temp;
  temp.clear();
  std::stringstream out;

  out << "_";
  out << (time_struct->tm_mon + 1); //month is zero-based
  out << "_";
  out << time_struct->tm_mday;
  out << "_";
  out << (time_struct->tm_hour);
  if(time_struct->tm_min < 10)
  {
    //prepend w/ 0 for small minutes
    out << "0" << time_struct->tm_min;
  }
  else
  {
    out << time_struct->tm_min;
  }
  out << "-";
  out << time_struct->tm_sec; //add seconds to be sure not to overwrite files

  temp = out.str();

  return temp;
}
