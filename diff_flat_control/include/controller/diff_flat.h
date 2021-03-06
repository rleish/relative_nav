 /* \copyright This work was completed by Robert Leishman while performing official duties as 
  * a federal government employee with the Air Force Research Laboratory and is therefore in the 
  * public domain (see 17 USC § 105). Public domain software can be used by anyone for any purpose,
  * and cannot be released under a copyright license
  */

 /*
  * 
  * \file estimator.h
  * \author Robert Leishman
  * \date September 2012
  *
  * \brief The diff_flat.h file is the header for the DiffFlat class.
*/

#ifndef DIFFFLAT_H
#define DIFFFLAT_H

#include <iostream>
#include <fstream>
#include <queue>
#include <math.h>
#include <stdio.h>
#include <sys/stat.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/TransformStamped.h>
#include "mikro_serial/mikoCmd.h"
#include "control_toolbox/pid.h"

extern pthread_mutex_t list_mutex_; //!< mutex just in case for the waypoint_list_ vector (used by multiple callbacks)
extern pthread_mutex_t hpt_mutex_; //!< and one just in case for and hoverpoint

/*!
 *  \class DiffFlat diff_flat.h "include/controller/diff_flat.h"
 *  \brief This class implements a differentially flat position controller for MikroKopter vehicles. It is based on the
 *  paper: "Differential Flatness-Based Control of a Rotorcraft for Aggressive Maneuvers" by
 *  Ferrin J., Leishman R., Beard R., and McLain T. IEEE IROS Sept. 2011.
 *
 *  This implementation expands the controller identified in the paper by allowing waypoint paths to be followed.
 *  The paper only discusses following twice-differentiable, time-dependent paths.
 *
 *  The controller states are position, linear velocity (in same frame as position), and yaw: [f s d fdot sdot ddot psi]
 *  In my current application, these will be relative to the current node that the estimator is navigating with respect to.
 *  This node can change, and the waypoints must be expressed in the new node frame when that occurs.
 *  However, as long as the waypoints and the state are in expressed in the same coordinate frame, this controller
 *  should work fine.  So this controller should work fine using motion capture data
 *  UPDATE: I have used this controller to fly the hex using truth data around the CAVE.  I still have yet to try it
 *  using relative states though...
 * 
*/
class DiffFlat
{
public:
  /// Eigen macro used when there are fixed-sized class member variables and you dynamically create an instance of the
  /// class.  (See: http://eigen.tuxfamily.org/dox/TopicStructHavingEigenMembers.html)
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /*!
   *  \brief The constructor.  Optionally, a waypoint list can be used to initialize the controller.
   *  \param allow_integrator is a flag for allowing integrator control for the position controller
   *  \param gain_file_location is the path to the file that contains the gain matricies
   *  \param wypt_list is an optional list of waypoints used to allocate the waypoint_list_ class member
   *  \param base_height is an optional term to set the desired height above ground for the paths.
  */
  DiffFlat(bool &allow_integrator, std::string &gain_file_location,
           std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > *wypt_list = NULL,
           double *base_height = NULL);


  /*!
   *  \brief Destructor...
   *
   *
  */
  ~DiffFlat();


  /*!
   *  \brief This function applies the provided transformation to the list of waypoints in the class.  This function
   *  should be called after a new node is created and before the function pathPlanner is called.  This function also
   *  transforms the hover_setpoint_ when hovering is enabled during hovering (shouldn't really occur, but it's here
   *  just in case)
   *
   *  \param translation is the 3D translation from the old coordinate frame to the new frame, expressed in the old frame
   *  \param rotation is the 3D rotation from the old coordinate frame to the new one,
  */
  void applyNewNodeToWaypoints(Eigen::Vector3d &translation,  Eigen::Quaterniond &rotation);


  /*!
   *  \brief Path that simply hovers at whatever state the rotorcraft is currently at or the optionally provided position.
   *  The error is computed and sent to applyDiffFlatness so that the control can be computed.
   *
   *  Additionally, while hovering, the setpoint can be changed (independently for the position and the yaw).  Just send
   *  a different value for either the position_hold or the yaw_hold, as desired, and the set point will be changed. The
   *  yaw value must be different from the old one by more than 1 deg (0.017 rad) and the new position hold point must
   *  be different by 0.01 meters (in the norm).
   *
   *  \param position is the position in the reference frame
   *  \param quat is the rotation between the current node and the body-fixed coordinate frame
   *  \param velocity is the velocity of the body-fixed frame w.r.t. the node frame, expressed in the <b> body </b> frame
   *  \param time is the current ros::Time
   *  \param dt is the ros::Duration since the last control was computed
   *  \param position_hold is the optional position to hover about.
   *  \param yaw_hold Is the optional yaw angle to hover at (along with the position)
   *  \returns A vector of commands in the order: [pitch; roll; thrust; yaw]
  */
  Eigen::Vector4d hoverPath(Eigen::Vector3d &position, Eigen::Quaterniond &quat,
                            Eigen::Vector3d &velocity, ros::Time &time, ros::Duration &dt,
                            Eigen::Vector3d *position_hold = NULL,
                            double *yaw_hold = NULL, double *voltage = NULL);

  /*!
   *  \brief This path follows the provided waypoint list.  This function calls the helper functions:
   *  pathManager, pathFollower, and applyDiffFlatness.
   *
   *  \param position is the position in the reference frame
   *  \param quat is the rotation from the current node to the body-fixed coordinate frame
   *  \param velocity is the velocity of the body-fixed frame w.r.t. the node frame, expressed in the <b> body </b> frame
   *  \param time is the current ros::Time
   *  \param dt is the ros::Duration since the last control was computed
   *  \param goal_achieved is a flag if the end of the waypoint path is reached, if it is, it will be set to true.
   *  \param wypt_list A new list of waypoints. Any old list is replaced with the new, and we begin the new list at the first waypoint   
   *  \returns A vector of commands in the order: [pitch; roll; thrust; yaw]
  */
  Eigen::Vector4d waypointPath(Eigen::Vector3d &position, Eigen::Quaterniond &quat,
                               Eigen::Vector3d &velocity, ros::Time &time, ros::Duration &dt,bool *goal_achieved,
                               std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > *wypt_list = NULL,
                               double *voltage = NULL);


  /*!
   *  \brief Has the rotorcraft follow a pre-defined circle path at a constant velocity.  Again, applyDiffFlatness is
   *  called to compute the control inputs once the error is calculated.
   *
   *  \param position is the position in the reference frame
   *  \param quat is the rotation between the current node and the body-fixed coordinate frame
   *  \param velocity is the velocity of the body-fixed frame w.r.t. the node frame, expressed in the <b> body </b> frame
   *  \param time is the current ros::Time
   *  \param dt is the ros::Duration since the last control was computed
   *  \returns A vector of commands in the order: [pitch; roll; thrust; yaw]

  */
  Eigen::Vector4d circlePath(Eigen::Vector3d &position, Eigen::Quaterniond &quat,
                             Eigen::Vector3d &velocity, ros::Time &time, ros::Duration &dt, double *voltage = NULL);

  //A few of other paths like circlePath are available on the windows "DEMOQuad" program.  Just need to be translated
  //into C++ (and improved - yes, it is horribly written code, sorry.  Just follow how I've done it with circlePath)


  /*!
   *  \brief Read/Write access to "flying", to set when the integral control will be used.
   *  \returns the value for the class variable flying_
  */
  bool modifyFlyingFlag(bool fly)
  {
    if(fly != flying_)
    {
      flying_ = fly;
    }
    return flying_;
  }


protected:

  /*!
   *  \brief Using the set of waypoints and the current waypoint, determine the direction of travel and the starting point.
   *  This function also determines when a waypoint has been reached and it's time to switch to the next one.
   *
   *  This function essentially implements the material outlined in Bead and McLain's book "Small Unmanned Aircraft".
   *  Only, since we can control path error directly by pitch/roll of rotorcraft, we don't implement the course angle stuff
   *
   *  \param position is the current position
   *  \param start_point is the path startig point computed and returned by this method
   *  \param direction is the path direction computed and returned by this method
   *  \param hoverpoint is returned when the end of the waypoint path is reached, start_point and direction will be null
   *  \returns true when the end of the waypoint list has been reached and the vehicle should hover at the last waypoint
  */
  bool pathManager(Eigen::Vector3d &position, Eigen::Vector3d *start_point, Eigen::Vector3d *direction, Eigen::Vector3d *hoverpoint);


  /*!
   *  \brief Given the direction of travel, the starting point and current state; compute the error in the control state
   *
   *  \param start_point is the path startig point
   *  \param direction is the path direction
   *  \param position is the position in the reference frame
   *  \param quat is the rotation between the current node and the body-fixed coordinate frame
   *  \param velocity is the velocity of the body-fixed frame w.r.t. the node frame, expressed in the <b> body </b> frame
   *  \param error_state is the error in the control state computed and returned by this method
  */
  void pathFollower(Eigen::Vector3d &start_point, Eigen::Vector3d &direction, Eigen::Vector3d &position,
                    Eigen::Quaterniond &rotation, Eigen::Vector3d &velocity, Eigen::Matrix<double,7,1> *error_state);


  /*!
   *  \brief This function computes the control commands from the error and feedforward accelerations given by whichever
   *  path above.  This is the function that really implements the differential flatness described in the paper.
   *  The control state is in the order: [n e d ndot edot ddot psi]
   *
   *  \param error The error in the control state (desired - actual)
   *  \param quat Is the current orientation of the body-fixed frame relative to the node frame (rotation between those two)
   *  \param dt Is the ros::Duration between the last control and this one.
   *  \param feed_forward_accels Is the feed forward accelerations and the feed forward yaw rate.  This is a key part of
   *  the differential flatness that allows the vehicle to follow aggressive paths.  It really isn't needed for waypoint
   *  following (at least when trying to use vision inputs on a vehicle). Ordering is [north, east, down, psi]
   *  \returns Control inputs in the order [theta; phi; thrust; yawrate]
  */
  Eigen::Vector4d applyDiffFlatness(Eigen::Matrix<double,7,1> &error, Eigen::Quaterniond &quat, ros::Duration &dt,
                                    Eigen::Vector4d *feed_forward_accels = NULL, double *voltage = NULL);


  /*!
   *  \brief This function computes the LQR feedback control terms, given the error in the control state.  The gain
   *  matrix is input from a file (which is genereated from the Matlab script "LQR_script_heavy.m").
   *
   *  \param error Is again the error in the control state (desired - actual)
   *  \param dt Is the ros::Duration between the last control and this one.
  */
  Eigen::Vector4d computeLQRControl(Eigen::Matrix<double,7,1> &error, ros::Duration &dt);


  /*!
   *  \brief This function creates a right-handed rotation about the inertial (nodal) z-axis for the yaw.
   *
   *  \param yaw Is the yaw angle between the node and the body-fixed frame
   *  \returns The rotation matrix
  */
  inline Eigen::Matrix3d createYawRotation(double yaw)
  {
    Eigen::Matrix3d rotation;
    rotation << cos(yaw),sin(yaw),0.d,-sin(yaw),cos(yaw),0.d,0.d,0.d,1.0;
    return rotation;
  }


  /*!
   *  \brief Reads the gain matrices LQR_K_ and LQR_KI_ from a file.  These gain matrices are genereated from a matlab
   *  script.
   *  \param filename Tells what file to read the gains from
  */
  void readKFromFile(std::string &filename);


  /*!
   *  \brief provides a timestamp for saving log files.
  */
  std::string timeStructFilename(tm *time_struct);


  /*!
   *  \brief This function computes a dirty derivative of the error in angle to feed to the PID controller.  The idea
   *  is to make the controller less sensitive to small dt values.
   *  \param dphi is the error in phi
   *  \param dtheta is the error in theta
   *  \param dt is the time between this and the last control command
   *  \param der_dphi is the derivative of dphi returned by the method
   *  \param der_dtheta is the derivative of dtheta returned by the method
  */
  inline void computeDirtyDerivativeError(double dphi, double dtheta, double dt, double *der_dphi, double *der_dtheta)
  {
    der_dphi_ = (2. * TAU_ - dt) / (2. * TAU_ + dt) * der_dphi_ + 2. / (2. * TAU_ + dt) * (dphi - dphi_old_);
    der_dtheta_ = (2. * TAU_ - dt) / (2. * TAU_ + dt) * der_dtheta_ + 2. / (2. * TAU_ + dt) * (dtheta - dtheta_old_);
    *der_dphi = der_dphi_;
    *der_dtheta = der_dtheta_;

    dphi_old_ = dphi;
    dtheta_old_ = dtheta;
  }


  /*!
   *  \brief This function provides the feedforward accel to combat gravity.  It takes into account the battery voltage
   *  level.  This curve was found when the mass was about 4.05 kg, KF_ = 0.00028, on a Hexacopter w/ 13" props.
   *  Probably need to find the curve again for any large deviations from that.
   *
   *  To find the curve, I hovered the hex using truth until the battery alarm went off.  I found the trend in the thrust
   *  command (pretty hard Low-Pass filtering) and then used some interpolation to find the thrust command at each time
   *  I had a battery voltage reading.  Then I plotted thrust as a function of battery voltage.
   *  (See figure voltage_with_thrust.jpg)
   *  I found that the command was pretty level at high voltages and low ones, so I just did a linear fit between those
   *  areas.  That is the function that is represented here.  (although to make it less susceptible to changes in mass,
   *  I used the relationship in the DiffFlat::applyDiffFlatness() function: T = sqrt(accel*mass/(6*kF)) to change the
   *  function to be a relationship between voltage and acceleration.
   *
   *  \param batt_volt is the most recent battery voltage in units of 0.1 V i.e. 15.0 V = 150,
   *  \param accel is the returned acceleration that should be added in the feedforward term (instead of gravity const)
  */
  inline void computeGravityOffset(double batt_volt, double *accel)
  {
    if(batt_volt > 150)
    {
      *accel = 9.393;
    }
    else if(batt_volt < 140)
    {
      *accel = 10.80;
    }
    else
    {
       //v(2) = sqrt(v(2)/(NUM_ROTORS_ * KF_));
      *accel = NUM_ROTORS_*KF_/MASS_*(SLOPE_*batt_volt + INTER_)*(SLOPE_*batt_volt + INTER_);
    }
  }

//  /*!
//   *  \brief Given a line from the file and the delimiter, parse the line into a vector of doubles. We assume that just
//   *  numbers are in the file (for reading in the gains)
//   *
//   *  \param line is a line from the file
//   *  \param delimeter is the delimeter character used to separate elements
//   *  \returns a vector of the doubles found in the file.
//   *  \todo Make this function inline
//  */
//  std::vector<double> parseLine(std::string line, std::string delimiter);


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

  //Important Flags:
  const static bool open_Loop_ = false; //!< allows calculation of control using only feedforward terms
  bool use_Integrator_; //!< enable/disable the use of integrators in the control (control with ros param - see ROSServer)
  bool flying_; //!< flag for when we are flying, used for the integral control.

  //variables:
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > waypoint_list_; //!< the waypoint list
  Eigen::Matrix<double,4,7> LQR_K_; //!< the LQR gain matrix K
  Eigen::Matrix<double,4,4> LQR_KI_; //!< the integrator LQR gain
  Eigen::Vector4d int_error_; //!< the integrated error states (only need position and psi)
  Eigen::Matrix<double,7,1> old_error_; //!< for computing the integrator terms of the LQR
  control_toolbox::Pid pid_phi_; //!< PID controller for the roll (necessary since we don't know correct converstion for Mikrokopter stuff
  control_toolbox::Pid pid_theta_; //!< PID controller for the pitch
  std::ifstream gain_reader_; //!< reader for the gain matrices file

  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >::iterator current_waypoint_; //!< iterator for the current waypoint
  const static double WAYPOINT_SPEED_ = 0.2;//0.15;// //!< the speed to pursue along the paths between waypoints
  bool first_time_; //!< flag for when it is first time in the path planner
  bool waypt_inserted_; //!< flag for if a beginning waypoint was placed in the list

  bool hover_flag_; //!< flag for if the hover position has been set (when a position to hold isn't provided)
  Eigen::Vector3d hover_setpoint_; //!< setpoint for hovering
  double hover_yawpoint_; //!< setpoint for yaw when hovering (in radians, as it's converted to yaw angle)
  double base_height_; //!< (m) the mean height of the path
  double batt_voltage_; //!< the most recent battery voltage

  //Circle Path Variables/Constants:
  /// \todo Make the path parameters available for editing using the parameter server (then they could be set in a launch file)
  bool circle_started_; //!< flag for defining when the circle started;
  ros::Time start_time_; //!< ros::Time that the circle started
  const static double VELOCITY_ = 0.5; //!< (m/s) velocity that the vehicle travels around the circle
  const static double RADIUS_ = 1.0; //!< (m) circle radius
  const static double HEIGHT_GAIN_ = 0.5; //!< (m) Amplitude of the height change
  const static double YAW_RATE_  = 0.0; //!< (rad/s) the desired yaw rate change during the flight
  const static double N_OFFSET_ = 0.0; //!< optional offset from the origin in "north"
  const static double E_OFFSET_ = 0.0; //!< optional offset from origin in "east"

  //Other (including vehicle) constants:
  const static double PI_ = 3.1415926535897932384626; //!< pi
  const static double GRAVITY_ = 9.80665; //!< gravity
  const static double MASS_ = 3.7; //4.05;//3.95; //!< (kg) mass of vehicle
  const static double YAW_RATE_GAIN_ = 73.0; //!< the gain to convert r/s to MikroKopter units (found by trial/error)
  const static double NUM_ROTORS_ = 6.0; //!< number of rotors on the platform
  const static double KF_ = 0.00028; //!< gain for the thrust model: kF*w^2, where w = motor speed
  const static double BOUNDS_ = 0.3; //!< the error sphere radius needed to be met to switch nodes (in addition to the plane)
  const static double MIN_BOUNDS_ =0.15; //!< if we are within this, switch waypoints, even if we haven't crossed the half plane
  const static double YAW_MAX_ = 0.15;//0.25; //!< if the abs(yaw_error) when following a path is > this, hover in place and yaw.

  //dirty derivative parameters:
  double der_dphi_; //!< derivative for dphi
  double der_dtheta_; //!< derivative for dtheta
  double dphi_old_; //!< self explainatory
  double dtheta_old_; //!< yeah, this one too
  const static double TAU_ = 0.1; //!< time constant for the LPF of the derivative


  //Thrust curve fit with voltage stuff
  static const double SLOPE_ = -1.0855; //!< the slope of the voltage line in between 14.0 and 15.0 volt battery level
  static const double INTER_ = 313.3019; //!< the intercept for the voltage line between 14.0 and 15.0 volts battery level


  std::ofstream cmd_file_; //!< for logging
  std::ofstream log_file_; //!< for writing the log file

};

#endif



/*!
 *
 *
 *
*/
