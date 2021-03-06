 /* \copyright This work was completed by Robert Leishman while performing official duties as 
  * a federal government employee with the Air Force Research Laboratory and is therefore in the 
  * public domain (see 17 USC § 105). Public domain software can be used by anyone for any purpose,
  * and cannot be released under a copyright license
  */

/*! \file estimator.h
  * \author Robert Leishman
  * \date June 2012
  *
  * \brief The estimator.h file is the header for the Estimator class.
  *
  *
*/

#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <queue>
#include <math.h>
#include <stdio.h>
#include <sys/stat.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <boost/math/special_functions/fpclassify.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Range.h>
#include "rel_estimator/constants.h"
#include "rel_estimator/navnode.h"
#include "rel_estimator/navedge.h"
#include "rel_estimator/statepacket.h"
#include "rel_MEKF/relative_state.h"
#include "rel_MEKF/edge.h"
#include <visualization_msgs/Marker.h>



/*!
 *  \class Estimator estimator.h "include/rel_estimator/estimtor.h"
 *
 *  \brief This class implements a Multiplicative Extended Kalman Filter to estimate the states of a hexacopter aircraft.
 *
 *  The EKF uses gyro measurements in the prediction and accelerometers, altimeter or laser, and visual odometry as measurement
 *  updates.  The rotation information is in quaternion form (different from my windows implementation).
 *
 *  As an MEKF, the error for the quaternion is computed using the quaternion product instead of subtraction.
 *  A result of this is that the covariance is smaller than the state (by one for each quaternion in the state).
 *  Consequently, the covariance is predicted forward using the error dyanmics.  Also, measurements update the error
 *  state, which is then used to update the real state.
 *
 *  The other major point is that the position and yaw angle information is relative to the current keyframe of the visual
 *  odometry (view matching); consequently those state change each time a new node is declared. The other states remain.
 *
 *  The estimator estimates [f r d qx qy qz qw u v w bp bq br ax ay | cqx cqy cqz cqw cx cy cz] where f,r,d are the
 *  front, right, and down displacements from the current node; qx, qy, qz, qw are the quaternion orientation
 *  (w/ qz relative); u, v, w are the body-fixed frame velocities; bp, bq, br are the gyro biases;
 *  ax, ay are the accel biases.  Optionally, the calibration between the body-fixed frame and the camera frame can be
 *  estimated by the filter as well - the option is enabled by the #defines STATE_LENGTH and COVAR_LENGTH in constants.h
 *
 *  A word about coordinate frames:  We are using a North-East-Down global frame.  The body-fixed coordinate frame is
 *  based at the center of mass (or rather the location of the microstrain IMU, which is close).  The x axis goes out
 *  the front, the z axis out the bottom, and the y axis out the right side (from a pilot's point of view) - to make the
 *  coordinate frame right handed.
 *  \image html coordinate_frames.jpg "This figure illustrates the coordinate frames used by the estimator."
 *
 *  \note Because this class implements an EKF, there are many variables that need to retain memory after the scope is
 *  changed, hence there are many class variables.  This class could not really be implemented in a distributed fashion
 *  very well either.
*/
class Estimator
{

public:
  /// Eigen macro used when there are fixed-sized class member variables and you dynamically create an instance of the
  /// class.  (See: http://eigen.tuxfamily.org/dox/TopicStructHavingEigenMembers.html)
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW


  /*!
   *  \brief Estimator is the constructor. It initializes the class variables (there are a lot in this class)
   *
   *  \param mk_const provides the common constants used througout the program
  */
  Estimator(Constants *mk_const);


  /*!
   *  \brief The destructor
  */
  ~Estimator();


  /*!
   *  \brief This function initializes the filter state and covariance.  It is called continuously until it is determined
   *  that the vehicle is flying and the estimator should begin estimating the states.
   *
   *  If truth information is available, the initial position will be initialized with respect to that truth.   
   *  \param imu_data is the most recent IMU packet recieved
   *  \param hex_data is the optional packet of debug data recieved by the Hexacopter.  It contains IMU, motorspeeds, voltage etc.
   *  \param alt_data is the optional altitude packet (to detect takeoff)
   *  \param truth_data is by default not available, when it is, the filter uses it to initialize.
  */
  void Initialize(IMU_message &imu_data, Hex_message *hex_data = NULL,
                  sensor_msgs::Range *alt_data = NULL, TRUTH_message *truth_data = NULL);


  /*!
   *  \brief This function is used to enable delayed data updates for vision data. It is called when vision data has
   *  been recieved.  It sorts through the queued IMU and state data and finds the data that should be applied
   *  This is the only function that removes items from the different sensor queues.  The queues are only accessed by
   *  delayedVisionUpdate().  This is to support that possiblity that before a picture is processed, another picture is
   *  taken and will need some of the IMU and altitude measurements that are used in the delayedVisionUpdate().
   *
   *  \note This function assumes that the IMU is the "drumbeat" of the EKF and everything happens when IMU is available.
   *  \param timestamp is from the most recent visual odometry message
  */
  void prepareQueuedItems(ros::Time timestamp);


  /*!
   *  \brief delayedVisionUpdata provides the measurement update for delayed vision data after it has been expressed
   *  properly using transformToNodeFrame.  This function retrieves the state and covariance when the image was taken,
   *  applies the update (through the function directVisionUpdate) and then repropogates the state and covariance
   *  forward using saved IMU and altitude data.
   *  Also, when new nodes are created, it applys this information (i.e. augmentMarginalize() is called here).
   *  If the timestamp of the vo is such that a delayed update is not necessary, this function calls directVisionUpdate()
   *  for the update to be processed immediately.
   *
   *  \param node_vo_data is vision (or truth) data expressed in the node frame, ready for the measurement update
   *  \param truth_data is optional, used when a new node is declared, the truth data is placed into the node (but isn't
   *  used, it's only there for comparisons)
  */
  void delayedVisionUpdate(VO_message &node_vo_data, TRUTH_message *truth_data = NULL);


  /*!
   *  \brief prediction takes the gyro measurement and predicts the state and covariance forward a timestep.  Actually,
   *  it brings the state and covariance from the previous timestep up to the current one.  It uses saved gyro values
   *  from the previous IMU data for the update.  So accels are used at the current timestep and gyros are used to
   *  to predict on the next timestep.  It was done this way since we do not have a completely stable dt value.  It is
   *  always slightly different and this way allows us to use the dt instead of guessing it.
   *
   *  /param N is the number of mini loops to complete to improve the linearization (Continuous-Discrete EKF)
   *  /param dt is the timestep defined as the difference between the current time and the previous time
  */
  void prediction(int N, double dt,IMU_message &imu_data);


  /*!
   *  \brief The imuMeasurementUpdate function updates the state and covariance using accelerometer measurements.  The
   *  gyro values are saved here for the next predition() call.
   *
   *  \param imu_data is the most recent IMU data recieved.
  */
  void imuMeasurementUpdate(IMU_message &imu_data);

#ifdef DETECT
  /*!
   *  \brief The altitudeMeasurementUpdate updates the state and covariance using the altimeter or laser data.  Since the down
   *  information is relative, this measurement data is made relative before applying the update (for the altimeter only).
   *
   *  The function checks to make sure data is available, if it is a null pointer, the update isn't applied.
   *
   *  \param alt_data is the pointer to the most recently recieved altitude data.
  */
  void altitudeMeasurementUpdate(sensor_msgs::Range *alt_data, bool normal_update);
#else
  /*!
   *  \brief The altitudeMeasurementUpdate updates the state and covariance using the altimeter or laser data.  Since the down
   *  information is relative, this measurement data is made relative before applying the update (for the altimeter only).
   *
   *  The function checks to make sure data is available, if it is a null pointer, the update isn't applied.
   *
   *  \param alt_data is the pointer to the most recently recieved altitude data.
  */
  void altitudeMeasurementUpdate(sensor_msgs::Range *alt_data);
#endif


//  /*!
//   *  \brief This is a pseudo-measurement update that is based on the unit quaternion constraint.  This function helps
//   *  maintain that constraint.
//   *  It uses the regular update equations, but the measurement is 0 and the modeled measurement is q^T*q - 1 (which
//   *  should be zero)
//  */
//  void quaternionMeasurementUpdate();


  /*!
   *  \brief saveData saves the state, covariance, IMU, and altitude data in queues to enable the delayed updates.
   *
   *  \param imu_data is the most recently recieved IMU
   *  \param alt_data is the most recently recieved altitude data, when available
  */
  void saveData(IMU_message &imu_data, sensor_msgs::Range *alt_data = NULL);


  /*!
   *  \brief This function checks the state and covariance to see if a new node is necessary.
   *
   *  \returns a bool indicating if a new node is needed (true) or not (false)
  */
  bool checkNewNode();


  /*!
   *  \brief This function converts the relative state (and saved edges) into a global pose estimate.
   *
   *  \attention This function returns coordinates in the ROS global {north west up} frame of reference!
   *
   *  \param stamp is the current (IMU based) ros::Time stamp
   *  \param global_name is the name of the global frame
   *  \param body_name is the name of the body frame
   *  \returns a transform corresponding to the estimated current global position and orientation
  */
  geometry_msgs::TransformStamped computeGlobalPoseEstimate(ros::Time stamp, std::string &global_name, std::string &body_name);


  /*!
   *  \brief The current relative pose is returned
   *
   *  \returns the current relative state vector
  */
  Eigen::Matrix<double,STATE_LENGTH,1> returnRelativeState(){return x_;}


  /*!
   *  \brief This function writes the current state and sensor information to a log file for further analysis
   *
   *  \param imu_data is the most recent IMU
   *  \param global_pose is the current estimate of the global position
   *  \param alt_data is altitude packet data, when available
   *  \param vo_data is the vo packet when available
   *  \param truth_data is the truth packet when available
  */
  void writeToLog(IMU_message &imu_data,
                  geometry_msgs::TransformStamped &global_pose,
                  sensor_msgs::Range *alt_data = NULL,
                  VO_message *vo_data = NULL,
                  TRUTH_message *truth_data = NULL);


  /*!
   *  \todo Implement this function.  We had this function in the windows version, but we haven't implemented it here yet.
   *  \brief makeDataRelative takes the truth data and makes it relative so it is just like data coming out of a
   *  visual odometry algorithm.  This way motion capture can be used as a measurement update in the filter.
   *  (noise and delay are added)
   *  \param truth_data is the most recently recieved truth data.
   *  \returns the transformation expressed as a visual odometry message
  */
  VO_message makeDataRelative(TRUTH_message &truth_data);

#ifdef LASER
#ifdef DETECT
  /*!
   *  \brief This function packs up the laser status into a message to be sent over ROS.
   *  \param timestamp Send in the current timestamp (defined be the current IMU message)
   *  \returns A Status_message
  */
  Status_message packageLaserStatus(ros::Time timestamp);
#endif
#endif


  /*!
   *  \brief This function packs up the state into a message to be sent over ROS.
   *  \note This returns the state in the relative node coordinate frame (front, right, and down)
   *  \param timestamp Send in the current timestamp (defined be the current IMU message)
   *  \returns A rel_MEKF::relative_state message
  */
  rel_MEKF::relative_state packageStateInMessage(ros::Time timestamp);


  /*!
   *  \brief Package up the current node into a message, to send out
   *
   *  \attention This is placed in a NWU (north west up) coordinate system that ROS expects for the global coordinates.
   *  The node information is maintained in the global NED coordinate frame in the estimator, but on export we change it to
   *  the NWU frame.
   *
   *  \param global_name is the name for the global coordinate frame that the system is using for global visualization
   *  \param base_name holds the name that you attach the node number to for publishing what frame it is.  (so we can map the nodes in the global frame)
  */
  geometry_msgs::TransformStamped packageCurrentNode(ros::Time timestamp, std::string &global_name, std::string &base_name);


  /*!
   *  \brief Pack up the current edge into a message.  Each edge is defined in the node coordinate system where it orginates.
   *  These are the relative transformations from node to node.  Only yaw is included as the node coordinate frames are aligned with global down
   *  So the coordinates are [f_j, r_j, d_j], with the yaw defined as the angle between f_j and f_{j+1}, positive about the d_j axis
  */
  rel_MEKF::edge packageCurrentEdge(ros::Time timestamp);

//  /*!
//   *  \brief This function takes in a quaternion, verifies it is a unit quat. and converts it to a rotation matrix
//  */
//  Eigen::Matrix3d convertQuaterionToRotation(Eigen::Quaterniond &q);

//  /*!
//   *  \brief This function takes a rotation and converts to a quaternion
//  */
//  Eigen::Quaterniond convertRotationToQuaternion(Eigen::Matrix3d &R);

  //Public variable:
  bool startup_flag_; //!< This PUBLIC Flag is false when the estimator is calculating (starts true)
  bool just_landed_; //!< public flag for ignoring trying to restart the estimator after we've just landed the hex

protected:

  /*!
   *  \brief directVisionUpdate contains the actual vision update code.  It is called by the delayedVisionUpdate funtion
   *  in two scenarios, if the vision update time is such that it doesn't need a delayed update, the vision update is
   *  processed immediately, using this function.  The next is when there is a delay, the delayedVisionUpdate rolls
   *  back the state, calls this function to process the update, and then the state and covariance is repropogated back
   *  to the current time in the delayedVisionUpdate function.
   *
   *  \param node_vo_data is vision (or truth) data expressed in the node frame, ready for the measurement update
   *  \param override_keyframe is used by delayedVisionUpdate so that this function does not process a new keyframe.
   *         When it is true, this function will not allow a new node and thus have the state augmented and marginalized
   *  \param truth_data is optional, used when a new node is declared, the truth data is placed into the node (but isn't
   *  used, it's only there for comparisons)
  */
  void directVisionUpdate(VO_message &node_vo_data, bool override_keyframe, TRUTH_message *truth_data = NULL);


//  /*!
//   *  \brief This version of the above function applies the position and rotation measurements separetly.
//  */
//  void directVisionUpdateSeparate(VO_message &node_vo_data, double inflate, TRUTH_message *truth_data = NULL);

  /*!
   *  \brief This function is used to augment the filter with new relative state and marginalize the old relative states.
   *  It is called by the delayedVisionUpdate() function.
   *  \param delta_quat is the delta quaternion calculated by the VO
  */
  void augmentMarginalize(Eigen::Vector3d &delta_quat);


  /*!
   *  \brief Provides a unique filename based on the time
   *
   *  \param time_struct is a time structure
   *  \returns a string containing the month, day, hour, and minute
  */
  std::string timeStructFilename(tm *time_struct);

  /*!
   *  \brief Given the delta_state, it applies the correction to the actual state after a measurement update
   *  \param delta_state is the error state computed by the measurement update, of length COVAR_LENGTH
  */
  void applyCorrection(Eigen::Matrix<double,COVAR_LENGTH,1> &delta_state);

  /*!
   *  \brief This function cheats in a way, we pull the quaternion out of the state, normalize it, and overwrite the
   *  state's version with the normalized version
  */
  inline void normalizeQuaternion()
  {
    double mag;
    mag = sqrt(x_(3,0)*x_(3,0) + x_(4,0)*x_(4,0) + x_(5,0)*x_(5,0) + x_(6,0)*x_(6,0));
    x_(3,0) = x_(3,0)/mag;
    x_(4,0) = x_(4,0)/mag;
    x_(5,0) = x_(5,0)/mag;
    x_(6,0) = x_(6,0)/mag;
  }


  /*!
    * \todo: Make this function general using templates so that it expands to whatever size is placed in it...
   * \brief Quick conversion between an Eigen Matrix and a Boost::array
   * \param matrix is the Eigen matrix (comes in full)
   * \param matptr is the boost::array which is returned as a copy of matrix
  */
  inline void eigenToMatrixPtr(const Eigen::Matrix<double,6,6> &matrix, boost::array<double,36> &matptr)
  {
    for( size_t row = 0; row < 6; row++ )
      for( size_t col = 0; col < 6; col++ )
      {
        matptr[ row * 6 + col ] = matrix(row,col);
      }
  }

  //
  //Variables: **********************************************************************************************
  //
  Constants *mk_consts_; //!< The pointer to the constants class

  //State and covariance
  Eigen::Matrix<double, STATE_LENGTH, 1> x_; //!< the state of the EKF: [f r d qx qy qz qw u v w bx by bz ax ay] (don't need az)
  Eigen::Matrix<double,COVAR_LENGTH,COVAR_LENGTH> P_; //!< the covariance of the EKF

  //Prediction variables:
  Eigen::Matrix<double,STATE_LENGTH,1> f_; //!< the nonlinear prediction equations for x
  Eigen::Matrix<double,COVAR_LENGTH,COVAR_LENGTH> A_; //!< the Jacobian of f_ with respect to (w.r.t.) the state x_
  Eigen::Matrix<double,COVAR_LENGTH,3> B_; //!< the Jacobian of f_ w.r.t. the inputs (gyro measurements)
  Eigen::Matrix3d G_; //!< the covariance of the inputs
  Eigen::Matrix<double,COVAR_LENGTH,COVAR_LENGTH> Q_; //!< the process noise (tuneable, but only have values for the gyro and accel biases)
  Eigen::Vector3d saved_gyros_; //!< the gyro readings that are saved during the IMU measurement update
  Eigen::Vector3d saved_deltatheta_; //!< save the deltatheta state from the measurement update for use in the next prediction
  Eigen::Vector3d saved_deltaV_; //!< save the deltaV portion of the delta state from the measurement update for the next prediction

  //Measurement update variables:
  //IMU
  Eigen::Matrix2d R_i_; //!< the measurement noise for the IMU
  //Altitude Sensor
  double R_a_; //!< the measurement noise for the altitude sensor
  //Quaternion
  double R_q_; //!< the measurment noise covariance for the quaternion contraint psuedo-update (very small)
  //Vision
  Eigen::Matrix<double,6,6> R_v_; //!< the measurement noise for the vision

  //Constants computed at runtime:
  double omega_h_; //!< the average motorspeed for hover
  double mu_; //!< the parameter for the drag, in the improved model

  //Queues for saving info:
  // I am using a deque instead of a regular queue, this allows me to iterate over the elements
  std::deque<IMU_message> i_queue_; //!< the IMU queue for delayed updates
  std::deque<sensor_msgs::Range> a_queue_; //!< the altitude queue for delayed updates
  /// \todo Probably need to find a better container for NavNode and NavEdge than a queue...
  /// \note When a class has fixed-size Eigen members, you must use an aligned allocator for standard containers:
  /// See: http://eigen.tuxfamily.org/dox/TopicStlContainers.html
  std::deque<NavNode, Eigen::aligned_allocator<NavNode> > node_queue_;  //!< the queue of nodes
  std::deque<NavEdge, Eigen::aligned_allocator<NavNode> > edge_queue_;  //!< the queue of edges between the nodes
  std::deque<StatePacket, Eigen::aligned_allocator<StatePacket> > state_queue_; //!< the queue of state and covariances


  Eigen::Vector3d global_node_position_; //!< the vector sum of the edges to provide the estimate of the global pose
  Eigen::Matrix3d global_R_yaw_; //!< the rotation matrix to express the relative edge information as global
  double global_yaw_; //!< the actual global psi

  int node_id_incrementer_; //!< the incrementer for the node ID (keeps account of the current node number)

  double lpf_accz_; //!< low pass filtered z acceleration for detecting takeoff
  double lpf_old_;  //!< the previous value of lpf_accz_
  static const double ACCZ_THRESHOLD_ = -10.1; //!< if the lpf_accz_ drops below this value, we've taken off (or pretty close)
  static const double ALPHA_ = 0.9; //!< lpf constant

  //Logging Info
  std::ofstream log_file_; //!< for writing the log file
  TRUTH_message node_truth_; //!< the truth recieved when a new node was declared

  //Fault Detection Variables
  int fault_flag_; //!< indicates which tests a given measurement has failed
  double residual_a_; //!< used for storing the residual for the laser measurement
  double residual_normalized_; //!< used for storing the normalized residual for the laser measurement
  double d_apriori_; //!< used for storing x_(2) before a laser measurement update takes place
  double d_aposteriori_; //!< used for storing x_(2) after a laser measurement update takes place
  std::deque<double> residual_queue_; //!< used to store the last n normalized residuals, where n is defined by window_size in constants.h
  std::deque<int> flag_queue_; //!< used to store the last n fault flags for determination of sensor failure
  double num_window_faults_; //!< counts the number of faults in the current window
  bool sensor_failure_; //!< decision variable about the health of the sensor (not the individual measurements)
  bool faulty_data_yet_; //!< will be used to tell if there is faulty data being injected yet (for analysis of delay to detection only)

  double window_mean_; //!< the mean of the normalized residuals in residual_queue_
  double mean_statistic_; //!< the statistic used for comparing to the mean threshold
  double window_covariance_; //!< the covariance of the normalized residuals in residual_queue_
  double covariance_statistic_; //!< the statistic used for comparing to the covariance threshold
  double threshold_outlier_; //!< stores the threshold for the outlier test
  double threshold_mean_; //!< stores the threshold for the test of mean
  double threshold_covariance_; //!< stores the threshold for the test of covariance

  int num_laser_updates_;
  bool normal_update_; //!< keeps track of when there is a normal update and when it is a repropagation


  //std::ofstream residual_file_; //

  /*!
   *
   *
   *
  */
};


#endif
