 /* \copyright This work was completed by Robert Leishman while performing official duties as 
  * a federal government employee with the Air Force Research Laboratory and is therefore in the 
  * public domain (see 17 USC § 105). Public domain software can be used by anyone for any purpose,
  * and cannot be released under a copyright license
  */

/*!
 *  \file estimator.cpp
 *  \author Robert Leishman
 *  \date June 2012
 *
*/

#include "rel_estimator/estimator.h"
#include "rel_estimator/eigen_utils.h"
#include <boost/math/distributions/normal.hpp>
#include <boost/math/distributions/chi_squared.hpp>

using namespace Eigen;

//
//  Constructor
//
Estimator::Estimator(Constants *mk_const): mk_consts_(mk_const)
{    
  int covar_len = COVAR_LENGTH;
  int state_len = STATE_LENGTH;
  ROS_ASSERT_MSG(covar_len != 14 || covar_len != 20, "Improper Covariance Length! Check the length in constants.h!");
  ROS_ASSERT_MSG(state_len != 15 || state_len != 22, "Improper State Length!  Check the length in constants.h!");

  //Flags
  startup_flag_ = true;
  just_landed_ = false;

  //Fault Detection Variables
  fault_flag_ = 0;
  residual_a_ = 0.0;
  residual_normalized_ = 0.0;
  d_apriori_ = 0.0;
  d_aposteriori_ = 0.0;
  num_laser_updates_ = 0;
  normal_update_ = false;

  //Constants computed at runtime:
  omega_h_ = sqrt(mk_consts_->g * mk_consts_->mass / (mk_consts_->kF * 6)); /// \note: 6 is for the number of rotors
  mu_ = mk_consts_->lam1x * 6 * omega_h_; /// 6 for number of rotors here as well

  //Initialize all the variables:
  //State and covariance
  x_.setZero(); //state x_ = [f r d qx qy qz qw u v w bp bq br ax ay | cqx cqy cqz cqw cx cy cz]
  x_(6,0) = 1.0; //the quaternion scaler
  P_.setZero();

  //Prediction variables:
  f_.setZero();
  A_.setZero();
  B_.setZero();
  G_.setZero();
  Q_.setZero();
  saved_deltatheta_.setZero();
  saved_deltaV_.setZero();

  //Measurement update variables:
  //IMU
  R_i_.setZero();

  //Altitude Sensor
  R_a_ = 0.d;

  //Quaternion
  R_q_ = 0.d;

  //Vision
  R_v_.setZero(); //variable, set with when vision data comes in

  node_id_incrementer_ = 0;
  global_R_yaw_.setIdentity();
  global_node_position_.setZero();
  global_yaw_ = 0.d;

  lpf_accz_ = 0;
  lpf_old_ = 0;

  //Setup the Log:
  time_t rawtime;
  struct tm *utc_time; 

  rawtime = time(NULL);
  utc_time = localtime(&rawtime);

  std::string file_name = "./estimator_logs/";
  bool error = mkdir(file_name.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH); // rwx permissions for owner and group, rx persmissions for others
  if(error && errno != EEXIST)
  {
    std::cout<<"ERROR: unable to create folder for log files.  Error code = "<<errno<<std::endl
       <<"Press enter to exit the program."<<std::endl;
    getchar();
    exit(1);
  }
  file_name += "estimate";
  file_name += timeStructFilename(utc_time);
  file_name.append(".txt");
  log_file_.open(file_name.c_str(), std::ios::out | std::ios::trunc);

//  std::string filename = "residual";
//  filename += timeStructFilename(utc_time);
//  filename.append(".txt");
//  residual_file_.open(filename.c_str(), std::ios::out | std::ios::trunc);
}



//global_node_position_
// Destructor
//
Estimator::~Estimator()
{
  //Destroy stuff...
  delete mk_consts_;

  //close stuff
  log_file_.close();
}


//
// Initialize the filter: (usually, we enter this function MANY times before taking off)
//
void Estimator::Initialize(IMU_message &imu_data, Hex_message *hex_data,
                           sensor_msgs::Range *alt_data, TRUTH_message *truth_data)
{

  /// \todo Make this function compatible with landing and taking off again!

  if(hex_data != NULL)
  {
    // state x_ = [f r d qx qy qz qw u v w bp bq br ax ay | cqx cqy cqz cqw cx cy cz]

    //Use the mikrokopter estimates for an idea:
    double phi, theta;
    phi = hex_data->angleRoll;
    theta = hex_data->anglePitch;
    x_(3,0) = cos(0.5*theta)*sin(0.5*phi);
    x_(4,0) = sin(0.5*theta)*cos(0.5*phi);
    x_(5,0) = -sin(0.5*theta)*sin(0.5*phi);
    x_(6,0) = cos(0.5*theta)*cos(0.5*phi);
  }
  else
  {
    x_(3,0) = 0.d;
    x_(4,0) = 0.d;
    x_(5,0) = 0.d;
    x_(6,0) = 1.d;
  }

  //zero out any velocities
  x_(7,0) = 0.0;
  x_(8,0) = 0.0;
  x_(9,0) = 0.0;
  x_(10,0) = mk_consts_->gyrox_bias;
  x_(11,0) = mk_consts_->gyroy_bias;
  x_(12,0) = mk_consts_->gyroz_bias;
  x_(13,0) = mk_consts_->accelx_bias;
  x_(14,0) = mk_consts_->accely_bias;
  if(STATE_LENGTH > 15)
  {
    //Calibrating!
    x_(15,0) = mk_consts_->qx;
    x_(16,0) = mk_consts_->qy;
    x_(17,0) = mk_consts_->qz;
    x_(18,0) = mk_consts_->qw;
    x_(19,0) = mk_consts_->cx;
    x_(20,0) = mk_consts_->cy;
    x_(21,0) = mk_consts_->cz;
  }

#ifndef LASER // ALTIMETER IS BEING USED
  //                 [0  1  2  3   4   5   6  7  8  9   10  11  12  13    14   15   16   17  18  19 ]
  // Covariance P_ = [df dr dd dqx dqy dqz du dv dw dbp dbq dbr dax day | dcqx dcqy dcqz dcx dxy dcz]
  //The P_ and Q_ happen to be tuned for the Roumeliotis method for voQ measurement
  P_(0,0)   = 0.000001;//mk_consts_->P_5mm;
  P_(1,1)   = 0.000001;//mk_consts_->P_5mm;
  P_(2,2)   = 0.000001;//mk_consts_->P_5mm;
  P_(3,3)   = 0.00000001;//mk_consts_->P_1deg;
  P_(4,4)   = 0.00000001;//mk_consts_->P_1deg;
  P_(5,5)   = 0.00000001;//mk_consts_->P_1deg;
  P_(6,6)   = 0.000001;//mk_consts_->P_05ms;
  P_(7,7)   = 0.000001;//mk_consts_->P_05ms;
  P_(8,8)   = 0.000001;//mk_consts_->P_05ms;
  P_(9,9)   = 0.000001;//mk_consts_->P_1;
  P_(10,10) = 0.000001;//mk_consts_->P_1;
  P_(11,11) = 0.000001;//mk_consts_->P_1;
  P_(12,12) = 0.0051;//mk_consts_->P_1;
  P_(13,13) = 0.0051;//mk_consts_->P_1;
  if(COVAR_LENGTH > 14)
  {
    //Calibrating
    P_(14,14) = 0.0001;//mk_consts_->P_1;
    P_(15,15) = 0.0001;//mk_consts_->P_1;
    P_(16,16) = 0.0001;//mk_consts_->P_1;
    P_(17,17) = 0.0001;//mk_consts_->P_1;
    P_(18,18) = 0.0001;//mk_consts_->P_1;
    P_(19,19) = 0.0001;//mk_consts_->P_1;
  }

  /// \note: Increase Q on a parameter for faster response. Increase R on a measurement for smoothing (more filtering)
  /// I have noticed that position is good when Q_ for v is high and velocity is good when Q_ for v is low.  So then I
  /// started adjusting the R values for the measurement updates...  I would set these and the R's back to bland values and start
  /// from there, if I needed to tune again...
//  Q_(0,0) = 0.01;//0.001;//0.035;//I can have these two at zero, but this gives better position performance
//  Q_(1,1) = 0.01;//0.001;//0.06;//0.065
//  Q_(2,2) = 0.0001;//Don't really need faster response on down, it's already quite fast
//  Q_(3,3) = 0.0025;//;//The IMU and enhanced model do a good job with attitude as is
//  Q_(4,4) = 0.0025;//;
//  Q_(5,5) = 0.0;
//  Q_(6,6) = 0.007;//0.0001;//0.05;// These parameters can be adjusted quite a bit, all the way down to 0.0001, but here seems ok
//  Q_(7,7) = 0.02;//0.0001;//0.07;
//  Q_(8,8) = 0.0007;//0.01;
//  //biases don't change fast:
//  Q_(9,9) =   0.00000000051;//
//  Q_(10,10) = 0.00000000005;//
//  Q_(11,11) = 0.000000000051;//
//  Q_(12,12) = 0.000000001;//
//  Q_(13,13) = 0.000000001;//
//  if(COVAR_LENGTH > 14)
//  {
//    //Shouldn't these be zero, as they are constants???  Need to think about that.
//    Q_(14,14) = 0.0000000001;//
//    Q_(15,15) = 0.0000000001;//
//    Q_(16,16) = 0.0000000001;//
//    Q_(17,17) = 0.000000001;//
//    Q_(18,18) = 0.000000001;//
//    Q_(19,19) = 0.000000001;//
//  }


  //Saving these original working params, just in case:
  Q_(0,0) = 0.01;//0.001;//0.035;//I can have these two at zero, but this gives better position performance
  Q_(1,1) = 0.01;//0.001;//0.06;//0.065
  Q_(2,2) = 0.0001;//Don't really need faster response on down, it's already quite fast
  Q_(3,3) = 0.00025;//;//The IMU and enhanced model do a good job with attitude as is
  Q_(4,4) = 0.00025;//;
  Q_(5,5) = 0.0;
  Q_(6,6) = 0.0002;//0.0;//0.0001;//0.05;// These parameters can be adjusted quite a bit, all the way down to 0.0001, but here seems ok
  Q_(7,7) = 0.0002;//0.0;//0.0001;//0.07;
  Q_(8,8) = 0.009;//0.01;
  //biases don't change fast:
  Q_(9,9) =   0.00000000051;//
  Q_(10,10) = 0.00000000005;//
  Q_(11,11) = 0.000000000051;//
  Q_(12,12) = 0.000000001;//
  Q_(13,13) = 0.000000001;//
  if(COVAR_LENGTH > 14)
  {
    //Shouldn't these be zero, as they are constants???  Need to think about that.
    Q_(14,14) = 0.0000000001;//
    Q_(15,15) = 0.0000000001;//
    Q_(16,16) = 0.0000000001;//
    Q_(17,17) = 0.000000001;//
    Q_(18,18) = 0.000000001;//
    Q_(19,19) = 0.000000001;//
  }


#else //LASER IN USE!

  //                 [0  1  2  3   4   5   6  7  8  9   10  11  12  13    14   15   16   17  18  19 ]
  // Covariance P_ = [df dr dd dqx dqy dqz du dv dw dbp dbq dbr dax day | dcqx dcqy dcqz dcx dxy dcz]
  //The P_ and Q_ happen to be tuned for the Roumeliotis method for voQ measurement
  P_(0,0)   = 0.000001;//mk_consts_->P_5mm;
  P_(1,1)   = 0.000001;//mk_consts_->P_5mm;
  P_(2,2)   = 0.000001;//mk_consts_->P_5mm;
  P_(3,3)   = 0.00000001;//mk_consts_->P_1deg;
  P_(4,4)   = 0.00000001;//mk_consts_->P_1deg;
  P_(5,5)   = 0.00000001;//mk_consts_->P_1deg;
  P_(6,6)   = 0.000001;//mk_consts_->P_05ms;
  P_(7,7)   = 0.000001;//mk_consts_->P_05ms;
  P_(8,8)   = 0.000001;//mk_consts_->P_05ms;
  P_(9,9)   = 0.000001;//mk_consts_->P_1;
  P_(10,10) = 0.000001;//mk_consts_->P_1;
  P_(11,11) = 0.000001;//mk_consts_->P_1;
  P_(12,12) = 0.001;//mk_consts_->P_1;
  P_(13,13) = 0.001;//mk_consts_->P_1;
  if(COVAR_LENGTH > 14)
  {
    //Calibrating
    P_(14,14) = 0.00001;//mk_consts_->P_1;
    P_(15,15) = 0.00001;//mk_consts_->P_1;
    P_(16,16) = 0.00001;//mk_consts_->P_1;
    P_(17,17) = 0.0001;//mk_consts_->P_1;
    P_(18,18) = 0.0001;//mk_consts_->P_1;
    P_(19,19) = 0.0001;//mk_consts_->P_1;
  }

  /// \note: Increase Q on a parameter for faster response. Increase R on a measurement for smoothing (more filtering)
  /// I have noticed that position is good when Q_ for v is high and velocity is good when Q_ for v is low.  So then I
  /// started adjusting the R values for the measurement updates...  I would set these and the R's back to bland values and start
  /// from there, if I needed to tune again...
  Q_(0,0) = 0.003;//0.0;//0.035;//I can have these two at zero, but this gives better position performance
  Q_(1,1) = 0.020;//0.0;//0.06;//0.065
  Q_(2,2) = 0.000001;//Don't really need faster response on down, it's already quite fast
  Q_(3,3) = 0.0002;//0.0001;//The IMU and enhanced model do a good job with attitude as is
  Q_(4,4) = 0.0008;//0.0001;
  Q_(5,5) = 0.00000000000000001;
  Q_(6,6) = 0.0001;//0.0;//0.0001;//0.05;// These parameters can be adjusted quite a bit, all the way down to 0.0001, but here seems ok
  Q_(7,7) = 0.0003;//0.0;//0.0001;//0.07;
  Q_(8,8) = 0.05;//0.01;
  //biases don't change fast:
  Q_(9,9) =   0.00000000051;//
  Q_(10,10) = 0.00000000005;//
  Q_(11,11) = 0.000000000051;//
  Q_(12,12) = 0.000000001;//
  Q_(13,13) = 0.000000001;//
  if(COVAR_LENGTH > 14)
  {
    Q_(14,14) = 0.0000000001;//
    Q_(15,15) = 0.0000000001;//
    Q_(16,16) = 0.0000000001;//
    Q_(17,17) = 0.00000001;//
    Q_(18,18) = 0.00000001;//
    Q_(19,19) = 0.00000001;//
  }

#endif




  G_(0,0) = mk_consts_->gyrox_std*mk_consts_->gyrox_std;
  G_(1,1) = mk_consts_->gyroy_std*mk_consts_->gyroy_std;
  G_(2,2) = mk_consts_->gyroz_std*mk_consts_->gyroz_std;

  R_i_(0,0) = mk_consts_->accel_x_std*mk_consts_->accel_x_std*mk_consts_->acc_x_inflate;
  R_i_(1,1) = mk_consts_->accel_y_std*mk_consts_->accel_y_std*mk_consts_->acc_y_inflate;

  R_a_ = mk_consts_->alt_std*mk_consts_->alt_std;  

//  R_q_ = mk_consts_->quat_std*mk_consts_->quat_std;

  //Initialize the first node (use truth if available)
  if(truth_data != NULL)
  {
    NavNode node1(*truth_data,node_id_incrementer_+1); //New node
    NavEdge edge1(*truth_data,node_id_incrementer_,node_id_incrementer_+1); //Node 0 is the global origin
    Vector3d temp(truth_data->transform.translation.x,truth_data->transform.translation.y,truth_data->transform.translation.z);

    Quaterniond qtemp(x_(6,0),x_(3,0),x_(4,0),x_(5,0));
    node1.setEstimatePosition(global_node_position_,global_yaw_,qtemp);

    //Use the most recent truth data to declare the node:
    if(!node_queue_.empty())
      node_queue_.clear();

    if(!edge_queue_.empty())
      edge_queue_.clear();

    node_queue_.push_front(node1);
    edge_queue_.push_front(edge1);
  }
  else
  {
    if((int)node_queue_.size() == 0)
    {
      //If truth is not avaliable, intialize to zero, this spot becomes the origin
      NavNode node1(node_id_incrementer_+1);
      NavEdge edge1(x_,P_,node_id_incrementer_,node_id_incrementer_+1);
      node_queue_.push_front(node1);
      edge_queue_.push_front(edge1);
    }
  }

  lpf_accz_ = (1.0 - ALPHA_)*imu_data.linear_acceleration.z + ALPHA_*lpf_old_;
  lpf_old_ = lpf_accz_;
  //Check to see if we are flying:
  //if(alt_data != NULL && USE_LASER == 0 && alt_data->range <= -0.35)//(imu_data->linear_acceleration.z > mk_consts_->acc_z_switch || imu_data.motor1 > 125)

#ifndef LASER
  if(lpf_accz_ <= ACCZ_THRESHOLD_ && !just_landed_)
  {
    //Accel indicates we are flying
    startup_flag_ = false;
    node_id_incrementer_++;//update the node now, when we won't go back into this function

    //initialize the global estimated position and orientation
    NavEdge firstedge;
    firstedge = edge_queue_.front();
    global_node_position_ = global_R_yaw_ * firstedge.getTranslation();

    global_R_yaw_ = global_R_yaw_*firstedge.getR_curr_next().transpose();
    global_yaw_ = firstedge.getPsi_i();

    //update saved gyros used in prediction
    saved_gyros_(0) = imu_data.angular_velocity.x;
    saved_gyros_(1) = imu_data.angular_velocity.y;
    saved_gyros_(2) = imu_data.angular_velocity.z;
  }
#else
  if (alt_data != NULL && alt_data->range > 0.10 && !just_landed_)//&*&
  {
      //Laser indicates we are flying
      startup_flag_ = false;
      node_id_incrementer_++;//update the node now, when we won't go back into this function

      //initialize the global estimated position and orientation
      NavEdge firstedge;
      firstedge = edge_queue_.front();
      global_node_position_ = global_R_yaw_ * firstedge.getTranslation();

      global_R_yaw_ = global_R_yaw_*firstedge.getR_curr_next().transpose();
      global_yaw_ = firstedge.getPsi_i();

      //update saved gyros used in prediction
      saved_gyros_(0) = imu_data.angular_velocity.x;
      saved_gyros_(1) = imu_data.angular_velocity.y;
      saved_gyros_(2) = imu_data.angular_velocity.z;
  }
#endif
}



//
//  Vision Data recieved: prepare to bring the state back to when the image was taken
//  This function assumes that the IMU is the "drum beat" of the system and that all actions take place when it is available
//
void Estimator::prepareQueuedItems(ros::Time timestamp)
{
  ros::Time time = timestamp; //Image was taken at "time"
  int elim = 0; //number of elements eliminated
  StatePacket tempstate1;
  StatePacket tempstate2;
  int count = (int)state_queue_.size();

  for (int j = 0; j < count; j++)
  {
    tempstate1 = state_queue_.front();

    if (tempstate1.getTime() > time)
    {
      //double dt1 = tempstate1.getTime().toSec() - time.toSec();
      //The camera time is before the first entry, use this time/state for the update
      ROS_INFO("There were not enough terms in the state queue - Using 1st Element!");
      break;
    }

    if (j + 1 < count)
    {
      //make sure that there is enough room in the queue:
      tempstate2 = state_queue_.at(1);
      if (tempstate2.getTime() > time)
      {
        //The Camera info applies at some time between these two packets. Use the first one to predict
        // the state up to the camera time and then predict the rest of the way up to the next IMU packet
        break;
      }
      else
      {
        //The camera time is not between the two statepackets, eliminate the older one and try again
        state_queue_.pop_front();
        elim++;
      }
    }
    else
    {
      ROS_INFO("Not enough elements in State_Queue to find closest one to camera time!");
      //apply the camera update at the current time, clear the queue and the delayed update will handle this:
      state_queue_.clear();
      elim = count + 1;   //this will clear the IMU queue as well
      break;
    }
  }

  //Console.WriteLine("Eliminated # packets from state queue: " + elim);

  if (elim > (int)i_queue_.size()) //error checking
  {
    ROS_INFO("Situation has occured: Camera update sent and recieved in-between 1 IMU update cycle!!!");
    i_queue_.clear();
  }
  else
  {
    for (int i = 0; i < elim; i++)
    {
      i_queue_.pop_front();
      a_queue_.pop_front();
    }
  }
}



//
//  Delayed Vision Update: called after transformToNodeFrame
//
void Estimator::delayedVisionUpdate(VO_message &vo_data,
                                    TRUTH_message *truth_data)
{
  if ((int)state_queue_.size() > 2 && (int)i_queue_.size() > 2)
  {
    //Do not attempt this version of the update if the state queue is empty!

    /// First step is to reverse time and replace x_ and P_ with the saved version:
    StatePacket tempstate;
    IMU_message tempimu;
    sensor_msgs::Range tempalt;

    tempstate = state_queue_.front();
    state_queue_.pop_front();
    x_ = tempstate.getState();
    P_ = tempstate.getCovariance();

    //Predict the state forward in time to the camera time and then apply the update
    tempimu = i_queue_.front();
    i_queue_.pop_front();
    a_queue_.pop_front();
    double dt_1 = vo_data.Timestamp().toSec() - tempimu.header.stamp.toSec();
    int start_at; //the IMU packet to start at for the repropagation below

    //approximate the first prediction, using the gyros from this sensor reading and zero out the delta values
    saved_gyros_(0) = tempimu.angular_velocity.x;
    saved_gyros_(1) = tempimu.angular_velocity.y;
    saved_gyros_(2) = tempimu.angular_velocity.z;
    saved_deltatheta_.setZero();
    saved_deltaV_.setZero();

    if (dt_1 > 0)
    {
      //VO should be applied between IMU timesteps
      //If there isn't sufficient info to go all the way back, do not do this step, it will be with a negative dt!

      prediction(mk_consts_->catchup_steps, dt_1,tempimu);  //predict based on the gyros for dt_1 timespan
    }

    //
    /// Process the visual odometry measurement update at the time the image was taken:
    // This is done in the directVisionUpdate function (that way we don't the same code twice)
    directVisionUpdate(vo_data, true, truth_data);

    if (dt_1 > 0)
    {
      //Predict the state forward to the next IMU timestep (we did the vision update between IMU measurements)
      tempimu = i_queue_.front();
      tempalt = a_queue_.front();
      start_at = 1; //the IMU packet to start at for the repropagation
      double dt_2 = tempimu.header.stamp.toSec() - vo_data.Timestamp().toSec(); //second dt to the next IMU packet time
      // This prediction will use the same values as the previous one did
      prediction(mk_consts_->catchup_steps, dt_2,tempimu);
      imuMeasurementUpdate(tempimu);  //complete the IMU measurement update at this timestep

#ifdef DETECT
      altitudeMeasurementUpdate(&tempalt, false);
#else
      altitudeMeasurementUpdate(&tempalt);
#endif

      //update the state and covariance that are saved in the queue:
      state_queue_.at(0).setState(x_);
      state_queue_.at(0).setCovariance(P_);
    }
    else
    {
      start_at = 0; //the IMU packet to start at for the repropagation
    }
    //
    /// If this data is from a new node image, need to replace the states!
    if (vo_data.NewReference())
    {
      //create the edge
      NavEdge newedge(x_, P_, node_id_incrementer_, node_id_incrementer_+1);
      edge_queue_.push_back(newedge);
      //create the new node
      NavNode newnode(node_id_incrementer_+1);
      node_id_incrementer_++; //increment to reflect the new current node

      if(truth_data)
        newnode.setTruePose(*truth_data);

      //Find the global estimated position and orientation of the new node
      global_node_position_ = global_node_position_ + global_R_yaw_ * newedge.getTranslation();
      //Save the global estimates
      Quaterniond temp(x_(6,0),x_(3,0),x_(4,0),x_(5,0));
      newnode.setEstimatePosition(global_node_position_,global_yaw_,temp);
      //update the global estimates for the next node
      global_R_yaw_ = global_R_yaw_ * newedge.getR_curr_next().transpose();  //update the rotation matrix, the current rotation is used for the NEXT translation
      global_yaw_ = global_yaw_ + newedge.getPsi_i();  //the angle applies to the next node!
      //store the node
      node_queue_.push_back(newnode);

      //Augment and Marginalize the State and Covariance!
      augmentMarginalize(saved_deltatheta_);
    }

    //
    /// Repropagate the IMU and altitude information back to current time
    //

    //Now, iterate through the IMU information in the queue and reapply it:
    int count = (int)i_queue_.size();
    double old_time = tempimu.header.stamp.toSec();

    for (int i = start_at; i < count; i++)
    {
      tempimu = i_queue_.at(i);
      tempalt = a_queue_.at(i);
      double dt = tempimu.header.stamp.toSec() - old_time;
      old_time = tempimu.header.stamp.toSec();
      prediction(mk_consts_->catchup_steps, dt,tempimu);
      imuMeasurementUpdate(tempimu);

#ifdef DETECT
      altitudeMeasurementUpdate(&tempalt, false);
#else
      altitudeMeasurementUpdate(&tempalt);
#endif

      // Update the state & covariance packet during this repropagation so that the info is correct
      state_queue_.at(i).setState(x_);
      state_queue_.at(i).setCovariance(P_);
    }
  }
  else
  {
      //If the state queue is empty, just apply the camera update at the current time:
      ROS_INFO("NON-DELAYED UPDATE OCCURING!!! Not sufficient data or VO coming in very fast!");
      directVisionUpdate(vo_data,false,truth_data);
  }
}



//
// The prediction function for bringing the states up to the current timestep
//
void Estimator::prediction(int N, double dt, IMU_message &imu_data)
{
  //babysteps to break up the linearization into smaller pieces
  for(int i = 0; i < N; i++)
  {    
    //            [0 1 2 3  4  5  6  7 8 9 10 11 12 13 14   15  16  17  18  19 20 21]
    // state x_ = [f r d qx qy qz qw u v w bp bq br ax ay | cqx cqy cqz cqw cx cy cz]

    double p,q,r; //gyros with biases subtracted:
    p = saved_gyros_(0) - x_(10,0);
    q = saved_gyros_(1) - x_(11,0);
    r = saved_gyros_(2) - x_(12,0);

    double u,v,w,qw,qx,qy,qz,qmag;
    qw = x_(6,0);
    qx = x_(3,0);
    qy = x_(4,0);
    qz = x_(5,0);
    qmag = sqrt(qw*qw + qx*qx + qy*qy + qz*qz); //magnitude of the quaternion
    u = x_(7,0);
    v = x_(8,0);
    w = x_(9,0);

    //Nonlinear equations of motion for quaternions, taken mostly from UAV Book (Beard, McLain,"Small Unmanned Aircraft")
    //, in Appendix B.  Drag terms from our paper: "Improved Use of Accelerometers in Estimating Quadrotor Attitude and
    // Velocity", Control Systems Magazine, to appear.
    f_.setZero();
    f_(0,0) = u*(qx*qx + qw*qw - qy*qy - qz*qz) + v*2.0*(qx*qy - qz*qw) + w*2.0*(qx*qz + qy*qw);  //fdot
    f_(1,0) = u*2.0*(qx*qy + qz*qw) + v*(qy*qy + qw*qw - qx*qx -qz*qz) + w*2.0*(qy*qz - qx*qw);   //rdot
    f_(2,0) = u*2.0*(qx*qz - qy*qw) + v*2.0*(qy*qz + qx*qw) + w*(qz*qz + qw*qw - qx*qx - qy*qy);  //ddot
    f_(3,0) = 0.5*(p*qw + r*qy - q*qz); //qxdot
    f_(4,0) = 0.5*(q*qw - r*qx + p*qz); //qydot
    f_(5,0) = 0.5*(r*qw + q*qx - p*qy); //qzdot
    f_(6,0) = 0.5*(-p*qx - q*qy -r*qz);  //qwdot
    f_(7,0) = r*v - q*w + mk_consts_->g*2.0*(qx*qz - qy*qw) - mu_/mk_consts_->mass*u; //udot: corriolis + gravity + drag
    f_(8,0) = p*w - r*u + mk_consts_->g*2.0*(qy*qz + qx*qw) - mu_/mk_consts_->mass*v; //vdot: corriolis + gravity + drag
    f_(9,0) = q*u - p*v + mk_consts_->g*(qz*qz + qw*qw - qx*qx - qy*qy) + imu_data.linear_acceleration.z;
              ///(mk_consts_->kF/mk_consts_->mass)*6.0*omega_h_*omega_h_; //wdot


        //Doesn't appear that I need the gradient-descent terms in there:
//    f_(3,0) = 0.5*(p*qw + qx*mk_consts_->lambda*(1 - qmag*qmag) + r*qy - q*qz); //qxdot
//    f_(4,0) = 0.5*(q*qw - r*qx + qy*mk_consts_->lambda*(1 - qmag*qmag) + p*qz); //qydot
//    f_(5,0) = 0.5*(r*qw + q*qx - p*qy + qz*mk_consts_->lambda*(1 - qmag*qmag)); //qzdot
//    f_(6,0) = 0.5*(qw*mk_consts_->lambda*(1 - qmag*qmag) - p*qx - q*qy -r*qz);  //qwdot

    /// Predict the covariance A = d(deltaf)/d(deltax) using the linear model from the error dynamics:
    //                 [0  1  2  3   4   5   6  7  8  9   10  11  12  13    14   15   16   17  18  19 ]
    // Covariance P_ = [df dr dd dqx dqy dqz du dv dw dbp dbq dbr dax dzy | dcqx dcqy dcqz dcx dxy dcz]

    //Used in the DeltaP equations:
    Vector3d v_hat(u,v,w);
    Vector3d gravity(0,0,mk_consts_->g*1.0);
    Vector3d beta(x_(10,0),x_(11,0),x_(12,0));
    Vector3d omega(saved_gyros_(0),saved_gyros_(1),saved_gyros_(2));
    Quaterniond q_t(qw,qx,qy,qz);
    Matrix3d MU;
    MU << -mu_ ,0 ,0,0,-mu_,0,0,0,0;

    A_.setZero();
    //DeltaPdot
    A_.block<3,3>(0,3) = -q_t.conjugate().toRotationMatrix()*skew(v_hat); //d(DeltaP dot)/d(delta theta)
    A_.block<3,3>(0,6) = q_t.conjugate().toRotationMatrix(); //d(DeltaP dot)/d(delta V)

    //DeltaQdot
    A_.block<3,3>(3,3) = -skew(omega); //d(delta theta dot)/d(delta theta)
    A_.block<3,3>(3,9) = -1.0*Matrix3d::Identity(); //d(delta theta dot)/d(delta beta)

    //DeltaVdot
    A_.block<3,3>(6,3) = skew(q_t.toRotationMatrix()*gravity); //d(deltaV dot)/d(delta theta)
    A_.block<3,3>(6,6) = -skew(omega - beta) + MU/(mk_consts_->mass*1.0); //d(deltaV dot)/d(deltaV)
    A_.block<3,3>(6,9) = skew(v_hat); //d(deltaV dot)/d(delta beta)

    //
    /// Populate B, the Jacobian of deltaf(x,u) w.r.t the input (gyro p,q,r)    
    B_.setZero();
    //DeltaQdot
    B_.block<3,3>(3,0) = skew(saved_deltatheta_);
    //DeltaVdot
    B_.block<3,3>(6,0) = skew(saved_deltaV_);

    // Propagate the state and covariance:
    x_ = x_ + (dt/(double)N)*f_;
    P_ = P_ + (dt/(double)N)*(A_*P_ + P_*A_.transpose() + Q_ + 1.0*mk_consts_->gamma*B_*G_*B_.transpose());

    //May need this:
    normalizeQuaternion();
  }
}


//
// IMU measurement update:
//
void Estimator::imuMeasurementUpdate(IMU_message &imu_data)
{
  Vector2d h_i, residual; // the nonlinear measurement function for the IMU (accelermeter) measurement update
  Matrix<double,COVAR_LENGTH,1> delta_x; // the error state, computed in the measurement update
  Matrix<double,2,COVAR_LENGTH> C_i; // the Jacobian of the h_i_ w.r.t. the delta state (error state)
  Matrix<double,COVAR_LENGTH,2> L_i; // the Kalman gain for the IMU measurement update
  delta_x.setZero();
  C_i.setZero();
  L_i.setZero();

//  //Expanded measurement update equations for Corriolis terms:
//  h_i(0) = (imu_data.angular_velocity.z-x_(12,0))*x_(8,0) - (imu_data.angular_velocity.y-x_(11,0))*x_(9,0) - mu_/mk_consts_->mass*x_(7,0) + x_(13,0);
//  h_i(1) = (imu_data.angular_velocity.x-x_(10,0))*x_(9,0) - (imu_data.angular_velocity.z-x_(12,0))*x_(7,0) - mu_/mk_consts_->mass*x_(8,0) + x_(14,0);

//  //Populate C_i, the Jacobian of h_i w.r.t. delta_x
//  C_i(0,6) = -mu_/mk_consts_->mass; //for u
//  C_i(0,7) = -imu_data.angular_velocity.z+x_(12,0); //corriolis
//  C_i(0,8) = imu_data.angular_velocity.y-x_(11,0);//corriolis
//  C_i(0,10) = -x_(9,0);
//  C_i(0,11) = x_(8,0);
//  C_i(0,12) = 1; //for accel x bias

//  C_i(1,6) = imu_data.angular_velocity.z-x_(12,0);
//  C_i(1,7) = -mu_/mk_consts_->mass; //for v
//  C_i(1,8) = -imu_data.angular_velocity.x+x_(10,0);
//  C_i(1,9) = x_(9,0);
//  C_i(1,11) = -x_(7,0);
//  C_i(1,13) = 1; //for accel y bias

  h_i(0) = -mu_/mk_consts_->mass*x_(7,0) + x_(13,0);
  h_i(1) = -mu_/mk_consts_->mass*x_(8,0) + x_(14,0);

  //Populate C_i, the Jacobian of h_i w.r.t. delta_x
  C_i(0,6) = -mu_/mk_consts_->mass; //for u
  C_i(0,12) = 1; //for accel x bias
  C_i(1,7) = -mu_/mk_consts_->mass; //for v
  C_i(1,13) = 1; //for accel y bias

  residual(0) = imu_data.linear_acceleration.x - h_i(0);
  residual(1) = imu_data.linear_acceleration.y - h_i(1);

  // Calculate the Kalman gain
  Matrix2d S;
  S.setZero();
  S = (R_i_ + C_i * P_ * C_i.transpose());
  L_i = P_ * C_i.transpose() * S.inverse();

  //Calc the error state & update the covariance
  delta_x = L_i * residual;
  P_ = (MatrixXd::Identity(COVAR_LENGTH,COVAR_LENGTH) - L_i * C_i) * P_*
      (MatrixXd::Identity(COVAR_LENGTH,COVAR_LENGTH) - L_i * C_i).transpose() + L_i * R_i_ * L_i.transpose();

  //Update the state
  applyCorrection(delta_x);

  //use these gyros for the next prediction
  saved_gyros_(0) = imu_data.angular_velocity.x;
  saved_gyros_(1) = imu_data.angular_velocity.y;
  saved_gyros_(2) = imu_data.angular_velocity.z;
  saved_deltatheta_ = delta_x.block<3,1>(3,0);
  saved_deltaV_ = delta_x.block<3,1>(6,0);
}

#ifdef DETECT
//
// Altitude Measurement update (need to check to make sure that the altitude data isn't empty)
//
void Estimator:: altitudeMeasurementUpdate(sensor_msgs::Range *alt_data, bool normal_update)
{
  //Don't apply the update if there is no data or if the range is 0 (range set to zero in the saveData funtion if there
  // wasn't an altimeter measurement that timestep (may need to change that)
  if(alt_data == NULL || fabs(alt_data->range) <= 0.00000001)
  {
    return;
  }

  NavNode current(0);
  current = node_queue_.back(); //last node in queue is the one we are navigating w/ respect to
  double measurement_a; // the nonlinear measurement for the altitude measurement update
  Matrix<double,COVAR_LENGTH,1> delta_x; // the error state, computed in the measurement update
  Matrix<double,1,COVAR_LENGTH> C_a; // the Jacobian of the h_a_ w.r.t. the error state (the nonlinear measurement function is x_(2.0))
  Matrix<double,COVAR_LENGTH,1> L_a; // the Kalman gain for the altitude measurement update
  Vector3d node_position = current.getEstimatePosition();

  delta_x.setZero();
  C_a.setZero();
  L_a.setZero();

  measurement_a = alt_data->range;

#ifndef LASER

  // Use the altimeter to perform the measurement update (no fault detection)
  measurement_a = measurement_a - node_position(2);  //make the measurement relative
  C_a(0,2) = 1.d; //Jacobian of h_a w.r.t. x_

  double more_uncertainty = 1.0;
  if(node_position(2) >= -0.30)
  {
    //When we are below this height, the altimeter measurements can be pretty incorrect.
    more_uncertainty = 20.0;
  }

  double residual = measurement_a - x_(2,0);
  double R_a2 = R_a_*more_uncertainty*mk_consts_->alt_inflate;
  // Calculate the Kalman gain
  L_a = P_ * C_a.transpose() * (1.d / (R_a2 + P_(2, 2)));

  //Calc the error state & update the covariance
  delta_x = L_a * residual;
  P_ = (MatrixXd::Identity(COVAR_LENGTH,COVAR_LENGTH) - L_a * C_a) * P_*
      (MatrixXd::Identity(COVAR_LENGTH,COVAR_LENGTH) - L_a * C_a).transpose() + L_a * R_a2 * L_a.transpose();

  //Update the state
  applyCorrection(delta_x);

#else
  // Use the laser to perform the measurement update (fault detection included)
  window_mean_ = 0.0;
  window_covariance_ = 0.0;
  fault_flag_ = 0;
  num_window_faults_ = 0;
  faulty_data_yet_ = alt_data->radiation_type;
  num_laser_updates_++;
  normal_update_ = normal_update;

  double qx, qy, qz, qw;
  qx = x_(3);
  qy = x_(4);
  qz = x_(5);
  qw = x_(6);
  double phi_current = atan2(2*(qy*qz+qw*qx),2*(qw*qw+qz*qz)-1);
  double theta_current = asin(2*(qw*qy-qx*qz));

  d_apriori_ = x_(2) + node_position(2);

//  double predicted_measurement = (-(x_(2) + node_position(2))-mk_consts_->delta_x_las*sin(theta_current)-mk_consts_->delta_z_las*cos(theta_current))/(cos(theta_current)*cos(phi_current)) - mk_consts_->las_bias;
  double predicted_measurement = -d_apriori_ - mk_consts_->delta_z_las + mk_consts_->las_bias;
//  C_a(0,2) = -1/std::pow(-(4*std::pow(qw*qy - qx*qz, 2) - 1)/(std::pow(2*qw*qy - 2*qx*qz, 2)/std::pow(2*qw*qw + 2*qz*qz - 1, 2) + 1), 1.0/2);

//  C_a(0,3) = (2*mk_consts_->delta_x_las*qz - (4*mk_consts_->delta_z_las*qz*(qw*qy - qx*qz))/std::pow(1 - 4*std::pow(qw*qy - qx*qz, 2), 1.0/2))/std::pow(-(4*std::pow(qw*qy - qx*qz, 2) - 1)/(std::pow(2*qw*qy - 2*qx*qz, 2)/std::pow(2*qw*qw + 2*qz*qz - 1, 2) + 1), 1.0/2) + (((8*qz*(qw*qy - qx*qz))/(std::pow(2*qw*qy - 2*qx*qz, 2)/std::pow(2*qw*qw + 2*qz*qz - 1, 2) + 1) - (4*qz*(4*std::pow(qw*qy - qx*qz, 2) - 1)*(2*qw*qy - 2*qx*qz))/(std::pow(std::pow(2*qw*qy - 2*qx*qz, 2)/std::pow(2*qw*qw + 2*qz*qz - 1, 2) + 1, 2)*std::pow(2*qw*qw + 2*qz*qz - 1, 2)))*(x_(2) + node_position(2) + mk_consts_->delta_z_las*std::pow(1 - 4*std::pow(qw*qy - qx*qz, 2), 1.0/2) + 2*mk_consts_->delta_x_las*(qw*qy - qx*qz)))/(2*std::pow(-(4*std::pow(qw*qy - qx*qz, 2) - 1)/(std::pow(2*qw*qy - 2*qx*qz, 2)/std::pow(2*qw*qw + 2*qz*qz - 1, 2) + 1), 3.0/2));

//  C_a(0,4) = -(2*mk_consts_->delta_x_las*qw - (4*mk_consts_->delta_z_las*qw*(qw*qy - qx*qz))/std::pow(1 - 4*std::pow(qw*qy - qx*qz, 2), 1.0/2))/std::pow(-(4*std::pow(qw*qy - qx*qz, 2) - 1)/(std::pow(2*qw*qy - 2*qx*qz, 2)/std::pow(2*qw*qw + 2*qz*qz - 1, 2) + 1), 1.0/2) - (((8*qw*(qw*qy - qx*qz))/(std::pow(2*qw*qy - 2*qx*qz, 2)/std::pow(2*qw*qw + 2*qz*qz - 1, 2) + 1) - (4*qw*(4*std::pow(qw*qy - qx*qz, 2) - 1)*(2*qw*qy - 2*qx*qz))/(std::pow(std::pow(2*qw*qy - 2*qx*qz, 2)/std::pow(2*qw*qw + 2*qz*qz - 1, 2) + 1, 2)*std::pow(2*qw*qw + 2*qz*qz - 1, 2)))*(x_(2) + node_position(2) + mk_consts_->delta_z_las*std::pow(1 - 4*std::pow(qw*qy - qx*qz, 2), 1.0/2) + 2*mk_consts_->delta_x_las*(qw*qy - qx*qz)))/(2*std::pow(-(4*std::pow(qw*qy - qx*qz, 2) - 1)/(std::pow(2*qw*qy - 2*qx*qz, 2)/std::pow(2*qw*qw + 2*qz*qz - 1, 2) + 1), 3.0/2));

//  C_a(0,5) = (2*mk_consts_->delta_x_las*qx - (4*mk_consts_->delta_z_las*qx*(qw*qy - qx*qz))/std::pow(1 - 4*std::pow(qw*qy - qx*qz, 2), 1.0/2))/std::pow(-(4*std::pow(qw*qy - qx*qz, 2) - 1)/(std::pow(2*qw*qy - 2*qx*qz, 2)/std::pow(2*qw*qw + 2*qz*qz - 1, 2) + 1), 1.0/2) + (((8*qx*(qw*qy - qx*qz))/(std::pow(2*qw*qy - 2*qx*qz, 2)/std::pow(2*qw*qw + 2*qz*qz - 1, 2) + 1) - ((4*std::pow(qw*qy - qx*qz, 2) - 1)*((8*qz*std::pow(2*qw*qy - 2*qx*qz, 2))/std::pow(2*qw*qw + 2*qz*qz - 1, 3) + (4*qx*(2*qw*qy - 2*qx*qz))/std::pow(2*qw*qw + 2*qz*qz - 1, 2)))/std::pow(std::pow(2*qw*qy - 2*qx*qz, 2)/std::pow(2*qw*qw + 2*qz*qz - 1, 2) + 1, 2))*(x_(2) + node_position(2) + mk_consts_->delta_z_las*std::pow(1 - 4*std::pow(qw*qy - qx*qz, 2), 1.0/2) + 2*mk_consts_->delta_x_las*(qw*qy - qx*qz)))/(2*std::pow(-(4*std::pow(qw*qy - qx*qz, 2) - 1)/(std::pow(2*qw*qy - 2*qx*qz, 2)/std::pow(2*qw*qw + 2*qz*qz - 1, 2) + 1), 3.0/2));

//  C_a(0,6) = -(2*mk_consts_->delta_x_las*qy - (4*mk_consts_->delta_z_las*qy*(qw*qy - qx*qz))/std::pow(1 - 4*std::pow(qw*qy - qx*qz, 2), 1.0/2))/std::pow(-(4*std::pow(qw*qy - qx*qz, 2) - 1)/(std::pow(2*qw*qy - 2*qx*qz, 2)/std::pow(2*qw*qw + 2*qz*qz - 1, 2) + 1), 1.0/2) - (((8*qy*(qw*qy - qx*qz))/(std::pow(2*qw*qy - 2*qx*qz, 2)/std::pow(2*qw*qw + 2*qz*qz - 1, 2) + 1) + ((4*std::pow(qw*qy - qx*qz, 2) - 1)*((8*qw*std::pow(2*qw*qy - 2*qx*qz, 2))/std::pow(2*qw*qw + 2*qz*qz - 1, 3) - (4*qy*(2*qw*qy - 2*qx*qz))/std::pow(2*qw*qw + 2*qz*qz - 1, 2)))/std::pow(std::pow(2*qw*qy - 2*qx*qz, 2)/std::pow(2*qw*qw + 2*qz*qz - 1, 2) + 1, 2))*(x_(2) + node_position(2) + mk_consts_->delta_z_las*std::pow(1 - 4*std::pow(qw*qy - qx*qz, 2), 1.0/2) + 2*mk_consts_->delta_x_las*(qw*qy - qx*qz)))/(2*std::pow(-(4*std::pow(qw*qy - qx*qz, 2) - 1)/(std::pow(2*qw*qy - 2*qx*qz, 2)/std::pow(2*qw*qw + 2*qz*qz - 1, 2) + 1), 3.0/2));
  C_a(0,2) = -1.d;

  residual_a_ = measurement_a - predicted_measurement;

#ifdef DETECT

  if(normal_update)
  {
  //////////////////////////////////////
  //////////////////////////////////////
  // Perform the fault detection stuff
  //////////////////////////////////////
  //////////////////////////////////////

  double residual_covariance = C_a*P_*C_a.transpose() + R_a_;
  residual_normalized_ = std::pow(1/residual_covariance, 1.0/2)*residual_a_;

  //////////////////////////////////////
  // OUTLIER REJECTION
  //////////////////////////////////////

  if (measurement_a >= mk_consts_->failed_return_distance || measurement_a <= alt_data->min_range)
  {
      //Then it is an outlier
      fault_flag_ += 4;
  }


  //////////////////////////////////////
  // TEST OF MEAN
  //////////////////////////////////////

  //add the current normalized residual to the deque of normalized residuals for windowed analysis
  residual_queue_.push_back(residual_normalized_);

  //remove elements which are outside the current window of data
  if (residual_queue_.size() > mk_consts_->window_size)
  {
      residual_queue_.pop_front();
  }

  std::deque<double>::iterator it;
  for (it = residual_queue_.begin(); it != residual_queue_.end(); it++)
  {
      window_mean_ += *it;
  }
  window_mean_ = window_mean_/residual_queue_.size();

  //Determine the mean statistic
  mean_statistic_ = residual_queue_.size()*window_mean_*window_mean_; //This works for the scalar case, but would need to be chaged for the vector case

  //Determine the threshold
  boost::math::chi_squared stat_distro_mean(1);//The dofs would increase for the vector case

  threshold_mean_ = mk_consts_->threshold_mean_inflate*quantile(stat_distro_mean, 1-mk_consts_->prob_false_allow);

  //Compare to threshold
  if (mean_statistic_ >= threshold_mean_)
  {
      fault_flag_ += 1;
  }


  //////////////////////////////////////
  // TEST OF COVARIANCE
  //////////////////////////////////////

  //Compute the Covariance
  for(it = residual_queue_.begin(); it !=residual_queue_.end(); it++)
  {
      window_covariance_ += std::pow(*it - window_mean_, 2);
  }
  if(residual_queue_.size() > 1)
  {
    window_covariance_ = window_covariance_/(residual_queue_.size()-1);
  }

  //Determine the covariance statistic
  covariance_statistic_ = (residual_queue_.size()-1)*window_covariance_; //really (N-1)*trace(S)
  //Determine the threshold
  if (residual_queue_.size() > 1)
  {
      boost::math::chi_squared covariance_distro(residual_queue_.size()-1);//if vector, need to multiply this dof with dim of vector
      threshold_covariance_ = mk_consts_->threshold_covariance_inflate*quantile(covariance_distro, 1-mk_consts_->prob_false_allow);
  }
  else
  {
      threshold_covariance_ = 0.000000000000001;
  }


  //Compare to threshold
  if (covariance_statistic_ >= threshold_covariance_)
  {
      fault_flag_ += 2;
  }

  //////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////
  // Determine if it is a sensor failure rather than noise
  //////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////
  flag_queue_.push_back(fault_flag_);

  if (flag_queue_.size() > mk_consts_->window_size)
  {
    flag_queue_.pop_front();
  }

  std::deque<int>::iterator iter;
  for(iter = flag_queue_.begin(); iter != flag_queue_.end(); iter++)
  {
    if(*iter != 0)
      num_window_faults_ += 1.d;
  }

  if(flag_queue_.size() == mk_consts_->window_size)
  {
    if(num_window_faults_>=6.0)
      sensor_failure_ = true;
    else
      sensor_failure_ = false;
  }
  else
  {
    sensor_failure_ = false;
  }

  }
#endif
  double R_a2 = R_a_*mk_consts_->alt_inflate;
  // Calculate the Kalman gain
  L_a = P_ * C_a.transpose() * (1.d / (R_a2 + P_(2, 2)));

  //Calc the error state & update the covariance
  delta_x = L_a * residual_a_;
  P_ = (MatrixXd::Identity(COVAR_LENGTH,COVAR_LENGTH) - L_a * C_a) * P_*
      (MatrixXd::Identity(COVAR_LENGTH,COVAR_LENGTH) - L_a * C_a).transpose() + L_a * R_a2 * L_a.transpose();

  //Update the state
  applyCorrection(delta_x);

  d_aposteriori_ = x_(2) + node_position(2);
#endif
}
#else
//
// Altitude Measurement update (need to check to make sure that the altitude data isn't empty)
//
void Estimator:: altitudeMeasurementUpdate(sensor_msgs::Range *alt_data)
{
  //Don't apply the update if there is no data or if the range is 0 (range set to zero in the saveData funtion if there
  // wasn't an altimeter measurement that timestep (may need to change that)
  if(alt_data == NULL || fabs(alt_data->range) <= 0.00000001)
  {
    return;
  }

  NavNode current(0);
  current = node_queue_.back(); //last node in queue is the one we are navigating w/ respect to
  double measurement_a; // the nonlinear measurement for the altitude measurement update
  Matrix<double,COVAR_LENGTH,1> delta_x; // the error state, computed in the measurement update
  Matrix<double,1,COVAR_LENGTH> C_a; // the Jacobian of the h_a_ w.r.t. the error state (the nonlinear measurement function is x_(2.0))
  Matrix<double,COVAR_LENGTH,1> L_a; // the Kalman gain for the altitude measurement update
  Vector3d node_position = current.getEstimatePosition();

  delta_x.setZero();
  C_a.setZero();
  L_a.setZero();

  measurement_a = alt_data->range;

#ifndef LASER

  // Use the altimeter to perform the measurement update (no fault detection)
  measurement_a = measurement_a - node_position(2);  //make the measurement relative
  C_a(0,2) = 1.d; //Jacobian of h_a w.r.t. x_

  double more_uncertainty = 1.0;
  if(node_position(2) >= -0.30)
  {
    //When we are below this height, the altimeter measurements can be pretty incorrect.
    more_uncertainty = 20.0;
  }

  double residual = measurement_a - x_(2,0);
  double R_a2 = R_a_*more_uncertainty*mk_consts_->alt_inflate;
  // Calculate the Kalman gain
  L_a = P_ * C_a.transpose() * (1.d / (R_a2 + P_(2, 2)));

  //Calc the error state & update the covariance
  delta_x = L_a * residual;
  P_ = (MatrixXd::Identity(COVAR_LENGTH,COVAR_LENGTH) - L_a * C_a) * P_*
      (MatrixXd::Identity(COVAR_LENGTH,COVAR_LENGTH) - L_a * C_a).transpose() + L_a * R_a2 * L_a.transpose();

  //Update the state
  applyCorrection(delta_x);

#else
  // Use the laser to perform the measurement update (fault detection included)
  window_mean_ = 0.0;
  window_covariance_ = 0.0;
  fault_flag_ = 0;
  num_window_faults_ = 0.d;
  faulty_data_yet_ = alt_data->radiation_type;
  num_laser_updates_++;

  double qx, qy, qz, qw;
  qx = x_(3);
  qy = x_(4);
  qz = x_(5);
  qw = x_(6);
  double phi_current = atan2(2*(qy*qz+qw*qx),2*(qw*qw+qz*qz)-1);
  double theta_current = asin(2*(qw*qy-qx*qz));

  d_apriori_ = x_(2) + node_position(2);

//  double predicted_measurement = (-(x_(2) + node_position(2))-mk_consts_->delta_x_las*sin(theta_current)-mk_consts_->delta_z_las*cos(theta_current))/(cos(theta_current)*cos(phi_current)) - mk_consts_->las_bias;
  double predicted_measurement = -d_apriori_ - mk_consts_->delta_z_las + mk_consts_->las_bias;
  C_a(0,2) = -1.d;

  residual_a_ = measurement_a - predicted_measurement;

  double R_a2 = R_a_*mk_consts_->alt_inflate;
  // Calculate the Kalman gain
  L_a = P_ * C_a.transpose() * (1.d / (R_a2 + P_(2, 2)));

  //Calc the error state & update the covariance
  delta_x = L_a * residual_a_;
  P_ = (MatrixXd::Identity(COVAR_LENGTH,COVAR_LENGTH) - L_a * C_a) * P_*
      (MatrixXd::Identity(COVAR_LENGTH,COVAR_LENGTH) - L_a * C_a).transpose() + L_a * R_a2 * L_a.transpose();

  //Update the state
  applyCorrection(delta_x);

  d_aposteriori_ = x_(2) + node_position(2);
#endif
}
#endif



//
// Save data in the queues
//
void Estimator::saveData(IMU_message &imu_data, sensor_msgs::Range *alt_data)
{
  sensor_msgs::Range alt_packet;
  if(alt_data == NULL)
  {
    alt_packet.range = 0.d;
  }
  else
  {
    alt_packet = *alt_data;
  }

  i_queue_.push_back(imu_data);
  a_queue_.push_back(alt_packet);
  StatePacket temp(x_,P_,imu_data.header.stamp);
  state_queue_.push_back(temp);
}


//
// Check to see if a new node should be requested
/// \todo Implement this function if we feel it's needed
//
bool Estimator::checkNewNode()
{
  //
  //If I do implement this function, it needs to be tied to a service call to the visual odometry.
  //


  //Here is the code from the windows version:
//  // The down direction is a global one, need to make it relative:
//  curr_node = node_queue.ElementAt<NavNode>(node_queue.Count - 1);

//  //double d_measure = x[2, 0] - curr_node.Estimate_Pose[2]; //the state - node down position = relative measurement

//  //This function checks to see if a new node is warranted
//  double length_sq = x[0, 0] * x[0, 0] + x[1, 0] * x[1, 0];
//  double height_sq = x[2, 0] * x[2, 0]; //for relative down     //d_measure * d_measure; //for global down
//  bool new_node;

//  if (!newNodeRequested && (length_sq > Hex_Constants.max_length || x[3, 0] > Hex_Constants.max_yaw || height_sq > Hex_Constants.max_height))
//  {
//      new_node = true;
//      newNodeRequested = true;
//      //Utilities.Log.SimpleWriteLine("New Node: (x&y, z, psi) " + length_sq + " , " + height_sq + " , " + x[3,0] );
//  }
//  else
//      new_node = false;

//  return new_node;
  return false;
}


//
// Compute the current global pose
//
geometry_msgs::TransformStamped Estimator::computeGlobalPoseEstimate(ros::Time stamp, std::string &global_name, std::string &body_name)
{
  //            [0 1 2 3  4  5  6  7 8 9 10 11 12 13 14]
  // state x_ = [f r d qx qy qz qw u v w bp bq br ax ay]

  //turns [f,s,d] into [n,e,d]
  Vector3d current_global = global_node_position_ + global_R_yaw_*x_.topRows(3);
  double yaw = global_yaw_ + atan2(2*(x_(6,0)*x_(5,0) + x_(3,0)*x_(4,0)),
                                   (x_(6,0)*x_(6,0) + x_(3,0)*x_(3,0) - x_(4,0)*x_(4,0) - x_(5,0)*x_(5,0)));
  double pitch, roll;
  pitch = asin(2.0*(x_(6,0)*x_(4,0) - x_(3,0)*x_(5,0)));
  roll = atan2(2.0*(x_(6,0)*x_(3,0) + x_(4,0)*x_(5,0)),
               (x_(6,0)*x_(6,0) + x_(5,0)*x_(5,0) - x_(3,0)*x_(3,0) - x_(4,0)*x_(4,0)));

//  //wrap the yaw between pi and -pi!
//  while (yaw > mk_consts_->pi)
//  {
//    yaw -= 2.0 * mk_consts_->pi;
//  }
//  while (yaw < -1.0*mk_consts_->pi)
//  {
//    yaw += 2.0 * mk_consts_->pi;
//  }

  Quaterniond q;
  q.w() = cos(yaw/2.0)*cos(pitch/2.0)*cos(roll/2.0) + sin(yaw/2.0)*sin(pitch/2.0)*sin(roll/2.0);
  q.x() = cos(yaw/2.0)*cos(pitch/2.0)*sin(roll/2.0) - sin(yaw/2.0)*sin(pitch/2.0)*cos(roll/2.0);
  q.y() = cos(yaw/2.0)*sin(pitch/2.0)*cos(roll/2.0) + sin(yaw/2.0)*cos(pitch/2.0)*sin(roll/2.0);
  q.z() = sin(yaw/2.0)*cos(pitch/2.0)*cos(roll/2.0) - cos(yaw/2.0)*sin(pitch/2.0)*sin(roll/2.0);
  //q.normalize();

  geometry_msgs::TransformStamped global_pose;
  global_pose.header.stamp = stamp;
  global_pose.header.frame_id = global_name; //"global_frame"; //frame the transformation is expressed in
  global_pose.child_frame_id = body_name; //"body-fixed_frame";
  global_pose.transform.translation.x = current_global(0);
  global_pose.transform.translation.y = -current_global(1);
  global_pose.transform.translation.z = -current_global(2);
  global_pose.transform.rotation.x = q.x();
  global_pose.transform.rotation.y = -q.y();
  global_pose.transform.rotation.z = -q.z();
  global_pose.transform.rotation.w = q.w();

  return global_pose;
}


//
// Write the log:
//
void Estimator::writeToLog(IMU_message &imu_data, geometry_msgs::TransformStamped &global_pose,
                           sensor_msgs::Range *alt_data, VO_message *vo_data, TRUTH_message *truth_data)
{
  NavNode current(0);
  current = node_queue_.back();
  Vector3d node = current.getEstimatePosition();
  double d,w;
  d = 0.d;
  w = 0.d;
  int num = 0;
  int new_node = 0;
  Vector3d true_pos,true_rel;
  Quaterniond true_angle;
  true_rel.setZero();
  true_pos.setZero();
  true_angle.setIdentity();

  if(alt_data != NULL)
  {
    d = alt_data->range;
    w = alt_data->min_range; //not necessarily right yet...
  }
  if(vo_data != NULL)
  {
    num = (int)vo_data->ImageNumber();
    if (vo_data->NewReference())
      new_node = 1;
    else
      new_node = 0;
  }
  if(truth_data != NULL)
  {
    true_pos(0) = truth_data->transform.translation.x;
    true_pos(1) = truth_data->transform.translation.y;
    true_pos(2) = truth_data->transform.translation.z;

    true_angle.x() = truth_data->transform.rotation.x;
    true_angle.y() = truth_data->transform.rotation.y;
    true_angle.z() = truth_data->transform.rotation.z;
    true_angle.w() = truth_data->transform.rotation.w;
    //Calc the true relative information:
    if(current.TruthSet())
    {
      Matrix3d psi;
      psi << cos(current.getTrueYaw()),sin(current.getTrueYaw()),0,-sin(current.getTrueYaw()),cos(current.getTrueYaw()),
          0,0,0,1;
      true_rel = psi*(true_pos - current.getTruePosition());//relative truth, in node frame
    }
  }

  log_file_.precision(20);


  //            [0 1 2 3  4  5  6  7 8 9 10 11 12 13 14   15  16  17  18  19 20 21]
  // state x_ = [f r d qx qy qz qw u v w bp bq br ax ay | cqx cqy cqz cqw cx cy cz]
#ifndef LASER
  if(STATE_LENGTH > 16)
  {
    log_file_ << imu_data.header.stamp.toSec() <<" "<< global_pose.transform.translation.x <<" "<<
            global_pose.transform.translation.y <<" "<< global_pose.transform.translation.z <<" "<<
            global_pose.transform.rotation.x <<" "<< global_pose.transform.rotation.y <<" "<<
            global_pose.transform.rotation.z <<" "<< global_pose.transform.rotation.w <<" "<< x_(0,0) <<" "<< x_(1,0)
            <<" "<< x_(2,0) <<" "<< x_(3,0) <<" "<< x_(4,0) <<" "<< x_(5,0) <<" "<< x_(6,0) <<" "<< x_(7,0) <<" "<< x_(8,0)
            <<" "<< x_(9,0) <<" "<< x_(10,0) <<" "<< x_(11,0) <<" "<< x_(12,0) <<" "<< x_(13,0) <<" "<< x_(14,0) <<" "<<
            x_(15,0) <<" "<< x_(16,0) <<" "<< x_(17,0) <<" "<< x_(18,0) <<" "<< x_(19,0) <<" "<< x_(20,0) <<" "<<
            x_(21,0) <<" "<<  node(0) <<" "<< node(1) <<" "<< node(2) <<" "<< current.getPsiGlobal() <<" "<<
            imu_data.linear_acceleration.x <<" "<< imu_data.linear_acceleration.y <<" "<< imu_data.linear_acceleration.z
            <<" "<< imu_data.angular_velocity.x <<" "<< imu_data.angular_velocity.y
            <<" "<< imu_data.angular_velocity.z <<" "<< d <<" "<< w <<" "<< num <<" "<< true_rel(0) <<" "<< true_rel(1)
            <<" "<< true_rel(2) <<" "<< true_angle.x() <<" "<< true_angle.y() <<" "<< true_angle.z() <<" "<< true_angle.w()
           <<" "<< new_node <<" "<< P_.trace() << std::endl;
  }
  else
  {
  log_file_ << imu_data.header.stamp.toSec() <<" "<< global_pose.transform.translation.x <<" "<<
          global_pose.transform.translation.y <<" "<< global_pose.transform.translation.z <<" "<<
          global_pose.transform.rotation.x <<" "<< global_pose.transform.rotation.y <<" "<<
          global_pose.transform.rotation.z <<" "<< global_pose.transform.rotation.w <<" "<< x_(0,0) <<" "<< x_(1,0)
          <<" "<< x_(2,0) <<" "<< x_(3,0) <<" "<< x_(4,0) <<" "<< x_(5,0) <<" "<< x_(6,0) <<" "<< x_(7,0) <<" "<< x_(8,0)
          <<" "<< x_(9,0) <<" "<< x_(10,0) <<" "<< x_(11,0) <<" "<< x_(12,0) <<" "<< x_(13,0) <<" "<< x_(14,0) <<" "<<
          node(0) <<" "<< node(1) <<" "<< node(2) <<" "<< current.getPsiGlobal() <<" "<<
            imu_data.linear_acceleration.x <<" "<< imu_data.linear_acceleration.y <<" "<< imu_data.linear_acceleration.z
            <<" "<< imu_data.angular_velocity.x <<" "<< imu_data.angular_velocity.y
            <<" "<< imu_data.angular_velocity.z <<" "<< d <<" "<< w <<" "<< num <<" "<< true_rel(0) <<" "<< true_rel(1)
          <<" "<< true_rel(2) <<" "<< true_angle.x() <<" "<< true_angle.y() <<" "<< true_angle.z() <<" "<< true_angle.w()
          <<" "<< new_node <<" "<< P_.trace() << std::endl;
  }
#else
  // Output the laser version of the log file with the fault detection parameters included (perhaps use other if include capability to do
  //laser without fault detection.  Then the if statement conditions would need to reflect that condition.
  if(STATE_LENGTH > 16)
  {
    log_file_ << imu_data.header.stamp.toSec() <<" "<< global_pose.transform.translation.x <<" "<<
            global_pose.transform.translation.y <<" "<< global_pose.transform.translation.z <<" "<<
            global_pose.transform.rotation.x <<" "<< global_pose.transform.rotation.y <<" "<<
            global_pose.transform.rotation.z <<" "<< global_pose.transform.rotation.w <<" "<< x_(0,0) <<" "<< x_(1,0)
            <<" "<< x_(2,0) <<" "<< x_(3,0) <<" "<< x_(4,0) <<" "<< x_(5,0) <<" "<< x_(6,0) <<" "<< x_(7,0) <<" "<< x_(8,0)
            <<" "<< x_(9,0) <<" "<< x_(10,0) <<" "<< x_(11,0) <<" "<< x_(12,0) <<" "<< x_(13,0) <<" "<< x_(14,0) <<" "<<
            x_(15,0) <<" "<< x_(16,0) <<" "<< x_(17,0) <<" "<< x_(18,0) <<" "<< x_(19,0) <<" "<< x_(20,0) <<" "<<
            x_(21,0) <<" "<<  node(0) <<" "<< node(1) <<" "<< node(2) <<" "<< current.getPsiGlobal() <<" "<<
            imu_data.linear_acceleration.x <<" "<< imu_data.linear_acceleration.y <<" "<< imu_data.linear_acceleration.z
            <<" "<< imu_data.angular_velocity.x <<" "<< imu_data.angular_velocity.y
            <<" "<< imu_data.angular_velocity.z <<" "<< d <<" "<< w <<" "<< num <<" "<< true_rel(0) <<" "<< true_rel(1)
            <<" "<< true_rel(2) <<" "<< true_angle.x() <<" "<< true_angle.y() <<" "<< true_angle.z() <<" "<< true_angle.w()
           <<" "<< new_node <<" "<< P_.trace() <<" "<< fault_flag_ <<" "<< residual_a_ <<" "<< residual_normalized_
          <<" "<< d_apriori_ <<" "<< d_aposteriori_ <<" "<< window_mean_ <<" "<< mean_statistic_ <<" "<< window_covariance_ <<" "
          << covariance_statistic_ <<" "<< threshold_outlier_ <<" "<< threshold_mean_ <<" "<< threshold_covariance_ <<" "<< sensor_failure_
          <<" "<< faulty_data_yet_ <<" "<< num_laser_updates_ <<" "<< normal_update_ << std::endl;
  }
  else
  {
  log_file_ << imu_data.header.stamp.toSec() <<" "<< global_pose.transform.translation.x <<" "<<
          global_pose.transform.translation.y <<" "<< global_pose.transform.translation.z <<" "<<
          global_pose.transform.rotation.x <<" "<< global_pose.transform.rotation.y <<" "<<
          global_pose.transform.rotation.z <<" "<< global_pose.transform.rotation.w <<" "<< x_(0,0) <<" "<< x_(1,0)
          <<" "<< x_(2,0) <<" "<< x_(3,0) <<" "<< x_(4,0) <<" "<< x_(5,0) <<" "<< x_(6,0) <<" "<< x_(7,0) <<" "<< x_(8,0)
          <<" "<< x_(9,0) <<" "<< x_(10,0) <<" "<< x_(11,0) <<" "<< x_(12,0) <<" "<< x_(13,0) <<" "<< x_(14,0) <<" "<<
          node(0) <<" "<< node(1) <<" "<< node(2) <<" "<< current.getPsiGlobal() <<" "<<
            imu_data.linear_acceleration.x <<" "<< imu_data.linear_acceleration.y <<" "<< imu_data.linear_acceleration.z
            <<" "<< imu_data.angular_velocity.x <<" "<< imu_data.angular_velocity.y
            <<" "<< imu_data.angular_velocity.z <<" "<< d <<" "<< w <<" "<< num <<" "<< true_rel(0) <<" "<< true_rel(1)
          <<" "<< true_rel(2) <<" "<< true_angle.x() <<" "<< true_angle.y() <<" "<< true_angle.z() <<" "<< true_angle.w()
          <<" "<< new_node <<" "<< P_.trace() <<" "<< fault_flag_ <<" "<< residual_a_ <<" "<< residual_normalized_
         <<" "<< d_apriori_ <<" "<< d_aposteriori_ <<" "<< window_mean_ <<" "<< mean_statistic_ <<" "<< window_covariance_ <<" "
        << covariance_statistic_<<" "<< threshold_outlier_ <<" "<< threshold_mean_ <<" "<< threshold_covariance_ <<" "<< sensor_failure_
         <<" "<< faulty_data_yet_ <<" "<< num_laser_updates_ <<" "<< normal_update_ << std::endl;
  }
#endif
  log_file_.flush();

  //DEBUG:
  //std::cout << P_.trace() << std::endl;
  ROS_INFO_THROTTLE_NAMED(0.5,"rel_MEKF","Current Trace of Covariance P: %7.5f",P_.trace());
}



//
// Take truth data and add noise, express it in a relative sense, and make it a VO data packet
//
VO_message Estimator::makeDataRelative(TRUTH_message &truth_data)
{
  //Not currently a high priority.  Would rather get the visual odometry working with the filter...
  //This code does exsist already in the windows version.  It is posted below in comments:

  //Current node is the last in the queue:
//  int count = node_queue.Count;
//  curr_node = node_queue.ElementAt<NavNode>(count - 1);
//  curr_node.True_Pose.CopyTo(node_pose, 0);
//  double phi_i = curr_node.Phi_i;
//  double theta_i = curr_node.Theta_i;

//  Trans_Cov = new Matrix(3, 3); //instantiate this

//  //Newer implementation:
//  //  Based on inverting these two equations to find R_cr_c and Delta_c:
//  //Delta_ni_2 = R_b_ni * T_b - R_b_ni * R_c_b * Matrix.Transpose(R_cr_c) * Delta_c - R_b_ni * R_c_b * Matrix.Transpose(R_cr_c) * Matrix.Transpose(R_c_b) * T_b;    //eqn 1
//  //R_nj_cs = R_c_b * R_cr_c * Matrix.Transpose(R_c_b) * Matrix.Transpose(R_b_I_node);  //eqn 1
//  //Necessary Rotations
//  Matrix R_I_cs = VehicleToBodyRotationMatrix(cortexStates[3], cortexStates[4], cortexStates[5]); //The rotation from inertial to current state (body frame)
//  Matrix R_I_bj = VehicleToBodyRotationMatrix(node_pose[3], node_pose[4], node_pose[5]); //The rotation from inertial to the reference body frame at node j
//  Matrix R_br_nj = BodyToNodeRotation(phi_i, theta_i); //The rotation from the current node frame to the reference body frame
//  Matrix R_I_nj = R_br_nj * R_I_bj; //this matrix rotates from the inertial frame to the current body frame
//  //Create the translation from the node frame to the current body, in the body frame
//  Delta_ni = new Matrix(3, 1);
//  Delta_ni[0, 0] = cortexStates[0] - node_pose[0];
//  Delta_ni[1, 0] = cortexStates[1] - node_pose[1];
//  Delta_ni[2, 0] = cortexStates[2] - node_pose[2];
//  Delta_ni = R_I_nj * Delta_ni; //Rotate the delta into the current node frame, from the inertial frame.

//  Matrix R_nj_cs = R_I_cs * Matrix.Transpose(R_I_nj); //the rotation from the current node to the current state (body frame)

//  //Find the Rotation from reference camera to current camera:
//  R_cr_c = Matrix.Transpose(R_c_b) * R_nj_cs * R_br_nj * R_c_b;
//  //Find the translation from the current camera to the reference camera:
//  //Depends on whether noise is wanted:
//  if (Hex_Constants.turnOffCameraNoise)
//  {
//    //Pull out the angles
//    relative_angles[0] = Math.Atan2(R_cr_c[1, 2], R_cr_c[2, 2]);
//    relative_angles[1] = Math.Asin(-R_cr_c[0, 2]);
//    relative_angles[2] = Math.Atan2(R_cr_c[0, 1], R_cr_c[0, 0]);

//    //Calculate the Translation:
//    Delta_c = R_cr_c * Matrix.Transpose(R_c_b) * Matrix.Transpose(R_br_nj) * (R_br_nj * T_b - Delta_ni - R_br_nj * R_c_b *
//              Matrix.Transpose(R_cr_c) * Matrix.Transpose(R_c_b) * T_b);
//  }
//  else
//  {
//    //Pull out the angles and add noise (eqn 15)
//    relative_angles[0] = Math.Atan2(R_cr_c[1, 2], R_cr_c[2, 2]) + randn.NextDouble() * Hex_Constants.cam_psi_std;
//    relative_angles[1] = Math.Asin(-R_cr_c[0, 2]) + randn.NextDouble() * Hex_Constants.cam_psi_std;
//    relative_angles[2] = Math.Atan2(R_cr_c[0, 1], R_cr_c[0, 0]) + randn.NextDouble() * Hex_Constants.cam_psi_std;

//    //Calculate the Translation:
//    Delta_c = R_cr_c * Matrix.Transpose(R_c_b) * Matrix.Transpose(R_br_nj) * (R_br_nj * T_b - Delta_ni - R_br_nj *
//              R_c_b * Matrix.Transpose(R_cr_c) * Matrix.Transpose(R_c_b) * T_b);
//    Delta_c[0, 0] = Delta_c[0, 0] + randn.NextDouble() * Hex_Constants.cam_f_std;
//    Delta_c[1, 0] = Delta_c[1, 0] + randn.NextDouble() * Hex_Constants.cam_s_std;
//    Delta_c[2, 0] = Delta_c[2, 0] + randn.NextDouble() * Hex_Constants.cam_d_std;
//  }
  VO_message temp;
  return temp;
}

#ifdef LASER
#ifdef DETECT
//
// Pack up laser condition into a message
//
Status_message Estimator::packageLaserStatus(ros::Time timestamp)
{
  Status_message temp_status;

  //Set the header information
  temp_status.header.frame_id = "/mocap";
  temp_status.header.stamp = timestamp;

  temp_status.type = Status_message::TEXT_VIEW_FACING;
  temp_status.action = Status_message::ADD;

  //Declare the namespace and id of this marker (will always be the same so we overwrite)
  temp_status.ns = "las_stat";
  temp_status.id = 0;

  //Give the marker a color
  temp_status.color.r = 0.0f;
  temp_status.color.g = 0.0f;
  temp_status.color.b = 0.0f;
  temp_status.color.a = 1.0;

  //Set the marker position (w/ respect to the frame above)
  temp_status.pose.position.x = 0;
  temp_status.pose.position.y = 0;
  temp_status.pose.position.z = 0;
  temp_status.pose.orientation.x = 0.0;
  temp_status.pose.orientation.y = 0.0;
  temp_status.pose.orientation.z = 0.0;
  temp_status.pose.orientation.w = 1.0;

  //Declare teh scale of the object
  temp_status.scale.x = 2.5;
  temp_status.scale.y = 2.5;
  temp_status.scale.z = 2.5;

  if(sensor_failure_ == 0)
  {
    temp_status.text = "OK";
    temp_status.color.g = 1.0f;
  }
  else
  {
    temp_status.text = "FAULT!";
    temp_status.color.r = 1.0f;
  }

  temp_status.lifetime = ros::Duration();

  return temp_status;
}
#endif
#endif

//
// Pack up the state into a message:
//
rel_MEKF::relative_state Estimator::packageStateInMessage(ros::Time timestamp)
{
  rel_MEKF::relative_state state_package;

  state_package.header.stamp = timestamp;
  state_package.header.frame_id = "node_frame"; /// \todo Change this to whatever we decide for the name for the node frame
  state_package.child_frame_id = "body-fixed";
  state_package.translation.x = x_(0,0);
  state_package.translation.y = x_(1,0);
  state_package.translation.z = x_(2,0); 
  state_package.rotation.x = x_(3,0);
  state_package.rotation.y = x_(4,0);
  state_package.rotation.z = x_(5,0);
  state_package.rotation.w = x_(6,0);
  state_package.velocity.x = x_(7,0);
  state_package.velocity.y = x_(8,0);
  state_package.velocity.z = x_(9,0);
  eigenToMatrixPtr(P_.block<6,6>(0,0), state_package.covariance);

  return state_package;
}


//
//  Pack up the node global state
//
geometry_msgs::TransformStamped Estimator::packageCurrentNode(ros::Time timestamp, std::string &global_name, std::string &base_name)
{
  NavNode new_node(0);
  new_node = node_queue_.back();
  Vector3d position;
  Quaterniond q;
  position = new_node.getEstimatePosition();
  q =  new_node.getEstimateOrientation(); //.getTrueOrientation();
  geometry_msgs::TransformStamped temp;
  temp.header.stamp = timestamp;
  temp.header.frame_id = global_name; //"/mocap"; /// \todo Rename this with the name for the global coordinate system
  std::ostringstream oss;
  oss << new_node.getNodeID();
  temp.child_frame_id = base_name + oss.str(); // append the id for a unique node name "/node_"
  temp.transform.translation.x = position(0);
  temp.transform.translation.y = -position(1);
  temp.transform.translation.z = -position(2);
  temp.transform.rotation.x = q.x();
  temp.transform.rotation.y = -q.y();
  temp.transform.rotation.z = -q.z();
  temp.transform.rotation.w = q.w();
  return temp;
}


//
//  Pack up the edge
//
rel_MEKF::edge Estimator::packageCurrentEdge(ros::Time timestamp)
{
  rel_MEKF::edge edge;
  NavEdge temp;
  std::string name1 = "node_";
  Vector3d pos;
  temp = edge_queue_.back();
  pos = temp.getTranslation();
  char buffer[20];
  sprintf(buffer,"node_%d",temp.getFromID());
  edge.header.stamp = timestamp;
  edge.header.frame_id = buffer;  
  sprintf(buffer,"node_%d",temp.getToID());
  edge.child_frame_id = buffer;
  edge.from_node_ID = temp.getFromID();
  edge.to_node_ID = temp.getToID();
  edge.yaw = temp.getPsi_i();
  edge.translation.x = pos(0);
  edge.translation.y = pos(1);
  edge.translation.z = pos(2);
  /// \todo Implement the covariance - need to make the function eigenToMatrixPtr into a template...
  //edge.covariance = ???
  return edge;
}


//
// Direct vision update - contains the code to process the vision measurement.  Called during the delayed update or
//  directly if there isn't much of a delay on the vo data.
//
void Estimator::directVisionUpdate(VO_message &vo_data, bool override_keyframe,
                                   TRUTH_message *truth_data)
{
  //  The state is transformed into the current camera frame to take the innovation.
  //            [0 1 2 3  4  5  6  7 8 9 10 11 12 13 14   15  16  17  18  19 20 21]
  // state x_ = [f r d qx qy qz qw u v w bp bq br ax ay | cqx cqy cqz cqw cx cy cz]

  // delta_x &       [0  1  2  3   4   5   6  7  8  9   10  11  12  13    14   15   16   17  18  19 ]
  // Covariance P_ = [df dr dd dqx dqy dqz du dv dw dbp dbq dbr dax dzy | dcqx dcqy dcqz dcx dxy dcz]

  /// First step: Express the state in the current camera frame (using the calibration)

  Quaterniond q_camera_to_body; //rotation from the camera to the body-fixed frame (CALIBRATION)
  Vector3d T_body; //location of the left-camera focal point expressed in body-fixed frame (CALIBRATION)

  if(STATE_LENGTH > 15)
  {
    //Calibration in the state:
    q_camera_to_body.x() = x_(15);
    q_camera_to_body.y() = x_(16);
    q_camera_to_body.z() = x_(17);
    q_camera_to_body.w() = x_(18);
    T_body << x_(19,0), x_(20,0), x_(21,0);
  }
  else
  {
    //fixed calibration
    q_camera_to_body = mk_consts_->q_camera_to_body;
    T_body = mk_consts_->T_camera_to_body;
  }

  //Current node (one that we are navigating with respect to, is the last element in the queue:
  NavNode current(0); //temp node to access the current one
  current = node_queue_.back();
  if(!current.TruthSet() && truth_data != NULL)
  {
    //fill the nodes' true data as soon as possible, if it wasn't set when created
    current.setTruePose(*truth_data);
    node_queue_.pop_back();
    node_queue_.push_back(current);
  }
  Quaterniond q_node_to_body(current.getNodetoBodyRotation()); //Rotation from the node frame to the body-fixed frame
  q_node_to_body = q_node_to_body.conjugate(); /// \note: Conjugate the quaternion when initialized from a rotation matrix
  Matrix3d R_node_to_body;
  R_node_to_body = current.getNodetoBodyRotation();//q_node_to_body.conjugate().toRotationMatrix();

  Matrix3d R_camera_to_body = q_camera_to_body.conjugate().toRotationMatrix(); //Rotation matrix version of CALIBRATION

  Vector3d T_node_x(x_(0,0),x_(1,0),x_(2,0)); //the transform in the current state
  Matrix3d R_node_x; //the rotation to the current body-fixed frame (current state)
  Quaterniond q_node_x(x_(6,0),x_(3,0),x_(4,0),x_(5,0)); //Quaternion form of R_node_x
  R_node_x = q_node_x.conjugate().toRotationMatrix();

//  Matrix3d R_cr_c_measure; //the measured rotation from the visual odometry algorithm
  Quaterniond q_cr_c_measured(vo_data.Rotation()); //no conjugate needed, initializing with a quaternion!
//  R_cr_c_measure = q_cr_c_measured.conjugate().toRotationMatrix();

  Vector3d T_c; //Estimated translation from current camera frame to the node camera frame (reference camera frame)
  Quaterniond q_cr_c; //Estimated Rotation from the camera reference frame to the current camera frame

  //INVERSE of Equation 2 from the "Steps for Nodes and Edges Nav" tech report
  //Translation: (because of how Eigen handles the transformation, I need to conjugate the quaternions before multiplying by a vector
  T_c = (q_node_to_body.conjugate()*q_node_x*q_camera_to_body.conjugate()).conjugate()*T_body -
        (q_node_x*q_camera_to_body.conjugate()).conjugate()*T_node_x - q_camera_to_body*T_body;
  //Rotation:
  q_cr_c = q_camera_to_body*q_node_to_body.conjugate()*q_node_x*q_camera_to_body.conjugate();

  /// Now do the update with the data expressed in the right reference frame
  /// First the position update:
  Vector3d residualP;
  Matrix<double,COVAR_LENGTH,1> delta_xP; // the error state, computed in the measurement update
  Matrix<double,3,COVAR_LENGTH> C_vP; // the Jacobian of the h_v_ w.r.t. the state
  Matrix<double,COVAR_LENGTH,3> L_vP; // the Kalman gain for the vision measurement update
  delta_xP.setZero();
  C_vP.setZero();
  L_vP.setZero();

  residualP = vo_data.Translation() - T_c; // camera x


  //Fill in the Jacobian:
  if(STATE_LENGTH > 15)
  {
    //Estimating Calibration
    Vector3d p_hat,cp_hat;
    p_hat << x_(0,0),x_(1,0),x_(2,0);
    cp_hat << x_(19,0),x_(20,0),x_(21,0);
    //Position Portion:
    /// \note When we switch to rotation matrix representation, need to switch the order of the rotations (compared to quat above)
    C_vP.block<3,3>(0,0) = -R_camera_to_body.transpose() * R_node_x;
    C_vP.block<3,3>(0,3) = R_camera_to_body.transpose()*R_node_x*skew(R_node_to_body.transpose()*T_body) -
        R_camera_to_body.transpose()*R_node_x*skew(T_node_x);
    C_vP.block<3,3>(0,14) = -skew(R_camera_to_body.transpose()*R_node_x*R_node_to_body.transpose()*T_body) +
        skew(R_camera_to_body.transpose()*R_node_x*T_node_x) + skew(R_camera_to_body.transpose()*T_body);
    C_vP.block<3,3>(0,17) = R_camera_to_body.transpose()*R_node_x*R_node_to_body.transpose()-R_camera_to_body.transpose();
  }
  else
  {
    //Position Portion:
    C_vP.block<3,3>(0,0) = -R_camera_to_body.transpose() * R_node_x;
    C_vP.block<3,3>(0,3) = R_camera_to_body.transpose()*R_node_x*skew(R_node_to_body.transpose()*T_body) -
        R_camera_to_body.transpose()*R_node_x*skew(T_node_x);
  }

  //double tuning = mk_consts_->camera_x_inflate;
  //Matrix3d tuner;
  //tuner << 1.0*mk_consts_->camera_x_inflate,0,0,0,1.0*mk_consts_->camera_x_inflate,0,0,0,1.0*mk_consts_->camera_x_inflate;
  Matrix3d R_voP;
  //R_voP << 0.00013,0,0,0,0.00014,0,0,0,0.00011;//tuner*vo_data.Covariance().block<3,3>(0,0)*tuner.transpose();
  R_voP = vo_data.Covariance().block<3,3>(0,0);
  R_voP(0,0)*=1.0;//0.5;
  R_voP*=1.0*mk_consts_->camera_x_inflate;

  // Calculate the Kalman gain
  Matrix3d S;
  S.setZero();
  S = (R_voP + C_vP * P_ * C_vP.transpose());
  L_vP = P_ * C_vP.transpose() * S.inverse();

  //Calc the error state & update the covariance
  delta_xP = L_vP * residualP;
  P_ = (MatrixXd::Identity(COVAR_LENGTH,COVAR_LENGTH) - L_vP * C_vP) * P_*
      (MatrixXd::Identity(COVAR_LENGTH,COVAR_LENGTH) - L_vP * C_vP).transpose() + L_vP * R_voP * L_vP.transpose();
  //Update the state
  applyCorrection(delta_xP);

  //DEBUG: For performing VO updates separetly
  bool apply_q = true;

  if(apply_q)
  {
  //*******************************************************************************************
  /// Apply the rotation update:
  Vector4d vq_cr_c(q_cr_c_measured.x(),q_cr_c_measured.y(),q_cr_c_measured.z(),q_cr_c_measured.w());
  Matrix<double,3,4> gammaT; //use this matrix to rotate the measured vector into the form needed
  gammaT.block<3,3>(0,0) = Matrix<double,3,3>::Identity()*q_cr_c.w() - skew(q_cr_c.vec());
  gammaT.block<3,1>(0,3) = -q_cr_c.conjugate().vec();
  Vector3d residualQ;
  Matrix3d R_voQ;
  double gain = mk_consts_->camera_qx_inflate;

  Matrix<double,COVAR_LENGTH,1> delta_xQ; // the error state, computed in the measurement update
  Matrix<double,3,COVAR_LENGTH> C_vQ; // the Jacobian of the h_v_ w.r.t. the state
  Matrix<double,COVAR_LENGTH,3> L_vQ; // the Kalman gain for the vision measurement update
  delta_xQ.setZero();
  C_vQ.setZero();
  L_vQ.setZero();

  /// Begin Roumeliotis Method:
  Matrix3d onesQ;
  onesQ.setOnes();
  onesQ(0,0) = -1.0;
  onesQ(0,2) = -1.0;
  onesQ(1,0) = -1.0;
  onesQ(1,1) = -1.0;
  onesQ(2,1) = -1.0;
  onesQ(2,2) = -1.0;

  C_vQ.block<3,3>(0,3) = 1./2.0*onesQ;

  //Fill in the Jacobian:
  if(STATE_LENGTH > 15)
  {
    //Estimating Calibration
    Matrix3d temp;
    temp(0,0) = -1.0;
    temp(0,1) = 0.5;
    temp(0,2) = 0.45;
    temp(1,0) = 0.45;
    temp(1,1) = -1.0;
    temp(1,2) = 0.5;
    temp(2,0) = 0.5;
    temp(2,1) = 0.45;
    temp(2,2) = -1.0;

    C_vQ.block<3,3>(0,14) = 1./2.0*temp;
  }

  residualQ = gammaT*vq_cr_c;
//  R_voQ << 4.8627e-05,0,0,0,
//          0,1.4040e-04,0,0,0,
//          0,0,3.1517e-05,0,
//          0,0,0,7.1446e-08;//gammaT*vo_data.Covariance().block<4,4>(3,3)*gammaT.transpose()*gain;
  R_voQ =gammaT*vo_data.Covariance().block<4,4>(3,3)*gammaT.transpose()*gain;
//  R_voQ*=gain;
  /// End Roumeliotis Method

  /// Begin ETH Method:
//    Quaterniond q_err;
//    q_err = q_cr_c_measured*q_cr_c.conjugate();
//    residualQ = q_err.vec()/(q_err.w()*2.0);
//    R_voQ = vo_data.Covariance().block<3,3>(3,3)*inflate*gain;

//    //Fill in the Jacobian: Rotation Portion (same for both - deltatheta for the calibration is all 2nd order stuff):
//    C_vQ.block<3,3>(0,3) = R_node_to_body.transpose()*R_camera_to_body*0.5;
  /// End ETH Method


  /// Apply the Update:
  // Calculate the Kalman gain
  Matrix3d Sq;
  Sq.setZero();
  Sq = (R_voQ + C_vQ* P_ * C_vQ.transpose());
  L_vQ = P_ * C_vQ.transpose() * Sq.inverse();

  //Calc the error state & update the covariance
  delta_xQ = L_vQ * residualQ;
  P_ = (MatrixXd::Identity(COVAR_LENGTH,COVAR_LENGTH) - L_vQ * C_vQ) * P_*
      (MatrixXd::Identity(COVAR_LENGTH,COVAR_LENGTH) - L_vQ * C_vQ).transpose() + L_vQ * R_voQ * L_vQ.transpose();
  //std::cout << P_ << std::endl;
  //Update the state
  applyCorrection(delta_xQ);


  //DEBUG:
  //ROS_DEBUG_STREAM("Current State: " << x_.transpose());

  }

  //If this data is from a new node image, need to replace the states! (as long as it isn't overriden by delayedVisionUpdate)
  if (!override_keyframe && vo_data.NewReference())
  {
    ROS_INFO("New Node!**********************************");
    //create the edge
    NavEdge newedge(x_, P_, node_id_incrementer_, node_id_incrementer_+1);
    edge_queue_.push_back(newedge);
    //create the new node
    NavNode newnode(node_id_incrementer_+1);
    node_id_incrementer_++; //increment to reflect the new current node

    if(truth_data)
      newnode.setTruePose(*truth_data);

    //Find the global estimated position and orientation of the new node
    global_node_position_ = global_node_position_ + global_R_yaw_ * newedge.getTranslation();
    //Save the global estimates
    Quaterniond temp(x_(6,0),x_(3,0),x_(4,0),x_(5,0));
    newnode.setEstimatePosition(global_node_position_,global_yaw_,temp);
    //update the global estimates for the next node
    global_R_yaw_ = global_R_yaw_ * newedge.getR_curr_next().transpose();  //update the rotation matrix, the current rotation is used for the NEXT translation
    global_yaw_ = global_yaw_ + newedge.getPsi_i();  //the angle applies to the next node!
    //store the node
    node_queue_.push_back(newnode);

    //Augment and Marginalize the State and Covariance!
    augmentMarginalize(saved_deltatheta_);
  }
}


//
//  Augment new relative state and marginalize out the old ones:
//
void Estimator::augmentMarginalize(Vector3d &delta_quat)
{
  //The position states are simply zeroed out & the corresponding rows and columns of the covariance as well.
  //The quaternion is tricky, we are zeroing out the yaw angle, but not the roll and pitch.

  //  The state is transformed into the current camera frame to take the innovation.
  //            [0 1 2 3  4  5  6  7 8 9 10 11 12 13 14   15  16  17  18  19 20 21]
  // state x_ = [f r d qx qy qz qw u v w bp bq br ax ay | cqx cqy cqz cqw cx cy cz]

  // delta_x &       [0  1  2  3   4   5   6  7  8  9   10  11  12  13    14   15   16   17  18  19 ]
  // Covariance P_ = [df dr dd dqx dqy dqz du dv dw dbp dbq dbr dax dzy | dcqx dcqy dcqz dcx dxy dcz]

  double qx,qy,qz,qw;
  qx = x_(3,0);
  qy = x_(4,0);
  qz = x_(5,0);
  qw = x_(6,0);

  double phi,theta;

  phi = atan2(2.0*(qw*qx + qy*qz),qw*qw+qz*qz-qx*qx-qy*qy);
  theta = asin(2.0*(qw*qy -qx*qz));

  Matrix<double,7,1> f;
  f(0,0) = 0.d;
  f(1,0) = 0.d;
  f(2,0) = 0.d;
  f(3,0) = cos(0.5*theta)*sin(0.5*phi);
  f(4,0) = sin(0.5*theta)*cos(0.5*phi);
  f(5,0) = -sin(0.5*theta)*sin(0.5*phi);
  f(6,0) = cos(0.5*theta)*cos(0.5*phi);

  //Replace the current state with the new state:
  x_(0,0) = f(0,0);
  x_(1,0) = f(1,0);
  x_(2,0) = f(2,0);
  x_(3,0) = f(3,0);
  x_(4,0) = f(4,0);
  x_(5,0) = f(5,0);
  x_(6,0) = f(6,0);
  //the remainder is not affected, keep it the same

  //As the covariance is in the reduced form, the procedure is slightly different.  The same equations are used, but
  //for the error quaternion.  As we are dealing with the error quaternion, I eliminated 2nd order terms (terms of the
  //error quaternion multiplying other error quaternion terms).  We are also zeroing out the position information.

  Matrix<double,6,COVAR_LENGTH> A;
  A.setZero();

  //Common terms for the derivatives:
  double asiny,atanx,x,y;
  x = delta_quat(0);
  y = delta_quat(1);
  asiny = 0.5*asin(y);
  atanx = 0.5*atan2(x,1);

  // Jacobian of the change w.r.t. delta theta
  A(3,3) = -0.5*cos(asiny)*cos(atanx)/(x*x + 1);
  A(3,4) = -0.5*sin(asiny)*sin(atanx)/sqrt(1-y*y);
  A(4,3) = 0.5*sin(asiny)*sin(atanx)/(x*x + 1);
  A(4,4) = 0.5*cos(asiny)*cos(atanx)/sqrt(1-y*y);
  A(5,3) = 0.5*sin(asiny)*cos(atanx)/(x*x + 1);
  A(5,4) = -0.5*cos(asiny)*sin(atanx)/sqrt(1-y*y);

  //Replace the Covariance:
  Matrix<double,6,6> center;
  Matrix<double,6,COVAR_LENGTH> right;
  Matrix<double,COVAR_LENGTH,6> left;
  center.setZero();
  right.setZero();
  left.setZero();

  center = A*P_*A.transpose();
  right = A*P_;
  left = P_.transpose()*A.transpose();

  P_.topLeftCorner(6,COVAR_LENGTH) = right;
  P_.topLeftCorner(COVAR_LENGTH,6) = left;
  P_.topLeftCorner(6,6) = center;
}


//
// Make a date/time string for a filename
//
std::string Estimator::timeStructFilename(tm *time_struct)
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


//
// Apply the delta_state correction to the current state
//
void Estimator::applyCorrection(Eigen::Matrix<double, COVAR_LENGTH,1> &delta_x)
{

  //            [0 1 2 3  4  5  6  7 8 9 10 11 12 13 14   15  16  17  18  19 20 21]
  // state x_ = [f r d qx qy qz qw u v w bp bq br ax ay | cqx cqy cqz cqw cx cy cz]

  // delta_x &       [0  1  2  3   4   5   6  7  8  9   10  11  12  13    14   15   16   17  18  19 ]
  // Covariance P_ = [df dr dd dqx dqy dqz du dv dw dbp dbq dbr dax dzy | dcqx dcqy dcqz dcx dxy dcz]

  //Update the state
  Quaterniond state(x_(6,0),x_(3,0),x_(4,0),x_(5,0));
  Quaterniond deltaq = quaternionFromSmallAngle(delta_x.block<3,1>(3,0));
  Quaterniond newq;
  newq = deltaq*state; //update the quaternion multiplicatively
  //newq.normalize(); //For some reason this step makes the scalar always positive (which can mess things up with large
  // yaws.

  x_.block<3,1>(0,0) += delta_x.block<3,1>(0,0);
  x_(3,0) = newq.x();
  x_(4,0) = newq.y();
  x_(5,0) = newq.z();
  x_(6,0) = newq.w();
  x_.block<8,1>(7,0) += delta_x.block<8,1>(6,0);

  if(STATE_LENGTH > 15)
  {
    //estimating calibration:
    Quaterniond state2(x_(18,0),x_(15,0),x_(16,0),x_(17,0));
    deltaq = quaternionFromSmallAngle(delta_x.block<3,1>(14,0));
    newq.setIdentity();
    newq = deltaq*state2; //update the quaternion multiplicatively
    //newq.normalize();
    x_(15,0) = newq.x();
    x_(16,0) = newq.y();
    x_(17,0) = newq.z();
    x_(18,0) = newq.w();
    x_.block<3,1>(19,0) += delta_x.block<3,1>(17,0);
  }


////
//// Apply the VO update separately
////
//void Estimator::directVisionUpdateSeparate(VO_message &vo_data, double inflate, TRUTH_message *truth_data)
//{
//  //  The state is transformed into the current camera frame to take the innovation.
//  //            [0 1 2 3  4  5  6  7 8 9 10 11 12 13 14   15  16  17  18  19 20 21]
//  // state x_ = [f r d qx qy qz qw u v w bp bq br ax ay | cqx cqy cqz cqw cx cy cz]

//  // delta_x &       [0  1  2  3   4   5   6  7  8  9   10  11  12  13    14   15   16   17  18  19 ]
//  // Covariance P_ = [df dr dd dqx dqy dqz du dv dw dbp dbq dbr dax dzy | dcqx dcqy dcqz dcx dxy dcz]

//  /// First step: Express the state in the current camera frame (using the calibration)

//  Quaterniond q_camera_to_body; //rotation from the camera to the body-fixed frame (CALIBRATION)
//  Vector3d T_body; //location of the left-camera focal point expressed in body-fixed frame (CALIBRATION)

//  if(STATE_LENGTH > 15)
//  {
//    //Calibration in the state:
//    q_camera_to_body.x() = x_(15);
//    q_camera_to_body.y() = x_(16);
//    q_camera_to_body.z() = x_(17);
//    q_camera_to_body.w() = x_(18);
//    T_body << x_(19,0), x_(20,0), x_(21,0);
//  }
//  else
//  {
//    //fixed calibration
//    q_camera_to_body = mk_consts_->q_camera_to_body;
//    T_body = mk_consts_->T_camera_to_body;
//  }

//  //Current node (one that we are navigating with respect to, is the last element in the queue:
//  NavNode current(0); //temp node to access the current one
//  current = node_queue_.back();
//  if(!current.TruthSet())
//  {
//    //fill the nodes' true data as soon as possible, if it wasn't set when created
//    current.setTruePose(*truth_data);
//    node_queue_.pop_back();
//    node_queue_.push_back(current);
//  }
//  Quaterniond q_node_to_body(current.getNodetoBodyRotation()); //Rotation from the node frame to the body-fixed frame
//  q_node_to_body = q_node_to_body.conjugate(); /// \note: Conjugate the quaternion when initialized from a rotation matrix
//  Matrix3d R_node_to_body;
//  R_node_to_body = current.getNodetoBodyRotation();//q_node_to_body.conjugate().toRotationMatrix();

//  Matrix3d R_camera_to_body = q_camera_to_body.conjugate().toRotationMatrix(); //Rotation matrix version of CALIBRATION

//  Vector3d T_node_x(x_(0,0),x_(1,0),x_(2,0)); //the transform in the current state
//  Matrix3d R_node_x; //the rotation to the current body-fixed frame (current state)
//  Quaterniond q_node_x(x_(6,0),x_(3,0),x_(4,0),x_(5,0)); //Quaternion form of R_node_x
//  R_node_x = q_node_x.conjugate().toRotationMatrix();

////  Matrix3d R_cr_c_measure; //the measured rotation from the visual odometry algorithm
//  Quaterniond q_cr_c_measured(vo_data.Rotation()); //no conjugate needed, initializing with a quaternion!
////  R_cr_c_measure = q_cr_c_measured.conjugate().toRotationMatrix();

//  Vector3d T_c; //Estimated translation from current camera frame to the node camera frame (reference camera frame)
//  Quaterniond q_cr_c; //Estimated Rotation from the camera reference frame to the current camera frame

//  //INVERSE of Equation 2 from the "Steps for Nodes and Edges Nav" tech report
//  //Translation:
//  T_c = (q_node_to_body.conjugate()*q_node_x*q_camera_to_body.conjugate()).conjugate()*T_body -
//        (q_node_x*q_camera_to_body.conjugate()).conjugate()*T_node_x - q_camera_to_body*T_body;
//  //Rotation:
//  q_cr_c = q_camera_to_body*q_node_to_body.conjugate()*q_node_x*q_camera_to_body.conjugate();

//  /// Now do the update with the data expressed in the right reference frame
//  /// First the position update:
//  Vector3d residualP;
//  Matrix<double,COVAR_LENGTH,1> delta_xP; // the error state, computed in the measurement update
//  Matrix<double,3,COVAR_LENGTH> C_vP; // the Jacobian of the h_v_ w.r.t. the state
//  Matrix<double,COVAR_LENGTH,3> L_vP; // the Kalman gain for the vision measurement update
//  delta_xP.setZero();
//  C_vP.setZero();
//  L_vP.setZero();

//  residualP = vo_data.Translation() - T_c; // camera x

//  //Fill in the Jacobian:
//  if(STATE_LENGTH > 15)
//  {
//    //Estimating Calibration
//    Vector3d p_hat,cp_hat;
//    p_hat << x_(0,0),x_(1,0),x_(2,0);
//    cp_hat << x_(19,0),x_(20,0),x_(21,0);
//    //Position Portion:
//    /// \note When we switch to rotation matrix representation, need to switch the order of the rotations (compared to quat above)
//    C_vP.block<3,3>(0,0) = -R_camera_to_body.transpose() * R_node_x;
//    C_vP.block<3,3>(0,3) = R_camera_to_body.transpose()*R_node_x*skew(R_node_to_body.transpose()*T_body) -
//        R_camera_to_body.transpose()*R_node_x*skew(T_node_x);
//    C_vP.block<3,3>(0,14) = -skew(R_camera_to_body.transpose()*R_node_x*R_node_to_body.transpose()*T_body) +
//        skew(R_camera_to_body.transpose()*R_node_x*T_node_x) + skew(R_camera_to_body.transpose()*T_body);
//    C_vP.block<3,3>(0,17) = R_camera_to_body.transpose()*R_node_x*R_node_to_body.transpose()-R_camera_to_body.transpose();
//  }
//  else
//  {
//    //Position Portion:
//    C_vP.block<3,3>(0,0) = -R_camera_to_body.transpose() * R_node_x;
//    C_vP.block<3,3>(0,3) = R_camera_to_body.transpose()*R_node_x*skew(R_node_to_body.transpose()*T_body) -
//        R_camera_to_body.transpose()*R_node_x*skew(T_node_x);
//  }

//  double tuning = mk_consts_->camera_x_inflate;
//  Matrix3d R_voP;
//  R_voP = vo_data.Covariance().block<3,3>(0,0)*inflate*tuning;

//  // Calculate the Kalman gain
//  Matrix3d S;
//  S.setZero();
//  S = (R_voP + C_vP * P_ * C_vP.transpose());
//  L_vP = P_ * C_vP.transpose() * S.inverse();

//  //Calc the error state & update the covariance
//  delta_xP = L_vP * residualP;
//  P_ = (MatrixXd::Identity(COVAR_LENGTH,COVAR_LENGTH) - L_vP * C_vP) * P_*
//      (MatrixXd::Identity(COVAR_LENGTH,COVAR_LENGTH) - L_vP * C_vP).transpose() + L_vP * R_voP * L_vP.transpose();
//  //Update the state
//  applyCorrection(delta_xP);

//  bool applyQ = true;

//  //*******************************************************************************************
//  if(applyQ)
//  {
//    /// Apply the rotation update:
//    Vector4d vq_cr_c(q_cr_c_measured.x(),q_cr_c_measured.y(),q_cr_c_measured.z(),q_cr_c_measured.w());
//    Matrix<double,3,4> gammaT; //use this matrix to rotate the measured vector into the form needed
//    gammaT.block<3,3>(0,0) = Matrix<double,3,3>::Identity()*q_cr_c.w() - skew(q_cr_c.vec());
//    gammaT.block<3,1>(0,3) = -q_cr_c.conjugate().vec();
//    Vector3d residualQ;
//    Matrix3d R_voQ;
//    double gain = mk_consts_->camera_qx_inflate;

//    Matrix<double,COVAR_LENGTH,1> delta_xQ; // the error state, computed in the measurement update
//    Matrix<double,3,COVAR_LENGTH> C_vQ; // the Jacobian of the h_v_ w.r.t. the state
//    Matrix<double,COVAR_LENGTH,3> L_vQ; // the Kalman gain for the vision measurement update
//    delta_xQ.setZero();
//    C_vQ.setZero();
//    L_vQ.setZero();

//    /// Begin Roumeliotis Method:
//    Matrix3d onesQ;
//    onesQ.setOnes();
//    onesQ(0,0) = -1.0;
//    onesQ(0,2) = -1.0;
//    onesQ(1,0) = -1.0;
//    onesQ(1,1) = -1.0;
//    onesQ(2,1) = -1.0;
//    onesQ(2,2) = -1.0;

//    C_vQ.block<3,3>(0,3) = 1./2.0*onesQ;

//    //Fill in the Jacobian:
//    if(STATE_LENGTH > 15)
//    {
//      //Estimating Calibration
//      Matrix3d temp;
//      temp(0,0) = -1.0;
//      temp(0,1) = 0.5;
//      temp(0,2) = 0.45;
//      temp(1,0) = 0.45;
//      temp(1,1) = -1.0;
//      temp(1,2) = 0.5;
//      temp(2,0) = 0.5;
//      temp(2,1) = 0.45;
//      temp(2,2) = -1.0;

//      C_vQ.block<3,3>(0,14) = 1./2.0*temp;
//    }

//    residualQ = gammaT*vq_cr_c;
//    R_voQ = gammaT*vo_data.Covariance().block<4,4>(3,3)*gammaT.transpose()*inflate*gain;
//    /// End Roumeliotis Method

//    /// Begin ETH Method:
////    Quaterniond q_err;
////    q_err = q_cr_c_measured*q_cr_c.conjugate();
////    residualQ = q_err.vec()/(q_err.w()*2.0);
////    R_voQ = vo_data.Covariance().block<3,3>(3,3)*inflate*gain;

////    //Fill in the Jacobian: Rotation Portion (same for both - deltatheta for the calibration is all 2nd order stuff):
////    C_vQ.block<3,3>(0,3) = R_node_to_body.transpose()*R_camera_to_body*0.5;
//    /// End ETH Method


//    /// Apply the Update:
//    // Calculate the Kalman gain
//    Matrix3d Sq;
//    Sq.setZero();
//    Sq = (R_voQ + C_vQ* P_ * C_vQ.transpose());
//    L_vQ = P_ * C_vQ.transpose() * Sq.inverse();

//    //Calc the error state & update the covariance
//    delta_xQ = L_vQ * residualQ;
//    P_ = (MatrixXd::Identity(COVAR_LENGTH,COVAR_LENGTH) - L_vQ * C_vQ) * P_*
//        (MatrixXd::Identity(COVAR_LENGTH,COVAR_LENGTH) - L_vQ * C_vQ).transpose() + L_vQ * R_voQ * L_vQ.transpose();
//    //std::cout << P_ << std::endl;
//    //Update the state
//    applyCorrection(delta_xQ);
//  }

//  //DEBUG:
//  ROS_DEBUG_STREAM("Current State: " << x_.transpose());



//  //If this data is from a new node image, need to replace the states!
//  if (vo_data.NewReference())
//  {
//    ROS_INFO("New Node!**********************************");
//    //create the edge
//    NavEdge newedge(x_, P_, node_id_incrementer_, node_id_incrementer_+1);
//    edge_queue_.push_back(newedge);
//    //create the new node
//    NavNode newnode(node_id_incrementer_+1);
//    node_id_incrementer_++; //increment to reflect the new current node

//    if(truth_data)
//      newnode.setTruePose(*truth_data);

//    //Find the global estimated position and orientation of the new node
//    global_node_position_ = global_node_position_ + global_R_yaw_ * newedge.getTranslation();
//    //Save the global estimates
//    newnode.setEstimatePose(global_node_position_, global_yaw_);
//    //update the global estimates for the next node
//    global_R_yaw_ = global_R_yaw_ * newedge.getR_curr_next().transpose();  //update the rotation matrix, the current rotation is used for the NEXT translation
//    global_yaw_ = global_yaw_ + newedge.getPsi_i();  //the angle applies to the next node!
//    //store the node
//    node_queue_.push_back(newnode);

//    //Augment and Marginalize the State and Covariance!
//    augmentMarginalize(saved_deltatheta_);
//  }
//}



  ////
  ////  Get a rotation matrix
  ////
  //Matrix3d Estimator::convertQuaterionToRotation(Quaterniond &q)
  //{
  //  Matrix3d R;
  //  q.normalize();

  //  R << 2*q.w()*q.w() - 1 + 2*q.x()*q.x(), 2*q.x()*q.y()+2*q.w()*q.z(), 2*q.x()*q.z()-2*q.w()*q.y(),
  //       2*q.x()*q.y()-2*q.w()*q.z(), 2*q.w()*q.w() - 1 + 2*q.y()*q.y(), 2*q.y()*q.z() + 2*q.w()*q.x(),
  //       2*q.x()*q.z() + 2*q.w()*q.y(), 2*q.y()*q.z() - 2*q.w()*q.x(), 2*q.w()*q.w() - 1 + 2*q.z()*q.z();

  //  return R;
  //}



  ////
  ////  Get a quaternion
  ////
  //Quaterniond Estimator::convertRotationToQuaternion(Matrix3d &R)
  //{
  //  Quaterniond q;
  //  q.w() = ((1.0/2.0)*sqrt(R(0,0) + R(1,1) + R(2,2) + 1.d));
  //  q.x() = ((R(1,2) - R(2,1))/(4.0*q.w()));
  //  q.y() = ((R(2,0) - R(0,2))/(4.0*q.w()));
  //  q.z() = ((R(0,1) - R(1,0))/(4.0*q.w()));
  //  q.normalize();
  //  return q;
  //}
}






