 /* \copyright This work was completed by Robert Leishman while performing official duties as 
  * a federal government employee with the Air Force Research Laboratory and is therefore in the 
  * public domain (see 17 USC § 105). Public domain software can be used by anyone for any purpose,
  * and cannot be released under a copyright license
  */

/*!
 *  \file ros_server.cpp
 *  \author Robert Leishman
 *  \date June 2012
 *
*/


#include "rel_estimator/ros_server.h"

pthread_mutex_t w_mutex_ = PTHREAD_MUTEX_INITIALIZER; //!< the mutex for the while_true_ access
pthread_mutex_t i_mutex_ = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t v_mutex_ = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t a_mutex_ = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t t_mutex_ = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t h_mutex_ = PTHREAD_MUTEX_INITIALIZER;

//
// Constructor
//
ROSServer::ROSServer(ros::NodeHandle &nh, Constants *mk_const): mk_consts_(mk_const)
{
  //setup the Estimator:
  estimator_ = new Estimator(mk_const);

  //initialize while_true_
  while_true_ = 1;

  std::string imu_topic,vo_topic,alt_topic,truth_topic,hex_topic,pose_topic,global_topic,global_node_topic,edge_topic;

  //retrieve names from server
#ifndef LASER
  ros::param::param<std::string>("~alt_topic", alt_topic, "/alt_msgs");
#else
  ROS_WARN("****************************************************************************************");
  ROS_WARN("LASER IN USE: SONAR ALTIMETER IS NOT IN USE - NOT RECOMMENDED FOR AUTONOMOUS BEHAVIOUR!!");
  ROS_WARN("****************************************************************************************");
  ros::param::param<std::string>("~alt_topic", alt_topic, "/scan");
#endif

  ros::param::param<std::string>("~imu_topic", imu_topic, "/imu/data");
  ros::param::param<std::string>("~vo_topic", vo_topic, "/kinect_visual_odometry/vo_transformation");
  ros::param::param<std::string>("~mocap_topic", truth_topic, "/evart/heavy_ros/base");
  ros::param::param<std::string>("~hex_debug_topic", hex_topic, "/mikoImu"); // /imu/data
  ros::param::param<std::string>("~output_rel_topic", pose_topic, "states");
  ros::param::param<std::string>("~estimated_global_topic", global_topic, "global_pose");
  ros::param::param<std::string>("~node_global_pose_topic",global_node_topic,"cur_node/global");
  ros::param::param<std::string>("~current_edge_topic",edge_topic,"cur_edge/pose");
  ros::param::param<std::string>("~node_frame_name", node_frame_name_, "/node_frame");
  ros::param::param<std::string>("~body_frame_name", body_frame_name_, "/node_frame/body_fixed");
  ros::param::param<std::string>("~global_frame_name", global_frame_name_, "/global_frame"); //gives the global frame a name
  ros::param::param<std::string>("~global_body_name", global_body_frame_name_, "/global_frame/body-fixed"); //the current pose in the global coordinates
  ros::param::param<std::string>("~base__node_name", base_node_name_, "/global_frame/node_"); //the basic name that is appended with the node id for the name


  /*!
    This segment documents the parameters available to modify on the parameter server:
    \code{.cpp}
  ros::param::param<std::string>("~imu_topic", imu_topic, "/imu/data");  //!< the IMU topic name
  ros::param::param<std::string>("~vo_topic", vo_topic, "/kinect_visual_odometry/vo_transformation"); //!< the visual odometry topic name
  ros::param::param<std::string>("~alt_topic", alt_topic, "/alt_msgs"); //!< the altimeter topic (not sure what to do about laser topic)
  ros::param::param<std::string>("~mocap_topic", truth_topic, "/evart/heavy_ros/base"); //!< the topic for the motion capture truth
  ros::param::param<std::string>("~hex_debug_topic", hex_topic, "/mikoImu"); //!< the topic for the hexacopter debug data
  ros::param::param<std::string>("~output_rel_topic", pose_topic, "relative/pose");  //!< the pose topic that is published by this class
  ros::param::param<std::string>("~estimated_global_topic", global_topic, "global_pose");
  ros::param::param<std::string>("~node_global_pose_topic",global_node_topic,"/cur_node/global"); //!< topic on which we publish the current node global pose
  ros::param::param<std::string>("~current_edge_topic",edge_topic,"/cur_edge/pose");
  ros::param::param<std::string>("~global_frame_name", global_frame_name_, "/node_frame");//!< name for the node coord. frame
  ros::param::param<std::string>("~body_frame_name", body_frame_name_, "/body_fixed");//!< name for the body fixed frame
    \endcode

  */

  //assign callbacks to the subscribers:
  imu_subscriber_ = nh.subscribe(imu_topic,5,&ROSServer::imuCallback,this);
  vo_subscriber_ = nh.subscribe(vo_topic,5,&ROSServer::visualCallback,this);
  alt_subscriber_ = nh.subscribe(alt_topic,5,&ROSServer::altCallback,this);
  truth_subscriber_ = nh.subscribe(truth_topic,5,&ROSServer::truthCallback,this);
  hex_subscriber_ = nh.subscribe(hex_topic,5,&ROSServer::hexCallback,this);

  //Call the VO and request that it start over with a new reference image:
  /// \todo Use the service to request a new reference image.

  //publishers:
  rel_state_publisher_ = nh.advertise<rel_MEKF::relative_state>(pose_topic,5);
  global_pose_publisher_ = nh.advertise<geometry_msgs::TransformStamped>(global_topic, 5);
  node_global_pub_ = nh.advertise<geometry_msgs::TransformStamped>(global_node_topic,5);
  edge_pub_ = nh.advertise<rel_MEKF::edge>(edge_topic,5);

#ifdef LASER
#ifdef DETECT
  status_pub_ = nh.advertise<Status_message>("/laser_status", 10);
#endif
#endif

  //Should we publish the edges as well?  I think we should... (then a place recognition could publish it's edges and
  // an optimization scheme could be incorporated as another ROS node bringing all those pieces in... TODO...

  past_accz_ = 0;

  ROS_INFO_ONCE("Listening to the IMU, VO, Altimeter, Hex, and Truth Topics!");
}


//
// Destructor
//
ROSServer::~ROSServer()
{
  estimator_->~Estimator();
  delete estimator_;
}


//
// Run function: the main loop
//
void ROSServer::Run()
{
  IMU_message imu_data;
  VO_message *vo_data;
  sensor_msgs::Range *alt_data;
  TRUTH_message *truth_data;
  Hex_message *hex_data;
  geometry_msgs::TransformStamped global_pose;  
  bool iflag,vflag; //flags for imu, altimeter, vision, and truth data

  //
  //Main Loop
  while(while_true_ == 1)
  {
    //Data for pointers to use
    VO_message vo_temp;
    sensor_msgs::Range alt_temp;
    TRUTH_message truth_temp;
    Hex_message hex_temp;
    int imu_queue_size;


    //check IMU:
    pthread_mutex_lock(&i_mutex_);
    if((int)imu_queue_.size() > 0)
    {
      ROS_INFO_ONCE("IMU DATA RECEIVED BY ESTIMATOR!");
      imu_data = imu_queue_.front();
      imu_queue_.pop();
      iflag = true;
      dt_ = imu_data.header.stamp.toSec() - old_time_;
      if(dt_ > 1000)
        dt_ = 0.d; //for the first time through
      old_time_ = imu_data.header.stamp.toSec();
      imu_queue_size = (int)imu_queue_.size();
    }
    else
    {
      iflag = false;
      imu_queue_size = 0;
    }
    pthread_mutex_unlock(&i_mutex_);


    //If there's IMU, check for other measurements and calc, if not, skip and wait until the next loop
    if(iflag)
    {
      //altitude data
      pthread_mutex_lock(&a_mutex_);     
        if((int)alt_queue_.size() > 5 && !estimator_->startup_flag_)
        {          
          ROS_WARN("Altitude Queue Size is Large!");
          if(imu_queue_size < 2 && (int)alt_queue_.size() > 2)
          {
            while((int)alt_queue_.size() > 2)
              alt_queue_.pop();
          }
        }

        if((int)alt_queue_.size() > 0)
        {
          ROS_INFO_ONCE("SONAR ALTIMETER DATA RECEIVED BY ESTIMATOR!");
          alt_temp = alt_queue_.front();
          alt_data = &alt_temp;
          alt_queue_.pop();
        }
        else
        {
          alt_data = 0;
        }
      pthread_mutex_unlock(&a_mutex_);

      //Vision data
      pthread_mutex_lock(&v_mutex_);
        if((int)vo_queue_.size() > 0)
        {
          ROS_INFO_ONCE("VO DATA RECEIVED BY ESTIMATOR!");
          if((int)vo_queue_.size() > 5 && !estimator_->startup_flag_)
          {
            ROS_WARN("Vision Queue Size is Large!");            
            //eliminate them if they are not a reference...
            vo_temp = vo_queue_.front();
            while(!vo_temp.NewReference() && vo_queue_.size() >1)
            {
              vo_queue_.pop_front();
              vo_temp = vo_queue_.front();
            }
            vo_data = &vo_temp;
            vflag = true;
            vo_queue_.pop_front();            
          }
          else
          {
            vo_temp = vo_queue_.front();
            vo_queue_.pop_front();
            vo_data = &vo_temp;
            vflag = true;
          }
        }
        else
        {
          vflag = false;
          vo_data = 0;
        }
      pthread_mutex_unlock(&v_mutex_);

      //Truth data
      pthread_mutex_lock(&t_mutex_);

        if((int)truth_queue_.size() > 5 && !estimator_->startup_flag_)
        {
          //ROS_WARN("Truth Queue Size is Large!");
          while((int)truth_queue_.size() > 1)
            truth_queue_.pop();
        }
        if((int)truth_queue_.size() > 0)
        {
          ROS_INFO_ONCE("TRUTH DATA RECEIVED BY ESTIMATOR!");
          truth_temp = truth_queue_.front();
          truth_queue_.pop();
          truth_data = &truth_temp;
        }
        else
        {
          truth_data = 0;
        }
      pthread_mutex_unlock(&t_mutex_);


      //Hex data
      pthread_mutex_lock(&h_mutex_);
        if((int)hex_queue_.size() > 5 && !estimator_->startup_flag_)
        {
          while((int)hex_queue_.size() > 1)
            hex_queue_.pop();
        }
        if((int)hex_queue_.size() > 0)
        {
          ROS_INFO_ONCE("HEX IMU/INFO DATA RECEIVED BY ESTIMATOR!");
          hex_temp = hex_queue_.front();
          hex_queue_.pop();
          hex_data = &hex_temp;
        }
        else
        {
          hex_data = 0;
        }
      pthread_mutex_unlock(&h_mutex_);

      //
      //Start Processing the data that was available:
      if(estimator_->startup_flag_)
      {
        //2 seconds is the min touchdown time
        if(estimator_->just_landed_ && (ros::Time::now() - landed_time_).toSec() > 2.0)
        {
          estimator_->just_landed_ = false;
        }

        estimator_->Initialize(imu_data,hex_data,alt_data,truth_data);

      }
      else
      {
        //
        //Run the filter
        //
        ROS_WARN_ONCE("ESTIMATOR IN RUN LOOP!"); 
        /// Here we assume that the vision is delayed, so we process it first, before getting to the prediction and other
        /// measurement updates.
        if(vflag)
        {
          estimator_->prepareQueuedItems(vo_data->Timestamp());

          estimator_->delayedVisionUpdate(*vo_data,truth_data);

          if(vo_data->NewReference())
          {
            rel_MEKF::edge edge_message;
            edge_message = estimator_->packageCurrentEdge(vo_data->Timestamp());
            geometry_msgs::TransformStamped transform;
            transform = estimator_->packageCurrentNode(vo_data->Timestamp(),global_frame_name_,base_node_name_);
            edge_pub_.publish(edge_message);
            node_global_pub_.publish(transform);
          }
        }

        /// \todo Add the capability to simulate vision using truth

        //Process IMU & Altitude
        estimator_->prediction(mk_consts_->normal_steps,dt_,imu_data);
        estimator_->imuMeasurementUpdate(imu_data);

#ifdef DETECT
        estimator_->altitudeMeasurementUpdate(alt_data, true);
#else
        estimator_->altitudeMeasurementUpdate(alt_data);
#endif

        //Save the IMU, Altitude, and State in queues for use with delayed updates
        estimator_->saveData(imu_data,alt_data);

        //Calc the global pose (May think about doing this at a slower rate than every loop - especially after we start
        //leaving the cortex room and there isn't really any global pose to compare this to)
        global_pose = estimator_->computeGlobalPoseEstimate(imu_data.header.stamp,global_frame_name_,global_body_frame_name_);

        //if((alt_data != NULL && USE_LASER == 0 && alt_data->range >= -0.30) || (hex_data != NULL && hex_data->motor1 < 50 && hex_data->motor2 < 50))
        if(imu_data.linear_acceleration.z <= ACCZ_LANDED_ && past_accz_ <= ACCZ_LANDED_)
        {
          //landed
          estimator_->startup_flag_ = true;
          estimator_->just_landed_ = true;
          landed_time_ = ros::Time::now();
          ROS_WARN("LANDING FLAG TRIPPED ON ESTIMATOR: SHOULD NOT BE FLYING!");
        }
        past_accz_ = imu_data.linear_acceleration.z;

        //Publish the global pose:
        global_pose_publisher_.publish(global_pose);

#ifdef LASER
#ifdef DETECT
        Status_message laser_status;
        laser_status = estimator_->packageLaserStatus(imu_data.header.stamp);

        status_pub_.publish(laser_status);
#endif
#endif        
      }
      //Log data (here so that it will log before we actual start the estimator
      estimator_->writeToLog(imu_data,global_pose,alt_data,vo_data,truth_data);

      //Publish the relative state and the tf:
      rel_MEKF::relative_state rel_state;
      rel_state = estimator_->packageStateInMessage(imu_data.header.stamp);

      //assign the frame names from the param server:
      rel_state.header.frame_id = node_frame_name_;
      rel_state.child_frame_id = body_frame_name_;
      rel_state_publisher_.publish(rel_state);

      //Publish transform for mapping in the node frame - ROS Convention is North-West-Up, we will follow that with the tf:
      tf::Transform transform;
      transform.setOrigin(tf::Vector3(rel_state.translation.x,
                                      -rel_state.translation.y,
                                      -rel_state.translation.z));
      transform.setRotation(tf::Quaternion(rel_state.rotation.x,
                                           -rel_state.rotation.y,
                                           -rel_state.rotation.z,
                                           rel_state.rotation.w));
      relative_tf_.sendTransform(tf::StampedTransform(transform,imu_data.header.stamp,node_frame_name_,
                                                      body_frame_name_));
    } // end if(iflag)
    else
    {
      //sleep the thread if no data is available
      boost::this_thread::sleep(boost::posix_time::milliseconds(3));
    }  
  } //end While

  pthread_mutex_lock(&w_mutex_);
    while_true_ = 0;
  pthread_mutex_unlock(&w_mutex_);

  //delete the pointers
  delete alt_data;
  delete vo_data;
  delete truth_data;
  delete hex_data;

}//end *ROSServer::Run(void *ptr)



//
//  Read/Write to the while_true variable
//
int ROSServer::accessWhileTrue(int while_true_value)
{
  int temp;
  if(while_true_value != 1)
  {
    pthread_mutex_lock(&w_mutex_);
      while_true_ = while_true_value;
    pthread_mutex_unlock(&w_mutex_);
    temp = while_true_value;
  }
  else
  {
    pthread_mutex_lock(&w_mutex_);
      temp = while_true_;
    pthread_mutex_unlock(&w_mutex_);
  }

  return temp;
}


//
// IMU Callback
//
void ROSServer::imuCallback(const IMU_message &imu_message)
{

  //queue the data:
  pthread_mutex_lock(&i_mutex_);
  imu_queue_.push(imu_message);
  pthread_mutex_unlock(&i_mutex_);

}


//
// VO Callback
//
void ROSServer::visualCallback(const k_message &vo_message)
{
  VOData vo_data(vo_message); //convert to a VOData message
  pthread_mutex_lock(&v_mutex_);
  vo_queue_.push_back(vo_data);
  pthread_mutex_unlock(&v_mutex_);
}


//
// Altitude Callback
//
void ROSServer::altCallback(const sensor_msgs::Range &alt_message)
{
  pthread_mutex_lock(&a_mutex_);
  alt_queue_.push(alt_message);
  pthread_mutex_unlock(&a_mutex_);
}


//
// Truth Callback
//
void ROSServer::truthCallback(const TRUTH_message &truth)
{
  pthread_mutex_lock(&t_mutex_);
  truth_queue_.push(truth);
  pthread_mutex_unlock(&t_mutex_);
}


//
// Hex Callback
//
void ROSServer::hexCallback(const Hex_message &hex_message)
{
  pthread_mutex_lock(&h_mutex_);
  hex_queue_.push(hex_message);
  pthread_mutex_unlock(&h_mutex_);
}


