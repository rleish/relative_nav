 /* \copyright This work was completed by Robert Leishman while performing official duties as 
  * a federal government employee with the Air Force Research Laboratory and is therefore in the 
  * public domain (see 17 USC § 105). Public domain software can be used by anyone for any purpose,
  * and cannot be released under a copyright license
  */

/*!
 *  \file hex_planner.cpp
 *  \author Robert Leishman
 *  \date October 2012
*/


#include "hex_planner/hex_planner.h"

using namespace Eigen;

//
// Constructor
//
HexPlanner::HexPlanner(ros::NodeHandle &nh, costmap_2d::Costmap2DROS *costmp)
{
  std::string goal_name, planner_name,edge_topic;
  ros::param::param<std::string>("~goal_topic_name", goal_name,"/hex_goal" );
  ros::param::param<std::string>("~planner_name", planner_name,"hex_planner" );
  ros::param::param<double>("~max_path_length",max_path_length_, 2 );
  ros::param::param<bool>("~use_relative_planning", relative_plans_,"true");
   ros::param::param<std::string>("~edge_topic", edge_topic, "/relative/cur_edge/pose");

  /*!
    \note Below are the private parameters that are available to change through the param server:
     \code{.cpp}
  ros::param::param<std::string>("~goal_topic_name", goal_name,"/hex_goal" ); //!< topic that the goal location is published on
  ros::param::param<std::string>("~planner_name", planner_name,"hex_planner" ); //!< name for the planner (need this name to access its parameters)
  ros::param::param<double>("~max_path_length",max_path_length_, "2" ); //!< Don't plan farther than x meters away
  ros::param::param<bool>("~use_relative_planning", rel_planner,"true"); //!< if using relative states, need to modify goal
     \endcode
  */

  initial_goal_location_recieved_ = false;

  goal_subscriber_ = nh.subscribe(goal_name,1,&HexPlanner::recieveGoalLocation,this);

  if(relative_plans_)
  {
    edge_sub_ = nh.subscribe(edge_topic,1,&HexPlanner::applyEdgeFromNewNode,this);
    ROS_WARN("PLANNER: Making RELATIVE Plans!");
  }
  else
  {
    ROS_WARN("PLANNER: Global Plans Being Computed!");
  }

  costmap_ = costmp;
  planner_.initialize(planner_name, costmap_);


  //DEBUG!!!
  std::cout << "Base frame ID in costmap is " << costmap_->getBaseFrameID() << std::endl;
  goal_location_.header.stamp = ros::Time::now();
  goal_location_.pose.position.x = 1.0;
  goal_location_.pose.position.y = -0.50;
  goal_location_.pose.position.z = 0.75;
  //initial_goal_location_recieved_ = true;
  //DEBUG!!!
}


//
// Destructor
//
HexPlanner::~HexPlanner()
{
  delete costmap_;
}



//
// Update the plan - do the work for this project
//
bool HexPlanner::updatePlan()
{
  //The costmap will be running on it's own thread, bringing in data at the rate set in the parameters.
  //The NavfnROS class accesses the costmap, as we sent in a pointer to it in the constructor.  Use the
  //NavfnROS functions to update everything:

  std::vector<geometry_msgs::PoseStamped> plan;

  if(costmap_ == NULL)
  {
    ROS_ERROR_NAMED("PLANNER","Cannot create a plan, there is not a cost map!!");
    return false;
  }


  tf::Stamped<tf::Pose> current_pose;
  if(!costmap_->getRobotPose(current_pose))
  {
    ROS_WARN_NAMED("PLANNER","Unable to get starting pose of the robot, unable to create a global plan");
    return false;
  }

  geometry_msgs::PoseStamped start;
  tf::poseStampedTFToMsg(current_pose, start);

  if(!planner_.makePlan(start, goal_location_,max_path_length_,plan) || plan.empty())
  {
    ROS_DEBUG_NAMED("PLANNER", "Failed to find a plan to point (%.2f, %.2f",goal_location_.pose.position.x,
                    goal_location_.pose.position.y );
    return false;
  }

  planner_.publishPlan(plan,255,0,0,0);
  last_plan_timestamp_ = ros::Time::now();

  return true;
}



//
// Callback for the goal locations
//
void HexPlanner::recieveGoalLocation(const geometry_msgs::PoseStamped goal_loc)
{
  if(!initGoalLocationRecieved())
  {
    initial_goal_location_recieved_ = true;
  }

  goal_location_ = goal_loc;

  //Have a new goal, create a new plan (in either relative or global)
  bool flag = updatePlan();

  if(!flag)
  {
    flag = updatePlan(); //immediately try again
  }
}


//
// Apply an edge to the goal location
//
void HexPlanner::applyEdgeFromNewNode(const rel_MEKF::edge &edge_message)
{
  if(relative_plans_)
  {
    //The edge is in NED coordinate system, the goal location needs to be in NWU:
    Vector3d translation(edge_message.translation.x,-edge_message.translation.y,-edge_message.translation.z);
    Vector3d temp,goal;
    Matrix3d rotation;
    goal << goal_location_.pose.position.x,goal_location_.pose.position.y,goal_location_.pose.position.z;
    // Negate the yaw to turn it into NWU:
    rotation << cos(-edge_message.yaw),sin(-edge_message.yaw),0,
               -sin(-edge_message.yaw),cos(-edge_message.yaw),0,
                0,0,1;

    /// NOTE: CHECK THIS ROTATION TO SEE IF IT IS RIGHT!
    temp = rotation*(-translation + goal);
    goal_location_.pose.position.x = temp(0);
    goal_location_.pose.position.y = temp(1);
    goal_location_.pose.position.z = temp(2);

    Quaterniond q(rotation),
        g_q(goal_location_.pose.orientation.w,goal_location_.pose.orientation.x,
            goal_location_.pose.orientation.y,goal_location_.pose.orientation.z);
    g_q = q*g_q; //new goal location orientation
    goal_location_.pose.orientation.x = g_q.x();
    goal_location_.pose.orientation.y = g_q.y();
    goal_location_.pose.orientation.z = g_q.z();
    goal_location_.pose.orientation.w = g_q.w();

    //Have a new goal, create a new plan
    bool flag = updatePlan();

    if(!flag)
    {
      flag = updatePlan(); //immediately try again
    }
  }


}

