 /* \copyright This work was completed by Robert Leishman while performing official duties as 
  * a federal government employee with the Air Force Research Laboratory and is therefore in the 
  * public domain (see 17 USC § 105). Public domain software can be used by anyone for any purpose,
  * and cannot be released under a copyright license
  */

/*! \file navnode.h
  * \author Robert Leishman
  * \date June 2012
  *
  * \brief The navnode.h file is the header for the NavNode class.
*/


#ifndef NAVNODE_H
#define NAVNODE_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/TransformStamped.h>
#include "rel_estimator/navedge.h"
#include "rel_estimator/constants.h"

/*!
 *  \class NavNode navnode.h "include/rel_estimator/navnode.h"
 *  \brief The NavNode class is the container for the nodes of the nodes and edges graph.  Each node is defined by
 *  a keyframe and a relative position (from the starting point).
 *  \note The node coordinate frame is defined with the down direction parrallel to the global down,
 *  therefore roll and pitch are zero when a node is declared.  The roll and pitch angles that define the rotation
 *  between the body-fixed coordinate frame to the node frame are saved with the node in phi_i and theta_i. These angles
 *  comprise the rotation matrix R_node_to_body_.
 *
 *  \attention The node expects that the global coordinate frame is North East Down.
 *
 *  The body-fixed coordinate frame is then related to the camera coordinate frame through a fixed transformation
 *  (found through calibration)
 *
 *  When truth information is available (through motion capture), that data is saved in the node as well.
*/
class NavNode
{
public:
  /// Eigen macro used when there are fixed-sized class member variables and you dynamically create an instance of the
  /// class.  (See: http://eigen.tuxfamily.org/dox/TopicStructHavingEigenMembers.html)
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW


  /*!
   *  \brief The default constructor, matricies are initialized to zero.
   *  \param id_number is the ID for the node.  Nodes will be able to be retrieved using this ID.
  */
  NavNode(int id_number);

  /*!
   *  \brief The constructor used when truth information is available to instantiate the node.
   *  \param truth is the packet of truth information from a motion capture system.
   *  \param id_number is the ID for the node.  Nodes will be able to be retrieved using this ID.
  */
  NavNode(TRUTH_message &truth_data, int id_number);

  /*!
   *  \brief setEstimatePosition sets the estimate of the "global" position and yaw.  Global as it is defined from the
   *  starting point by summing up the edges.
   *  \param position is the 3d vector relative to the starting point (which is relative to the global when an initial
   *  edge is provided)
   *  \param psi is the global psi with respect to the starting point
   *  \param orientation is the quaternion estimate when the node is created.  It is used to assign phi_i and theta_i
  */
  void setEstimatePosition(Eigen::Vector3d &position, double psi_g, Eigen::Quaterniond &orientation);

  /*!
   *  \brief setTruePose sets the true global position and yaw from truth data
   *  \param truth_data is the message from the motion capture
  */
  void setTruePose(TRUTH_message truth_data);

  /*!
   *  \brief Retrieves the Node to Camera Rotation matrix made from phi_i and theta_i
  */
  Eigen::Matrix3d getNodetoBodyRotation(){return R_node_to_body_;}

  /*!
   *  \brief Retrives the true position (when available, if not, it returns NULL)
  */
  Eigen::Vector3d getTruePosition() {if(truth_set_) return true_global_position_; else return true_global_position_.setZero();}

  /*!
   *  \brief Returns the estimated global position of the node
   *  \returns estimated global position
  */
  Eigen::Vector3d getEstimatePosition(){return estimate_global_position_;}

  /*!
   *  \brief Provides read access to the Euler angle phi_i
  */
  double getPhi_i(){return phi_i_;}

  /*!
   *  \brief Read access for the Euler angle theta_i
  */
  double getTheta_i(){return theta_i_;}

  /*!
   *  \brief Read access for the global yaw
  */
  double getPsiGlobal(){return estimate_yaw_;}

  /// \brief Read access for true yaw (for logging)
  double getTrueYaw(){return true_yaw_;}

  bool TruthSet(){return truth_set_;}

  // TEMP: Send out the true orientation. I need to replace this with estimate
  Eigen::Quaterniond getTrueOrientation(){return true_orientation_;}

  /// \brief Read access to the orientation estimate at the node
  Eigen::Quaterniond getEstimateOrientation(){return estimate_orientation_;}

  /// \brief Read access to the node id number
  int getNodeID(){return node_id_;}

protected:


  /*!
   *  \brief This function uses phi_i and theta_i to create the rotation matrix R_node_to_body
   *  Essentially, this matrix transforms points, expressed in the node frame, to the body frame.
  */
  void createRotationMatrix();


  bool truth_set_; //!< variable to keep track of when truth data is set for the node
  double phi_i_, theta_i_;  //!< the Euler angles between the node coordinate system and the keyframe image coord system
  Eigen::Vector3d true_global_position_; //!< the true global position, as measured by some motion capture
  Eigen::Quaterniond true_orientation_; //!< the true orientation of the keyframe
  Eigen::Vector3d estimate_global_position_; //!< the estimated global position
  Eigen::Quaterniond estimate_orientation_; //!< estimated orientation
  double estimate_yaw_; //!< the estimated global yaw
  double true_yaw_; //!< the true yaw
  int node_id_; //!< the id of the node
  Eigen::Matrix3d R_node_to_body_; //!< the rotation matrix using phi_i and theta_i that goes from the node frame to
  //! the body-fixed frame when the keyframe image was taken
};

#endif // NAVNODE_H
