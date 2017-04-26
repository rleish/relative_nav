 /* \copyright This work was completed by Robert Leishman while performing official duties as 
  * a federal government employee with the Air Force Research Laboratory and is therefore in the 
  * public domain (see 17 USC § 105). Public domain software can be used by anyone for any purpose,
  * and cannot be released under a copyright license
  */

/*! \file navedge.h
  * \author Robert Leishman
  * \date June 2012
  *
  * \brief The navedge.h file is the header for the NavEdge class.
*/


#ifndef NAVEDGE_H
#define NAVEDGE_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/TransformStamped.h>
#include "rel_estimator/constants.h"

/*!
 *  \class NavEdge navedge.h "include/rel_estimator/navedge.h"
 *  \brief The NavEdge class provides edges between keyframe nodes.  It provdes the translation (and covariance) betweeen
 *  nodes, and the yaw angle difference between the two (psi_i).  Also, the rotation matrix (using psi_i) is provided
 *  by this class as well (R_curr_next).
 *
*/
class NavEdge
{
public:
  /// Eigen macro used when there are fixed-sized class member variables and you dynamically create an instance of the
  /// class.  (See: http://eigen.tuxfamily.org/dox/TopicStructHavingEigenMembers.html)
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW


    /*!
     *  \brief Simple constructor without initializations.
    */
    inline NavEdge()
            {translation_.setZero();cov_translation_.setZero();R_Curr_Next_.setZero();
            psi_i_=0;node_from_id_=0;node_to_id_=0;}

    /*!
     *  \brief Constructor that is used when state and covariance estimates are provided.  This is the most common
     *  way to initialize an NavEdge.  Because of the relative nature of the state, it provides an edge between nodes
     *  directly.
     *
     *  \param state is the current state vector from the filter
     *  \param cov is the current covariance matrix from the filter
     *  \param from_id is the node id that is the base for this edge (from node "from_id")
     *  \param to_id is the node id to which this edge points (to node "to_id")
    */
    NavEdge(Eigen::Matrix<double,STATE_LENGTH,1> &state, Eigen::Matrix<double,COVAR_LENGTH,COVAR_LENGTH> &cov, int from_id, int to_id);


    /*!
     *  \brief This constructor is used usually only once, at the beginning, when truth information is available.  This
     *  edge links the relative map to the global coordinate system.
     *
     *  \param truth_data is a packet of motion capture data
    */
    NavEdge(TRUTH_message &truth_data, int from_id, int to_id);


    /*!
     *  \brief This function returns the Euler angle sequence 321 (Yaw,Pitch,Roll) from a quaternion
     *  This if from Jack Kuipers book on Quaternions
     *  \returns the 3 Euler angles in a vector as [roll,pitch,yaw] i.e. [phi,theta,psi]
    */
    Eigen::Vector3d computeEulerFromQuat(Eigen::Quaterniond &orientation);


    /*!
     *  \brief Provides read access to translation
    */
    Eigen::Vector3d getTranslation(){return translation_;}


    /*!
     *  \brief Provides read access to the translation covariance
    */
    Eigen::Matrix3d getTranslationCovariance(){return cov_translation_;}


    /*!
     * \brief Provides access to the rotation matrix R_curr_next which is the rotation matrix of the relative yaw psi_i
    */
    Eigen::Matrix3d getR_curr_next(){return R_Curr_Next_;}


    /*!
     *  \brief Retrives the relative yaw angle, psi_i
    */
    double getPsi_i(){return psi_i_;}


    /*!
     *  \brief Provides read access to the node_from_id
    */
    int getFromID(){return node_from_id_;}


    /*!
     *  \brief Provides read access to the node_to_id
    */
    int getToID(){return node_to_id_;}



//Don't know if I need these or not yet:
//    void setPsi_i(Eigen::Quaterniond orientation);
//    void setR_curr_next(Eigen::MatrixXd value);
//    void setTranslationCovariance(Eigen::Matrix<double,15,15> covariance);
//    void setTranslation(Eigen::Matrix<double,15,1> state);

protected:

    /*!
     *  \brief Using a quaternion, this function extracts the yaw rotation matrix for the relative yaw component.
     *  The function also sets psi_i (the class variable)
     *  \param orientation is the 3D orientation quaternion
    */
    void yawRotation(Eigen::Quaterniond &orientation);


    Eigen::Vector3d translation_;  //!< the 3D translation between the "from" node and the "to" node, expressed in the
                                  //!< "from" node coordinate frame.
    Eigen::Matrix3d cov_translation_; //!< the covariance on the translation
    //! \todo GET COVARIANCE ON THE YAW ROTATION!!!
    Eigen::Matrix3d R_Curr_Next_;  //!< the yaw-based right-handed rotation expressed as the angle from the the "from"
                                  //!< node f axis to the "to" node f axis.
    double psi_i_; //!< the relative yaw angle.
    int node_from_id_; //!< the node id that is the origin of this edge
    int node_to_id_; //!< the node id to which this edge points

};

#endif // NAVEDGE_H
