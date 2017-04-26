 /* \copyright This work was completed by Robert Leishman while performing official duties as 
  * a federal government employee with the Air Force Research Laboratory and is therefore in the 
  * public domain (see 17 USC § 105). Public domain software can be used by anyone for any purpose,
  * and cannot be released under a copyright license
  */

#include "rel_estimator/navedge.h"

using namespace Eigen;

//
//  Constructor for state and covariance data
//
NavEdge::NavEdge(Matrix<double, STATE_LENGTH, 1> &state, Matrix<double, COVAR_LENGTH, COVAR_LENGTH> &cov,
                 int from_id, int to_id):node_from_id_(from_id), node_to_id_(to_id)
{
    translation_ << state(0,0), state(1,0), state(2,0);
    //translation(2, 0) = 0.0; //for global down
    cov_translation_ << cov(0,0), cov(0,1), cov(0,2),
                       cov(1,0), cov(1,1), cov(1,2),
                       cov(2,0), cov(2,1), cov(2,2);
    Quaterniond temp;
    temp.x() = state(3,0);
    temp.y() = state(4,0);
    temp.z() = state(5,0);
    temp.w() = state(6,0);

    yawRotation(temp);
}


//
//  Constructor for truth data
//
NavEdge::NavEdge(TRUTH_message &truth_data, int from_id, int to_id)
  :node_from_id_(from_id), node_to_id_(to_id)
{
    translation_ << truth_data.transform.translation.x, truth_data.transform.translation.y,
                   truth_data.transform.translation.z;

    cov_translation_ << 0.00000001, 0, 0,
                       0, 0.00000001, 0,
                       0, 0, 0.00000001; //make it very certain of where it is!

    Quaterniond temp;
    temp.x() = truth_data.transform.rotation.x;
    temp.y() = truth_data.transform.rotation.y;
    temp.z() = truth_data.transform.rotation.z;
    temp.w() = truth_data.transform.rotation.w;

    yawRotation(temp);
}



//
// Calc the Euler angles from quaternion
//
Vector3d NavEdge::computeEulerFromQuat(Quaterniond &q)
{    
  Vector3d temp;  
  temp(0) = atan2(2.0*(q.w()*q.x() + q.y()*q.z()), (q.w()*q.w() + q.z()*q.z() - q.x()*q.x() - q.y()*q.y()));  //roll (phi)
  temp(1) = asin(2.0*(q.w()*q.y() - q.x()*q.z())); //pitch (theta)
  temp(2) = atan2(2.0*(q.w()*q.z() + q.x()*q.y()), (q.w()*q.w() +q.x()*q.x() - q.y()*q.y() - q.z()*q.z())); //yaw (psi)
  return temp;
}


//
//  Calc the yaw rotation matrix
//
void NavEdge::yawRotation(Quaterniond &orientation)
{
  Vector3d angles = computeEulerFromQuat(orientation);

  psi_i_ = angles(2);

  R_Curr_Next_ << cos(psi_i_), sin(psi_i_), 0,
                 -sin(psi_i_),cos(psi_i_), 0,
                 0,0,1;
}



