 /* \copyright This work was completed by Robert Leishman while performing official duties as 
  * a federal government employee with the Air Force Research Laboratory and is therefore in the 
  * public domain (see 17 USC § 105). Public domain software can be used by anyone for any purpose,
  * and cannot be released under a copyright license
  */

#include "rel_estimator/constants.h"

using namespace Eigen;

Constants::Constants()
{
  //Vision platform constants :
  // Calibration Constants: the static transformation from the camera coordinate frame to the body-fixed coordinate frame.
  q_camera_to_body.x() = qx;
  q_camera_to_body.y() = qy;
  q_camera_to_body.z() = qz;
  q_camera_to_body.w() = qw;
  T_camera_to_body << 1.0*cx, 1.0*cy, 1.0*cz; //Offset of left camera focal point in body-fixed frame coordinates (x,y,z)
}

//Matrix3d Constants::cameraToBodyFixedRotation(double psi)
//{
//  Matrix3d rotation,rotation2,rotation3;

//  rotation << cos(psi), sin(psi), 0,
//             -sin(psi), cos(psi), 0,
//              0, 0, 1;

//  rotation2 << 0,0,1,
//               1,0,0,
//               0,1,0;

//  rotation3 = rotation * rotation2;

//  return rotation3;
//}
