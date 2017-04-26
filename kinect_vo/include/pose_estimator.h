 /* \copyright This work was completed by Robert Leishman while performing official duties as 
  * a federal government employee with the Air Force Research Laboratory and is therefore in the 
  * public domain (see 17 USC § 105). Public domain software can be used by anyone for any purpose,
  * and cannot be released under a copyright license
  */

/*!
 *  \package kinect_visual_odometry
 *  \file pose_esimator.h
 *  \author Robert Leishman
 *  \date March 2012
 *
 *  \brief This file provides the header for the PoseEstimator class.  This class provides the 6 DOF pose estimates.
 *
 *  \copyright GNU PUblic License
*/

// guard against multiple inclusion
#ifndef POSE_ESTIMATOR_H
#define POSE_ESTIMATOR_H

//
/// \note: Here "EIGEN_NO_DEBUG" is defined to prevent Eigen from doing range checking
#define EIGEN_NO_DEBUG
//

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>
#include <algorithm>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <tr1/unordered_set>
#include <tr1/unordered_map>
#include <boost/lexical_cast.hpp>
#include <sstream>

#include "ransac.h"
#include "image_display.h"
#include "lsh.h"

//#include "g2o/solvers/csparse/g2o_csparse_api.h"
//#include "g2o/core/sparse_optimizer.h"
////#include "g2o/core/graph_optimizer_sparse.h"
//#include "g2o/core/block_solver.h"
//#include "g2o/core/solver.h"
//#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
//#include "g2o/solvers/dense/linear_solver_dense.h"
////#include "g2o/types/icp/types_icp.h"
//#include "g2o/types/sba/types_sba.h"
////#include "g2o/core/structure_only_solver.h"
//#include "g2o/core/hyper_graph.h"





/*!
 *  \class PoseEstimator pose_estimator.h "include/pose_estimator.h"
 *  \brief The PoseEstimator class provides the 6 DOF pose estimates produced using a reference image and the current image.
 *  The reference image is saved and a flag can be set to make the current image the reference image.  Set the flag
 *  repeatedly to perform typical VO or set the flag at every node to do "View Matching".
*/
class PoseEstimator
{

public:

  /*!
   *  \brief The constructor instantiates all the image processing components: feature and descriptor extractors, and
   *  the other classes that provide necessary functions
   *  \todo Fill out the description of the constructor
   *
   *  \param optimize is a flag for whether or not to optimize the transformation using non-linear least squares (g2o)
   *  \param display is a flag for displaying debug images (features, correspondence, etc)
  */
  PoseEstimator(bool optimize, bool display);



  /*!
   *  \brief The destructor
   *  \todo comment anything that is destroyed here...
  */
  ~PoseEstimator();



  /*!
   *  \brief setReferenceView takes in the image information, finds the features and descriptors on the image,
   *  and finds the 3D points.
   *
   *  \attention This routine must be called before "setCurrentView", otherwise the algorithm will not function!
   *
   *  The features are found using the feature descriptor used in a griddetector (to evenly space features over the image)
   *  The 3D points are extracted based on the calibration of the kinect.  If the method "setKinectParameters" is never
   *  called, this method uses the default calibration.
   *
   *  \param visual_image is the color image provided by the kinect.
   *  \param depth_image_float is the depth image off the kinect, that is parameterized using floats.  It is used to calculate
   *  the 3D points.
   *  \param depth_image_CV8UC1 is the reformatted depth image, this is used as a mask in the feature detection algorithm,
   *  that way points without depth information are not found on the image.
  */
  void setReferenceView(cv::Mat &visual_image, cv::Mat &depth_image_float, cv::Mat &depth_image_CV8UC1);



  /*!
   *  \brief setCurrentView does what setReferenceView does along with running the matcher (estimateTransformation).
   *
   *  This method finds the features and descriptors (with the mask provideded by depth_curr_image_CV8UC1), and then
   *  calculates the 3D points using the kinect calibration information.  After this, the "estimateTransformation" function is
   *  called from within, which estimates the tranformation using RANSAC, and if \param optimize is true, will refine the
   *  estimate using g2o.
   *
   *  \param visual_cur_image is the color image provided by the kinect.
   *  \param depth_curr_image_float is the float version of the depth image, used for calculating the 3D points
   *  \param depth_curr_image_CV8UC1 is the depth image used as a mask in the feature detection
   *  \param rotation is the rotation part of the 6DOF transfromation returned by the algorithm in a quaternion
   *  \param translation is the 2nd part of the 6DOF transformation (returned) in a 3D vector
   *  \param covariance is the 7x7 covariance matrix (estimate) of the 6 DOF transformation. Order: [x y z qx qy qz qw]
   *  \param inliers is the number of inliers returned from RANSAC
   *  \param corresponding is the number of corresponding features (desriptors) from the matcher
   *  \param total is the total number of features possible (based on the reference image)
   *  \param setAsReference is a flag for setting this current image as the next reference image.  This occurs at the end
   *  of the function, AFTER the transformation with the established reference is computed.
   *  \param rotation_guess can be provided to help the visual odometry (it adjusts the mask used for matching if there are
   *  larger rotations). The default value is a matrix of zeros.
   *  \param rot_opt is the quaternion found by the optimization
   *  \param tran_opt is the translation found Matby the optimization
   *  \returns zero if not enough features correspond and the outputs should be ignored, one if everything functioned correctly
  */
  int setCurrentAndFindTransform(cv::Mat &visual_cur_image, cv::Mat &depth_curr_image_float, cv::Mat &depth_curr_image_CV8UC1,
                                  Eigen::Quaterniond *rotation, Eigen::Vector3d *translation,
                                  Eigen::Matrix<double,7,7> *covariance, int *inliers, int *corresponding, int *total,
                                  bool setAsReference,Eigen::Quaterniond *rot_opt, Eigen::Vector3d *tran_opt,
                                  cv::Mat rotation_guess = cv::Mat::zeros(3,3,CV_64FC1));



  /*!
   *  \brief The setKinectCalibration method brings in the calibration information for the kinect into this class.
   *
   *  \attention This routine must be called at least once, before "setReferenceView" or "setCurrentView" for the algorithm to function properly!
   *  Alternately, the methods to set the image view could just bring it in with every image, but I wanted to try and avoid
   *  passing more information than necessary.
   *
   *  \param depth_info is the pointer to the depth camera information.
   *  \param rbg_info is the pointer to the rgb camera information.
  */
  void setKinectCalibration(const sensor_msgs::CameraInfoConstPtr &depth_info,
                            const sensor_msgs::CameraInfoConstPtr &rbg_info);


  /// Read access to "reference_set_":
  bool readReferenceSet(){return reference_set_;}

  /*!
   *  \brief Temp function for when we publish two messages for comparing the vo covariance info.  This function
   *  reports what the calc_hess_covariance_ variable is.  If provided an argument, it will modify the variable.
   *  calc_hess_covariance_ is initialiazed to "false"
   *  \param calc is an optional parameter that is used to change whether or not the Hessian approach to the covariance
   *  is also calculated.
  */
  inline bool calcHessianCovariance(bool *calc = NULL)
  {
    if(calc != NULL)
    {
      calc_hess_covariance_ = *calc;
    }
    return calc_hess_covariance_;
  }


  /*!
   * \brief If calc_hess_covariance is true, a covariance will be calculated with the Hessian approach.  It can be
   *  retrieved using this function.
   *  \param hess_covariance will return with the covariance matrix if it was calculated, if not, it returns a null pointer.
  */
  inline void returnHessianCovariance(Eigen::Matrix<double,7,7> *hess_covariance)
  {
    if (calc_hess_covariance_)
    {
      *hess_covariance = hess_covariance_;
    }
    else
    {
      hess_covariance = NULL;
    }
  }


protected:
  //variables

  //reference image member variables
  cv::Mat reference_img_gray_; //!< the reference image, it is a class variable so it can be used over and over
  cv::Mat reference_img_RGB_; //!< the color version of the reference image (just in case)
  cv::Mat reference_img_depth_; //!< the depth image (float)
  std::vector<cv::KeyPoint> reference2D_features_; //!< the 2D features found on the image
  std::vector<cv::Point2f> reference2D_idealized_; //!< the idealized 2D feature locations on the image
  std::vector<cv::Point3d > reference3D_features_; //!< the 3D features for reference
  cv::Mat reference_descriptors_;   //!< the descriptors found for the 2D features
  bool reference_set_; //!< flag for when the reference has been set, if it has not, setCurrentView will stop on an Assert

  //sensor information
  sensor_msgs::CameraInfo depth_info_; //!< the kinect depth camera calibration information
  sensor_msgs::CameraInfo rgb_info_;  //!< the kinect rgb camera calibration information
  cv::Mat rgb_camera_matrix_; //!< the rgb camera matrix parameters to use in undistortPoints
  cv::Mat rgb_camera_P_;  //!< the new projection matrix used by undistortPoints.
  std::vector<double> rgb_camera_distortion_; //!< the distortion coefficients.
  Eigen::Matrix3d image_noise_; //The image noise in pixels and in Z

  //image processing member variables
  cv::GridAdaptedFeatureDetector *grid_detector_; //!< the gridded detector for finding features all over the image
  cv::Ptr<cv::FeatureDetector> feature_detector_ptr_; //!< the pointer to a feature detector;
  //cv::Ptr<cv::DescriptorExtractor> descriptor_extractor_; //!< the pointer to the descriptor detector
  cv::BriefDescriptorExtractor *descriptor_extractor_;   //!< the descriptor extractor, I needed 64 bits, couldn't use general
  //cv::Ptr<cv::DescriptorMatcher> descriptor_matcher_;   /*!< the pointer for the matcher that finds the matching features
   //                                                       between images */
  //cv::BruteForceMatcher<cv::HammingLUT> matcher_forward_;  //!< the matcher for forward finding feature matches between images
  cv::BFMatcher *matcher_forward_;  //!< the matcher for forward finding feature matches between images
  //cv::BruteForceMatcher<cv::HammingLUT> matcher_reverse_; //!< the matcher for reverse feature matching
  cv::BFMatcher *matcher_reverse_; //!< the matcher for reverse feature matching
  //lsh::LshMatcher lsh_matcher_;   //!< a flann-based matcher that should be faster...

  //Optimization stuff:
  bool enable_optimizer_; //!< flag for enabling the optimization
  //g2o::SparseOptimizer optimizer_; //!< the g2o SBA optimizer
  //int point_vertex_offset_; //!< offset for the point vertex id's (initialized in constructor)
  int pose_vertex_id_; //!< id for camera pose verticies (rotation, translation)
  //int max_poses_; //!< max number of poses in the sliding window (approximated) bundle adjustment
  //int num_pose_verticies_; //!< keeps track of the number of camera pose verticies in the optimizer
  //int edge_id_; //!< the id for the edges (I assume that there are < 1000 edges for each camera pose)

  int num_features_; //!< number of max features
  int num_iterations_; //!< number of ransac interations

  //Tunable Params:
  //Eigen::Matrix<double,7,7> image_noise_; //!< matrix of (inverse) noise that is multiplied by the Hessian to calc the covariance
  Eigen::Matrix<double,7,7> deltaI_;  //!< a small amount of identity added to make sure the inverse Hessian converges

  //DEBUG Stuff:
  bool enable_display_; //!< flag for enabling displaying images (and saving correspondence ones)
  std::string MATCHEDWINDOW;
  std::string reference_window_;
  std::string current_window_; //for displaying the current points
  ImageDisplay *association_;
  int counter_; //for counting current images

  //Temp stuff for multiple covariances:
  bool calc_hess_covariance_; //enable calculation of hessian covariance
  Eigen::Matrix<double,7,7> hess_covariance_;

  //methods
  /*!
   *  \brief Calculate an approximate Covariance matrix of the transformation.  This method is the typical approach that
   *  uses the inverted Hessian of the reprojection error.
   *  \param rotation is the rotation matrix estimate
   *  \param translation is the translation vector estimate
   *  \param current2D are the ordered current 2D image points
   *  \param reference3D are the ordered reference 3D points
  */
  Eigen::Matrix<double,7,7> calculateCovariance(Eigen::Quaterniond rotation, Eigen::Vector3d translation,
                                                std::vector<cv::Point3d> reference3D);


  /*!
   *  \brief Calculates an approximate covariance matrix for the transformation.  This method starts with noise in pixels
   *  and noise in Z (camera Z) and brings it all the way up through the chain of equations to the transformation
   *  (including a partial derivative on the SVD).
   *  \param reference_image_pts is the sample of ref image pts (sample of 3 used in solution from RANSAC)
   *  \param urrent_image_pts is the sample of curr image pts (sample of 3 used in solution from RANSAC)
   *  \param reference_3D_pts is the 3D version of the above
   *  \param current_3D_pts is the 3D version
   *  \param reference_cent_3D_pts, the CENTERED sample of the reference 3D points (sample of 3 used in solution from RANSAC)
   *  \param current_cent_3D_pts is the CENTERED sample of the current 3D points (sample of 3 used in solution from RANSAC)
   *  \param reference_centroid_pt The centroid of the reference points
   *  \param current_centroid_pt the centroid of the current points
   *  \param svd_U Matricies from the SVD
   *  \param svd_V Matricies from the SVD
   *  \param svd_D Matricies from the SVD
   *  \param R is the rotation solution
   *  \param T is the translation solution
  */
  Eigen::Matrix<double,7,7> calculateNewCovariance(std::vector<cv::Point2f> &reference_image_pts,
                                                   std::vector<cv::Point2f> &current_image_pts,
                                                   std::vector<cv::Point3d> &reference_3D_pts,
                                                   std::vector<cv::Point3d> &current_3D_pts,
                                                   std::vector<cv::Point3d> &reference_cent_3D_pts,
                                                   std::vector<cv::Point3d> &current_cent_3D_pts,
                                                   cv::Point3d &reference_centroid_pt,
                                                   Eigen::Matrix3d &svd_U,
                                                   Eigen::Matrix3d &svd_V,
                                                   Eigen::Matrix3d &svd_D,
                                                   Eigen::Matrix3d &R,
                                                   Eigen::Vector3d &T);



  /*!
   *  \brief Finds the 3D point locations of the features.
   *
   *  This algorithm uses the raw rgb image points to find the depth Z of the 3D point and the undistorted points to find the
   *  X and Y portions of the 3D point.
   *  \todo Pass in the Descriptors, if features are deleted because of bad depth, delete the corresponding descriptor too!
   *
   *  \param depth_float is the depth image from the kinect that has depth information on it.
   *  \param kinect_calibration is the calibration of the RGB camera of the kinect sensor.
   *  \param features2D is a vector of 2D points that have the feature locations on the image plane, any unvalid points are removed.
   *  \param features2D_undistorted is the vector of 2D features that have been undistored using calibration info and OpenCV's undistortPoints
   *  \param features3D is the vector of 3D points extracted using the depth image, 2D feature locations, and calibration
  */
  void calc3DPoints(cv::Mat &depth_float,
                    sensor_msgs::CameraInfo &kinect_calibration,
                    std::vector<cv::Point2f> *features2D,
                    std::vector<cv::Point2f> *features2D_undistorted,
                    std::vector<cv::Point3d> *features3D);

  /*!
   *  \brief This function is called when the current image should become the reference image.
   *
   *  This function speeds up the process, as features, descriptors, and 3D points have already been extracted from the
   *  current image, transferring them over is faster than recalculating them from the image.
   *
   *  \attention Class variables are edited in this function, if multithreaded is desired, need to handle this appropriately!
   *
   *  \param color_image is the current color image.
   *  \param mono_image is the gray version of the color image.
   *  \param depth_image is the float version of the depth image.
   *  \param features2D are the FAST features extracted from mono_image
   *  \param idealized_pts are the 2D features at the ideal location assuming a pin-hole camera model and the camera parameters
   *  \param features3D are the 3D points of the 2D features
   *  \param descriptors are the BRIEF descriptors extracted around the FAST features.
  */
  void setCurrentAsReference(cv::Mat color_image,
                             cv::Mat mono_image,
                             cv::Mat depth_image,
                             std::vector<cv::KeyPoint> features2D,
                             std::vector<cv::Point2f> idealized_pts,
                             std::vector<cv::Point3d> features3D,
                             cv::Mat descriptors);


  /*!
   *  \brief Paints the features on the image, with the inliers highlighted.
   *
   *  \param ref_features are the features from the reference image (correspoinding points w/ current)
   *  \param cur_features are the features from the current image
   *  \param inliers are the location of the liners
   *  \param image is the image to draw these features on.
   */
  void drawFeatureAssociations(std::vector<cv::Point2f> ref_features, std::vector<cv::Point2f> cur_features,
                               std::vector<int> inliers,cv::Mat image);

  /*!
   *  \brief Used to extract three angles from a rotation matrix
   *
   *  The method does check to make sure that the rotation matrix is not zero.  If it is, roll, pitch, and yaw are returned zero
   *
   *  \param rotation_matrix is a 3x3 rotation matrix from the reference camera frame to the current camera frame
   *  \param roll is the returned roll angle as defined by the 321 Euler angle sequence (w.r.t. an inertial NED frame)
   *  \param pitch is the returned pitch angle as defined by a 321 Euler angle sequence (w.r.t. an inertial NED frame)
   *  \param yaw is the returned yaw angle " " " " .
  */
  void extractAngles(cv::Mat &rotation_matrix, double *roll, double *pitch, double *yaw);


  /*!
   *  \brief Same method as in ROSRelay, only using an Eigen::Quaterniond
   *
   *  \param R is the cv::Mat rotation matrix from RANSAC
   *  \param q is the returned quaternion
  */
  inline void convertRToQuaternion(cv::Mat R, Eigen::Quaterniond *q)
  {
    ROS_ASSERT(R.size() == cv::Size(3,3));

    q->w() = ((1.0/2.0)*sqrt(R.at<double>(0,0) + R.at<double>(1,1) + R.at<double>(2,2) + 1.d));
    q->x() = ((R.at<double>(1,2) - R.at<double>(2,1))/(4.0*q->w()));
    q->y() = ((R.at<double>(2,0) - R.at<double>(0,2))/(4.0*q->w()));
    q->z() = ((R.at<double>(0,1) - R.at<double>(1,0))/(4.0*q->w()));
  }


  /*!
   *  Quick max of 3 numbers: x,y,z
  */
  inline double maximum(double x, double y, double z)
  {
    double max = x;
    if (y > max)
    {
      max = y;
    }
    if (z > max)
    {
      max = z;
    }
    return max;
  }

};

#endif
