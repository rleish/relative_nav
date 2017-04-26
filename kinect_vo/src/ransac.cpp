 /* \copyright This work was completed by Robert Leishman while performing official duties as 
  * a federal government employee with the Air Force Research Laboratory and is therefore in the 
  * public domain (see 17 USC § 105). Public domain software can be used by anyone for any purpose,
  * and cannot be released under a copyright license
  */

/*!
 *  \file ransac.cpp
 *  \author Robert Leishman
 *  \date March 2012
 *
 *  \brief This implements the methods outlined in ransac.h
*/

#include "ransac.h"

using namespace cv;

//
//the contructor, initialize everything
//
RANSAC::RANSAC(int iters,
               int inlier_thres,
               double consensus_thres,
               sensor_msgs::CameraInfo &camera_params,
               bool optimize
  ): iterations_(iters),
  inlier_threshold_(inlier_thres),
  consensus_threshold_(consensus_thres),
  optimizer_enabled_(optimize)
{
  //initialize the camera intrinisics: (These are the intrinsics for the RBG camera)
  //camera_intrinsics = Mat::zeros(3,4, DataType<float>::type);
//  rgb_camera_intrinsics = (Mat_<double>(3,3) <<  camera_params.P[0], 0, camera_params.P[2],
//                                              0, camera_params.P[5], camera_params.P[6],
//                                              0, 0, 1);
  rgb_camera_intrinsics = (Mat_<double>(3,3) <<  camera_params.K[0], 0, camera_params.K[2],
                                                 0, camera_params.K[4], camera_params.K[5],
                                                 0, 0, 1);

  rgb_distortion = (Mat_<double>(5,1) << camera_params.D[0], camera_params.D[1], camera_params.D[2],
                camera_params.D[3], camera_params.D[4]);

  rnd_gen = new RNG();  //instatiate the random number generator

}


//
//put stuff here when it should be destroyed!
//
RANSAC::~RANSAC()
{

}




//
//run RANSAC!
//
void RANSAC::runRANSAC(std::vector<Point3d> &reference3D,
                       std::vector<Point3d> &current3D,
                       std::vector<cv::Point2f> &current2D,
                       Mat *final_rotation,
                       Mat *final_translation,
                       int *inliers,
                       std::vector<int> *inlier_list,
                       std::vector<int> *solution_list,
                       Mat *svd_D, Mat *svd_U, Mat *svd_V)
{
  //check dimensions:
  ROS_ASSERT(reference3D.size() == current3D.size());
  ROS_ASSERT(current3D.size() == current2D.size());
  ROS_ASSERT(reference3D.size() > 3);  //need more than three matching features

  double best_total_error = 99999999999999999999.9; //something high...
  int best_size = 3; //keeping track of most inliers
  std::vector<double> best_errors; //best vector showing the error
  std::vector<int> best_inliers; //initialize to three numbers...
  best_inliers.push_back(1);
  best_inliers.push_back(2);
  best_inliers.push_back(3);
  std::vector<int> best_solution_list; //list of indicies of the pts used in the best solution
//  Mat D_best,U_best,V_best; //SVD matricies from best solution

  //containers for the best solution:
  Mat best_rotation;
  Mat best_translation;


  //main loop for RANSAC:
  #pragma omp parallel for shared( best_total_error, best_size,best_solution_list)
  for(int i = 0; i < iterations_; i++)
  {
    //containers for the solution at each iteration
    Mat rotation;
    Mat translation;
//    Mat D,V,U; //matricies from the SVD
    std::vector<double> error_temp;  //container for the error
    std::vector<int> inliers_temp;   //container for the inliers
    double temp_err; //sum of the error at each iteration    
    std::vector<Point3d> reference_sample; //the container for the sample of the reference
    std::vector<Point3d> current_sample; //the container for the sample of the current

    //Containers for the P3P solution
    std::vector<Point2d> current2D_sample; //the container for the sample of the current 2D
    //std::vector<Mat> rotation_P3P, translation_P3P;

    std::vector<int> list; //the container for the indicies (to be passed on for covariance estimation)

    //sample
    sample(reference3D, current3D, current2D, &reference_sample, &current_sample, &current2D_sample, &list);

    //find a solution using the 3 sampled points
    computeSampleTransformation(reference_sample, current_sample, &rotation, &translation);

    //P3P Method:
    //computeKneipP3P( reference_sample,current2D_sample,&rotation_P3P,&translation_P3P);
    //temp_err = computeErrorModelP3P(reference3D, current2D, &rotation_P3P, &translation_P3P, &error_temp, &inliers_temp);    

    //check the error against all the points
    temp_err = computeErrorModel(reference3D, current2D, rotation, translation, &error_temp, &inliers_temp);    

    //check to see if it is the best so far or better than our minimum threshold
    //two conditions, (better error and same or greater # inliers) OR (greater # inliers)
    #pragma omp critical
    if((temp_err < best_total_error && (int)inliers_temp.size() >= best_size) || (int)inliers_temp.size() > best_size)
    {
      //Have an improved guess, replace with new information:
      best_size = (int)inliers_temp.size();
      best_total_error = temp_err;
      best_errors = error_temp;
      best_inliers = inliers_temp;
      best_rotation = rotation.clone();
      best_translation = translation.clone();
      best_solution_list = list;
//      D_best = D;
//      U_best = U;
//      V_best = V;
    }
  }
  //End of main loop

//  *svd_D = D_best;
//  *svd_U = U_best;
//  *svd_V = V_best;

//  //
//  //Find the inlier that is furthest from the plane defined from the 3 points, and return that least-squares solution

//  Vec3d A(reference3D[best_solution_list[0]].x,reference3D[best_solution_list[0]].y,reference3D[best_solution_list[0]].z);
//  Vec3d B(reference3D[best_solution_list[1]].x,reference3D[best_solution_list[1]].y,reference3D[best_solution_list[1]].z);
//  Vec3d C(reference3D[best_solution_list[2]].x,reference3D[best_solution_list[2]].y,reference3D[best_solution_list[2]].z);

//  Vec3d N,n;
//  N = (A-B).cross(A-C); //plane normal eqn.
//  n = N*(1.0/(double)norm(N));
//  double biggest = 0.0;
//  int inlierpick = 0;

//  for(int i = 0; i<(int)best_inliers.size(); i++)
//  {
//    Vec3d D(reference3D[best_inliers[i]].x,reference3D[best_inliers[i]].y,reference3D[best_inliers[i]].z);
//    double d = fabs(n.dot(A-D));
//    if(d > biggest)
//    {
//      biggest = d;
//      inlierpick = i;
//    }
//  }
//  best_solution_list.push_back(inlierpick);

//  //Use these 4 (non-planar) pts to create a least-squares solution to return:
//  std::vector<Point3d> best_reference;
//  std::vector<Point3d> best_current;
//  for(int i = 0; i < (int)best_solution_list.size(); i++)
//  {
//    best_reference.push_back(reference3D[best_solution_list[i]]);
//    best_current.push_back(current3D[best_solution_list[i]]);
//  }

//  Mat final_rot, final_trans, D1,V1,U1; //matricies from the SVD;

//  computeSampleTransformation(best_reference, best_current, &final_rot, &final_trans,&D1,&U1,&V1);

//  std::cout << D << std::endl;

  //
  //Use all the inliers to generate the SVD terms (for the uncertainty):
    std::vector<Point3d> best_reference;
    std::vector<Point3d> best_current;
  for(int i = 0; i < (int)best_inliers.size(); i++)
  {
    best_reference.push_back(reference3D[best_inliers[i]]);
    best_current.push_back(current3D[best_inliers[i]]);
  }

  Mat final_rot, final_trans, D,V,U; //matricies from the SVD;

  computeSampleTransformation(best_reference, best_current, &final_rot, &final_trans,&D,&U,&V);

  *svd_D = D;
  *svd_U = U;
  *svd_V = V;

//  std::cout << D1 << " " << D << std::endl;

  //copy over the solution from least-squares solution using all inliers
//  final_rot.copyTo(*final_rotation);
//  final_trans.copyTo(*final_translation);

//  //use the 3pt solution:
  best_rotation.copyTo(*final_rotation);
  best_translation.copyTo(*final_translation);

//  if(!optimizer_enabled_)
//  {
//    //Use all the inliers to create a least-squares solution to return:
//    std::vector<Point3d> best_reference;
//    std::vector<Point3d> best_current;
//    for(int i = 0; i < (int)best_inliers.size(); i++)
//    {
//      best_reference.push_back(reference3D[best_inliers[i]]);
//      best_current.push_back(current3D[best_inliers[i]]);
//    }

//    Mat final_rot, final_trans, D,V,U; //matricies from the SVD;

//    computeSampleTransformation(best_reference, best_current, &final_rot, &final_trans,&D,&U,&V);

//    *svd_D = D;
//    *svd_U = U;
//    *svd_V = V;

//    //copy over the solution
//    final_rot.copyTo(*final_rotation);
//    final_trans.copyTo(*final_translation);

//    best_rotation.copyTo(*final_rotation);
//    best_translation.copyTo(*final_translation);
//  }
//  else
//  {
//    best_rotation.copyTo(*final_rotation);
//    best_translation.copyTo(*final_translation);
//  }

  *solution_list = best_solution_list;
  *inliers = (int)best_inliers.size(); //report # inliers
  *inlier_list = best_inliers;

  //ROS_INFO_THROTTLE(1,"RANSAC Inliers = %d", *inliers); //advertise the # of inliers
}



//
//For computing the inliers and the error from the proposed model (called within runRANSAC)
//
double RANSAC::computeErrorModel(std::vector<Point3d> &reference3D,
                                  std::vector<cv::Point2f> &current2D,
                                  Mat &rotation,
                                  Mat &translation,
                                  std::vector<double> *error,
                                  std::vector<int> *inliers)
{
  Mat rodrigues_rotation;
  Rodrigues(rotation,rodrigues_rotation); //change the rotation to a Rodrigues rotation (3 parameters)
  std::vector<Point2d> predicted_pts;
  double err;  //error  //temp points px, py,
  double error_total = 0;

  //This function projects the reference 3D points onto the current image plane
  projectPoints(reference3D,rodrigues_rotation,translation,rgb_camera_intrinsics,rgb_distortion,predicted_pts);

  //go through each element and compute the error
  for(int i = 0; i < (int)predicted_pts.size(); i++)
  {
    err = (current2D[i].x - predicted_pts[i].x)*(current2D[i].x - predicted_pts[i].x) +
          (current2D[i].y - predicted_pts[i].y)*(current2D[i].y - predicted_pts[i].y);

    error->push_back(err);  //!< \note The error is squared, not square-rooted!
    error_total += err; //sum up the error for all the points, not just the inliers!

    if (err <= inlier_threshold_)
    {
      //we have an inlier!
      inliers->push_back(i);
    }
  }

  return error_total;
}


//
//given a sample, computes the sample solution using the SVD technique
//
void RANSAC::computeSampleTransformation(std::vector<Point3d> &reference3D,
                                         std::vector<Point3d> &current3D,
                                         Mat *rotation, Mat *translation,
                                         cv::Mat *D, cv::Mat *U, cv::Mat *V)
{
  std::vector<Point3d> centered_reference; //The reference3D w/out mean
  std::vector<Point3d> centered_current;  //the current3D w/out mean
  Matx31d centroid_reference(0.d, 0.d, 0.d), centroid_current(0.d, 0.d, 0.d);; //the mean/centriods
  Mat R; //rotation matrix

  //calc centriods
  findCentroid(reference3D, &centered_reference, &centroid_reference);
  findCentroid(current3D, &centered_current, &centroid_current);

  //calculate the 3x3 matrix H
  Mat H, HU,HVt,w;
  H = Mat::zeros(3,3, DataType<double>::type);

  //Make a matrix out of the vectors of points, resize to Nx3
  Mat pts_Mat_ref = Mat(centered_reference).reshape(1);
  Mat pts_Mat_cur = Mat(centered_current).reshape(1);

  //Matrix form of the sum of inner products
  H = pts_Mat_ref.t()*pts_Mat_cur;

  //compute the SVD:
  SVD::compute(H,w,HU,HVt,SVD::MODIFY_A);   //V comes out as V.transpose (V.t), be aware of this.  

  R = HVt.t()*HU.t();

  //check determinate and find the rotation
  double det_R;
  det_R = determinant(R);

  ROS_ASSERT(!std::isnan(det_R));

  if(det_R < 0)
  {
    //sign of last column of V needs to be switched:
    Mat v_temp;
    v_temp = HVt.t();
    v_temp.col(2) *= -1.0;
    //recompute
    R = v_temp * HU.t();
  }

  //Compute the translation
  Mat T;
  T = Mat(centroid_current) - R*Mat(centroid_reference);

  *rotation = R;
  *translation = T;
  if(D != NULL)
  {
    *D = w;
    *U = HU;
    *V = HVt.t();
  }
}




////
//// P3P algroithm by Laurent Kneip
////
int RANSAC::computeKneipP3P(std::vector<cv::Point3d> &reference3D,
                             std::vector<cv::Point2d> &current2D,
                             std::vector<cv::Mat> *rotation,
                             std::vector<cv::Mat> *translation)
{
  // Extraction of world points

  Mat_<double> P1(3,1),P2(3,1),P3(3,1);
  P1 << reference3D[0].x,reference3D[0].y,reference3D[0].z;
  P2 << reference3D[1].x,reference3D[1].y,reference3D[1].z;
  P3 << reference3D[2].x,reference3D[2].y,reference3D[2].z;

  std::cout << "Point 1 " << P1 << std::endl;
  std::cout << "Point 2 " << P2 << std::endl;
  std::cout << "Point 3 " << P3 << std::endl;

  // Verification that world points are not colinear

  Mat_<double> temp1 = P2 - P1;
  Mat_<double> temp2 = P3 - P1;

  if(norm(temp1.cross(temp2)) == 0)
    return -1;

  // Use 2D points and focal point to creat the feature vectors:
  Mat_<double> f1(3,1),f2(3,1),f3(3,1);
  f1 << current2D[0].x,current2D[0].y,rgb_camera_intrinsics.at<double>(0,0); //use the x axis focal length
  f2 << current2D[1].x,current2D[1].y,rgb_camera_intrinsics.at<double>(0,0);
  f3 << current2D[2].x,current2D[2].y,rgb_camera_intrinsics.at<double>(0,0);

  std::cout << "Vector 1 " << f1 << std::endl;
  std::cout << "Vector 2 " << f2 << std::endl;
  std::cout << "Vector 3 " << f3 << std::endl;

  //Normalize for unit vectors:
  f1 = f1/norm(f1);
  f2 = f2/norm(f2);
  f3 = f3/norm(f3);

  std::cout << "Vector 1 " << f1 << std::endl;
  std::cout << "Vector 2 " << f2 << std::endl;
  std::cout << "Vector 3 " << f3 << std::endl;

  // Creation of intermediate camera frame
  Mat_<double> e1(3,1),e2(3,1),e3(3,1);
  e1 = f1;
  e3 = f1.cross(f2);
  e3 = e3 / norm(e3);
  e2 = e3.cross(e1);

  std::cout << "Vector 1 " << e1 << std::endl;
  std::cout << "Vector 2 " << e2 << std::endl;
  std::cout << "Vector 3 " << e3 << std::endl;

  Mat_<double> T(3,3);
  T.row(0) = e1.t();
  T.row(1) = e2.t();
  T.row(2) = e3.t();

  std::cout << "T " << T << std::endl;

  f3 = T*f3;

  // Reinforce that f3[2] > 0 for having theta in [0;pi]

  if( f3(2,0) > 0 )
  {
    f2 << current2D[0].x,current2D[0].y,rgb_camera_intrinsics.at<double>(0,0); //use the x axis focal length
    f1 << current2D[1].x,current2D[1].y,rgb_camera_intrinsics.at<double>(0,0);
    f3 << current2D[2].x,current2D[2].y,rgb_camera_intrinsics.at<double>(0,0);

    //Normalize for unit vectors:
    f1 = f1/norm(f1);
    f2 = f2/norm(f2);
    f3 = f3/norm(f3);

    e1 = f1;
    e3 = f1.cross(f2);
    e3 = e3 / norm(e3);
    e2 = e3.cross(e1);

    T.row(0) = e1.t();
    T.row(1) = e2.t();
    T.row(2) = e3.t();

    f3 = T*f3;

    P2 << reference3D[0].x,reference3D[0].y,reference3D[0].z;
    P1 << reference3D[1].x,reference3D[1].y,reference3D[1].z;
    P3 << reference3D[2].x,reference3D[2].y,reference3D[2].z;
  }

  // Creation of intermediate world frame

  Mat_<double> n1(3,1),n2(3,1),n3(3,1);
  n1 = P2-P1;
  n1 = n1 / norm(n1);
  n3 = n1.cross(P3-P1);
  n3 = n3 / norm(n3);
  n2 = n3.cross(n1);

  Mat_<double> N(3,3);
  N.row(0) = n1.t();
  N.row(1) = n2.t();
  N.row(2) = n3.t();

  // Extraction of known parameters

  P3 = N*(P3-P1);

  double d_12 = norm(P2-P1);
  double f_1 = f3(0,0)/f3(2,0);
  double f_2 = f3(1,0)/f3(2,0);
  double p_1 = P3(0,0);
  double p_2 = P3(1,0);

  double cos_beta = f1.dot(f2);
  double b = 1/(1-pow(cos_beta,2)) - 1;

  if (cos_beta < 0)
    b = -sqrt(b);
  else
    b = sqrt(b);

  // Definition of temporary variables for avoiding multiple computation

  double f_1_pw2 = pow(f_1,2);
  double f_2_pw2 = pow(f_2,2);
  double p_1_pw2 = pow(p_1,2);
  double p_1_pw3 = p_1_pw2 * p_1;
  double p_1_pw4 = p_1_pw3 * p_1;
  double p_2_pw2 = pow(p_2,2);
  double p_2_pw3 = p_2_pw2 * p_2;
  double p_2_pw4 = p_2_pw3 * p_2;
  double d_12_pw2 = pow(d_12,2);
  double b_pw2 = pow(b,2);

  // Computation of factors of 4th degree polynomial
  Mat_<double> factors(5,1);

  factors(0,0) = -f_2_pw2*p_2_pw4
         -p_2_pw4*f_1_pw2
         -p_2_pw4;

  factors(1,0) = 2*p_2_pw3*d_12*b
         +2*f_2_pw2*p_2_pw3*d_12*b
         -2*f_2*p_2_pw3*f_1*d_12;

  factors(2,0) = -f_2_pw2*p_2_pw2*p_1_pw2
           -f_2_pw2*p_2_pw2*d_12_pw2*b_pw2
         -f_2_pw2*p_2_pw2*d_12_pw2
         +f_2_pw2*p_2_pw4
         +p_2_pw4*f_1_pw2
         +2*p_1*p_2_pw2*d_12
         +2*f_1*f_2*p_1*p_2_pw2*d_12*b
         -p_2_pw2*p_1_pw2*f_1_pw2
         +2*p_1*p_2_pw2*f_2_pw2*d_12
         -p_2_pw2*d_12_pw2*b_pw2
         -2*p_1_pw2*p_2_pw2;

  factors(3,0) = 2*p_1_pw2*p_2*d_12*b
         +2*f_2*p_2_pw3*f_1*d_12
         -2*f_2_pw2*p_2_pw3*d_12*b
         -2*p_1*p_2*d_12_pw2*b;

  factors(4,0) = -2*f_2*p_2_pw2*f_1*p_1*d_12*b
         +f_2_pw2*p_2_pw2*d_12_pw2
         +2*p_1_pw3*d_12
         -p_1_pw2*d_12_pw2
         +f_2_pw2*p_2_pw2*p_1_pw2
         -p_1_pw4
         -2*f_2_pw2*p_2_pw2*p_1*d_12
         +p_2_pw2*f_1_pw2*p_1_pw2
         +f_2_pw2*p_2_pw2*d_12_pw2*b_pw2;

  // Computation of roots

  Mat_<double> realRoots(4,1);

  this->solveQuartic( factors, realRoots );

  // Backsubstitution of each solution

  for(int i=0; i<4; i++)
  {
    double cot_alpha = (-f_1*p_1/f_2-realRoots(i,0)*p_2+d_12*b)/(-f_1*realRoots(i,0)*p_2/f_2+p_1-d_12);

    double cos_theta = realRoots(i,0);
    double sin_theta = sqrt(1-pow(realRoots(i,0),2));
    double sin_alpha = sqrt(1/(pow(cot_alpha,2)+1));
    double cos_alpha = sqrt(1-pow(sin_alpha,2));

    if (cot_alpha < 0)
      cos_alpha = -cos_alpha;

    Mat_<double> C(3,1);
    C = Mat::zeros(3,1,CV_64F);
    C << d_12*cos_alpha*(sin_alpha*b+cos_alpha), cos_theta*d_12*sin_alpha*(sin_alpha*b+cos_alpha), sin_theta*d_12*sin_alpha*(sin_alpha*b+cos_alpha);

    C = P1 + N.t()*C;

    Mat_<double> R(3,3);    
    R(0,0) =  -cos_alpha;
    R(0,1) = -sin_alpha*cos_theta;
    R(0,2) = -sin_alpha*sin_theta;
    R(1,0) = sin_alpha;
    R(1,1) = -cos_alpha*cos_theta;
    R(1,2) = -cos_alpha*sin_theta;
    R(2,0) = 0.d;
    R(2,1) = -sin_theta;
    R(2,2) = cos_theta;

//    std::cout << "N =  " << N << std::endl;
//    std::cout << "R_before = " << R << std::endl;
//    std::cout << "T = " << T << std::endl;

    R = N.t()*R.t()*T;

    rotation->push_back(R);
    translation->push_back(C);

    std::cout << "Rotation = " << R << std::endl;
    std::cout << "Translation1 = " << C << std::endl;
  }

  return 1;
}


//
// Compute the correct solution and the error the the P3P algorithm
//
double RANSAC::computeErrorModelP3P(std::vector<cv::Point3d> &reference3D, std::vector<cv::Point2f> &current2D, std::vector<cv::Mat> *rotation, std::vector<cv::Mat> *translation, std::vector<double> *error, std::vector<int> *inliers)
{
//  std::cout << "Rotation1 = " << rotation->at(0) << std::endl;
//  std::cout << "Rotation2 = " << rotation->at(1) << std::endl;
//  std::cout << "Rotation3 = " << rotation->at(2) << std::endl;
//  std::cout << "Rotation4 = " << rotation->at(3) << std::endl;

//  std::cout << "Translation1 = " << translation->at(0) << std::endl;
//  std::cout << "Translation2 = " << translation->at(1) << std::endl;
//  std::cout << "Translation3 = " << translation->at(2) << std::endl;
//  std::cout << "Translation4 = " << translation->at(3) << std::endl;


  Mat rodrigues_rotation1,rodrigues_rotation2,rodrigues_rotation3,rodrigues_rotation4;
  Rodrigues(rotation->at(0),rodrigues_rotation1); //change the rotation to a Rodrigues rotation (3 parameters)
  Rodrigues(rotation->at(1),rodrigues_rotation2);
  Rodrigues(rotation->at(2),rodrigues_rotation3);
  Rodrigues(rotation->at(3),rodrigues_rotation4);
  std::vector<Point2d> predicted_pts1, predicted_pts2,predicted_pts3,predicted_pts4;
  double err1, err2,err3,err4;  //error  //temp points px, py,
  double error_total = 0;
  double errt1,errt2,errt3,errt4;
  errt1 = 0;
  errt2 = 0;
  errt3 = 0;
  errt4 = 0;

  //This function projects the reference 3D points onto the current image plane
  projectPoints(reference3D,rodrigues_rotation1,translation->at(0),rgb_camera_intrinsics,rgb_distortion,predicted_pts1);
  projectPoints(reference3D,rodrigues_rotation2,translation->at(1),rgb_camera_intrinsics,rgb_distortion,predicted_pts2);
  projectPoints(reference3D,rodrigues_rotation3,translation->at(2),rgb_camera_intrinsics,rgb_distortion,predicted_pts3);
  projectPoints(reference3D,rodrigues_rotation4,translation->at(3),rgb_camera_intrinsics,rgb_distortion,predicted_pts4);

  //go through each element and compute the error
  for(int i = 0; i < (int)predicted_pts1.size(); i++)
  {
    //predicted_temp = camera_intrinsics*sample_solution*reference3D[i];//sample solution needs to be 4x4, camera_intrinsics is 3x4
    //px = predicted_temp(0)/predicted_temp(2);
    //py = predicted_temp(1)/predicted_temp(2);
    //err = (current2D[i].pt.x - px)*(current2D[i].pt.x - px) + (current2D[i].pt.y - py)*(current2D[i].pt.y - py);

    err1 = (current2D[i].x - predicted_pts1[i].x)*(current2D[i].x - predicted_pts1[i].x) +
           (current2D[i].y - predicted_pts1[i].y)*(current2D[i].y - predicted_pts1[i].y);

    err2 = (current2D[i].x - predicted_pts2[i].x)*(current2D[i].x - predicted_pts2[i].x) +
           (current2D[i].y - predicted_pts2[i].y)*(current2D[i].y - predicted_pts2[i].y);
    err3 = (current2D[i].x - predicted_pts3[i].x)*(current2D[i].x - predicted_pts3[i].x) +
           (current2D[i].y - predicted_pts3[i].y)*(current2D[i].y - predicted_pts3[i].y);
    err4 = (current2D[i].x - predicted_pts4[i].x)*(current2D[i].x - predicted_pts4[i].x) +
           (current2D[i].y - predicted_pts4[i].y)*(current2D[i].y - predicted_pts4[i].y);

    error->push_back(err1);  //!< \note The error is squared, not square-rooted!
    error_total += err1; //sum up the error for all the points, not just the inliers!

    errt1 += err1;
    errt2 += err2;
    errt3 += err3;
    errt4 += err4;


    //! \note Implement some type of scheme to find the best solution from the possible ones:

    if (err1 <= inlier_threshold_ || err2 <= inlier_threshold_ || err3 <= inlier_threshold_ || err4 <= inlier_threshold_)
    {
      //we have an inlier!
      inliers->push_back(i);
    }
  }

  //! \note Revise the inputs and return the correct translation and rotation

  return error_total;
}


//
//  Root finder for 4th order polynomials, by Laurent Kneip
//
void RANSAC::solveQuartic(cv::Mat_<double> factors, cv::Mat_<double> real_roots)
{
  double A = factors(0,0);
  double B = factors(1,0);
  double C = factors(2,0);
  double D = factors(3,0);
  double E = factors(4,0);

  double A_pw2 = A*A;
  double B_pw2 = B*B;
  double A_pw3 = A_pw2*A;
  double B_pw3 = B_pw2*B;
  double A_pw4 = A_pw3*A;
  double B_pw4 = B_pw3*B;

  double alpha = -3*B_pw2/(8*A_pw2)+C/A;
  double beta = B_pw3/(8*A_pw3)-B*C/(2*A_pw2)+D/A;
  double gamma = -3*B_pw4/(256*A_pw4)+B_pw2*C/(16*A_pw3)-B*D/(4*A_pw2)+E/A;

  double alpha_pw2 = alpha*alpha;
  double alpha_pw3 = alpha_pw2*alpha;

  std::complex<double> P (-alpha_pw2/12-gamma,0);
  std::complex<double> Q (-alpha_pw3/108+alpha*gamma/3-pow(beta,2)/8,0);
  std::complex<double> R = -Q/2.0+sqrt(pow(Q,2.0)/4.0+pow(P,3.0)/27.0);

  std::complex<double> U = pow(R,(1.0/3.0));
  std::complex<double> y;

  if (U.real() == 0)
    y = -5.0*alpha/6.0-pow(Q,(1.0/3.0));
  else
    y = -5.0*alpha/6.0-P/(3.0*U)+U;

  std::complex<double> w = sqrt(alpha+2.0*y);

  std::complex<double> temp;

  temp = -B/(4.0*A) + 0.5*(w+sqrt(-(3.0*alpha+2.0*y+2.0*beta/w)));
  real_roots(0,0) = temp.real();
  temp = -B/(4.0*A) + 0.5*(w-sqrt(-(3.0*alpha+2.0*y+2.0*beta/w)));
  real_roots(1,0) = temp.real();
  temp = -B/(4.0*A) + 0.5*(-w+sqrt(-(3.0*alpha+2.0*y-2.0*beta/w)));
  real_roots(2,0) = temp.real();
  temp = -B/(4.0*A) + 0.5*(-w-sqrt(-(3.0*alpha+2.0*y-2.0*beta/w)));
  real_roots(3,0) = temp.real();
}


//
// This algorithm finds computes the average for a set of points, and returns the average and a centered set of points
//
void RANSAC::findCentroid(std::vector<Point3d> &inpoints3D,
                          std::vector<Point3d> *centered3D,
                          Matx31d *centroid_final)
{
  Point3d bar(0.d, 0.d, 0.d), centroid(0.d, 0.d, 0.d);;  //for averages
  Matx31d temp_cent;

  //calculate the mean
  for(int i = 0; i<(int)inpoints3D.size(); i++)
  {
    bar += inpoints3D[i];
  }

  centroid = bar*(1.0/inpoints3D.size());

  Point3d temp;

  if(!centered3D->empty())
  {
    centered3D->clear();
  }

  //remove the mean from the points
  for(int i = 0; i<(int)inpoints3D.size(); i++)
  {
    temp = inpoints3D[i] - centroid;
    centered3D->push_back(temp);
  }

  //pass back the centroid as a matrix, to facilitate other computation
  temp_cent(0,0) = centroid.x;
  temp_cent(1,0) = centroid.y;
  temp_cent(2,0) = centroid.z;

  *centroid_final = temp_cent;
}



















