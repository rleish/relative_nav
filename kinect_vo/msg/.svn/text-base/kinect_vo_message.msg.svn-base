# This message sends a transform between two camera poses, 
# the current and the reference.  It is similar to 
# geometry_msgs/TransformStamped, except that it also	
# contains a flag for when the reference has been reset. 

Header header	
string child_frame_id #the frame of the current camera
bool newReference #when true, the "current" image from this result is now the reference
geometry_msgs/Transform transform
int32 corresponding #number of corresponding features in the matching
int32 inliers #number of inliers from the RANSAC
float64[49] covariance #the covariance for the transformation [x y z qx qy qz qw]

