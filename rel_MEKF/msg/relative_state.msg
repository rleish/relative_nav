# This message provides the relative state and covariance that the estimator 
# tracks. 
Header header	
string child_frame_id #the body-fixed frame
geometry_msgs/Vector3 translation
geometry_msgs/Quaternion rotation
geometry_msgs/Vector3 velocity
float64[36] covariance #the covariance on the relative position and orientation error 

