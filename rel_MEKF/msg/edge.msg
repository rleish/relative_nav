#This message provides the edge between two nodes
Header header	
string child_frame_id #the new node
int32 from_node_ID #The old node ID number
int32 to_node_ID #the new node ID number
geometry_msgs/Vector3 translation
float64 yaw #the relative yaw change between the two nodes
float64[] covariance #the covariance on the translation part
