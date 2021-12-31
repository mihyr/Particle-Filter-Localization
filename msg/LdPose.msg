std_msgs/Header header
string ns
int32 num_of_landmarks
int32[] landmark_ids
geometry_msgs/Point[] landmark_coordinates #x,y,Radius of the circles
geometry_msgs/Point[] clusters  #x,y,id of the clusters
geometry_msgs/Point[] outliers #x,y of the outliers