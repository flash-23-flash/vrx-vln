# pose is in the world frame, velocity is in the robot frame
std_msgs/Header header
std_msgs/String robot_name
std_msgs/Bool reach_goal
std_msgs/Float64 travel_time
geometry_msgs/Pose pose 
geometry_msgs/Twist velocity