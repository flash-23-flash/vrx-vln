# self_pose is in the world frame, all others are in the robot frame
std_msgs/Header header
geometry_msgs/Point goal
geometry_msgs/Pose self_pose 
geometry_msgs/Twist self_velocity
float32 self_radius
geometry_msgs/Point[] object_positions
float32[] object_radii
geometry_msgs/Point[] object_velocities