# Ellipses are expressed in the local frame anchored at origin_pose.
# Each row of data is: [center_x, center_y, semi_major, semi_minor, yaw].
std_msgs/Header header
geometry_msgs/Pose origin_pose
float64[] data
uint32 columns
