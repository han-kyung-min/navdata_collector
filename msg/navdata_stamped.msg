# navdata stamped

Header header

std_msgs/Bool joystick
std_msgs/Float32MultiArray waypoints
std_msgs/Int32 sg_idx        # SG idx of topomap which is seeked by the robot
std_msgs/Float32 xy_dist     # the prediction of xy dist to sg 
std_msgs/Float32 orient_dist # the prediction of orient_dist to sg 
std_msgs/Float32MultiArray pose_diff  # dx dy dth   
std_msgs/Bool is_collision   # the collision status
