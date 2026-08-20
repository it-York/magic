# ROS 2 requires message resource names to start with an uppercase letter.
# Keep fields synchronized with wheel_info.msg, which is retained for ROS 1 API compatibility.
std_msgs/Header header
float32 front_angle_fb_l
float32 front_angle_fb_r
float32 rear_angle_fb_l
float32 rear_angle_fb_r
float32 lf_wheel_fb_velocity
float32 lr_wheel_fb_velocity
float32 rf_wheel_fb_velocity
float32 rr_wheel_fb_velocity
