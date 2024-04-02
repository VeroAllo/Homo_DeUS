##########################################
#
# Robot Pose Stamped data
#
##########################################

#####################
# Robot Pose header
#####################
# https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Header.html
std_msgs/Header header

#####################
# Robot Pose status
#####################
# 0: not lost, 1: lost
uint8 lost

#####################
# Robot Pose pose
#####################
geometry_msgs/Vector3 pose