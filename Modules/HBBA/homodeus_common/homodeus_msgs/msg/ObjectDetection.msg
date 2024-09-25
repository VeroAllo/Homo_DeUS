##########################################
#
# Object Detection data
#
##########################################

#####################
# Object header
#####################
# https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Header.html
std_msgs/Header header

#####################
# Object bounding box
#####################
homodeus_msgs/BoundingBox box

#####################
# Object distance (m) & angle (rad) from robot
#####################
float32 distance        # Ne pas utiliser (herite)
float32 angle           # Ne pas utiliser (herite)

#####################
# Object pose (frame /map)
#####################
geometry_msgs/Pose pose
homodeus_msgs/DesireID id