#!/bin/bash

# Get the IP address of the enp3s0 interface
IP_ADDRESS=$(ifconfig enp3s0 | grep 'inet ' | awk '{print $2}')

# Extract the last octet of the IP address
LAST_OCTET=$(echo $IP_ADDRESS | awk -F. '{print $4}')

# Set the ROS_IP environment variable
export ROS_IP="10.68.0.$LAST_OCTET"

# Print the ROS_IP value to verify
echo "ROS_IP is set to $ROS_IP"

# Continue with the rest of your script
cd ~/Homo_DeUS/homodeus_ws
source /opt/ros/noetic/setup.bash
source ~/tiago_public_ws/devel/setup.bash
source devel/setup.bash
source ~/ws_moveit/devel/setup.bash
export ROS_MASTER_URI=http://10.68.0.1:11311

# --- Accueillir Client ---

# Navigation :
gnome-terminal -- bash -c "rosrun NavigationSelector main_navSelector.py; exec bash"
# cd Homo_DeUS/homodeus_ws/src/Navigation
# rosrun NavigationSelector main_navSelector.py

# Vision :
gnome-terminal -- bash -c "cd ~/Homo_DeUS/homodeus_ws/src/pseudo_detection/scripts/object_detection_package && python -m detect --weights ./yolov7-tiny.pt --conf-thres 0.4; exec bash"
#cd ~/Homo_DeUS/homodeus_ws/src/pseudo_detection/scripts/object_detection_package
#python -m detect --weights ./yolov7-tiny.pt --conf-thres 0.4

# Dialog :
gnome-terminal -- bash -c "rosrun HD_audio talkInterface.py; exec bash"
#cd ~/Homo_DeUS/homodeus_ws/src/HD_audio/script
#rosrun HD_audio talkInterface.py


# HBBA :
gnome-terminal -- bash -c "rosrun hbba_state filter_node.py; exec bash"
#cd ~/Homo_DeUS/homodeus_ws/src/hbba_state/scripts
#rosrun hbba_state filter_node.py

# trusssss c'est dans le cmake ;)
gnome-terminal -- bash -c "rosrun hbba_state hbba_state_node; exec bash"
#cd ~/Homo_DeUS/homodeus_ws/src/hbba_state
#rosrun hbba_state hbba_state_node

# --- Prendre Commande ---
# TODO
gnome-terminal -- bash -c "rosrun HD_audio oneDiscuss.py; exec bash"
# --- Chercher Commande ---

# Navigation :
#gnome-terminal -- bash -c "rosrun NavigationSelector main_navSelector.py; exec bash"
# rosrun NavigationSelector main_navSelector.py

# Vision :
#gnome-terminal -- bash -c "cd ~/tiago_public_ws/src/HomoDeUS/pseudo_detection/scripts/object_detection_package && python -m detect_can --weights ./tiny_canettes.pt --conf-thres 0.4; exec bash"
#cd ~/tiago_public_ws/src/HomoDeUS/pseudo_detection/scripts/object_detection_package
#python -m detect_can --weights ./tiny_canettes.pt --conf-thres 0.4

# Prehension: lets pray
gnome-terminal -- bash -c "roslaunch homodeus_prehension object_segmentation.launch; exec bash"
#roslaunch homodeus_prehension object_segmentation.launch