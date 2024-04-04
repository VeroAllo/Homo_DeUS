#! /bin/bash

homodeus_home=${PWD}

filter_node_name="Filters"
filter_node="rosrun hbba_lite_main filter_node.py"

hbba_lite_main_node_name="HBBA"
hbba_lite_main_node="rosrun hbba_lite_main hbba_lite_main_node"

nav_node_name="Navigation"
nav_node="rosrun Navigation main_navSelector.py"

pseudo_detection_node_name="PseudoDetection"
pseudo_detection_node="rosrun pseudo_detection pseudo_detection" 

hd_audio_node_name="Audio"
hd_audio_node="rosrun HD_audio talkInterface.py" 

eval "source ../../devel/setup.bash"

# Commands to run in each terminal window

gnome-terminal \
  --tab --title=${filter_node_name} -e "bash -c '$filter_node; exec bash'" \
  --tab --title=${nav_node_name}  -e "bash -c '$nav_node; exec bash'" \
  --tab --title=${hd_audio_node_name}  -e "bash -c '$hd_audio_node; exec bash'" \
  --tab --title=${pseudo_detection_node_name} -e "bash -c '$pseudo_detection_node; exec bash'" \
  --tab --title=${hbba_lite_main_node_name} -e "bash -c '$hbba_lite_main_node; exec bash'" \


