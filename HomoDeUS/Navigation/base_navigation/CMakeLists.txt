cmake_minimum_required(VERSION 3.0.2)
project(base_navigation)

find_package(
    catkin REQUIRED COMPONENTS
        rospy
        geometry_msgs
        homodeus_msgs
        nav_msgs
        sensor_msgs
        std_msgs
)

catkin_python_setup()

catkin_package(
    CATKIN_DEPENDS
        rospy
        geometry_msgs
        homodeus_msgs
        nav_msgs
        sensor_msgs
        std_msgs
)

include_directories(
    # include
    ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
    python/base_navigation/BaseRotate.py
    python/base_navigation/obstacle_detector_sonar.py
    python/base_navigation/perception_pose.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# foreach(dir launch)
#     install(DIRECTORY ${dir}/
#         DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
# endforeach(dir)

install(FILES
    launch/base_navigation.launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
