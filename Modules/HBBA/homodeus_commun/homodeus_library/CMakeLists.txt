cmake_minimum_required(VERSION 2.8.3)
project(homodeus_library)

find_package(catkin REQUIRED COMPONENTS
    rospy
    actionlib
    actionlib_msgs
    geometry_msgs
    move_base_msgs
    std_msgs
    message_generation
)

catkin_python_setup()

catkin_package(
    CATKIN_DEPENDS
        rospy
        actionlib
        actionlib_msgs
        geometry_msgs
        move_base_msgs
        std_msgs
        message_runtime
)


include_directories(
#   SYSTEM
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
    python/homodeus_library/homodeus_precomp.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
