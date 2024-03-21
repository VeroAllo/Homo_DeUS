cmake_minimum_required(VERSION 2.8.3)
project(base_navigation)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    geometry_msgs
    roscpp
    rospy
    actionlib
    actionlib_msgs
    std_msgs
    move_base_msgs
)

catkin_python_setup()

catkin_package(
    CATKIN_DEPENDS
        geometry_msgs
        roscpp
        rospy
        std_msgs
        move_base_msgs
        message_runtime
        actionlib
        actionlib_msgs
)

include_directories(
    # include
    ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
    src/base_navigation/BaseRotate.py
    src/base_navigation/NavSelector.py
    # src/base_navigation/NavGoalDeserializer.py
    # src/base_navigation/NavGoal.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# foreach(dir launch)
#     install(DIRECTORY ${dir}/
#         DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
# endforeach(dir)

install(FILES
    launch/nav_simul.launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
