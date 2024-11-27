cmake_minimum_required(VERSION 3.0.2)
project(homodeus_parameters)

find_package(
    catkin REQUIRED COMPONENTS
        rospy
        geometry_msgs
        pal_control_msgs
        std_msgs
)

catkin_python_setup()

catkin_package(
    CATKIN_DEPENDS
        rospy
        geometry_msgs
        pal_control_msgs
        std_msgs
)

include_directories(
    # include
    ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
    scripts/set_tiago_parameters.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# foreach(dir launch)
#     install(DIRECTORY ${dir}/
#         DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
# endforeach(dir)

# install(FILES
#     launch/nav_simul.launch
#     DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
