cmake_minimum_required(VERSION 3.0.2)
project(homodeus_prehension)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  dynamic_reconfigure
  tf
  tf2
  tf2_ros
  tf2_geometry_msgs 
  homodeus_msgs
  sensor_msgs
  std_msgs
  geometry_msgs
  message_generation
  
  rospy
  aruco_ros

  
  
  joint_state_publisher
  moveit_planners_ompl
  moveit_ros_move_group
  moveit_ros_visualization
  moveit_simple_controller_manager
  robot_state_publisher
  xacro
)

generate_messages(
   DEPENDENCIES
   geometry_msgs
   std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS joint_state_publisher moveit_planners_ompl moveit_ros_move_group moveit_ros_visualization moveit_simple_controller_manager robot_state_publisher roscpp xacro dynamic_reconfigure tf2 tf2_ros tf2_geometry_msgs sensor_msgs message_runtime
  DEPENDS system_lib
)

include_directories(SYSTEM 
  include
  ${catkin_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME}
            src/pcl_filters.cpp
            src/ArmInterface.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(segment_table src/nodes/segment_table.cpp)
target_link_libraries(segment_table ${catkin_LIBRARIES})

add_executable(arm_interface_node
  src/homodeus_arm_interface_node.cpp
  src/ArmInterface.cpp
)
target_link_libraries(arm_interface_node ${catkin_LIBRARIES})

add_executable(object_detector src/nodes/object_detector.cpp src/pcl_filters.cpp)
target_link_libraries(object_detector ${PROJECT_NAME}
                                        ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME} segment_table object_detector
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)



