cmake_minimum_required(VERSION 2.8.3)
project(pseudo_detection)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  homodeus_msgs
  image_geometry
  image_transport
  sensor_msgs
  tf
  cv_bridge
)

find_package(OpenCV REQUIRED 
    COMPONENTS
    opencv_highgui)

catkin_package(
  CATKIN_DEPENDS geometry_msgs homodeus_msgs sensor_msgs
)

include_directories(
  SYSTEM
  ${catkin_INCLUDE_DIRS}
)

add_executable(pseudo_detection src/pseudo_detection.cpp)
target_link_libraries(pseudo_detection
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES})

### INSTALL ###

install(TARGETS pseudo_detection
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
