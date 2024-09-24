cmake_minimum_required(VERSION 3.0.2)
project(homodeus_hbba_lite)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  hbba_lite REQUIRED
  std_msgs
  homodeus_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
    roscpp 
    hbba_lite
    std_msgs
    homodeus_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
src/HDDesires.cpp
src/HDStrategy.cpp
src/HDStrategies.cpp
src/HDMotivations.cpp
# src/HDStrategyToMotivationInterface.cpp
)

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)