cmake_minimum_required(VERSION 2.8.3)
project(tactile_state_publisher)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  sensor_msgs
  tactile_msgs
  #urdfdom # should probably be a separate find_package
  urdf
  cmake_modules
)

find_package(TinyXML REQUIRED)
#include_directories(SYSTEM ${TinyXML_INCLUDE_DIRS})

catkin_package(
CATKIN_DEPENDS  tactile_msgs sensor_msgs 
INCLUDE_DIRS include
LIBRARIES
#DEPENDS TinyXML #urdfdom
)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS}) # ${urdfdom_INCLUDE_DIRS}

add_executable(tactile_state_publisher src/tactile_state_publisher.cpp)
target_link_libraries(tactile_state_publisher ${catkin_LIBRARIES} ${Boost_LIBRARIES}) # ${TinyXML_LIBRARIES} ${urdfdom_LIBRARIES}

install(TARGETS tactile_state_publisher
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
