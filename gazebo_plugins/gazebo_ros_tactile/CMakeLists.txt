cmake_minimum_required(VERSION 2.8.3)
project(gazebo_ros_tactile)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  urdf
  urdf_tactile
  tactile_msgs
)

## System dependencies are found with CMake's conventions
find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gazebo_ros_tactile
  CATKIN_DEPENDS gazebo_ros roscpp tactile_msgs
  DEPENDS gazebo 
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

option(PUB_DEBUG_CONTACT_STATE "Build with contact state publisher" OFF)

if(PUB_DEBUG_CONTACT_STATE)
  message( "Building with contact state publisher in addition to tactile_state.")
  add_definitions( -DPUB_DEBUG_CONTACT_STATE )
endif(PUB_DEBUG_CONTACT_STATE)

## Declare a C++ library
add_library(gazebo_ros_tactile src/gazebo_ros_tactile.cpp)
add_dependencies(gazebo_ros_tactile ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_ros_tactile ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ContactPlugin ${catkin_LIBRARIES})

#############
## Install ##
#############
install(DIRECTORY include/
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)


install(TARGETS gazebo_ros_tactile
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_gazebo_ros_tactile.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
