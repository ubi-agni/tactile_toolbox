cmake_minimum_required(VERSION 3.1.3)
project(urdf_tactile)

set(CMAKE_CXX_STANDARD 14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(Boost REQUIRED system)
find_package(urdfdom REQUIRED)
find_package(catkin REQUIRED pluginlib urdf)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  # Don't export the plugin-lib! This should be loaded dynamically only.
  LIBRARIES ${PROJECT_NAME}_tools
  CATKIN_DEPENDS urdf
  DEPENDS Boost
)

#############
## sub dir ##
#############
include_directories(SYSTEM ${Boost_INCLUDE_DIR})

include_directories(include ${catkin_INCLUDE_DIRS})
file(GLOB_RECURSE PROJECT_INCLUDES *.h)

add_subdirectory(src)
if(CATKIN_ENABLE_TESTING)
   add_subdirectory(test)
endif()

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

install(FILES
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY
  include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
