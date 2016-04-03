cmake_minimum_required(VERSION 2.8.3)
project(urdf_tactile)
set (${PROJECT_NAME}_VERSION_MAJOR 0)
set (${PROJECT_NAME}_VERSION_MINOR 1)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
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
#  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS urdf
#  DEPENDS system_lib
)

#############
## sub dir ##
#############
include_directories(include ${catkin_INCLUDE_DIRS})

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
