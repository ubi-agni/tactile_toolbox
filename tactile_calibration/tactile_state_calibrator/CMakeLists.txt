cmake_minimum_required(VERSION 3.1.3)
project(tactile_state_calibrator)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS roscpp tactile_msgs)
find_package(tactile_filters REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_search_module(YAML REQUIRED yaml-cpp)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES
CATKIN_DEPENDS tactile_msgs roscpp
#  DEPENDS
)

include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${YAML_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} src/tactile_state_calibrator.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} tactile_filters ${YAML_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
