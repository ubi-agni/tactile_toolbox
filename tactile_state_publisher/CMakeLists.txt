cmake_minimum_required(VERSION 2.8.3)
project(tactile_state_publisher)

find_package(catkin REQUIRED COMPONENTS
  tactile_msgs
  urdf
)
find_package(urdf_tactile REQUIRED)

message("${urdf_DIR}")
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES
#  CATKIN_DEPENDS tactile_msgs sensor_msgs
#  DEPENDS
)

include_directories(include ${catkin_INCLUDE_DIRS} ${urdf_tactile_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} src/tactile_state_publisher.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_definitions(-std=c++11)
# only works since cmake 3.1
# target_compile_features(${PROJECT_NAME} PUBLIC cxx_auto_type)

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
