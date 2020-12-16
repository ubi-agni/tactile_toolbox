cmake_minimum_required(VERSION 3.1.3)
project(tactile_state_publisher)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  tactile_msgs
  urdf_tactile
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES
CATKIN_DEPENDS tactile_msgs
#  DEPENDS
)

catkin_python_setup()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} src/tactile_state_publisher.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

catkin_install_python(PROGRAMS scripts/tactile_bias.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
