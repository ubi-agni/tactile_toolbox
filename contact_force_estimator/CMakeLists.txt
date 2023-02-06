cmake_minimum_required(VERSION 3.1.3)
project(contact_force_estimator)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED urdf_tactile tactile_msgs)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS urdf_tactile tactile_msgs
  DEPENDS
)

include_directories(include)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

# Enforce a RUNPATH on the installed target (by cmake-default it is empty).
# Usually libs are found by LD_LIBRARY_PATH in a ROS environment.
# However, without an RUNPATH, sudo-elevated binaries will not find their libs,
# because the LD_LIBRARY_PATH is cleared in this case for security reasons.
# https://gitlab.kitware.com/cmake/community/-/wikis/doc/cmake/RPATH-handling
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

add_library(${PROJECT_NAME}
  src/taxel.cpp
  src/taxel_group.cpp
  src/contact_force_estimator.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_node src/node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME ${PROJECT_NAME})

# Install rules
install(TARGETS
  ${PROJECT_NAME}_node ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")
