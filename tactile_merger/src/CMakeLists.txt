include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${tactile_filters_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
   taxel.cpp
   taxel_group.cpp
   merger.cpp
   merger_node.cpp
)
## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${tactile_filters_LIBRARIES})

add_definitions(-std=c++11)
# only works since cmake 3.1:
# target_compile_features(${PROJECT_NAME} PUBLIC cxx_auto_type)

# Install rules
install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
