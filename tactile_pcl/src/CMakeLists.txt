set (CMAKE_CXX_STANDARD 14)

add_executable(tactile_pcl_node
	pcl_collector.cpp
	tactile_pcl_node.cpp
	conversions.h
)
target_link_libraries(tactile_pcl_node
	${catkin_LIBRARIES}
)

## Mark executables and/or libraries for installation
install(TARGETS tactile_pcl_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
