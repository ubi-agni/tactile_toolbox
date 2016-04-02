add_executable(tactile_pcl_node
	pcl_collector.cpp
	tactile_pcl_node.cpp
	conversions.h
)
target_link_libraries(tactile_pcl_node
	${catkin_LIBRARIES}
)
