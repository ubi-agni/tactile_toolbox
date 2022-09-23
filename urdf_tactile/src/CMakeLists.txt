add_library(${PROJECT_NAME} SHARED
	parser.cpp
	utils.cpp
	${PROJECT_INCLUDES}
)
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES} ${urdfdom_LIBRARIES} ${catkin_LIBRARIES})

add_library(${PROJECT_NAME}_tools SHARED
	taxel_info_iterator.cpp
	sort.cpp
	${PROJECT_INCLUDES}
)
target_link_libraries(${PROJECT_NAME}_tools ${Boost_LIBRARIES} ${catkin_LIBRARIES})

# install rules
install(TARGETS
  ${PROJECT_NAME} ${PROJECT_NAME}_tools
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
