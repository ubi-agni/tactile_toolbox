find_package(Boost REQUIRED system unit_test_framework)
include_directories(SYSTEM ${Boost_INCLUDE_DIR})

# unit tests
add_executable(test_parser parser.cpp)
target_link_libraries(test_parser ${PROJECT_NAME} ${Boost_LIBRARIES})
add_test(NAME test_parser
   WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	COMMAND test_parser)

add_executable(test_tools tools.cpp)
target_link_libraries(test_tools ${PROJECT_NAME}_tools ${Boost_LIBRARIES})
add_test(NAME test_tools
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	COMMAND test_tools)
