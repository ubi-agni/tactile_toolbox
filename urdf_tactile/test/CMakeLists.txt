find_package(Boost REQUIRED unit_test_framework)
include_directories(../src ${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})

# unit tests
add_executable(unit_test unit_test.cpp)
target_link_libraries(unit_test ${PROJECT_NAME} ${Boost_LIBRARIES})
add_test(NAME unit_test
   WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
   COMMAND unit_test)
