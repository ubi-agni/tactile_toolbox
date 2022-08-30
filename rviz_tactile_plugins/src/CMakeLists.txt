set(CMAKE_CXX_STANDARD 14)

## QT stuff ##
# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct cmake to run moc automatically when needed
set(CMAKE_AUTOMOC ON)

include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

set(HDR_FILES
  ../include/${PROJECT_NAME}/tactile_state_display.h
  ../include/${PROJECT_NAME}/tactile_visual_base.h
  ../include/${PROJECT_NAME}/tactile_taxels_visual.h
  ../include/${PROJECT_NAME}/tactile_array_visual.h
  ../include/${PROJECT_NAME}/range_property.h
  ../include/${PROJECT_NAME}/group_property.h
  ../include/${PROJECT_NAME}/color_map.h
  ../include/${PROJECT_NAME}/tactile_contact_display.h
)

set(SRC_FILES
  tactile_state_display.cpp
  tactile_visual_base.cpp
  tactile_taxels_visual.cpp
  tactile_array_visual.cpp
  range_property.cpp
  group_property.cpp
  color_map.cpp

  tactile_contact_display.cpp
  plugin_init.cpp
)

add_library(${PROJECT_NAME} MODULE
  ${SRC_FILES}
  ${HDR_FILES}
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  tactile_filters
  ${QT_LIBRARIES}
)

# Install rules
install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
