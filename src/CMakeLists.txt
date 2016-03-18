## QT stuff ##
# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct cmake to run moc automatically when needed
set(CMAKE_AUTOMOC ON)

include_directories(
  ${QT_HEADERS_DIR}
  ${QT_QTCORE_INCLUDE_DIR}
  ${QT_QTGUI_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${urdf_tactile_INCLUDE_DIRS}
  ${tactile_filters_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} MODULE
  tactile_state_display.cpp
  tactile_visual_base.cpp
  tactile_taxels_visual.cpp
  tactile_array_visual.cpp
  range_property.cpp
  color_map.cpp
  plugin_init.cpp
)
## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${tactile_filters_LIBRARIES})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_auto_type)

# Install rules
install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
