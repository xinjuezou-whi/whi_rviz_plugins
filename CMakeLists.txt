cmake_minimum_required(VERSION 3.5)

project(whi_rviz_plugins)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wnon-virtual-dtor -Woverloaded-virtual)
endif()

# Tests currently only run on OS X @ OSRF jenkins
# Enable on Linux by providing a display, enable on Windows via EnableDisplayTests=True
option(EnableDisplayTests "EnableDisplayTests")
set(DisplayTests "False" CACHE STRING "DisplayTestsVariable")

if(DEFINED ENV{DISPLAY})
  set(DISPLAYPRESENT TRUE)
endif()

if(APPLE OR DISPLAYPRESENT OR EnableDisplayTests STREQUAL "True")
  message(STATUS "Enabling tests requiring a display")
else()
  set(SKIP_DISPLAY_TESTS "SKIP_TEST")
endif()

# options and directories for visual tests (see visual_testing_framework documentation)
option(EnableVisualTests "decides whether or not to enable the tests")

add_definitions(-D_BUILD_DIR_PATH="${CMAKE_CURRENT_BINARY_DIR}")
add_definitions(-D_SRC_DIR_PATH="${CMAKE_CURRENT_SOURCE_DIR}")

file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/test_images)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/test/reference_images)

# Run visual tests only if "EnableVisualTests=True"
if(EnableVisualTests STREQUAL "True")
  message(STATUS "Enabling visual tests")
else()
  set(SKIP_VISUAL_TESTS "SKIP_TEST")
endif()

if(MSVC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")
endif()

# We specifically don't turn on CMAKE_AUTOMOC, since it generates one huge
# mocs_compilation.cpp file that takes a lot of memory to compile.  Instead
# we create individual moc files that can be compiled separately.

find_package(ament_cmake REQUIRED)

find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Test)
find_package(geometry_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(map_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(resource_retriever REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(angles REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(pcl_conversions REQUIRED)
find_package(whi_interfaces REQUIRED)

# UI Files
set(UIC_FILES
  ui/navi_waypoints.ui
  ui/navi_teleop.ui
  ui/navi_multiple_ns.ui
  ui/navi_battery.ui
  ui/navi_state.ui
  ui/navi_robot_model_viewer.ui
  ui/line_chart_2D.ui
)
qt5_wrap_ui(UIC_SOURCES ${UIC_FILES})

include_directories(${CMAKE_CURRENT_BINARY_DIR})

# Source files
file(GLOB_RECURSE WHI_SDK_H
  "include/${PROJECT_NAME}/*.h"
)
file(GLOB_RECURSE WHI_SDK_SRC
  "src/*.cpp"
)
file(GLOB QWT_LIBRARIES
  "/usr/lib/libqwt*"
)

set(CMAKE_AUTOMOC ON)
qt5_wrap_cpp(MOC_FILES
  include/${PROJECT_NAME}/display_2D_line_chart.h
  include/${PROJECT_NAME}/display_battery.h
  include/${PROJECT_NAME}/display_state.h
  include/${PROJECT_NAME}/display_teleop.h
  include/${PROJECT_NAME}/display_video_stream.h
  include/${PROJECT_NAME}/display_waypoints.h
  include/${PROJECT_NAME}/display_tf.h
  include/${PROJECT_NAME}/display_robot_model_viewer.h
  include/${PROJECT_NAME}/panel_2D_line_chart.h
  include/${PROJECT_NAME}/panel_battery.h
  include/${PROJECT_NAME}/panel_map_saver.h
  include/${PROJECT_NAME}/panel_state.h
  include/${PROJECT_NAME}/panel_teleop.h
  include/${PROJECT_NAME}/panel_waypoints.h
  include/${PROJECT_NAME}/panel_robot_model_viewer.h
  include/${PROJECT_NAME}/panel_pcd_selection.h
  include/${PROJECT_NAME}/frame_info.h
  include/${PROJECT_NAME}/widget_twist.h
  )
# foreach(header "${WHI_SDK_H}")
#   qt5_wrap_cpp(whi_rviz_plugins_moc_files "${header}")
# endforeach()

add_library(${PROJECT_NAME} SHARED
  # ${whi_rviz_plugins_moc_files}
  ${MOC_FILES}
  ${WHI_SDK_SRC}
  ${UIC_SOURCES}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Qt5Widgets_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  /usr/include/qwt/
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  rviz_ogre_vendor::OgreMain
  rviz_ogre_vendor::OgreOverlay
  rviz_common::rviz_common
  rviz_rendering::rviz_rendering
  rviz_default_plugins::rviz_default_plugins
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${QWT_LIBRARIES}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "WHI_RVIZ_PLUGINS_BUILDING_LIBRARY")

# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(rviz_common whi_rviz_plugins.xml)

ament_target_dependencies(${PROJECT_NAME}
  PUBLIC
  geometry_msgs
  interactive_markers
  laser_geometry
  nav2_msgs
  map_msgs
  rclcpp
  resource_retriever
  rviz_common
  rviz_rendering
  rviz_default_plugins
  tf2
  tf2_geometry_msgs
  tf2_ros
  urdf
  visualization_msgs
  cv_bridge
  angles
  pcl_conversions
  whi_interfaces
)

ament_export_include_directories(include)
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  Qt5
  rviz_common
  geometry_msgs
  interactive_markers
  laser_geometry
  nav2_msgs
  map_msgs
  rclcpp
  resource_retriever
  rviz_ogre_vendor
  tf2
  tf2_geometry_msgs
  tf2_ros
  urdf
  visualization_msgs
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/icons" config
  DESTINATION "share/${PROJECT_NAME}"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
