cmake_minimum_required(VERSION 3.5)
project(gaden2_rviz)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gaden2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(gaden2_rviz
  src/environment_visualisation_plane.cpp
  src/filament_visualisation.cpp
  src/gas_source_visualisation.cpp
  src/visualisation_base.cpp
  src/wind2d_visualisation.cpp
  src/helpers/ros_type_conversions.cpp
)

#target_link_libraries(gaden2_rviz
#  rclcpp
#)

ament_target_dependencies(gaden2_rviz
  gaden2
  rclcpp
  visualization_msgs
)

set_property(TARGET gaden2_rviz PROPERTY POSITION_INDEPENDENT_CODE ON)

target_include_directories(gaden2_rviz PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_export_dependencies(
  rclcpp
  visualization_msgs
)

ament_export_targets(export_gaden2_rviz HAS_LIBRARY_TARGET)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}/
)

install(
  TARGETS gaden2_rviz
  EXPORT export_gaden2_rviz
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
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
