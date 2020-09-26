cmake_minimum_required(VERSION 3.10)

project(rl_logging)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g)
endif()

find_package(rclcpp REQUIRED)

## rl_logging library
#add_library(rl_logging INTERFACE)
add_library(rl_logging
  src/numbered_instance.cpp
)

set_property(TARGET rl_logging PROPERTY POSITION_INDEPENDENT_CODE ON)

target_include_directories(rl_logging PUBLIC 
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(TARGETS rl_logging
    EXPORT rl_loggingTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    INCLUDES DESTINATION include
)

install(DIRECTORY include/ DESTINATION include)

install(EXPORT rl_loggingTargets
  FILE rl_loggingTargets.cmake
  DESTINATION lib/cmake/rl_logging
)

include(CMakePackageConfigHelpers)
# generate the config file that is includes the exports
configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/Config.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/rl_loggingConfig.cmake"
  INSTALL_DESTINATION "lib/cmake/example"
  NO_SET_AND_CHECK_MACRO
  NO_CHECK_REQUIRED_COMPONENTS_MACRO
  )
# generate the version file for the config file
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/rl_loggingConfigVersion.cmake"
  VERSION "${Tutorial_VERSION_MAJOR}.${Tutorial_VERSION_MINOR}"
  COMPATIBILITY AnyNewerVersion
)

# install the configuration file
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/rl_loggingConfig.cmake
  DESTINATION lib/cmake/rl_logging
)

## rl_logging examples
add_executable(rl_logging_example_std_cout src/example_std_cout.cpp)
target_link_libraries(rl_logging_example_std_cout PRIVATE rl_logging)

add_executable(rl_logging_example_ros2 src/example_ros2.cpp)
target_link_libraries(rl_logging_example_ros2 PRIVATE
  rl_logging
  ${rclcpp_LIBRARIES}
)

target_include_directories(rl_logging_example_ros2 PRIVATE
  ${rclcpp_INCLUDE_DIRS}
)
