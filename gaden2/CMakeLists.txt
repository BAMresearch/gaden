cmake_minimum_required(VERSION 3.5)
project(gaden2)

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
find_package(rl_logging REQUIRED)
find_package(gaden2_eigen REQUIRED)

add_library(gaden2
  src/environment_model_plane.cpp
  src/filament_model.cpp
  src/gas_source.cpp
  src/gas_source_filament_model.cpp
  src/logger.cpp
  src/simulator.cpp
  src/wind_model_farrell.cpp
  src/wind_model_farrell_noise.cpp
  src/helpers/interpolation.cpp
  src/sensors/open_path.cpp
)

ament_target_dependencies(gaden2
  gaden2_eigen
)

target_link_libraries(gaden2
  rl_logging
)

set_property(TARGET gaden2 PROPERTY POSITION_INDEPENDENT_CODE ON)

target_include_directories(gaden2 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_export_dependencies(
  gaden2_eigen
  rl_logging
)

ament_export_targets(export_gaden2 HAS_LIBRARY_TARGET)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}/
)

install(
  TARGETS gaden2
  EXPORT export_gaden2
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
