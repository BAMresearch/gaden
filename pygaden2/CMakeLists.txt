cmake_minimum_required(VERSION 3.5)
project(pygaden2)

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
find_package(pybind11 REQUIRED)
find_package(gaden2 REQUIRED)
find_package(gaden2_rviz REQUIRED)

ament_python_install_package(gaden2)

pybind11_add_module(pygaden2 src/example.cpp)

target_link_libraries(pygaden2 PRIVATE
  gaden2::gaden2
  gaden2_rviz::gaden2_rviz
)

install(TARGETS pygaden2
  DESTINATION "${PYTHON_INSTALL_DIR}/gaden2"
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
