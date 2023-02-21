cmake_minimum_required(VERSION 3.8)
project(map_merge_2d)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Enable debugging
add_compile_options(-g)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package( OpenCV REQUIRED )

set(
  DEPENDENCIES
  rclcpp
  std_msgs
  tf2_ros
  tf2
  nav_msgs
  geometry_msgs
  OpenCV
)

set(
  SOURCES
  src/map_merger_node.cpp
  src/map_merger.cpp
  src/util/topic_name_utils.cpp
  src/util/tf_utils.cpp
  src/map_discovery/topic_discovery.cpp
  src/submaps/submap.cpp
  src/submaps/submap_matcher.cpp
  src/submaps/submap_merger.cpp
  src/cv_core/cv_core.cpp
)

add_executable(map_merger ${SOURCES})
ament_target_dependencies(map_merger ${DEPENDENCIES})

target_include_directories(map_merger PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(map_merger PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS map_merger
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY include config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
