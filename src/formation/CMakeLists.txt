cmake_minimum_required(VERSION 3.8)
project(formation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# cmake flag
set(CMAKE_BUILD_TYPE Debug)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(px4_ros_com REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include model)

add_subdirectory(model)

add_executable(formation_node src/formation_node.cpp)
target_link_libraries(formation_node FMS Eigen3::Eigen)
ament_target_dependencies(formation_node rclcpp px4_msgs px4_ros_com)
install(TARGETS formation_node DESTINATION lib/${PROJECT_NAME})

add_executable(cross_node src/cross_node.cpp)
ament_target_dependencies(cross_node rclcpp px4_msgs px4_ros_com)
install(TARGETS cross_node DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)


set(TEST false)
if (TEST)
  add_executable(test_node src/test.cpp)
  ament_target_dependencies(test_node rclcpp px4_msgs px4_ros_com)
  install(TARGETS test_node DESTINATION lib/${PROJECT_NAME})
endif ()


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
