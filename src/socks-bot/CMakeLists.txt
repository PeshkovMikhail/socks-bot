cmake_minimum_required(VERSION 3.8)
project(socks_robot_tf2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(xacro REQUIRED)

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

add_executable(socksbot_tf2_broadcaster src/socksbot_tf2_broadcaster.cpp)
ament_target_dependencies(
   socksbot_tf2_broadcaster
   geometry_msgs
   rclcpp
   tf2
   tf2_ros
   nav_msgs
)

install(DIRECTORY
  launch
  description
  config
  DESTINATION share/${PROJECT_NAME}/
)


install(TARGETS
   socksbot_tf2_broadcaster
   DESTINATION lib/${PROJECT_NAME})

ament_package()
