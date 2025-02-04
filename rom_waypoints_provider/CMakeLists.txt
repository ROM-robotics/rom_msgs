cmake_minimum_required(VERSION 3.8)
project(rom_waypoints_provider)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(rom_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

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

add_executable(send_waypoints src/send_waypoints.cpp)
target_link_libraries(send_waypoints yaml-cpp)
ament_target_dependencies(send_waypoints rclcpp rclcpp_action nav2_msgs geometry_msgs visualization_msgs yaml-cpp ament_index_cpp)

add_executable(construct_yaml_server src/construct_yaml_server.cpp)
target_link_libraries(construct_yaml_server yaml-cpp)
ament_target_dependencies(construct_yaml_server rclcpp rclcpp_action nav2_msgs geometry_msgs visualization_msgs yaml-cpp ament_index_cpp rom_interfaces)



install(TARGETS 
  send_waypoints 
  construct_yaml_server
DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY 
    config
    launch
  DESTINATION share/${PROJECT_NAME}  # Destination directory in install space
)

ament_package()
