cmake_minimum_required(VERSION 3.8)
project(which_maps)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rom_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(example_interfaces REQUIRED)

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


add_executable(which_maps_server src/which_maps_server.cpp)
ament_target_dependencies(which_maps_server rclcpp rom_interfaces)

add_executable(which_vel_server src/which_vel_server.cpp)
ament_target_dependencies(which_vel_server rclcpp rom_interfaces)

add_executable(which_nav_server src/which_navs_server.cpp)
ament_target_dependencies(which_nav_server rclcpp rom_interfaces)

# add_executable(ntmgr src/rom_node_thread_mgr.cpp)
# ament_target_dependencies(ntmgr rclcpp rom_interfaces)

add_executable(which_maps_client src/which_maps_client.cpp)
ament_target_dependencies(which_maps_client rclcpp rom_interfaces)

install(
  DIRECTORY maps       # Source directory
  DESTINATION share/${PROJECT_NAME}  # Destination directory in install space
)

# install(TARGETS
#   server
#   client
#   DESTINATION lib/${PROJECT_NAME})

#add_executable(add_two_ints_server src/add_two_ints_server.cpp)
#ament_target_dependencies(add_two_ints_server rclcpp example_interfaces)

install(TARGETS
  #add_two_ints_server
  which_maps_server
  which_maps_client
  which_nav_server
  which_vel_server
  DESTINATION lib/${PROJECT_NAME})

ament_package()
