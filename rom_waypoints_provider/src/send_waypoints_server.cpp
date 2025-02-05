#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "rom_interfaces/srv/which_maps.hpp"
#include <filesystem>

#include <memory>

#include <std_msgs/msg/string.hpp>
#include <stdexcept>
#include <string>
#include <memory>
#include <cstdlib>
#include <unistd.h>
#include <sys/wait.h>
#include "rom_interfaces/srv/construct_yaml.hpp"
#include <sstream>
#include <iostream>

#define ROM_DEBUG 1

// switch mode parameters
pid_t launch_pid = -1;

// Package and launch file names
const std::string robot_name = std::getenv("ROM_ROBOT_MODEL");

const std::string my_nav2_package = "rom_waypoint_provider";                 
const std::string execu_name = "send_waypoints_goals";

std::vector<std::string> wp_names_;
bool state=false;

std::string joinVector(const std::vector<std::string>& waypoints, const std::string& delimiter = " ") {
    std::ostringstream oss;
    for (size_t i = 0; i < waypoints.size(); ++i) {
        if (i > 0) {
            oss << delimiter;
        }
        oss << waypoints[i];
    }
    return oss.str();
}

// switch_mode functions
void startLaunch(const std::string &package, const std::string &execu_file, const std::vector<std::string> waypoints,const bool &state = false) 
{
  std::string loop;
  if(state) { loop = "--loop"; }
  else{ loop= ""; }

  std::string result = joinVector(waypoints);

      #ifdef ROM_DEBUG
        RCLCPP_INFO(rclcpp::get_logger("waypoint_server"), "Starting ros2 run : %s/%s", package.c_str(), execu_file.c_str());
      #endif
        // Fork a process  to run the launch file
        launch_pid = fork();
        if (launch_pid == 0) {
            // In child process
            execlp("ros2", "ros2", "run", package.c_str(), execu_file.c_str(), result, state, (char *)NULL);
            perror("execlp failed");
            std::exit(EXIT_FAILURE);
        }

        if (launch_pid < 0) {
            perror("fork failed");
            throw std::runtime_error("Failed to start ros2 run process");
        }
}

void shutdownLaunch() 
{
        if (launch_pid > 0) {
          #ifdef ROM_DEBUG
            RCLCPP_INFO(rclcpp::get_logger("waypoint_server"), "Shutting down current ros2 run process (PID: %d)...", launch_pid);
          #endif
            kill(launch_pid, SIGINT);
            int status;
            waitpid(launch_pid, &status, 0);
            launch_pid = -1;
            sleep(3); // Ensure the process has fully terminated
        }
}

void waypoints_select(const std::shared_ptr<rom_interfaces::srv::ConstructYaml::Request> request,
          std::shared_ptr<rom_interfaces::srv::ConstructYaml::Response> response)
{
  if(request->pose_names.size()<1) { return; }

  state = request->loop;
  for (size_t i = 0; i < request->pose_names.size(); ++i) 
  {
    wp_names_.emplace_back(request->pose_names[i]);
  }

  if (launch_pid < 0) 
  {
    startLaunch(my_nav2_package,execu_name,wp_names_,state);
  }
  else {
    shutdownLaunch(); 
    startLaunch(my_nav2_package,execu_name,wp_names_,state);
  }
    #ifdef ROM_DEBUG
        RCLCPP_INFO(rclcpp::get_logger("Selected_Waypoints"), "Selected_Waypoints Activate");
    #endif
  
  
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("waypoint_server");

  rclcpp::Service<rom_interfaces::srv::ConstructYaml>::SharedPtr service = node->create_service<rom_interfaces::srv::ConstructYaml>("waypoints_selected", &waypoints_select);

  #ifdef ROM_DEBUG
    RCLCPP_INFO(rclcpp::get_logger("waypoint_server"), "Ready to selected waypoints.");
   
  #endif

  rclcpp::spin(node);
  rclcpp::shutdown();
}