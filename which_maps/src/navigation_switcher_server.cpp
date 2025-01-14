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



//std::string package_name = "which_maps";
//bool map_topic_exists = false;

 // switch mode parameters
    std::string current_mode = "navi";
    pid_t launch_pid = -1;

    // Package and launch file names
    const std::string navigation_pkg = "rom2109_nav2";                 
    const std::string navigation_launch = "navigation.launch.py";
    
    const std::string navigation_map_server_launch = "navigation_with_map_server.launch.py";
    
    const std::string something_launch = "something.launch.py";



// switch_mode functions

void startLaunch(const std::string &package, const std::string &launch_file) 
{
        RCLCPP_INFO(rclcpp::get_logger("nav_switcher"), "Starting launch file: %s/%s", package.c_str(), launch_file.c_str());

        // Fork a proce  to run the launch file
        launch_pid = fork();
        if (launch_pid == 0) {
            // In child process
            execlp("ros2", "ros2", "launch", package.c_str(), launch_file.c_str(), (char *)NULL);
            perror("execlp failed");
            std::exit(EXIT_FAILURE);
        }

        if (launch_pid < 0) {
            perror("fork failed");
            throw std::runtime_error("Failed to start launch process");
        }
}

void shutdownLaunch() 
{
        if (launch_pid > 0) {
            RCLCPP_INFO(rclcpp::get_logger("nav_switcher"), "Shutting down current launch process (PID: %d)...", launch_pid);
            kill(launch_pid, SIGINT);
            int status;
            waitpid(launch_pid, &status, 0);
            launch_pid = -1;
            sleep(3); // Ensure the process has fully terminated
        }
}




// TODO 1. GO TO HOME 'S MAP DIRECTORY OR NOT
// INPUT ( which_map_do_you_have | save_map | select_map | mapping | navi | remapping )
void which_map_answer(const std::shared_ptr<rom_interfaces::srv::WhichMaps::Request> request,
          std::shared_ptr<rom_interfaces::srv::WhichMaps::Response>      response)
{
 
  /// ၁။ mapping mode change ပါ။
  if (request->request_string == "mapping")
  {
    if (current_mode == "mapping" ) 
    {
      RCLCPP_INFO(rclcpp::get_logger("nav_switcher"), "Mode '%s' is already active.", current_mode.c_str());
    }
    else 
    {
      shutdownLaunch();
      
      RCLCPP_INFO(rclcpp::get_logger("nav_switcher"), "Sending : Response Status OK");
      response->status = 1; // ok
      startLaunch(navigation_pkg, navigation_launch );
      current_mode = "mapping";
      RCLCPP_INFO(rclcpp::get_logger("nav_switcher"), "Mode '%s' is already active.", current_mode.c_str());
    }
  }
  
  /// ၂။ nav mode change ပါ။
  else if (request->request_string == "navi")
  {
    if (current_mode == "navi" ) 
    {
      RCLCPP_INFO(rclcpp::get_logger("nav_switcher"), "Mode '%s' is already active.", current_mode.c_str());
    }
    else 
    {
      shutdownLaunch();
      response->status = 1; // ok
      startLaunch(navigation_pkg, navigation_map_server_launch);
      RCLCPP_INFO(rclcpp::get_logger("nav_switcher"), "Sending : Response Status OK");
      current_mode = "navi";
      RCLCPP_INFO(rclcpp::get_logger("nav_switcher"), "Mode '%s' is already active.", current_mode.c_str());
    }
  }
  
  
  /// ၃။ remapping mode change ပါ။
  else if (request->request_string == "remapping")
  {
    if (current_mode == "remapping") 
    {
      RCLCPP_INFO(rclcpp::get_logger("nav_switcher"), "Mode '%s' is already active.", request->request_string.c_str());
    }
    else 
    {
      shutdownLaunch();
      startLaunch(navigation_pkg, something_launch);
      response->status = 1; // ok
      RCLCPP_INFO(rclcpp::get_logger("nav_switcher"), "Sending : Response Status OK");
      current_mode = "remapping";
    }   
  }

  /// 7. ဘာမှမလုပ်
  else 
  {
    response->total_maps = 0;
    response->status = -1; // not ok
    RCLCPP_INFO(rclcpp::get_logger("nav_switcher"), "Sending : Response Status not OK");
    RCLCPP_WARN(rclcpp::get_logger("nav_switcher"), "Unknown mode '%s'.", request->request_string.c_str());
  }

  /// ၇။ ဘာမှမဟုတ်။

}




int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("nav_switcher");

  rclcpp::Service<rom_interfaces::srv::WhichMaps>::SharedPtr service = node->create_service<rom_interfaces::srv::WhichMaps>("which_nav", &which_map_answer);

  //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to answer maps.");
  RCLCPP_INFO(rclcpp::get_logger("nav_switcher"), "Fist Time trigger to Nav mode");

  startLaunch(navigation_pkg, navigation_map_server_launch);
  

  rclcpp::spin(node);
  rclcpp::shutdown();
}