#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <stdexcept>
#include <string>
#include <memory>
#include <cstdlib>
#include <unistd.h>
#include <sys/wait.h>

// switch mode parameters
std::string current_mode = "navi";
pid_t launch_pid = -1;

// Package and launch file names
const std::string robot_name = std::getenv("ROM_ROBOT_MODEL");

const std::string nav2_pkg = robot_name +"_nav2";                
const std::string nav2_mapping_launch = "navigation.launch.py";
const std::string nav2_localization_launch = "navigation_with_map_server.launch.py";
const std::string remapping_launch = "something.launch.py";

void startLaunch(const std::string &package, const std::string &launch_file) 
{
        RCLCPP_INFO(rclcpp::get_logger("which_maps_server"), "Starting launch file: %s/%s", package.c_str(), launch_file.c_str());

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
            RCLCPP_INFO(rclcpp::get_logger("which_maps_server"), "Shutting down current launch process (PID: %d)...", launch_pid);
            kill(launch_pid, SIGINT);
            int status;
            waitpid(launch_pid, &status, 0);
            launch_pid = -1;
            sleep(3); // Ensure the process has fully terminated
        }
}

void topic_callback(const std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_INFO(rclcpp::get_logger("which_nav_switcher"), "Get : '%s'", msg->data.c_str());
    
    std::string request_string = msg->data;

  if (request_string == "mapping")
  {
    if (current_mode == "mapping" ) 
    {
      RCLCPP_INFO(rclcpp::get_logger("which_nav_switcher"), "Mode '%s' is already active.", current_mode.c_str());
    }
    else 
    {
      shutdownLaunch();
      
      RCLCPP_INFO(rclcpp::get_logger("which_nav_switcher"), "Sending : Response Status OK");
      
      startLaunch(nav2_pkg, nav2_mapping_launch );
      current_mode = "mapping";
    }
  }
  
  /// ၅။ nav mode change ပါ။
  else if (request_string == "navi")
  {
    // if (current_mode == "navi" ) 
    // {
    //   RCLCPP_INFO(rclcpp::get_logger("which_nav_switcher"), "Mode '%s' is already active.", current_mode.c_str());
    // }
   
      shutdownLaunch();
      
      startLaunch(nav2_pkg, nav2_localization_launch);
      RCLCPP_INFO(rclcpp::get_logger("which_nav_switcher"), "Sending : Response Status OK");
      current_mode = "navi";
    
  }
  
  
  /// 6။ remapping mode change ပါ။
  else if (request_string == "remapping")
  {
    if (current_mode == "remapping") 
    {
      RCLCPP_INFO(rclcpp::get_logger("which_nav_switcher"), "Mode '%s' is already active.", request_string.c_str());
    }
    else 
    {
      shutdownLaunch();
      startLaunch(nav2_pkg, remapping_launch);
      
      RCLCPP_INFO(rclcpp::get_logger("which_nav_switcher"), "Sending : Response Status OK");
      current_mode = "remapping";
    }   
  }
}

int main(int argc, char **argv)
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create a node
    auto node = rclcpp::Node::make_shared("which_nav_switcher");

    // Create a subscriber
    auto subscription = node->create_subscription<std_msgs::msg::String>("which_nav", 10, topic_callback);

    RCLCPP_INFO(rclcpp::get_logger("which_nav_server"), "Fist Time trigger to Nav mode");

    startLaunch(nav2_pkg, nav2_localization_launch);

    // Spin the node to process callbacks
    rclcpp::spin(node);

    // Shutdown ROS 2
    rclcpp::shutdown();

    return 0;
}
