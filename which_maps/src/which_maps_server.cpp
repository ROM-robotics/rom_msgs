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


std::string package_name = "which_maps";
bool map_topic_exists = true;

std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>> global_publisher;
auto trigger_msg = std_msgs::msg::String();

// switch mode parameters
std::string current_mode = "navi";
pid_t launch_pid = -1;

// Package and launch file names
const std::string cartographer_pkg = "rom2109_carto";                 
const std::string carto_mapping_launch = "mapping.launch.py";
    
const std::string carto_localization_launch = "localization.launch.py";
    
const std::string remapping_launch = "something.launch.py";

// switch_mode functions

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




// TODO 1. GO TO HOME 'S MAP DIRECTORY OR NOT
// INPUT ( which_map_do_you_have | save_map | select_map | mapping | navi | remapping )
void which_map_answer(const std::shared_ptr<rom_interfaces::srv::WhichMaps::Request> request,
          std::shared_ptr<rom_interfaces::srv::WhichMaps::Response>      response)
{
  /// ၁။ ဘယ်မြေပုံတွေရှိလဲ?။
  if(request->request_string == "which_map_do_you_have"){
    // if(check existance of map directory)
    
    std::string package_directory = ament_index_cpp::get_package_share_directory(package_name) + "/maps/";
    //RCLCPP_INFO(rclcpp::get_logger("which_maps"), "%s", package_directory.c_str());
    int yaml_file_count = 0;
    //std::string[] name_array;

    try 
    {
        for (const auto& entry : std::filesystem::directory_iterator(package_directory)) 
        {
            if (entry.is_regular_file() && entry.path().extension() == ".yaml") 
            {
                //std::cout << "YAML File: " << entry.path().filename() << std::endl;
                response->map_names.push_back(entry.path().filename().c_str());
                RCLCPP_INFO(rclcpp::get_logger("which_maps"), "%s", entry.path().filename().c_str());
                ++yaml_file_count;
            }
        }
        std::cout << "Total .yaml files: " << yaml_file_count << std::endl;
        response->status = 1; // ok
        response->total_maps = yaml_file_count;
        RCLCPP_INFO(rclcpp::get_logger("which_maps_server"), "Sending : Response Status OK");

    } 
    catch (const std::filesystem::filesystem_error& e) 
    {
        std::cerr << "Error accessing directory: " << e.what() << std::endl;
        response->total_maps = 0;
        response->status = -1; // not ok
        RCLCPP_INFO(rclcpp::get_logger("which_maps_server"), "Sending : Response Status not OK");
    }
  }

  
  /// ၂။ မြေပုံ save ပါ။
  else if (request->request_string == "save_map"){
    // map saver
    std::string map_name = request->map_name_to_save;
    
    if (map_topic_exists) 
    {
      RCLCPP_INFO(rclcpp::get_logger("which_maps_server"), "The /map topic exists.");
      RCLCPP_INFO(rclcpp::get_logger("which_maps_server"), "map: %s", map_name.c_str());

      // Save the map
      // command စစ်ဆေးရန်။
      std:: string cmd = "cd /home/mr_robot/data/maps/ && touch " + map_name + "_hack; rm " + map_name + "* && ros2 service call /write_state cartographer_ros_msgs/srv/WriteState '{filename: '/home/mr_robot/data/maps/" +  map_name + ".pbstream', include_unfinished_submaps: true}'";

      // first command
      int ret_code = std::system(cmd.c_str());

      if(ret_code == 0) {
        RCLCPP_INFO(rclcpp::get_logger("which_map_server"), "Map saver command 1 executed successfully.");

        std::string cmd2 = std::string("ros2 run cartographer_ros cartographer_pbstream_to_ros_map ") +
                            "-pbstream_filename /home/mr_robot/data/maps/" + map_name + ".pbstream " +
                            "-map_filename /home/mr_robot/data/maps/" + map_name + ".pgm " +
                            "-yaml_filename /home/mr_robot/data/maps/" + map_name + ".yaml && "  
                            "mv /home/mr_robot/map.pgm /home/mr_robot/data/maps/"  + map_name + ".pgm && "
                            "mv /home/mr_robot/map.yaml /home/mr_robot/data/maps/"  + map_name + ".yaml";
        // second command
        int ret_code2 = std::system(cmd2.c_str());

        if (ret_code2 == 0)
        {
          RCLCPP_INFO(rclcpp::get_logger("which_map_server"), "Map saver command 2 executed successfully.");

          std::string cmd3 = "sed 's|/*.pbstream|/custom.pbstream|g' /home/mr_robot/devel_ws/install/rom2109_carto/share/rom2109_carto/launch/localization.launch.py";
          int ret_code3 = std::system(cmd3.c_str());
          if(ret_code3==0)
          {
            RCLCPP_INFO(rclcpp::get_logger("which_maps_server"), "sed command 3 OK"); 

            std::string cmd4 = "sed 's|/*.yaml|/custom.yaml|g' /home/mr_robot/devel_ws/install/rom2109_nav2/share/rom2109_nav2/config/nav2_params.yaml";

            int ret_code4 = std::system(cmd4.c_str());

            if(ret_code4==0) { RCLCPP_INFO(rclcpp::get_logger("which_maps_server"), "sed command 4 OK"); response->status = 1; }
            else             { RCLCPP_INFO(rclcpp::get_logger("which_maps_server"), "sed command 4 Fail"); response->status = -1; }
          }
          else
          {
            response->status = -1; // not ok
            RCLCPP_INFO(rclcpp::get_logger("which_maps_server"), "sed command 3 Fail");
            return;
          }
          
          //response->status = 1; // ok
          RCLCPP_INFO(rclcpp::get_logger("which_maps_server"), "Sending : Response Status OK");
        } 
        else 
        {
          RCLCPP_ERROR(rclcpp::get_logger("which_map_server"), "Map saver command 2 failed with return code: %d", ret_code2);
          response->status = -1; // not ok
          RCLCPP_INFO(rclcpp::get_logger("which_maps_server"), "Sending : Response Status not OK");
        }
      }
      else 
      {
        RCLCPP_ERROR(rclcpp::get_logger("which_map_server"), "Map saver command 1 failed with return code: %d", ret_code);
        response->status = -1; // not ok
        RCLCPP_INFO(rclcpp::get_logger("which_maps_server"), "Sending : Response Status not OK");
      }    
    } 
    else 
    {
        RCLCPP_WARN(rclcpp::get_logger("which_maps_server"), "The /map topic does not exist.");
        response->status = -1; // not ok
        RCLCPP_INFO(rclcpp::get_logger("which_maps_server"), "Sending : Response Status not OK");
    }

    
  }


  /// ၃။ မြေပုံရွေးပါ။
  else if (request->request_string == "select_map") // Select map ထည့်၇န်
  {   
    // map selector

    std::string map_name = request->map_name_to_select;
    if(map_name == "")
    {
      response->status = -1; // not ok
      RCLCPP_INFO(rclcpp::get_logger("which_maps_server"), "Sending : Response Status not OK");
    }
    else 
    {
      response->status = 1; // ok
      RCLCPP_INFO(rclcpp::get_logger("which_maps_server"), "Sending : Response Status OK");
    }
  }

  /// ၄။ mapping mode change ပါ။
  else if (request->request_string == "mapping")
  {
    if (current_mode == "mapping" ) 
    {
      RCLCPP_INFO(rclcpp::get_logger("which_maps_server"), "Mode '%s' is already active.", current_mode.c_str());
    }
    else 
    {
      shutdownLaunch();
      
      RCLCPP_INFO(rclcpp::get_logger("which_maps_server"), "Sending : Response Status OK");
      response->status = 1; // ok
      startLaunch(cartographer_pkg, carto_mapping_launch );
      current_mode = "mapping";

      trigger_msg.data = "mapping";
      global_publisher->publish(trigger_msg);
    }
  }
  
  /// ၅။ nav mode change ပါ။
  else if (request->request_string == "navi")
  {
    if (current_mode == "navi" ) 
    {
      RCLCPP_INFO(rclcpp::get_logger("which_maps_server"), "Mode '%s' is already active.", current_mode.c_str());
    }
    else 
    {
      shutdownLaunch();
      response->status = 1; // ok
      startLaunch(cartographer_pkg, carto_localization_launch);
      RCLCPP_INFO(rclcpp::get_logger("which_maps_server"), "Sending : Response Status OK");
      current_mode = "navi";
      
      trigger_msg.data = "navi";
      global_publisher->publish(trigger_msg);
    }
  }
  
  
  /// 6။ remapping mode change ပါ။
  else if (request->request_string == "remapping")
  {
    if (current_mode == "remapping") 
    {
      RCLCPP_INFO(rclcpp::get_logger("which_maps_server"), "Mode '%s' is already active.", request->request_string.c_str());
    }
    else 
    {
      shutdownLaunch();
      startLaunch(cartographer_pkg, remapping_launch);
      response->status = 1; // ok
      RCLCPP_INFO(rclcpp::get_logger("which_maps_server"), "Sending : Response Status OK");
      current_mode = "remapping";

      trigger_msg.data = "remapping";
      global_publisher->publish(trigger_msg);
    }   
  }

  /// 7. ဘာမှမလုပ်
  else 
  {
    response->total_maps = 0;
    response->status = -1; // not ok
    RCLCPP_INFO(rclcpp::get_logger("which_maps_server"), "Sending : Response Status not OK");
    RCLCPP_WARN(rclcpp::get_logger("which_maps_server"), "Unknown mode '%s'.", request->request_string.c_str());
  }

  /// ၇။ ဘာမှမဟုတ်။

}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("which_maps_server");

  rclcpp::Service<rom_interfaces::srv::WhichMaps>::SharedPtr service = node->create_service<rom_interfaces::srv::WhichMaps>("which_maps", &which_map_answer);

  // latch publisher
  global_publisher = node->create_publisher<std_msgs::msg::String>("which_nav", rclcpp::QoS(1).transient_local());

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to answer maps.");

  RCLCPP_INFO(rclcpp::get_logger("which_maps_server"), "Fist Time trigger to Nav mode");

  startLaunch(cartographer_pkg, carto_localization_launch);
  
  trigger_msg.data = current_mode;
  global_publisher->publish(trigger_msg);

  rclcpp::spin(node);
  rclcpp::shutdown();
}