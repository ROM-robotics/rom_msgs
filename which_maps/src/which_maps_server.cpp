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
bool map_topic_exists = false;



 // switch mode parameters
    std::string current_mode_;
    pid_t launch_pid_ = -1;

    // Package and launch file names
    const std::string cartographer_pkg = "rom2109_carto";                 
    const std::string carto_mapping_launch = "mapping.launch.py";
    
    const std::string carto_localization_launch = "localization.launch.py";
    
    const std::string remapping_launch = "something.launch.py";



// switch_mode functions

void startLaunch(const std::string &package, const std::string &launch_file) {
        RCLCPP_INFO(rclcpp::get_logger("which_maps_server"), "Starting launch file: %s/%s", package.c_str(), launch_file.c_str());

        // Fork a proce  to run the launch file
        launch_pid_ = fork();
        if (launch_pid_ == 0) {
            // In child process
            execlp("ros2", "ros2", "launch", package.c_str(), launch_file.c_str(), (char *)NULL);
            perror("execlp failed");
            std::exit(EXIT_FAILURE);
        }

        if (launch_pid_ < 0) {
            perror("fork failed");
            throw std::runtime_error("Failed to start launch process");
        }
    }

    void shutdownLaunch() {
        if (launch_pid_ > 0) {
            RCLCPP_INFO(rclcpp::get_logger("which_maps_server"), "Shutting down current launch process (PID: %d)...", launch_pid_);
            kill(launch_pid_, SIGINT);
            int status;
            waitpid(launch_pid_, &status, 0);
            launch_pid_ = -1;
            sleep(3); // Ensure the process has fully terminated
        }
    }




// TODO 1. GO TO HOME 'S MAP DIRECTORY OR NOT
// INPUT ( which_map_do_you_have | save_map | select_map | mapping | navi | remapping )
void which_map_answer(const std::shared_ptr<rom_interfaces::srv::WhichMaps::Request> request,
          std::shared_ptr<rom_interfaces::srv::WhichMaps::Response>      response)
{
  /// ၁။ ဘယ်မြေပုံတွေရှိလဲ?။
  if(request->request_string == "which_map_do_you_have")
  {
    // if(check existance of map directory)
    
    std::string package_directory = ament_index_cpp::get_package_share_directory(package_name) + "/maps/";
    //RCLCPP_INFO(rclcpp::get_logger("which_maps"), "%s", package_directory.c_str());
    int yaml_file_count = 0;
    //std::string[] name_array;

    try 
    {
        for (const auto& entry : std::filesystem::directory_iterator(package_directory)) {
            if (entry.is_regular_file() && entry.path().extension() == ".yaml") {
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

      // Save the map
      // command စစ်ဆေးရန်။
      std:: string cmd = "cd /home/mr_robot/Desktop/Git/rom_dynamics_robots/developer_packages/rom2109/rom2109_nav2/maps && ros2 run nav2_map_server map_saver_cli -f " + map_name;

      int ret_code = std::system(cmd.c_str());

      if (ret_code == 0)
      {
        RCLCPP_INFO(rclcpp::get_logger("which_map_server"), "Map saver command executed successfully.");
        response->status = 1; // ok
        RCLCPP_INFO(rclcpp::get_logger("which_maps_server"), "Sending : Response Status OK");
      } 
      else 
      {
        RCLCPP_ERROR(rclcpp::get_logger("which_map_server"), "Map saver command failed with return code: %d", ret_code);
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
  else if (request->request_string == "select_map"){   // Select map ထည့်၇န်
    // map selector

    std::string map_name = request->map_name_to_select;
    if(map_name == "")
    {
      response->status = -1; // not ok
      RCLCPP_INFO(rclcpp::get_logger("which_maps_server"), "Sending : Response Status not OK");
      return;
    }
    else 
    {
      response->status = 1; // ok
      RCLCPP_INFO(rclcpp::get_logger("which_maps_server"), "Sending : Response Status OK");
    }
  }
  
  
  
  else if (request->request_string == current_mode_) {
            RCLCPP_INFO(rclcpp::get_logger("which_maps_server"), "Mode '%s' is already active.", request->request_string.c_str());
            return;
        }


  shutdownLaunch();

  /// ၄။ mapping mode change ပါ။
  if (request->request_string == "mapping"){
    startLaunch(cartographer_pkg, carto_mapping_launch );
    response->status = 1; // ok
    RCLCPP_INFO(rclcpp::get_logger("which_maps_server"), "Sending : Response Status OK");
  }
  
  /// ၅။ nav mode change ပါ။
  else if (request->request_string == "navi"){

    startLaunch(cartographer_pkg, carto_localization_launch);

    response->status = 1; // ok
    RCLCPP_INFO(rclcpp::get_logger("which_maps_server"), "Sending : Response Status OK");
  }
  
  
  /// 6။ remapping mode change ပါ။
  else if (request->request_string == "remapping"){
    startLaunch(cartographer_pkg, remapping_launch);
    response->status = 1; // ok
    RCLCPP_INFO(rclcpp::get_logger("which_maps_server"), "Sending : Response Status OK");
  }
  else {

    response->total_maps = 0;
    response->status = -1; // not ok
    RCLCPP_INFO(rclcpp::get_logger("which_maps_server"), "Sending : Response Status not OK");

    RCLCPP_WARN(rclcpp::get_logger("which_maps_server"), "Unknown mode '%s'.", request->request_string.c_str());
    return;
        }

  /// ၇။ ဘာမှမဟုတ်။

}




int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("which_maps_server");

  rclcpp::Service<rom_interfaces::srv::WhichMaps>::SharedPtr service = node->create_service<rom_interfaces::srv::WhichMaps>("which_maps", &which_map_answer);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to answer maps.");

  auto topics = node->get_topic_names_and_types();
  for (const auto &topic : topics) 
  {
    if (topic.first == "/map") 
    {
      map_topic_exists = true;
      break;
    }
  }

  if (map_topic_exists) 
  {
    RCLCPP_INFO(node->get_logger(), "The /map topic exists.");
  } 
  else 
  {
    RCLCPP_WARN(node->get_logger(), "The /map topic does not exist.");
  }

  rclcpp::spin(node);
  rclcpp::shutdown();
}