#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "rom_interfaces/srv/which_maps.hpp"
#include <filesystem>

#include <memory>

std::string package_name = "which_maps";

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

    // lifecycle မ run ရသေးလို့ ပြသနာရှိနေသေးတယ်။
    std:: string cmd = "ros2 run nav2_map_server map_saver_cli -f " + map_name;

    int ret_code = std::system(cmd.c_str());

    if (ret_code == 0){
        RCLCPP_INFO(rclcpp::get_logger("which_map_server"), "Map saver command executed successfully.");
        response->status = 1; // ok
        RCLCPP_INFO(rclcpp::get_logger("which_maps_server"), "Sending : Response Status OK");
    } 
    else {
        RCLCPP_ERROR(rclcpp::get_logger("which_map_server"), "Map saver command failed with return code: %d", ret_code);
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
  
  
  /// ၄။ mapping mode change ပါ။
  else if (request->request_string == "mapping"){
    response->status = 1; // ok
    RCLCPP_INFO(rclcpp::get_logger("which_maps_server"), "Sending : Response Status OK");
  }
  

  /// ၅။ nav mode change ပါ။
  else if (request->request_string == "navi"){
    response->status = 1; // ok
    RCLCPP_INFO(rclcpp::get_logger("which_maps_server"), "Sending : Response Status OK");
  }
  
  
  /// 6။ remapping mode change ပါ။
  else if (request->request_string == "remapping"){
    response->status = 1; // ok
    RCLCPP_INFO(rclcpp::get_logger("which_maps_server"), "Sending : Response Status OK");
  }
  
  
  /// ၇။ ဘာမှမဟုတ်။
  else {
    response->total_maps = 0;
    response->status = -1; // not ok
    RCLCPP_INFO(rclcpp::get_logger("which_maps_server"), "Sending : Response Status not OK");
  }
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("which_maps_server");

  rclcpp::Service<rom_interfaces::srv::WhichMaps>::SharedPtr service =
    node->create_service<rom_interfaces::srv::WhichMaps>("which_maps", &which_map_answer);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to answer maps.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}