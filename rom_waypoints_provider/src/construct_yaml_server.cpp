#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "rom_interfaces/srv/construct_yaml.hpp"
#include "rom_interfaces/msg/construct_yaml.hpp"
#include <iostream>
#include <filesystem>
#include <fstream>
#include <string>

#define ROM_DEBUG 1

std::string package_path;
std::string yaml_path;

rclcpp::Publisher<rom_interfaces::msg::ConstructYaml>::SharedPtr publisher_;

void construct_yaml_file(const std::shared_ptr<rom_interfaces::srv::ConstructYaml::Request> request,
          std::shared_ptr<rom_interfaces::srv::ConstructYaml::Response>      response)
{
  //check yaml, if exists delete
  if (std::filesystem::exists(yaml_path)) 
  {
    #ifdef ROM_DEBUG
      RCLCPP_INFO_STREAM(rclcpp::get_logger("yaml constructor"), "File exists: ");
    #endif
    
    if (std::filesystem::remove(yaml_path)) 
    {
      #ifdef ROM_DEBUG
        RCLCPP_INFO_STREAM(rclcpp::get_logger("yaml constructor"), "File deleted successfully.");
      #endif
    } 
    else 
    {
      #ifdef ROM_DEBUG
        RCLCPP_INFO_STREAM(rclcpp::get_logger("yaml constructor"), "Error deleting file!");
      #endif
      return; // Exit with error
    }
  }

  // create waypoints.yaml in your specific packages
  std::ofstream file(yaml_path);
  if (!file.is_open()) 
  {
    #ifdef ROM_DEBUG
      RCLCPP_ERROR(rclcpp::get_logger("yaml constructor"), "file open error!!");
    #endif
    return;
  }
  // write data to yaml files
  file << "waypoints:\n";
        for (size_t i = 0; i < request->pose_names.size(); ++i) 
        {
            file << "  - name: " << request->pose_names[i] << "\n";
            file << "    frame_id: " << "map" << "\n";
            file << "    pose:\n";
            file << "      position:\n";
            file << "        x: " << request->poses[i].pose.position.x << "\n";
            file << "        y: " << request->poses[i].pose.position.y << "\n";
            file << "        z: " << request->poses[i].pose.position.z << "\n";
            file << "      orientation:\n";
            file << "        x: " << request->poses[i].pose.orientation.x << "\n";
            file << "        y: " << request->poses[i].pose.orientation.y << "\n";
            file << "        z: " << request->poses[i].pose.orientation.z << "\n";
            file << "        w: " << request->poses[i].pose.orientation.w << "\n";
        }

  // close yaml file
  file.close();
  #ifdef ROM_DEBUG
    RCLCPP_INFO_STREAM(rclcpp::get_logger("yaml constructor"), "waypoints.yaml created successfully!");
  #endif     

  // publish or gui apps
  auto message = rom_interfaces::msg::ConstructYaml();
    
  for (size_t i = 0; i < request->pose_names.size(); ++i) 
  {
    message.pose_names[i] = request->pose_names[i];
    message.poses[i] = request->poses[i];
  }
  publisher_->publish(message);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // node ဖန်တီး
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("construct_yaml_server");

  package_path = ament_index_cpp::get_package_share_directory("rom_waypoints_provider");
  yaml_path = package_path + "/config/waypoints.yaml";

  // publisher
  auto qos = rclcpp::QoS(10); // Keep last 10 messages
  qos.transient_local(); // Set durability to TRANSIENT_LOCAL

  publisher_ = node->create_publisher<rom_interfaces::msg::ConstructYaml>("waypoints_list", qos);


  // service server နဲ့ callback ချိတ်တယ်။
  rclcpp::Service<rom_interfaces::srv::ConstructYaml>::SharedPtr service = node->create_service<rom_interfaces::srv::ConstructYaml>("construct_yaml", &construct_yaml_file);

  #ifdef ROM_DEBUG
    RCLCPP_INFO(rclcpp::get_logger("construct_yaml"), "Ready to construct waypoints.yaml");
  #endif

  rclcpp::spin(node);
  rclcpp::shutdown();
}