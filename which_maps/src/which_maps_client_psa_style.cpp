#include "rclcpp/rclcpp.hpp"
#include "rom_interfaces/srv/which_maps.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

// INPUT ( which_map_do_you_have | save_map | select_map | mapping | navi | remapping )

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("which_map_client");
  rclcpp::Client<rom_interfaces::srv::WhichMaps>::SharedPtr client = node->create_client<rom_interfaces::srv::WhichMaps>("which_maps");

  auto request = std::make_shared<rom_interfaces::srv::WhichMaps::Request>();

  
  // // Wait for "which_maps" service
  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("which_maps_client"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("which_maps_client"), "Service not available, waiting again...");
  }

  request->request_string = "which_map_do_you_have";
  auto result = client->async_send_request(request);
  // // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("which_maps_client"), "Total_map: %d", result.get()->total_maps);
    if(result.get()->status == 1) { RCLCPP_INFO(rclcpp::get_logger("which_maps_client"), "which_map_do_you_have response OK"); }
    else { RCLCPP_INFO(rclcpp::get_logger("which_maps_client"), "which_map_do_you_have response NOT OK"); }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("which_maps_client"), "Failed to call service which_maps");
    RCLCPP_INFO(rclcpp::get_logger("which_maps_client"), "which_map_do_you_have response NOT OK"); 
  }


  request->request_string = "save_map";
  request->map_name_to_save = "default_map";
  auto result2 = client->async_send_request(request);
  // // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result2) == rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("which_maps_client"), "Map save %d ", result2.get()->status);
    if(result2.get()->status == 1) { RCLCPP_INFO(rclcpp::get_logger("which_maps_client"), "save_map response OK"); }
    else { RCLCPP_INFO(rclcpp::get_logger("which_maps_client"), "save_map response NOT OK"); }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("which_maps_client"), "Map save fail");
    RCLCPP_INFO(rclcpp::get_logger("which_maps_client"), "save_map response NOT OK");
  }


  request->request_string = "select_map";
  request->map_name_to_select = "default_map";
  auto result3 = client->async_send_request(request);
  // // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result3) == rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("which_maps_client"), "Selecting map  %d ", result3.get()->status);
    if(result3.get()->status == 1) { RCLCPP_INFO(rclcpp::get_logger("which_maps_client"), "select_map response OK"); }
    else { RCLCPP_INFO(rclcpp::get_logger("which_maps_client"), "select_map response NOT OK"); }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("which_maps_client"), "Selecting map fail");
    RCLCPP_INFO(rclcpp::get_logger("which_maps_client"), "select_map response NOT OK");
  }


  request->request_string = "mapping";
  auto result4 = client->async_send_request(request);
  // // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result4) == rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("which_maps_client"), "Mapping mode  %d ", result4.get()->status);
    if(result4.get()->status == 1) { RCLCPP_INFO(rclcpp::get_logger("which_maps_client"), "Mapping mode response OK"); }
    else { RCLCPP_INFO(rclcpp::get_logger("which_maps_client"), "Mapping mode response NOT OK"); }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("which_maps_client"), "Mapping mode fail");
    RCLCPP_INFO(rclcpp::get_logger("which_maps_client"), "Mapping mode response NOT OK");
  }


  request->request_string = "navi";
  auto result5 = client->async_send_request(request);
  // // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result5) == rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("which_maps_client"), "Navigation mode  %d ", result5.get()->status);
    if(result5.get()->status == 1) { RCLCPP_INFO(rclcpp::get_logger("which_maps_client"), "Navigation mode response OK"); }
    else { RCLCPP_INFO(rclcpp::get_logger("which_maps_client"), "Navigation mode response NOT OK"); }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("which_maps_client"), "Navigation fail");
    RCLCPP_INFO(rclcpp::get_logger("which_maps_client"), "Navigation mode response NOT OK");
  }


  request->request_string = "remapping";
  auto result6 = client->async_send_request(request);
  // // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result6) == rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("which_maps_client"), "Remapping mode  %d ", result6.get()->status);
    if(result6.get()->status == 1) { RCLCPP_INFO(rclcpp::get_logger("which_maps_client"), "Remapping mode response OK"); }
    else { RCLCPP_INFO(rclcpp::get_logger("which_maps_client"), "Remapping mode response NOT OK"); }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("which_maps_client"), "Remapping mode fail");
    RCLCPP_INFO(rclcpp::get_logger("which_maps_client"), "Remapping mode response NOT OK");
  }


  rclcpp::shutdown();
  return 0;
}