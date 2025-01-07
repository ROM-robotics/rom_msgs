#include "rclcpp/rclcpp.hpp"
#include "rom_interfaces/srv/which_maps.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  if (argc != 2) {
      RCLCPP_INFO(rclcpp::get_logger("which_maps_client"), "usage: client {1 or 0?}");
      return 1;
  }

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("which_map_client");
  rclcpp::Client<rom_interfaces::srv::WhichMaps>::SharedPtr client =
    node->create_client<rom_interfaces::srv::WhichMaps>("which_maps");

  auto request = std::make_shared<rom_interfaces::srv::WhichMaps::Request>();

  request->which_maps_do_you_has = atoll(argv[1]);

  if (request->which_maps_do_you_has != 1 ) {

    return -1;
  }
  // request->b = atoll(argv[2]);

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("which_maps_client"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("which_maps_client"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("which_maps_client"), "total_map: %d", result.get()->total_maps);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("which_maps_client"), "Failed to call service which_maps");
  }

  rclcpp::shutdown();
  return 0;
}