#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher;
int counter = 0;

void publish_stop_signal() {
    auto msg = std_msgs::msg::Bool();

    // Publish True first (Stop signal)
    if (counter == 0) {
        msg.data = true;
        RCLCPP_INFO(rclcpp::get_logger("navigation_stop_publisher"), "Publishing: STOP navigation (True)");
    }

    publisher->publish(msg);
    counter++;

    // Exit after sending 2 messages (True and False)
    if (counter > 1) {
        RCLCPP_INFO(rclcpp::get_logger("navigation_stop_publisher"), "Exiting publisher after sending stop signals.");
        rclcpp::shutdown();
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("navigation_stop_publisher");
    publisher = node->create_publisher<std_msgs::msg::Bool>("navigation_stop", 10);
    
    // Publish stop signals once or twice
    publish_stop_signal();
    publish_stop_signal();
    
    // Keep the node alive until the signals are sent and then exit
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
