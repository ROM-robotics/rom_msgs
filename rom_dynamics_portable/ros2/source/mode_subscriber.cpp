#include "mode_subscriber.h"

ModeSubscriber::ModeSubscriber(const std::string &topic_name) : Node("mode_subscriber")
{
    rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::KeepLast(1)).durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

    subscription_ = this->create_subscription<std_msgs::msg::String>(
        topic_name, qos_profile, std::bind(&ModeSubscriber::modeCallback, this, std::placeholders::_1));
};


void ModeSubscriber::modeCallback(const std_msgs::msg::String::SharedPtr msg) 
{
    emit modeReceived(msg);
}

