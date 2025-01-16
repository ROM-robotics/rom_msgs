#include "mode_subscriber.h"

ModeSubscriber::ModeSubscriber(const std::string &topic_name) : Node("mode_subscriber")
{
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        topic_name, 10, std::bind(&ModeSubscriber::modeCallback, this, std::placeholders::_1));
};


void ModeSubscriber::modeCallback(const std_msgs::msg::String::SharedPtr msg) 
{
    emit modeReceived(msg);
}
   
