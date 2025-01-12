#include "pose_subscriber.h"

Subscriber::Subscriber(const std::string &topic_name) : Node("pose_subscriber")
{
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        topic_name, 10, std::bind(&Subscriber::mapCallback, this, std::placeholders::_1));
};


void Subscriber::mapCallback(const nav_msgs::msg::Odometry::SharedPtr msg) 
{
    emit logReceived(msg);
}
   
