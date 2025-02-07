#include "pose_subscriber.h"

Subscriber::Subscriber(const std::string &topic_name) : Node("qt_robot_pose_publisher")
{
    subscription_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
        topic_name, 10, std::bind(&Subscriber::poseCallback, this, std::placeholders::_1));
};


void Subscriber::poseCallback(const geometry_msgs::msg::Pose2D::SharedPtr msg) 
{
    emit logReceived(msg);
}
   
