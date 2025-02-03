#include <map_subscriber.h>

MapSubscriber::MapSubscriber(const std::string &topic_name) :  Node("map_receiver") 
{

    rclcpp::QoS qos_profile(rclcpp::KeepLast(5)); 
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos_profile.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

    map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        topic_name, qos_profile, std::bind(&MapSubscriber::mapCallback, this, std::placeholders::_1));
}


void MapSubscriber::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) 
{
    #ifdef ROM_DEBUG
        qDebug() << "emit updateMap(msg)";
    #endif
    emit updateMap(msg);
}
   