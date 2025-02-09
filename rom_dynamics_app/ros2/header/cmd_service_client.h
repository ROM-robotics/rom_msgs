#ifndef ROM_ROS2_PUBLISHER_H
#define ROM_ROS2_PUBLISHER_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <QObject>
#include <QtWidgets/QApplication>

#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rom_interfaces/srv/which_vel.hpp>
#include <rom_interfaces/srv/construct_yaml.hpp>
#include "rom_interfaces/msg/construct_yaml.hpp"
#include <QDebug>
#include <geometry_msgs/msg/pose_stamped.hpp>

class CmdServiceClient : public QObject, public rclcpp::Node
{
    Q_OBJECT

public:
    explicit CmdServiceClient(const std::string &service_name, QObject *parent = nullptr);
    void sendCommand(const QString &command); 

signals:
    void commandResponse(bool success); // Signal to notify command status
    void serviceResponse(bool success);

public slots:
    void setForward();
    void setLeft();
    void setRight();
    void setStop(); 

private:
    rclcpp::Client<rom_interfaces::srv::WhichVel>::SharedPtr client_;
    std::string service_name_;
};

class ConstructYamlServiceClient : public QObject, public rclcpp::Node
{
    Q_OBJECT

public:
    explicit ConstructYamlServiceClient(const std::string &service_name, QObject *parent = nullptr);


public slots:
    void onSendWaypoints(std::shared_ptr<std::unordered_map<std::string, geometry_msgs::msg::Pose>> wp_list, std::shared_ptr<std::unordered_map<std::string, geometry_msgs::msg::Pose>> scene_wp_list);

private:
    rclcpp::Client<rom_interfaces::srv::ConstructYaml>::SharedPtr goal_client_;
    std::string service_name_;
};

class WaypointListSubscriber : public QObject, public rclcpp::Node {
    Q_OBJECT

public:
    explicit WaypointListSubscriber(const std::string &topic_name);
    //~WaypointListSubscriber() {}
public slots:
    void onMapReadyForWaypointsSubscriber();

signals:
    void updateWpUI(rom_interfaces::msg::ConstructYaml::SharedPtr wplist_ptr);

private:
    rclcpp::Subscription<rom_interfaces::msg::ConstructYaml>::SharedPtr wp_subscription_;
    void wpCallback(const rom_interfaces::msg::ConstructYaml::SharedPtr wplist);

    std::vector<std::string> wp_names_;
    rom_interfaces::msg::ConstructYaml::SharedPtr wplistPtr_;
};

class SendWaypointsClient : public QObject, public rclcpp::Node
{
    Q_OBJECT

public:
    explicit SendWaypointsClient(const std::string &service_name, QObject *parent = nullptr);

signals:
    void serviceWpResponse(bool success);
    
public slots:
    void onSendWaypointsGoal(std::vector<std::string> wp_names);
    //void onSendWaypointsGoalzz();

private:
    std::mutex mutex_;
    
    rclcpp::Client<rom_interfaces::srv::ConstructYaml>::SharedPtr wp_goal_client_;
    std::string service_name_;
};

#endif // PUBLISHER_H
