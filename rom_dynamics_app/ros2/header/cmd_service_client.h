#ifndef ROM_ROS2_PUBLISHER_H
#define ROM_ROS2_PUBLISHER_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <QObject>
#include <QtWidgets/QApplication>


#include <rclcpp/rclcpp.hpp>
#include <rom_interfaces/srv/which_vel.hpp>
#include <rom_interfaces/srv/construct_yaml.hpp>
#include <QObject>

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
    void onSendWaypointsGoal(std::shared_ptr<std::unordered_map<std::string, geometry_msgs::msg::Pose>> wp_list);

private:
    rclcpp::Client<rom_interfaces::srv::ConstructYaml>::SharedPtr goal_client_;
    std::string service_name_;
};


#endif // PUBLISHER_H
