#ifndef ROM_ROS2_PUBLISHER_H
#define ROM_ROS2_PUBLISHER_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <QObject>
#include <QtWidgets/QApplication>


#include <rclcpp/rclcpp.hpp>
#include <rom_interfaces/srv/which_vel.hpp> // Replace with your custom service if needed
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



#endif // PUBLISHER_H
