#ifndef SUBSCRIBER_H
#define SUBSCRIBER_H

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <QObject>
#include <QtWidgets/QApplication>

class Subscriber : public QObject, public rclcpp::Node
{
    Q_OBJECT

    public:
        explicit Subscriber(const std::string &topic_name);

    signals:
        void logReceived(const nav_msgs::msg::Odometry::SharedPtr msg);

    private:
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
        void poseCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
};

#endif // SUBSCRIBER_H



