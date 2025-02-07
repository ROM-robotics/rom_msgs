#ifndef SUBSCRIBER_H
#define SUBSCRIBER_H

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <QObject>
#include <QtWidgets/QApplication>
#include <geometry_msgs/msg/pose2_d.hpp>

class Subscriber : public QObject, public rclcpp::Node
{
    Q_OBJECT

    public:
        explicit Subscriber(const std::string &topic_name);

    signals:
        void logReceived(const geometry_msgs::msg::Pose2D::SharedPtr msg);

    private:
        rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr subscription_;
        void poseCallback(const geometry_msgs::msg::Pose2D::SharedPtr msg);
};

#endif // SUBSCRIBER_H



