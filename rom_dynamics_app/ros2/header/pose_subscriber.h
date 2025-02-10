#ifndef SUBSCRIBER_H
#define SUBSCRIBER_H

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <QObject>
#include <QtWidgets/QApplication>
#include <geometry_msgs/msg/pose2_d.hpp>

#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <QObject>

#include <memory>
#include <chrono>

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

//------------------------------------------------------------


class TfListener : public QObject, public rclcpp::Node
{
    Q_OBJECT

public:
    explicit TfListener(const std::string &node_name);

    void listenForTransform();

signals:
    void transformReceived(std::shared_ptr<geometry_msgs::msg::TransformStamped> transform);

private:
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};



#endif // SUBSCRIBER_H



