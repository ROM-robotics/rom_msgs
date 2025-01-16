#ifndef MODE_SUBSCRIBER_H
#define MODE_SUBSCRIBER_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <QObject>
#include <QtWidgets/QApplication>

class ModeSubscriber : public QObject, public rclcpp::Node
{
    Q_OBJECT

    public:
        explicit ModeSubscriber(const std::string &topic_name);

    signals:
        void modeReceived(const std_msgs::msg::String::SharedPtr msg);

    private:
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
        void modeCallback(const std_msgs::msg::String::SharedPtr msg);
};

#endif // MODE_SUBSCRIBER_H

