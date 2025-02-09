#ifndef MAPHANDLER_H
#define MAPHANDLER_H

#include <QObject>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <QImage>
#include <QDebug>
//#define ROM_DEBUG 1

class MapSubscriber : public QObject, public rclcpp::Node {
    Q_OBJECT

public:
    explicit MapSubscriber(const std::string &topic_name);
    //~MapSubscriber() {}

signals:
    void updateMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void testEmit();

private:
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
};

class LaserSubscriber : public QObject, public rclcpp::Node {
    Q_OBJECT

public:
    explicit LaserSubscriber(const std::string &topic_name);
    //~LaserSubscriber() {}

signals:
    void updateLaser(const sensor_msgs::msg::LaserScan::SharedPtr msg);

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscriber_;
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
};

#endif // MAPHANDLER_H
