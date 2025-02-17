#ifndef ROM_STRUCTURES_H
#define ROM_STRUCTURES_H

#include <rclcpp/rclcpp.hpp>

#include <QObject>
#include <QtWidgets/QApplication>

struct ROMTransform
{
    // Transform between "map" and "odom" frame
    double map_odom_x = 0.0;   // X coordinate of the transform (map -> odom)
    double map_odom_y = 0.0;   // Y coordinate of the transform (map -> odom)
    double map_odom_yaw = 0.0; // Yaw (orientation) of the transform (map -> odom)

    // Transform between "odom" and "base_footprint" frame
    double odom_base_footprint_x = 0.0;  // X coordinate of the transform (odom -> base_footprint)
    double odom_base_footprint_y = 0.0;  // Y coordinate of the transform (odom -> base_footprint)
    double odom_base_footprint_yaw = 0.0; // Yaw (orientation) of the transform (odom -> base_footprint)

    // Default constructor
    ROMTransform() = default;

    // Parameterized constructor for easier initialization
    ROMTransform(double map_odom_x, double map_odom_y, double map_odom_yaw,
                 double odom_base_footprint_x, double odom_base_footprint_y, double odom_base_footprint_yaw)
        : map_odom_x(map_odom_x), map_odom_y(map_odom_y), map_odom_yaw(map_odom_yaw),
          odom_base_footprint_x(odom_base_footprint_x), odom_base_footprint_y(odom_base_footprint_y), 
          odom_base_footprint_yaw(odom_base_footprint_yaw) {}
};



#endif // 