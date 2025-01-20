#ifndef ROM_ALGORITHM_H
#define ROM_ALGORITHM_H

#ifndef UNUSED
#define UNUSED(x) (void)(x)
#endif

#include <rclcpp/rclcpp.hpp>

#include <QObject>
#include <QtWidgets/QApplication>

double degree_to_radian_constant = 0.017453292519943295;
double radian_to_degree_constant = 57.29577951308232;
double foot_to_meter_constant = 0.3048;
double meter_to_foot_constant = 3.28084;

void quaternion_to_euler(double x, double y, double z, double w, double &roll, double &pitch, double &yaw) {
    double t0 = +2.0 * (w * x + y * z);
    double t1 = +1.0 - 2.0 * (x * x + y * y);
    roll = atan2(t0, t1);

    double t2 = +2.0 * (w * y - z * x);
    double t3 = +1.0 - 2.0 * (y * y + z * z);
    pitch = atan2(t2, t3);

    double t4 = +2.0 * (w * z + x * y);

    double t5 = +1.0 - 2.0 * (z * z + x * x);

    yaw = atan2(t4, t5);
}

double quaternion_to_euler_yaw(double x, double y, double z, double w)
{
    double yaw = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    return yaw;
}

void yaw_to_quaternion(double yaw, double &z, double &w)
{
    w = cos(yaw / 2);
    //x = 0;
    //y = 0;
    z = sin(yaw / 2);
}
#endif // SUBSCRIBER_H

