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

double quaternion_to_euler(double x, double y, double z, double w) {
    double t0 = +2.0 * (w * x + y * z);
    double t1 = +1.0 - 2.0 * (x * x + y * y);
    double x_rot = atan2(t0, t1);
    double t2 = +2.0 * (w * y - z * x);
    double t3 = +1.0 - 2.0 * (y * y + z * z);
    double y_rot = atan2(t2, t3);
    double t4 = +2.0 * (w * z + x * y);
    double t5 = +1.0 - 2.0 * (z * z + x * x);
    double z_rot = atan2(t4, t5);

    UNUSED(y_rot); UNUSED(z_rot);
    return x_rot; // * radian_to_degree_constant;
}

#endif // SUBSCRIBER_H

