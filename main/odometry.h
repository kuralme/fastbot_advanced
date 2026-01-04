#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <nav_msgs/msg/odometry.h>

void configure_encoders(void);
void update_odometry(nav_msgs__msg__Odometry *msg);

#endif // ODOMETRY_H