#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <nav_msgs/msg/odometry.h>

typedef struct
{
    float left;
    float right;
} wheel_vel_t;

wheel_vel_t get_wheel_velocities(float dt);
void configure_encoders(void);
void update_odometry(nav_msgs__msg__Odometry *msg);

#endif // ODOMETRY_H