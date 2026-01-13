#ifndef ODOMETRY_H
#define ODOMETRY_H

typedef struct
{
    float vel_l;  // Filtered velocity Left
    float vel_r;  // Filtered velocity Right
    double x;     // Odom X
    double y;     // Odom Y
    double theta; // Odom Yaw
} robot_state_t;

void configure_encoders(void);
void get_robot_state(robot_state_t *copy);
void update_robot_state(float dt);

#endif // ODOMETRY_H