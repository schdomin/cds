#include "ros/ros.h"

extern ros::Publisher cmd_vel_pub;

void _set_speed(float v, float omega);
