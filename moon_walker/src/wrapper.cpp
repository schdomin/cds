#include "wrapper.h"
#include "geometry_msgs/Twist.h"

void _set_speed(float v, float omega){
    geometry_msgs::Twist a;
    a.linear.x=v;
    a.angular.z=omega;
    cmd_vel_pub.publish(a);
}
