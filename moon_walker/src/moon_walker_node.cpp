#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"
#include "function.h"
#include "wrapper.h"
#include <sstream>

ros::Publisher cmd_vel_pub;

void gpsCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
 // ROS_INFO("I received this pose from gps: x[%f] y[%f] theta[%f]", msg->x,msg->y, msg->theta);

  gps_callback(msg->x,msg->y, msg->theta, ros::Time::now().toSec());



}

void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg){
    //ROS_INFO("I received this odometry: x[%f] y[%f] theta[%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, tf::getYaw(msg->pose.pose.orientation));

   odometry_callback(msg->pose.pose.position.x,msg->pose.pose.position.y, tf::getYaw(msg->pose.pose.orientation),ros::Time::now().toSec());
}

int main(int argc, char **argv)
{
    /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
    ros::init(argc, argv, "talker");

    /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
    ros::NodeHandle n;

    /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
    cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

    /**
    * The subscribe() call is how you tell ROS that you want to receive messages
    * on a given topic.  This invokes a call to the ROS
    * master node, which keeps a registry of who is publishing and who
    * is subscribing.  Messages are passed to a callback function, here
    * called chatterCallback.  subscribe() returns a Subscriber object that you
    * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
    * object go out of scope, this callback will automatically be unsubscribed from
    * this topic.
    *
    * The second parameter to the subscribe() function is the size of the message
    * queue.  If messages are arriving faster than they are being processed, this
    * is the number of messages that will be buffered up before beginning to throw
    * away the oldest ones.
    */
    ros::Subscriber gps_sub = n.subscribe("gps_pose", 1000, gpsCallback);
    ros::Subscriber odometry_sub = n.subscribe("odometry", 1000, odometryCallback);

    ros::spin();


    return 0;
}
