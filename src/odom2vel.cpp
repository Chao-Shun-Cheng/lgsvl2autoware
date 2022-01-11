#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include "ros/ros.h"

geometry_msgs::TwistStamped data;

void gps_odometry_callback(const nav_msgs::Odometry &msg)
{
    data.header = msg.header;
    data.twist = msg.twist.twist;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom2vel");
    ros::NodeHandle n;
    ros::Publisher vel_pub = n.advertise<geometry_msgs::TwistStamped>("odom_vel", 1);
    ros::Subscriber sub1 = n.subscribe("/gps_odometry", 1, gps_odometry_callback);
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        vel_pub.publish(data);
        loop_rate.sleep();
    }
    return 0;
}
