#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>

sensor_msgs::PointCloud2 data;

void callback(const sensor_msgs::PointCloud2 &msg)
{
    data = msg;
    data.header.stamp = ros::Time::now();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pointscloud");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2>("points_raw", 10);
    ros::Subscriber sub = n.subscribe("/points_raw_lg", 1, callback);
    ros::Rate loop_rate(10);

    while (ros::ok()) {
        ros::spinOnce();
        pub.publish(data);
        loop_rate.sleep();
    }
    return 0;
}
