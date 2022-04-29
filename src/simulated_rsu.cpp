#include <autoware_msgs/DetectedObjectArray.h>
#include <autoware_msgs/DetectedObject.h>
#include <time.h>
#include "iostream"
#include "ros/ros.h"

using namespace std;

int latency_upper_, latency_lower_, noise_;
autoware_msgs::DetectedObjectArray rsu;

void callback(const autoware_msgs::DetectedObjectArray &msg)
{
    float latency = rand() % latency_upper_ + latency_lower_;
    latency = latency > latency_upper_ ? latency_upper_ / 1000.0 : latency / 1000.0;
    while (ros::Time::now().toSec() - msg.header.stamp.toSec() < latency) {}

    float x_noise = (rand() % (noise_ << 1) - noise_) / 1000.0;
    float y_noise = (rand() % (noise_ << 1) - noise_) / 1000.0;

    autoware_msgs::DetectedObjectArray data;
    data.header = msg.header;
    data.header.stamp = ros::Time::now();
    data.header.frame_id = "velodyne";

    for (int i = 0; i < msg.objects.size(); i++) {
        autoware_msgs::DetectedObject object;
        object = msg.objects[i];
        object.header = data.header;
        object.convex_hull.header = data.header;
        object.latency = latency;
        object.pose.position.x += x_noise;
        object.pose.position.y += y_noise;
        data.objects.push_back(object);
    }
    rsu = data;
    
}

int main(int argc, char **argv)
{
    srand(time(NULL));
    ros::init(argc, argv, "simulated_rsu");

    ros::NodeHandle private_nh_("~");
    
    private_nh_.param<int>("latency_upper", latency_upper_, 500);
    private_nh_.param<int>("latency_lower", latency_lower_, 100);
    private_nh_.param<int>("noise", noise_, 500);

    ros::NodeHandle n;
    ros::Publisher rsu_pub = n.advertise<autoware_msgs::DetectedObjectArray>("/lgsvl/rsu/objects", 1);
    ros::Subscriber sub = n.subscribe("/lgsvl/ground_truth/objects", 1, callback);
    ros::Rate loop_rate(10);

    while (ros::ok()) {
        ros::spinOnce();
        rsu_pub.publish(rsu);
        loop_rate.sleep();
    }
    return 0;
}