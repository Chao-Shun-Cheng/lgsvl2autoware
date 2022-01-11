#include <autoware_msgs/DetectedObjectArray.h>
#include <autoware_msgs/DetectedObject.h>
#include <lgsvl_msgs/Detection3DArray.h>
#include "iostream"
#include "ros/ros.h"

using namespace std;

autoware_msgs::DetectedObjectArray result;

void d3_detections_callback(const lgsvl_msgs::Detection3DArray &msg)
{
    ros::Time now = ros::Time::now();
    float latency = now.toSec() - msg.header.stamp.toSec();
    autoware_msgs::DetectedObjectArray data;
    data.header = msg.header;
    data.header.frame_id = "velodyne";
    for (int i = 0; i < msg.detections.size(); i++) {
        autoware_msgs::DetectedObject object;
        object.header = data.header;
        object.id = msg.detections[i].id;
        object.label = msg.detections[i].label;
        object.score = msg.detections[i].score;
        object.velocity = msg.detections[i].velocity;
        object.pose = msg.detections[i].bbox.position;
        object.source = "lgsvl";
        object.latency = latency;
        object.dimensions = msg.detections[i].bbox.size;
        object.pose_reliable = true;
        object.velocity_reliable = true;
        object.valid = true;
        object.space_frame = "velodyne";
        object.convex_hull.header = data.header;
        data.objects.push_back(object);
    }
    result = data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ground_truth");
    ros::NodeHandle n;
    ros::Publisher ground_truth_pub = n.advertise<autoware_msgs::DetectedObjectArray>("/simulator/ground_truth/objects", 1);
    ros::Subscriber sub = n.subscribe("/simulator/ground_truth/3d_detections", 1, d3_detections_callback);
    ros::Rate loop_rate(10);

    while (ros::ok()) {
        ros::spinOnce();
        ground_truth_pub.publish(result);
        loop_rate.sleep();
    }
    return 0;
}
