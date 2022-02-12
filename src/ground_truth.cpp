#include <autoware_msgs/DetectedObjectArray.h>
#include <autoware_msgs/DetectedObject.h>
#include <lgsvl_msgs/Detection3DArray.h>
#include "iostream"
#include "ros/ros.h"

using namespace std;

autoware_msgs::DetectedObjectArray result;
void generatePolygon(autoware_msgs::DetectedObject &object);
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
        object.convex_hull.header = data.header;
        object.space_frame = "velodyne";
        object.pose_reliable = true;
        object.velocity_reliable = true;
        object.valid = true;
        object.source = "lgsvl";
        object.latency = latency;

        object.id = msg.detections[i].id;
        object.label = msg.detections[i].label;
        object.score = msg.detections[i].score;
        object.velocity = msg.detections[i].velocity;
        object.pose = msg.detections[i].bbox.position;
        object.dimensions = msg.detections[i].bbox.size;
        generatePolygon(object);
        
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

void generatePolygon(autoware_msgs::DetectedObject &object)
{
    float longitudinal_distance = object.dimensions.y / 2;
    float lateral_distance = object.dimensions.x / 2;
    float vertial_distance = object.dimensions.z;

    geometry_msgs::Point32 p;
    
    // FRU (front, right, up)
    p.y = object.pose.position.y + longitudinal_distance;
    p.x = object.pose.position.x + lateral_distance;
    p.z = vertial_distance;
    object.convex_hull.polygon.points.push_back(p);

    // FRD
    p.y = object.pose.position.y + longitudinal_distance;
    p.x = object.pose.position.x + lateral_distance;
    p.z = 0;
    object.convex_hull.polygon.points.push_back(p);

    // FLU
    p.y = object.pose.position.y + longitudinal_distance;
    p.x = object.pose.position.x - lateral_distance;
    p.z = vertial_distance;
    object.convex_hull.polygon.points.push_back(p);

    // FLD
    p.y = object.pose.position.y + longitudinal_distance;
    p.x = object.pose.position.x - lateral_distance;
    p.z = 0;
    object.convex_hull.polygon.points.push_back(p);

    // BRU (back, right, up)
    p.y = object.pose.position.y - longitudinal_distance;
    p.x = object.pose.position.x + lateral_distance;
    p.z = vertial_distance;
    object.convex_hull.polygon.points.push_back(p);

    // BRD
    p.y = object.pose.position.y - longitudinal_distance;
    p.x = object.pose.position.x + lateral_distance;
    p.z = 0;
    object.convex_hull.polygon.points.push_back(p);

    // BLU
    p.y = object.pose.position.y - longitudinal_distance;
    p.x = object.pose.position.x - lateral_distance;
    p.z = vertial_distance;
    object.convex_hull.polygon.points.push_back(p);

    // BLD
    p.y = object.pose.position.y - longitudinal_distance;
    p.x = object.pose.position.x - lateral_distance;
    p.z = 0;
    object.convex_hull.polygon.points.push_back(p);
}