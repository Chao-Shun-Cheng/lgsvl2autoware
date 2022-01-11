#!/usr/bin/env python

import rospy
import tf
import math
import pymap3d as pm

from nmea_msgs.msg import Sentence
from geometry_msgs.msg import PoseStamped

# The local coordinate origin (NCKU EE)
lat0 = 22.99665575170912 # deg      
lon0 = 120.22259226369121  # deg
h0 = 98.211     # meters

# initial global variable
orientation_time_ = position_time_ = current_time_ = 0
x_ = y_ = z_ = 0
roll_ = pitch_ = yaw_ = 0
br_ = tf.TransformBroadcaster()

# initial publisher
pub1_ = rospy.Publisher('gnss_pose', PoseStamped, queue_size=10)

def nmea2llh(sentence):
    # global variable
    global position_time_, orientation_time_, current_time_
    global x_, y_, z_, roll_, pitch_, yaw_ 

    # parse nmea sentence
    #rospy.loginfo("I heard %s", sentence.sentence)
    split_sentence = sentence.sentence.split(',')

    current_time_ = sentence.header.stamp
    # current_time_ = rospy.get_rostime() # add by Kenny

    # only grep $GPGGA sentence
    if split_sentence[0] == "$GPGGA":
        position_time_ = float(split_sentence[1])

        # Deprecate

        #lat_decimal_ = nmea2decimal(float(split_sentence[2]), split_sentence[3])
        #lon_decimal_ = nmea2decimal(float(split_sentence[4]), split_sentence[5])
        #height_ = float(split_sentence[9])

        #ell = pm.Ellipsoid("wgs84")

        #x_, y_, z_ = pm.geodetic2enu(lat_decimal_, lon_decimal_, height_, lat0, lon0, h0, ell)

        # Magic Happen: Just pass the vehicle transform which is ENU data to /gnss_pose.
        # This can eliminate errors caused by coordinate transformation.

        x_ =  float(split_sentence[13])
        y_ =  float(split_sentence[14])
        z_ =  float(split_sentence[15])   

        # print("------debug------")
        # print(split_sentence)

    elif split_sentence[0] == "QQ02C":
        orientation_time_ = float(split_sentence[3])
        roll_ = float(split_sentence[4]) * math.pi / 180
        pitch_ = -1 * float(split_sentence[5]) * math.pi / 180
        yaw_ = -1 * float(split_sentence[6]) * math.pi / 180 + math.pi / 2

    e = 1e-2
    if abs(orientation_time_ - position_time_) < e:
        publishPoseStamped()
        publishTF()


# Convert the nmea longtitude into decimal form.
# Source: https://scwrm.pixnet.net/blog/post/304778087-how-to-convert-gps-nmea-to-decimal 
       
def nmea2decimal(nmea, dir):
    nmea_dd = (int)(nmea) / 100
    nmea_mm = nmea - (int)(nmea_dd * 100)
    nmea_decimal = (int)(nmea_dd) + (nmea_mm / 60)

    if (dir == "W" or dir == "S"):
        nmea_decimal *= -1

    return nmea_decimal

def publishPoseStamped():
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = current_time_
    pose.pose.position.x = x_
    pose.pose.position.y = y_
    pose.pose.position.z = z_
    quaternion = tf.transformations.quaternion_from_euler(roll_, pitch_, yaw_)
    pose.pose.orientation.x = quaternion[0]
    pose.pose.orientation.y = quaternion[1]
    pose.pose.orientation.z = quaternion[2]
    pose.pose.orientation.w = quaternion[3]
    pub1_.publish(pose)

def publishTF():
    br_.sendTransform((x_, y_, z_), 
                    tf.transformations.quaternion_from_euler(roll_, pitch_, yaw_),
                    current_time_,
                    "gps",
                    "map")

# void Nmea2TFPoseNode::publishTF()
# {
#   tf::Transform transform;
#   transform.setOrigin(tf::Vector3(geo_.y(), geo_.x(), geo_.z()));
#   tf::Quaternion quaternion;
#   quaternion.setRPY(roll_, pitch_, yaw_);
#   transform.setRotation(quaternion);
#   br_.sendTransform(tf::StampedTransform(transform, current_time_, MAP_FRAME_, GPS_FRAME_));
# }

if __name__ == '__main__':
    rospy.init_node('nmea2enu')
    rospy.Subscriber("nmea_sentence", Sentence, nmea2llh)

    rospy.spin()
    #print(pm.geodetic2enu(lat, lon, h, lat0, lon0, h0))
