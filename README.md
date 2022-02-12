# lgsvl2autoware
This package is to generate the positon and velocity of the ego vehicle via lgsvl_bridge. On the other hand, it can provide the 3D ground truth of surroundung vehicles.  
## nmea2enu.py
This python file can produce the pose via unity accuracy position. This position is same as real world, and the origin point is in NCKUEE building.  
### How to use
#### LG Simulator
Publish the `nmea_sentence`
#### Autoware
```bash
roslaunch lgsvl2autoware nmea2tfpose.launch
```
## odom2vel.cpp
This cpp file can produce the velocity of ego vehicle.  
### How to use
#### LG Simulator
Publish the `/gps_odometry`
#### Autoware
```bash
rosrun lgsvl2autoware odom2vel
```
## ground_truth.cpp
This cpp file can produce the ground truth of surrounding vehicles including pose, velocity and acceleration.
### How to use
#### LG Simulator
Publish the `/simulator/ground_truth/3d_detections`
#### Autoware
```bash
roslaunch lgsvl2autoware ground_truth.launch
```
