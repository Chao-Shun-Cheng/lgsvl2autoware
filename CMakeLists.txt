cmake_minimum_required(VERSION 3.0.2)
project(lgsvl2autoware)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  autoware_msgs
  lgsvl_msgs
  geometry_msgs
  nav_msgs
)

catkin_package(
  CATKIN_DEPENDS 
  roscpp
  autoware_msgs
  lgsvl_msgs
  geometry_msgs
  nav_msgs
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

# ground_truth
add_executable(ground_truth src/ground_truth.cpp)
target_link_libraries(ground_truth ${catkin_LIBRARIES})

# odom2vel
add_executable(odom2vel src/odom2vel.cpp)
target_link_libraries(odom2vel ${catkin_LIBRARIES})

# nmea2enu
install(PROGRAMS
  scripts/nmea2enu.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)