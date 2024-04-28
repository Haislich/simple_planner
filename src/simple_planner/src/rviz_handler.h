#pragma once
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <visualization_msgs/Marker.h>

#include <vector>

class RvizCoord {
 public:
  float x;
  float y;
  int n;
  RvizCoord(int x, int y, int n);
};
class RvizMap {
 public:
  visualization_msgs::Marker robot_marker;
  ros::Publisher rviz_pub;

  RvizMap(ros::NodeHandle node_handle, RvizCoord coord);
  void update_robot_position(RvizCoord coord);
};

class RvizPath {
 public:
  ros::Publisher path_pub;
  nav_msgs::Path path;

  RvizPath(ros::NodeHandle node_handle);
  void update_path(RvizCoord coord);
};