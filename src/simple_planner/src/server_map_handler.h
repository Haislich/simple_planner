#pragma once
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <stdio.h>

enum MapElement { Obstacle, Road, Goal, NotValid };
class MapCoord {
 public:
  int x;
  int y;
  int idx;
  MapCoord(int x, int y, int w, int h);
};
class Map {
 private:
  nav_msgs::OccupancyGrid occupancy_grid;
  ros::Subscriber map_sub;

 public:
  bool msg_recieved = false;
  Map(ros::NodeHandle node_handle);
  void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
  std::vector<int8_t, std::allocator<int8_t>> get_data();
  int get_width();
  int get_height();
  MapElement get_element_at(MapCoord coords);
};