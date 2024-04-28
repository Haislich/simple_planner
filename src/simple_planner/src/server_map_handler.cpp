#include "server_map_handler.h"

MapCoord::MapCoord(int x, int y, int w, int h) {
  int x_map = x;
  int y_map = (h - 1) - y;
  this->idx = x_map + (y_map * w);
  this->x = x;
  this->y = y;
}

Map::Map(ros::NodeHandle node_handle) {
  ROS_INFO("Subscribed to map_serve_node");
  map_sub = node_handle.subscribe("map", 100, &Map::mapCallBack, this);
}
void Map::mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  occupancy_grid = *msg;
  msg_recieved = true;
}
std::vector<int8_t, std::allocator<int8_t>> Map::get_data() {
  return occupancy_grid.data;
}
int Map::get_width() { return occupancy_grid.info.width; }
int Map::get_height() { return occupancy_grid.info.height; }
MapElement Map::get_element_at(MapCoord coords) {
  switch (occupancy_grid.data[coords.idx]) {
    case 0:
      return MapElement::Road;
    case -1:
      return MapElement::Goal;
    case 100:
      return MapElement::Obstacle;
    default:
      return MapElement::NotValid;
  }
}
