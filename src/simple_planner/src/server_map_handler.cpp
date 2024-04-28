#include "server_map_handler.h"

MapCoord::MapCoord(int x, int y, int n) {
  int x_map = x;
  int y_map = (n - 1) - y;
  this->idx = x_map + (y_map * n);
  this->x = x;
  this->y = y;
}
bool MapCoord::operator==(const MapCoord& other_point) const {
  return x == other_point.x && y == other_point.y;
}
size_t MapCoord::HashFunction::operator()(const MapCoord& point) const {
  size_t xHash = std::hash<int>()(point.x);
  size_t yHash = std::hash<int>()(point.y) << 1;
  return xHash ^ yHash;
}

Map::Map(ros::NodeHandle node_handle) {
  ROS_INFO("Created map node and subscribed to map.");
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
      // std::cout << "Road" << std::endl;
      return MapElement::Road;
    case -1:
      // std::cout << "Goal" << std::endl;
      return MapElement::Goal;
    case 100:
      // std::cout << "Obstacle" << std::endl;
      return MapElement::Obstacle;
    default:
      // std::cout << "Not Valid" << std::endl;
      return MapElement::NotValid;
  }
}
