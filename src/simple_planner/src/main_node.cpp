#include "planner.h"
#include "rviz_handler.h"
#include "server_map_handler.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "simple_planner_main_node");

  ros::NodeHandle node_handle;
  ros::Rate r(3);
  Map map(node_handle);
  std::vector<RvizCoord> path;
  int initial_x = atoi(argv[1]);
  int initial_y = atoi(argv[2]);
  while (ros::ok()) {
    while (!map.msg_recieved) {
      ros::spinOnce();
      ROS_INFO_ONCE("Waiting to get data from map_server...");
    }
    ROS_INFO("Map data recieved.");

    if (map.get_height() != map.get_width()) {
      ROS_ERROR("Unsopported Map. Maps must be square matrixes.");
      return 1;
    }
    int n = map.get_height();
    // if (n % 2 != 0) {
    //   ROS_ERROR(
    //       "Unsopported Map. Maps must have an even number of row and
    //       columns.");
    //   return 1;
    // }
    MapCoord initial_position(MapCoord(initial_x, initial_y, n));
    RvizMap rviz_map(node_handle,
                     RvizCoord(initial_position.x, initial_position.y, n));

    RvizPath rviz_path(node_handle);

    if (map.get_element_at(initial_position) == MapElement::Obstacle) {
      ROS_ERROR("No path could be found from this sarting position.");
      return 1;
    }
    if (initial_x >= n || initial_x < 0 || initial_y >= n || initial_y < 0) {
      ROS_ERROR("Invalid starting position.");
      return 1;
    }
    BFS bfs(map);
    ROS_INFO("Planning...");
    path = bfs.plan(initial_position);

    while (rviz_map.rviz_pub.getNumSubscribers() < 1 &&
           rviz_path.path_pub.getNumSubscribers() < 1) {
      ROS_INFO_ONCE("Waiting for rviz to subscribe.");
    }
    ROS_INFO("Press any key to start.");
    std::cin.get();
    if (path.size() == 0) {
      ROS_WARN("No path could be found.");
      return 0;
    }
    for (size_t i = 0; i < path.size(); i++) {
      rviz_map.update_robot_position(path[i]);
      rviz_path.update_path(path[i]);
      r.sleep();
    }
    ROS_INFO("Press any key to stop.");
    std::cin.get();
    return 0;
  }
}