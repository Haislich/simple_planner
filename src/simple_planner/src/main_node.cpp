#include <ros/ros.h>

#include "planner.h"
#include "rviz_handler.h"
#include "server_map_handler.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "simple_planner_main_node");

  ros::NodeHandle node_handle;
  ros::Rate r(2);
  Map map(node_handle);
  int n = map.get_height();
  MapCoord initial_position(MapCoord(0, 0, n));
  RvizMap rviz_map(node_handle,
                   RvizCoord(n, initial_position.x, initial_position.y));
  RvizPath rviz_path(node_handle);
  bool already_planned = false;
  std::vector<RvizCoord> path;
  while (ros::ok()) {
    ros::spinOnce();
    if (map.msg_recieved) {
      if (!already_planned) {
        BFS bfs(map);
        path = bfs.plan(MapCoord(0, 0, n));
        already_planned = true;
      }

      if (already_planned && rviz_map.rviz_pub.getNumSubscribers() >= 1 &&
          rviz_path.path_pub.getNumSubscribers() >= 1) {
        std::cout << path.size() << std::endl;
        for (size_t i = 0; i < path.size(); i++) {
          rviz_map.update_robot_position(path[i]);
          rviz_path.update_path(path[i]);
          r.sleep();
          // std::cout << path[i].x << path[i].y << std::endl;
        }
        return 0;
      }
    }
    // rviz_map.update_robot_position()
  }
}
