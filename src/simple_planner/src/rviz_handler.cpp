#include "rviz_handler.h"
RvizCoord::RvizCoord(int x, int y, int n) {
  if (n % 2) {
    this->x = x - n / 2;
    this->y = n / 2 - y;
  } else {
    this->x = (x + 1) - n / 2 - .5;
    this->y = n / 2 - y - .5;
  }
  this->n = n;
}

RvizMap::RvizMap(ros::NodeHandle node_handle, RvizCoord coord) {
  ROS_INFO("Created rviz node and started broadcasting.");
  rviz_pub = node_handle.advertise<visualization_msgs::Marker>(
      "visualization_marker", 0);
  robot_marker.header.frame_id = "map";
  robot_marker.header.stamp = ros::Time();
  robot_marker.ns = "robot_pose";
  robot_marker.id = 0;
  robot_marker.type = visualization_msgs::Marker::SPHERE;
  robot_marker.action = visualization_msgs::Marker::ADD;
  robot_marker.pose.position.x = coord.x;
  robot_marker.pose.position.y = coord.y;
  robot_marker.pose.position.z = 0;
  robot_marker.pose.orientation.x = 0.0;
  robot_marker.pose.orientation.y = 0.0;
  robot_marker.pose.orientation.z = 0.0;
  robot_marker.pose.orientation.w = 1.0;
  robot_marker.scale.x = .3;
  robot_marker.scale.y = .3;
  robot_marker.scale.z = .3;
  robot_marker.color.a = 1;  // Don't forget to set the alpha!
  robot_marker.color.r = 0.0;
  robot_marker.color.g = 1.0;
  robot_marker.color.b = 0.0;
  robot_marker.lifetime = ros::Duration();
  rviz_pub.publish(robot_marker);
  ROS_INFO("Created Robot_Marker and published.");
}
void RvizMap::update_robot_position(RvizCoord coord) {
  robot_marker.pose.position.x = coord.x;
  robot_marker.pose.position.y = coord.y;
  rviz_pub.publish(robot_marker);
}

RvizPath::RvizPath(ros::NodeHandle node_handle) {
  // Create a publisher to visualize in real time the path.
  path_pub = node_handle.advertise<nav_msgs::Path>("/path", 1);
  // Populate Header Values
  path.header.stamp = ros::Time();
  path.header.frame_id = "map";
}
void RvizPath::update_path(RvizCoord coord) {
  // A path contains a vector of poses.
  // The idea is to add each new visited position
  // as a new pose.
  geometry_msgs::PoseStamped pose;
  // Populate pose header.
  pose.header.stamp = ros::Time();
  pose.header.frame_id = "map";
  // Each pose is determined by a position and orientation
  geometry_msgs::Point position;
  position.x = coord.x;
  position.y = coord.y;
  position.z = 0;

  geometry_msgs::Quaternion orientation;

  orientation.x = 0;
  orientation.y = 0;
  orientation.z = 0;
  orientation.w = 0;
  pose.pose.orientation = orientation;
  pose.pose.position = position;
  // We add at the end of the vector the new pose
  path.poses.push_back(pose);
  // We comunicate the new path vector to Rviz
  path_pub.publish(path);
}
