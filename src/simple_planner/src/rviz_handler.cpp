#include "rviz_handler.h"
RvizCoord::RvizCoord(int x, int y, int w, int h) {
  this->x = x - w / 2 + (1 - w % 2) * .5;
  this->y = h / 2 - y - (1 - h % 2) * .5;
  this->w = w;
  this->h = h;
}

RvizMap::RvizMap(ros::NodeHandle node_handle) {
  ROS_INFO("Created rviz node and started broadcasting.");
  rviz_pub = node_handle.advertise<visualization_msgs::Marker>(
      "visualization_marker", 0);
  robot_marker.header.frame_id = "map";
  robot_marker.header.stamp = ros::Time();
  robot_marker.ns = "robot_pose";
  robot_marker.id = 0;
  robot_marker.type = visualization_msgs::Marker::SPHERE;
  robot_marker.action = visualization_msgs::Marker::ADD;
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

  // Get initial position
  initial_pos_sub = node_handle.subscribe("initialpose", 1,
                                          &RvizMap::initial_pos_callback, this);
}
void RvizMap::initial_pos_callback(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
  geometry_msgs::PoseWithCovarianceStamped initial_pose = *msg;
  ROS_INFO("Received initial pose: %0.2f, %0.2f",
           initial_pose.pose.pose.position.x,
           initial_pose.pose.pose.position.y);
  initial_pos_x = initial_pose.pose.pose.position.x < 0
                      ? floor(initial_pose.pose.pose.position.x) + .5
                      : ceil(initial_pose.pose.pose.position.x) - .5;
  initial_pos_y = ceil(initial_pose.pose.pose.position.y) - .5;
  std::cout << initial_pos_x << ", " << initial_pos_y << std::endl;
  initial_pos_recieved = true;
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
