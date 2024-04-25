#include "rviz_handler.h"
RvizCoord::RvizCoord(int n,int x, int y){
        this->x = (x+1) - n/2 - .5;
        this->y = n/2 - y -.5;
    }

Rviz::Rviz(ros::NodeHandle node_handle, RvizCoord coord){
            ROS_INFO("Created rviz node and started broadcasting.");
            rviz_pub = node_handle.advertise<visualization_msgs::Marker>("visualization_marker",0);
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
            robot_marker.color.a = 1; // Don't forget to set the alpha!
            robot_marker.color.r = 0.0;
            robot_marker.color.g = 1.0;
            robot_marker.color.b = 0.0;
            robot_marker.lifetime = ros::Duration();
            ROS_INFO("Created Robot_Marker and published.");
        }
        void Rviz::update_robot_position(RvizCoord coord){
            robot_marker.pose.position.x = coord.x;
            robot_marker.pose.position.y = coord.y;
            rviz_pub.publish(robot_marker);
        }
