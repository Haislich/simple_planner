#include <ros/ros.h>

#include <iostream>
#include <nav_msgs/Path.h>

#include <std_msgs/Header.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

#include "server_map_handler.h"
#include "rviz_handler.h"

using namespace std;

 
// void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){}
// void goalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){}

int main( int argc, char** argv ){
    ros::init(argc, argv, "simple_planner_main_node");
    int n = 10;
    ros::NodeHandle node_handle;
    ros::Rate r(2);
    // Map map_node(node_handle);
    ros::Publisher path_pub = node_handle.advertise<nav_msgs::Path>("/path",1);
    nav_msgs::Path path;
    
    path.header.stamp = ros::Time();
    path.header.frame_id = "map";

    RvizCoord starting_position(10,0,0);
    Rviz rviz(node_handle,starting_position);
    while(ros::ok()){
        ros::spinOnce();
            for(int y = 0; y < n; y++){
                for(int x = 0; x<n; x++){
                    
                    RvizCoord rviz_coord(10,x,y);
                    rviz.update_robot_position(rviz_coord);
                    geometry_msgs::PoseStamped pose;
                    pose.header.stamp = ros::Time();
                    pose.header.frame_id ="map";
                        // position 
                        geometry_msgs::Point position;
                        position.x = rviz_coord.x;
                        position.y =rviz_coord.y;
                        position.z = 0;
                        // direction
                        geometry_msgs::Quaternion direction;
                        direction.x = 1;
                        direction.y = 1;
                        direction.z = 0;
                        direction.w = 0;
                    pose.pose.orientation = direction;
                    pose.pose.position = position;
                    path.poses.push_back(pose);
                    r.sleep(); 
            }
            path_pub.publish(path);
        }
    }  
}

