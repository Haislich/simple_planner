#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <iostream>
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
    Map map_node(node_handle);
    RvizCoord starting_position(10,0,0);
    Rviz rviz(node_handle,starting_position);
    while(ros::ok()){
        ros::spinOnce();
        if (map_node.msg_recieved){
            for(int y = 0; y < n; y++){
                for(int x = 0; x<n; x++){
                    RvizCoord rviz_coord(10,x,y);
                    rviz.update_robot_position(rviz_coord);

                    MapCoord map_coord(10, x, y);
                    map_node.get_element_at(map_coord);
                    r.sleep();
                }
            }
        }
    }  
}

