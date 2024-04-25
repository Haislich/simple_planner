#pragma once 
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

class RvizCoord{
    public:
    float x;
    float y;
    RvizCoord(int n,int x, int y);
};
class Rviz{
    private:
        visualization_msgs::Marker robot_marker;
        ros::Publisher rviz_pub;
    public:
        Rviz(ros::NodeHandle node_handle, RvizCoord coord);
        void update_robot_position(RvizCoord coord);
};