#include <opencv2/opencv.hpp>
#include <iostream>

#include <ros/ros.h> 

using namespace std;
int main(int argc, char **argv)
{
    ros:: init(argc, argv, "simple_planner_renderer_node");
    ros::NodeHandle n;
    ROS_INFO("Renderer node started");
    
    return 0;
}