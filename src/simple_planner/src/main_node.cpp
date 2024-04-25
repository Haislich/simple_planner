#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h> 
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <iostream>
#include <vector>

using namespace std;

 
// void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){}
// void goalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){}

enum MapElement{
    Obstacle,
    Road,
    Goal
};
class MapCoord{
    public: 
        int idx;
        MapCoord(int n, int x, int y){
            int x_map = x;
            int y_map = (n-1) - y;
            this->idx = x_map + (y_map*n);
        }
};
class Map{
    private:
        nav_msgs::OccupancyGrid occupancy_grid;
        ros::Subscriber map_sub;
    public:
        bool msg_recieved = false;
        Map(ros::NodeHandle node_handle){
            ROS_INFO("Created map node and subscribed to map.");
            map_sub = node_handle.subscribe("map",1,&Map::mapCallBack,this);
        }
        void mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg){
            msg_recieved = true;
            cout << msg->info.height << endl;
            occupancy_grid = *msg;
        }
        std::vector<int8_t, std::allocator<int8_t>> get_data(){
            return occupancy_grid.data;
        }
        int get_width(){
            return occupancy_grid.info.width;
        }
        int get_height(){
            return occupancy_grid.info.height;
        }
        MapElement get_element_at(MapCoord coords){
            switch (occupancy_grid.data[coords.idx]){
                case 0:
                    cout << "Road" << endl;
                    return MapElement::Road;
                case -1:
                    cout << "Goal" << endl;
                    return MapElement::Goal;
                case 100:
                    cout << "Obstacle" << endl;
                    return MapElement::Obstacle;
                // default: handle exception
            }  
        }
};
class RvizCoord{
    public:
    float x;
    float y;
    RvizCoord(int n,int x, int y){
        this->x = (x+1) - n/2 - .5;
        this->y = n/2 - y -.5;
    }
};
class Rviz{
    private:
        nav_msgs::OccupancyGrid occupancy_grid;
        visualization_msgs::Marker robot_marker;
        ros::Publisher rviz_pub;
    public:
        Rviz(ros::NodeHandle node_handle, RvizCoord coord){
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
        void update_robot_position(RvizCoord coord){
            robot_marker.pose.position.x = coord.x;
            robot_marker.pose.position.y = coord.y;
            rviz_pub.publish(robot_marker);
        }
};

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

