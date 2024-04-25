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
        
        
};


visualization_msgs::Marker init_marker(visualization_msgs::Marker marker, float x, float y){
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "robot_pose";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = .3;
    marker.scale.y = .3;
    marker.scale.z = .3;
    marker.color.a = 1; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.lifetime = ros::Duration();
    return marker;
}
// class MapCoords{
//     public: 
//         float x;
//         float y;
//         MapCoords(int x, int y, int n){
//             this->x = (x+1) - n/2 -.5;
//             this->y = y - n/2 -.5;
//         }
// };

int main( int argc, char** argv ){
    ros::init(argc, argv, "simple_planner_main_node");
    ros::NodeHandle node_handle;
    Map map_node(node_handle);

    ros::Publisher pos_pub = node_handle.advertise<visualization_msgs::Marker>("visualization_marker",0);
    ros::Rate r(2);
    int n = 10;
    visualization_msgs::Marker marker;
   
    // for(int i = (h-1); i >=0 ;i--){ //Start from the last 
    //     for(int j  = 0; j < w; j++){
    //         int idx = (i*w) + j;
    // }
    while(ros::ok()){
        ros::spinOnce();
        if (map_node.msg_recieved){
            for(int y = 0; y < n; y++){
                for(int x = 0; x<n; x++){
                    // // OK RVIZ
                    float x_rviz = (x+1) - n/2 - .5;
                    float y_rviz = n/2 - y -.5;
                    int x_map = x;
                    int y_map = (n-1) - y;
                    int idx = x_map + (y_map*n);
                    if(map_node.get_data()[idx] == 100){
                            cout << "obstacle" << endl;
                        }
                        else if (map_node.get_data()[idx] == 0)
                        {
                            cout << "road" << endl;
                        
                        }
                        else {
                            cout << "goal" << endl;
                        }
                    marker = init_marker(marker,x_rviz,y_rviz);
                    pos_pub.publish(marker);
                    r.sleep();
                }
            }
        }
    }  
}

