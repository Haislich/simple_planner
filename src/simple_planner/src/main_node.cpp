#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h> 
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <iostream>
#include <vector>

using namespace std;

void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
    ROS_INFO("Dentro a InitialPose");
    cout << msg->pose.pose << endl;
    ROS_INFO("esco da InitialPose");

}
void goalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    ROS_INFO("Dentro a goalPose");
    ROS_INFO("Esco da goalPose");


}
void mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    int h = msg->info.height;
    int w = msg->info.width;
    // For absolutely no reason
    // a matrix like this:
    // 1 2 3
    // 4 5 6 
    // Is converted into this vector
    // 4 5 6 1 2 3  
    for(int i =(h-1); i >=0 ;i--){ //Start from the last 
        for(int j  = 0; j < w; j++){
            int idx = (i*w) + j;
            if (msg->data[idx] == 100){
                cout << "0 " ;
            }
            else if ((msg->data[idx]) == 0)
            {
                cout << "2 " ;
               
            }
            else {
               cout << "1 ";
            }
            
        }
        cout << endl;
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
class MapCoords{
    public: 
        float x;
        float y;
        MapCoords(int x, int y, int n){
            this->x = (x+1) - n/2 -.5;
            this->y = y - n/2 -.5;
        }
};

int main( int argc, char** argv ){
    ros::init(argc, argv, "simple_planner_main_node");
    // cv::Mat image = cv::imread("/home/lattinone/catkin_ws/simple_planner_ws/maps/map1.pgm",cv::IMREAD_GRAYSCALE);
    // for(int i = 0; i < image.cols; i ++){
    //     for (int j =0; j< image.rows;j++){
    //         cout << static_cast<int>(image.at<uint8_t>(i,j)) << " ";
    //     }
    //     cout << endl;
    // }
    // cv::Mat resized;
    // cv::resize(image,resized,cv::Size2d(400,400),0,0,cv::InterpolationFlags::INTER_NEAREST);
    // cv::imshow("Culo",resized);
    // cv::waitKey(0);
    ros::NodeHandle node_handle;
    // // ros::Subscriber sub2 = n.subscribe("map", 1000, mapCallBack);

    // ros::Subscriber sub2 = n.subscribe("initialpose", 1000, &initialPoseCallback);
    // ros::Subscriber sub3 = n.subscribe("move_base_simple/goal", 1000, &goalPoseCallback);
    ros::Publisher pos_pub = node_handle.advertise<visualization_msgs::Marker>("visualization_marker",0);
    ros::Rate r(2);
    int n = 10;
    visualization_msgs::Marker marker;
    while(ros::ok()){
        for(int j = n; j > 0;j--){
            for(int i = 0; i < n ; i ++){
                float x = (i+1) - n/2 -.5;
                float y = j - n/2 -.5;
                marker = init_marker(marker,x,y);
                pos_pub.publish(marker);
                r.sleep();
            }
        }
    }
    // while (marker_pub.getNumSubscribers() < 1)
    // {
    //   if (!ros::ok())
    //   {
    //     return 0;
    //   }
    //   ROS_WARN_ONCE("Please create a subscriber to the marker");
    //   sleep(1);
    // }

    
  
}

