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
    ros::NodeHandle n;
    ros::Subscriber sub2 = n.subscribe("map", 1000, mapCallBack);

    // ros::Subscriber sub2 = n.subscribe("initialpose", 1000, &initialPoseCallback);
    // ros::Subscriber sub3 = n.subscribe("move_base_simple/goal", 1000, &goalPoseCallback);
    while(ros::ok()){
        ros::spinOnce();
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

