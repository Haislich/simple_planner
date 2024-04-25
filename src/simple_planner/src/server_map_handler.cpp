#include "server_map_handler.h"

MapCoord::MapCoord(int n, int x, int y){
            int x_map = x;
            int y_map = (n-1) - y;
            this->idx = x_map + (y_map*n);
        }

Map::Map(ros::NodeHandle node_handle){
            ROS_INFO("Created map node and subscribed to map.");
            map_sub = node_handle.subscribe("map",1,&Map::mapCallBack,this);
        }
void Map::mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    msg_recieved = true;
    std::cout << msg->info.height << std::endl;
    occupancy_grid = *msg;
}
std::vector<int8_t, std::allocator<int8_t>> Map::get_data(){
    return occupancy_grid.data;
}
int Map::get_width(){
    return occupancy_grid.info.width;
}
int Map::get_height(){
    return occupancy_grid.info.height;
}
MapElement Map::get_element_at(MapCoord coords){
    switch (occupancy_grid.data[coords.idx]){
        case 0:
            std::cout << "Road" << std::endl;
            return MapElement::Road;
        case -1:
            std::cout << "Goal" << std::endl;
            return MapElement::Goal;
        case 100:
            std::cout << "Obstacle" << std::endl;
            return MapElement::Obstacle;
        default:
            std::cout << "Not Valid" << std::endl;
            return MapElement::NotValid;
    }  
}