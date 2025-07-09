#ifndef __DFS__HPP__
#define __DFS__HPP__

#include "rclcpp/rclcpp.hpp"
#include <chrono>


#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/srv/get_map.hpp"
#include <std_msgs/msg/string.hpp>
#include "custom_interfaces/msg/point.hpp"


using namespace std::chrono_literals;
using namespace std::this_thread;
using namespace std::chrono;

class DFS : public  rclcpp::Node 
{
public :

explicit DFS();
~DFS();

void get_map_callback(rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture future);
void path_planning_callback();
void create_timer();

private:



rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr map_client;

int free_space;
int start_value;
int goal_value;
float path_finding_color;
int final_color;
int color_update;
bool data_recieved;

bool first_run;
bool goal_reached;

std::vector<std::vector<int>> parent_map;


int m_height;
int m_width;


nav_msgs::srv::GetMap::Response map_data;
std::shared_ptr<nav_msgs::srv::GetMap_Response> result;
  
   
rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;

nav_msgs::msg::Path path_message; 
rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr finding_map_pub;


rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr DFS_pub;
nav_msgs::msg::OccupancyGrid occ_msg;

rclcpp::TimerBase::SharedPtr timer;

custom_interfaces::msg::Point start = custom_interfaces::msg::Point();
custom_interfaces::msg::Point goal = custom_interfaces::msg::Point();
custom_interfaces::msg::Point path_node = custom_interfaces::msg::Point();

std::vector<std::vector<int>> map_2d_data;
// nav_msgs::msg::OccupancyGrid occupancy_msg =  nav_msgs::msg::OccupancyGrid();


};
#endif