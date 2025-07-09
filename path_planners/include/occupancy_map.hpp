#ifndef __MAP_HPP__
#define __MAP_HPP__


#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/srv/get_map.hpp"

#include <string>
#include <vector>
#include <random>

#include <memory>



using namespace std::chrono_literals;

class OccupancyMap : public rclcpp::Node
{
public:

    explicit OccupancyMap();

    ~OccupancyMap();
    void create_map();
    void map_callback(const std::shared_ptr<nav_msgs::srv::GetMap::Request> request,
                           const std::shared_ptr<nav_msgs::srv::GetMap::Response> response);
private:
   int m_start[2];
   int m_goal[2];

   int m_rows;
   int m_columns; 
   bool m_first_run;
   // std::vector<std::vector<int>> map_data(m_columns, std::vector<int>(m_rows, 0));

   rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_map_pub;
   nav_msgs::msg::OccupancyGrid occupancy_msg =  nav_msgs::msg::OccupancyGrid();

   rclcpp::Service<nav_msgs::srv::GetMap>::SharedPtr get_map_srv;
   
   std::string topic_occ_map;
   std::string topic_getmap;
   std::string frame_id;

   
   


   rclcpp::TimerBase::SharedPtr timer;

};




#endif