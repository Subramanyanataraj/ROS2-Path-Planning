#ifndef __A_STAR_HPP__/*define*/
#define __A_STAR_HPP__/*define*/

#include "rclcpp/rclcpp.hpp"

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

class Astar : public rclcpp::Node
{
    public:

   explicit Astar();
    ~Astar();
    void get_map_callback(rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture future);
    void path_planning_callback();
    void create_timer();
    float calculate_euclidean_heuristic(int x1, int y1, int x2, int y2);
    float calculate_manhattan_heuristic(int x1, int y1, int x2, int y2);
    float calculate_chebyshev_heuristic(int x1, int y1, int x2, int y2);

    nav_msgs::msg::Path catmull_rom_spline(
    const nav_msgs::msg::Path  path, int interpolation_points);
private:



    rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr map_client;

    int free_space;
    int start_value;
    int goal_value;
    float path_finding_color;
    int final_color;
    int color_update;
    bool data_recieved;
    int g_cost;
    float h_cost;
    float f_cost;
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


    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr a_star_pub;
    nav_msgs::msg::OccupancyGrid occ_msg;

    rclcpp::TimerBase::SharedPtr timer;

    custom_interfaces::msg::Point start = custom_interfaces::msg::Point();
    custom_interfaces::msg::Point goal = custom_interfaces::msg::Point();
    custom_interfaces::msg::Point path_node = custom_interfaces::msg::Point();

    std::vector<std::vector<int>> map_2d_data;
    // nav_msgs::msg::OccupancyGrid occupancy_msg =  nav_msgs::msg::OccupancyGrid();


};




#endif ///*define*/
