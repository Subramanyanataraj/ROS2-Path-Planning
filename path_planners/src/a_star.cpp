#include "a_star.hpp"
#include <cmath>

Astar::Astar() : free_space(0), start_value(-2), goal_value(-120), path_finding_color(0.00005), 
final_color(-127), color_update(98), data_recieved(false), first_run(true), goal_reached(false), m_height(0),
 m_width(0),g_cost(0),h_cost(0.0),f_cost(0.0), Node("a_star_node")
{
    map_client = create_client<nav_msgs::srv::GetMap>("get_map");
    auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();

    while (!map_client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            rclcpp::shutdown();
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
    }

    auto result_future = map_client->async_send_request(
        request, std::bind(&Astar::get_map_callback, this, std::placeholders::_1)
    );

    finding_map_pub = create_publisher<nav_msgs::msg::OccupancyGrid>("occ_algo_map", 10);
    path_pub = create_publisher<nav_msgs::msg::Path>("algo_path", 10);

    timer = create_wall_timer(0.001s, std::bind(&Astar::path_planning_callback, this));

}

void Astar::get_map_callback(rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture future)
{   
    data_recieved = true;
    result = future.get();
    map_data = *result;

    int height = map_data.map.info.height;
    int width = map_data.map.info.width;

    m_height = height;
    m_width = width;

    std::vector<int> map_list(map_data.map.data.begin(), map_data.map.data.end());
    map_2d_data.resize(height, std::vector<int>(width, 0));

    for(int j = 0; j < width; j++)
    {
        for(int i = 0; i < height; i++)
        {
            map_2d_data[j][i] = map_list[j * width + i];

            if(map_2d_data[j][i] == start_value)
            {
                start.x = i;
                start.y = j;
            }

            if(map_2d_data[j][i] == goal_value)
            {
                goal.x = i;
                goal.y = j;
            }
        }
    }

    path_node = goal;
}

void Astar::path_planning_callback()
{
    static float color_value = start_value;
    int start_x = start.x;
    int start_y = start.y;
    static std::vector<std::vector<int>> parent_map(m_height, std::vector<int>(m_width, -1));
    static std::vector<std::vector<float>> open_heap;

    RCLCPP_INFO_ONCE(this->get_logger(), "start: %d, %d", start.x, start.y);
    RCLCPP_INFO_ONCE(this->get_logger(), "goal: %d, %d", goal.x, goal.y);

    h_cost = calculate_euclidean_heuristic(start.x, start.y, goal.x, goal.y);
    f_cost = g_cost + h_cost;

    if(first_run)
    {
        first_run = false;
        open_heap.push_back({f_cost, start_x, start_y});
    }

    std::vector<std::vector<int>> movements {{1, 0}, {0, 1}, {-1, 0}, {0, -1},
                                                {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};

    if(data_recieved)
    {   static int explored = 0;

        if(!open_heap.empty())
        {
            sort(open_heap.begin(), open_heap.end());
            reverse(open_heap.begin(), open_heap.end());
            std::vector<float> next = open_heap.back();
            open_heap.pop_back();



           

            float f_cost = next[0];
            int x = static_cast<int> (next[1]);
            int y = static_cast<int> (next[2]);

/*        
        // Navigating through corner obstacles
           for(int i = 0; i < movements.size(); i++)
            {
                int neighbor_x = x + movements[i][0];
                int neighbor_y = y + movements[i][1];

                color_value -= path_finding_color;
                if(map_2d_data[neighbor_y][neighbor_x] == free_space)
                {   g_cost++;
                    explored++;
                    h_cost = calculate_chebyshev_heuristic(neighbor_x, neighbor_y, goal.x, goal.y);
                    f_cost = g_cost + h_cost;
                    // RCLCPP_INFO(this->get_logger(), " cost:  %d", g_cost);

                    open_heap.push_back({f_cost, static_cast<float> (neighbor_x), static_cast<float> (neighbor_y)});
                    map_2d_data[neighbor_y][neighbor_x] = color_value;

                    parent_map[neighbor_y][neighbor_x] = i;
                    // RCLCPP_INFO(this->get_logger(), "Set parent_map[%d][%d] = %d", neighbor_y, neighbor_x, i);
                }
                else if(map_2d_data[neighbor_y][neighbor_x] == goal_value)
                {   g_cost++;
                    h_cost = calculate_chebyshev_heuristic(neighbor_x, neighbor_y,goal.x, goal.y);
                    explored++;
                    f_cost = g_cost + h_cost;
                    data_recieved = false;
                    goal_reached = true;
                    RCLCPP_INFO_ONCE(this->get_logger(), "Number of cells explored: %d ", explored);

                    open_heap.push_back({f_cost, static_cast<float> (neighbor_x),static_cast<float> (neighbor_y)});

                    parent_map[neighbor_y][neighbor_x] = i;

                    RCLCPP_INFO_ONCE(this->get_logger(), "Goal Reached");
                }
            }

*/      
    for(int i = 0; i < movements.size(); i++)
    {
        int neighbor_x = x + movements[i][0];
        int neighbor_y = y + movements[i][1];

        if(neighbor_x >= 0 && neighbor_x < m_width && neighbor_y >= 0 && neighbor_y < m_height)
        {
            if(i < 4) // Cardinal directions
            {
                if(map_2d_data[neighbor_y][neighbor_x] == 0)
                {
                    g_cost++;
                    explored++;
                    h_cost = calculate_chebyshev_heuristic(neighbor_x, neighbor_y, goal.x, goal.y);
                    f_cost = g_cost + h_cost;

                    open_heap.push_back({f_cost, static_cast<float> (neighbor_x), static_cast<float> (neighbor_y)});
                    map_2d_data[neighbor_y][neighbor_x] = -30;

                    parent_map[neighbor_y][neighbor_x] = i;
                }
                else if(map_2d_data[neighbor_y][neighbor_x] == goal_value)
                {
                    data_recieved = false;
                    goal_reached = true;
                    g_cost++;
                    explored++;
                    h_cost = calculate_chebyshev_heuristic(neighbor_x, neighbor_y, goal.x, goal.y);
                    f_cost = g_cost + h_cost;

                    open_heap.push_back({f_cost, static_cast<float> (neighbor_x), static_cast<float> (neighbor_y)});
                    map_2d_data[neighbor_y][neighbor_x] = -30;

                    parent_map[neighbor_y][neighbor_x] = i;
                    RCLCPP_INFO_ONCE(this->get_logger(), "Number of cells explored: %d ", explored);


                }
            }
            else // Diagonal directions
            {
                // Determine which cardinal cells need to be checked
                int cardinal1_x, cardinal1_y;
                int cardinal2_x, cardinal2_y;
                // Set cardinal cells based on the diagonal direction
                switch (i)
                {  //{1, 0}, {0, 1}, {-1, 0}, {0, -1}
                // {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
                   case 4: // Diagonal {1, 1} // cardinals {1, 0}, {0, 1}
                    cardinal1_x = x + 1; cardinal1_y = y;
                    cardinal2_x = x; cardinal2_y = y + 1;
                    break;
                   case 5: // Diagonal {1, -1} // cardinals {0, 1}, {-1, -0}
                    cardinal1_x = x ; cardinal1_y = y-1;
                    cardinal2_x = x-1; cardinal2_y = y;
                    break;
                   case 6: // Diagonal {-1, 1} // cardinals {1, 0}, {0, -1}
                    cardinal1_x = x + 1; cardinal1_y = y;
                    cardinal2_x = x; cardinal2_y = y - 1;
                    break;
                   case 7: // Diagonal {-1, -1} // cardinals {-1, 0}, {0, -1}
                    cardinal1_x = x - 1; cardinal1_y = y;
                    cardinal2_x = x; cardinal2_y = y - 1;
                    break;
                   default:
                    continue; 
                }

                 // Check if the cardinal cells are not blocked
                bool cardinal1_free = (cardinal1_x >= 0 && cardinal1_x < m_width &&
                                        cardinal1_y >= 0 && cardinal1_y < m_height &&
                                        map_2d_data[cardinal1_y][cardinal1_x] != 100);
                bool cardinal2_free = (cardinal2_x >= 0 && cardinal2_x < m_width &&
                                        cardinal2_y >= 0 && cardinal2_y < m_height &&
                                        map_2d_data[cardinal2_y][cardinal2_x] != 100);

                if(cardinal1_free && cardinal2_free)
                {  

                    if(map_2d_data[neighbor_y][neighbor_x] == 0)
                    {
                        g_cost++;
                        explored++;
                        h_cost = calculate_chebyshev_heuristic(neighbor_x, neighbor_y, goal.x, goal.y);
                        f_cost = g_cost + h_cost;
                        // RCLCPP_INFO(this->get_logger(), " cost:  %d", g_cost);

                        open_heap.push_back({f_cost, static_cast<float> (neighbor_x), static_cast<float> (neighbor_y)});
                        map_2d_data[neighbor_y][neighbor_x] = -30;

                        parent_map[neighbor_y][neighbor_x] = i;

                    }
                    else if(map_2d_data[neighbor_y][neighbor_x] == goal_value)
                    {
                        data_recieved = false;
                        goal_reached = true;
                        g_cost++;
                        explored++;
                        h_cost = calculate_chebyshev_heuristic(neighbor_x, neighbor_y, goal.x, goal.y);
                        f_cost = g_cost + h_cost;
                        // RCLCPP_INFO(this->get_logger(), " cost:  %d", g_cost);

                        open_heap.push_back({f_cost, static_cast<float> (neighbor_x), static_cast<float> (neighbor_y)});
                        map_2d_data[neighbor_y][neighbor_x] = -30;

                        parent_map[neighbor_y][neighbor_x] = i;
                        RCLCPP_INFO_ONCE(this->get_logger(), "Number of cells explored: %d ", explored);


                    }
                }
            }
    }
        }

            

            if(color_value < final_color)
            {
                color_value = final_color;
            }

        
    }

    else if(open_heap.empty())
    {
        RCLCPP_INFO_ONCE(this->get_logger(), "Path cannot be generated");
        rclcpp::shutdown();
    }

    }
    else if(goal_reached)
    {
        goal_reached = false;

        int x = goal.x;
        int y = goal.y;
        static int count=0;

        while(x != start.x || y != start.y)
        {   
            count++;
            int movement_index = parent_map[y][x];
            if(movement_index == -1)
            {
                RCLCPP_ERROR(this->get_logger(), "Invalid movement index: %d", movement_index);
                break;
            }

            int x2 = x - movements[movement_index][0];
            int y2 = y - movements[movement_index][1];

            geometry_msgs::msg::PoseStamped pose_message;
            pose_message.header.stamp = get_clock()->now();
            pose_message.header.frame_id = "map";
            pose_message.pose.position.x = x + 0.5;
            pose_message.pose.position.y = y + 0.5;
            pose_message.pose.position.z = 0.0;
            pose_message.pose.orientation.w = 1.0;
            pose_message.pose.orientation.x = 0.0;
            pose_message.pose.orientation.y = 0.0;
            pose_message.pose.orientation.z = 0.0;
            path_message.header.frame_id = "map";
            path_message.header.stamp = get_clock()->now();
            path_message.poses.push_back(pose_message);

            if(x2 >= 0 && x2 < m_width && y2 >= 0 && y2 < m_height)
            {
                map_2d_data[y2][x2] = 101;
                x = x2;
                y = y2;
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Out of bounds: x=%d, y=%d", x2, y2);
                break;
            }

            // Smooth the path using Catmull-Rom spline
            nav_msgs::msg::Path smoothed_path = catmull_rom_spline(path_message, 10);
      
            path_pub -> publish(smoothed_path);
        }

         RCLCPP_INFO_ONCE(this->get_logger(), "Number of cells to path: %d ", count);

    }
   

    occ_msg.data.clear();
    occ_msg.info.resolution = 1.0;
    occ_msg.info.width = m_width;
    occ_msg.info.height = m_height;
    occ_msg.info.origin.position.x = 0.0;
    occ_msg.info.origin.position.y = 0.0;
    occ_msg.info.origin.position.z = 0.0;
    occ_msg.info.origin.orientation.x = 0.0;
    occ_msg.info.origin.orientation.y = 0.0;
    occ_msg.info.origin.orientation.z = 0.0;
    occ_msg.info.origin.orientation.w = 1.0;
    occ_msg.header.frame_id = "map";

    for(const auto& row : map_2d_data)
    {
        occ_msg.data.insert(occ_msg.data.end(), row.begin(), row.end());
    }

    finding_map_pub->publish(occ_msg);

   
    
}


 nav_msgs::msg::Path Astar::catmull_rom_spline(const nav_msgs::msg::Path path, int interpolation_points) {
    nav_msgs::msg::Path smooth_path;
    smooth_path.header.frame_id = "map";
    smooth_path.header.stamp = get_clock()-> now();
            
    int num_points = path.poses.size();

    if (num_points < 4) {
        // Not enough points for Catmull-Rom spline
        return path;
    }

    for (int i = 1; i < num_points - 2; ++i) {
        for (int j = 0; j < interpolation_points; ++j) {
            float t = static_cast<float>(j) / interpolation_points;
            float t2 = t * t;
            float t3 = t2 * t;

            geometry_msgs::msg::PoseStamped p;
            
            p.pose.position.x =
                0.5f * ((2.0f * path.poses[i].pose.position.x) +
                        (-path.poses[i - 1].pose.position.x + path.poses[i + 1].pose.position.x) * t +
                        (2.0f * path.poses[i - 1].pose.position.x - 5.0f * path.poses[i].pose.position.x + 4.0f * path.poses[i + 1].pose.position.x - path.poses[i + 2].pose.position.x) * t2 +
                        (-path.poses[i - 1].pose.position.x + 3.0f * path.poses[i].pose.position.x - 3.0f * path.poses[i + 1].pose.position.x + path.poses[i + 2].pose.position.x) * t3);
            p.pose.position.y =
                0.5f * ((2.0f * path.poses[i].pose.position.y) +
                        (-path.poses[i - 1].pose.position.y + path.poses[i + 1].pose.position.y) * t +
                        (2.0f * path.poses[i - 1].pose.position.y - 5.0f * path.poses[i].pose.position.y + 4.0f * path.poses[i + 1].pose.position.y - path.poses[i + 2].pose.position.y) * t2 +
                        (-path.poses[i - 1].pose.position.y + 3.0f * path.poses[i].pose.position.y - 3.0f * path.poses[i + 1].pose.position.y + path.poses[i + 2].pose.position.y) * t3);
            smooth_path.poses.push_back(p);

        }
    }

    // Add the last point to the smoothed path
    smooth_path.poses.push_back(path.poses.back());

    return smooth_path;
}

// Function to calculate the Euclidean distance heuristic
inline float Astar::calculate_euclidean_heuristic(int x1, int y1, int x2, int y2) {
    return static_cast<float>(std::sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1)));
}

// Function to calculate the Manhattan distance heuristic
inline float Astar::calculate_manhattan_heuristic(int x1, int y1, int x2, int y2) {
    return std::abs(x2 - x1) + std::abs(y2 - y1);
}

// Function to calculate the Chebyshev distance heuristic
 inline float Astar::calculate_chebyshev_heuristic(int x1, int y1, int x2, int y2) {
    return std::max(std::abs(x2 - x1), std::abs(y2 - y1));
}


Astar::~Astar(){}


int main(int argc, char* argv[])
{
rclcpp::init(argc,argv);
rclcpp::spin(std::make_shared<Astar>());
rclcpp::shutdown();
return 0;
}