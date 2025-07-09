#include "dfs.hpp"
#include <stack> // 

DFS::DFS() : free_space(0), start_value(-2), goal_value(-120), path_finding_color(0.00005), 
final_color(-127), color_update(98), data_recieved(false), first_run(true), goal_reached(false), m_height(0), m_width(0), Node("dfs_node")
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
        request, std::bind(&DFS::get_map_callback, this, std::placeholders::_1)
    );

    finding_map_pub = create_publisher<nav_msgs::msg::OccupancyGrid>("occ_algo_map", 10);
    path_pub = create_publisher<nav_msgs::msg::Path>("algo_path", 10);

    timer = create_wall_timer(0.001s, std::bind(&DFS::path_planning_callback, this));
}

void DFS::get_map_callback(rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture future)
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

void DFS::path_planning_callback()
{
    static float color_value = start_value;
    int start_x = start.x;
    int start_y = start.y;
    static std::vector<std::vector<int>> parent_map(m_height, std::vector<int>(m_width, -1));
    static std::stack<std::vector<int>> open_stack;

    RCLCPP_INFO_ONCE(this->get_logger(), "start: %d, %d", start.x, start.y);
    RCLCPP_INFO_ONCE(this->get_logger(), "goal: %d, %d", goal.x, goal.y);

    if(first_run)
    {
        first_run = false;
        open_stack.push({ start_x, start_y});
    }

    std::vector<std::vector<int>> movements {{1, 0}, {0, 1}, {-1, 0}, {0, -1}};

    if(data_recieved)
    {
        if(!open_stack.empty())
        {
            std::vector<int> next = open_stack.top();
            open_stack.pop();

            int x = next[0];
            int y = next[1];

            for(int i = 0; i < movements.size(); i++)
            {
                int neighbor_x = x + movements[i][0];
                int neighbor_y = y + movements[i][1];

                color_value -= path_finding_color;

                if(map_2d_data[neighbor_y][neighbor_x] == free_space)
                {
                    open_stack.push({ neighbor_x, neighbor_y});
                    map_2d_data[neighbor_y][neighbor_x] = color_value;

                    parent_map[neighbor_y][neighbor_x] = i;
                    // RCLCPP_INFO(this->get_logger(), "Set parent_map[%d][%d] = %d", neighbor_y, neighbor_x, i);
                }
                else if(map_2d_data[neighbor_y][neighbor_x] == goal_value)
                {
                    data_recieved = false;
                    goal_reached = true;
                
                    open_stack.push({ neighbor_x, neighbor_y});

                    parent_map[neighbor_y][neighbor_x] = i;

                    RCLCPP_INFO_ONCE(this->get_logger(), "Goal Reached");
                }
            }

            if(color_value < final_color)
            {
                color_value = final_color;
            }
        }

         else if(open_stack.empty())
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

        while(x != start.x || y != start.y)
        {
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

            path_pub->publish(path_message);
        }
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

DFS::~DFS(){} 

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DFS>());
    rclcpp::shutdown();
    return 0;
}