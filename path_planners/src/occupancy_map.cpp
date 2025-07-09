#include "occupancy_map.hpp"




OccupancyMap::OccupancyMap() : m_first_run (true), m_rows(100), m_columns(100), m_start({0, 0}), m_goal({79, 49}), Node("occupancy_map_pub")
{
    declare_parameter<std::string>("frame_id", "map");
    declare_parameter<std::string>("topic_occ_map", "occ_map");
    declare_parameter<std::string>("topic_getmap", "get_map");

    get_parameter("frame_id", frame_id);
    get_parameter("topic_occ_map", topic_occ_map);
    get_parameter("topic_getmap", topic_getmap);

    occupancy_map_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>(topic_occ_map, 10);

    

    // timer = this->create_wall_timer(1s, std::bind(&OccupancyMap::create_map, this));
    create_map();
    get_map_srv = this->create_service<nav_msgs::srv::GetMap>(
    topic_getmap,
    std::bind(&OccupancyMap::map_callback, this, std::placeholders::_1, std::placeholders::_2));
}

void OccupancyMap::map_callback(const std::shared_ptr<nav_msgs::srv::GetMap::Request> request,
                                const std::shared_ptr<nav_msgs::srv::GetMap::Response> response)
{
    response->map= occupancy_msg;
    RCLCPP_INFO(this->get_logger(), "GetMAp service successful");
}

void OccupancyMap::create_map()
{
    std::vector<std::vector<int>> map_data(m_columns, std::vector<int>(m_rows, 0));

    // Fill outer walls with 100 (obstacles)
    for (int i = 0; i < m_rows; ++i)
    {
        map_data[0][i] = 100;           // Top wall
        map_data[m_columns - 1][i] = 100; // Bottom wall
    }

    for (int i = 0; i < m_columns; ++i)
    {
        map_data[i][0] = 100;           // Left wall
        map_data[i][m_rows - 1] = 100;  // Right wall
    }

    // Create structured wall-like obstacles
    std::random_device rd;
    std::mt19937 rng(rd());
    std::uniform_int_distribution<int> dist_row(1, m_rows - 2);
    std::uniform_int_distribution<int> dist_col(1, m_columns - 2);
    std::uniform_int_distribution<int> dist_length(5, 10); // Length of the wall segments

    for (int i = 0; i < 250; ++i)  // Adjust the number of wall segments for desired complexity
    {
        int start_row = dist_row(rng);
        int start_col = dist_col(rng);
        int length = dist_length(rng);
        bool horizontal = rng() % 2;

        for (int j = 0; j < length; ++j)
        {
            int row = start_row + (horizontal ? 0 : j);
            int col = start_col + (horizontal ? j : 0);

            if (row < (m_rows - 1) && col < (m_columns - 1))
            {
                map_data[col][row] = 100;  // Set obstacles
            }
        }
    }


     // Generate random start position
    std::uniform_int_distribution<int> dist_start_row(1 ,m_rows-2 );
    std::uniform_int_distribution<int> dist_start_col(1, int(m_columns/4));

    do {
        m_start[0] = dist_start_col(rng);
        m_start[1] = dist_start_row(rng);
    } while (map_data[m_start[0]][m_start[1]] == 100);

    // Generate random goal position
    std::uniform_int_distribution<int> dist_goal_row( 1 ,m_rows-2);
    std::uniform_int_distribution<int> dist_goal_col( int(m_columns/1.3),m_columns-2);

    do {
        m_goal[0] = dist_goal_col(rng);
        m_goal[1] = dist_goal_row(rng);
    } while ((map_data[m_goal[0]][m_goal[1]] == 100) || (m_goal[0] == m_start[0] && m_goal[1] == m_start[1]));
    // Set start and goal positions
    map_data[m_start[0]][m_start[1]] = -2;
    map_data[m_goal[0]][m_goal[1]] = -120;

    RCLCPP_INFO(this->get_logger(), "start:  {%d,%d}",m_start[0], m_start[1] );
    RCLCPP_INFO(this->get_logger(), "Goal:  {%d,%d}",m_goal[0], m_goal[1] );

    // Populate occupancy_msg with map data
    occupancy_msg.header.frame_id = frame_id;
    occupancy_msg.info.resolution = 1.0;
    occupancy_msg.info.width = m_rows;
    occupancy_msg.info.height = m_columns;
    occupancy_msg.info.origin.position.x = 0.0;
    occupancy_msg.info.origin.position.y = 0.0;
    occupancy_msg.info.origin.position.z = 0.0;
    occupancy_msg.info.origin.orientation.x = 0.00;
    occupancy_msg.info.origin.orientation.y = 0.00;
    occupancy_msg.info.origin.orientation.z = 0.00;
    occupancy_msg.info.origin.orientation.w = 1.00;

    // Flatten map_data into occupancy_msg.data
    if(m_first_run==true){    
        occupancy_msg.data.clear();
        m_first_run = false;
        }
    RCLCPP_INFO(this->get_logger(), "create map successful");
    for (const auto &row : map_data)
    {
        occupancy_msg.data.insert(occupancy_msg.data.end(), row.begin(), row.end());
    }

    // Publish occupancy_msg
    occupancy_map_pub->publish(occupancy_msg);
    
}

OccupancyMap::~OccupancyMap() {}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OccupancyMap>());
    rclcpp::shutdown();
    return 0;
}

