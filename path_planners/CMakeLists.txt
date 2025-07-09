cmake_minimum_required(VERSION 3.8)
project(path_planners)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


add_executable(occupancy_map src/occupancy_map.cpp)
ament_target_dependencies(occupancy_map rclcpp nav_msgs)


add_executable(bfs src/bfs.cpp)
ament_target_dependencies(bfs rclcpp custom_interfaces nav_msgs)

add_executable(dfs src/dfs.cpp)
ament_target_dependencies(dfs rclcpp custom_interfaces nav_msgs)

add_executable(dijkstra src/dijkstra.cpp)
ament_target_dependencies(dijkstra rclcpp custom_interfaces nav_msgs)

add_executable(a_star src/a_star.cpp)
ament_target_dependencies(a_star rclcpp custom_interfaces nav_msgs)


# target_include_directories(occupancy_map PRIVATE ${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/include)


install(TARGETS
  occupancy_map 
  dfs 
  bfs 
  dijkstra
  a_star
  DESTINATION lib/${PROJECT_NAME})


install(FILES
DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
launch
config
DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
