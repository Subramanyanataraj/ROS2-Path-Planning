import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (DeclareLaunchArgument, GroupAction)
from launch.substitutions import LaunchConfiguration
from launch.substitutions import Command, LaunchConfiguration

import launch.actions
import launch_ros
from launch.substitutions import Command, LaunchConfiguration
import launch
import launch.substitutions
import launch_ros.actions
from launch_ros.substitutions import FindPackageShare
from launch.actions import TimerAction
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch_ros.actions import Node
from launch.event_handlers import OnExecutionComplete
  



def generate_launch_description():
    rviz_config = os.path.join(get_package_share_directory('path_planners'), 'config','rviz.rviz') 

    rviz_node =   Node(
        package='rviz2', 
        executable='rviz2', 
        arguments=['-d', rviz_config],
        output='screen',
        on_exit=launch.actions.Shutdown())
    
    map_node = Node(
        package='path_planners',
        executable='occupancy_map', 
        )
    
    a_star_node = Node(
        package='path_planners',
        executable='a_star', 
        )
    ld = LaunchDescription()

   
    ld.add_action(rviz_node)
    ld.add_action(TimerAction(
        period  = 2.0,
        actions =[map_node]
    ))

    ld.add_action(TimerAction(
        period  = 5.0,
        actions =[a_star_node]
    ))

    
    return ld