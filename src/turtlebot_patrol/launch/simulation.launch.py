from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    share=get_package_share_directory('turtlebot_patrol')
    return LaunchDescription([
        DeclareLaunchArgument('gui',default_value='true'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share,'launch','patrol.launch.py'))),
        Node(package='turtlebot_patrol',executable='simulator',output='screen'),
        Node(package='turtlebot_patrol',executable='dashboard',output='screen',condition=IfCondition(LaunchConfiguration('gui')))])
