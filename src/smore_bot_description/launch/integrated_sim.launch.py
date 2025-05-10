import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Packages
    pkg_share = get_package_share_directory('smore_bot_description')
    moveit_pkg_share = get_package_share_directory('simple_robot_moveit_config')
    
    # Launch Gazebo simulation
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share, 'launch', 'gazebo.launch.py')
        ])
    )
    
    # Launch MoveIt with the robot in simulation
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(moveit_pkg_share, 'launch', 'move_group.launch.py')
        ])
    )
    
    # Launch RViz with MoveIt configuration
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(moveit_pkg_share, 'launch', 'moveit_rviz.launch.py')
        ])
    )
    
    return LaunchDescription([
        gazebo_launch,
        moveit_launch,
        rviz_launch
    ])
