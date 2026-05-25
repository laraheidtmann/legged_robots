from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_name = "ros_visuals"

    rviz_config= "/home/ubuntu/Documents/legged_robots/rviz_config/cage.rviz"




    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
    )

    return LaunchDescription([
        rviz_node,
    ])