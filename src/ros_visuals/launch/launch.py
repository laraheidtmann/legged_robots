from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_name = "ros_visuals"

    rviz_config = os.path.join(
        get_package_share_directory(package_name),
        "rviz",
        "cage_visualization.rviz",
    )

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