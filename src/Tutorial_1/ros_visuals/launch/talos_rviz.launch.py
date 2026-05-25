

from launch import LaunchDescription

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os



def generate_launch_description():

    package_name = "ros_visuals"


    urdf_path = os.path.abspath("src/Tutorial_2/talos_description/robots/talos_reduced.urdf")
    path_meshes = "src/Tutorial_2/talos_description/meshes/../.."
    rviz_config= "/home/ubuntu/Documents/legged_robots/rviz_config/talos.rviz"


    with open(urdf_path, "r") as f:

        robot_description = f.read()

   


    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
    )


    sim_node = Node(
        package="bullet_sims",
        executable="t23",
        name="t23",
        output="screen",
    )


    robot_state_publisher_node = Node(

        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {

                "use_sim_time": False,
                "robot_description": robot_description,

            }

        ],

    )


    return LaunchDescription([

        rviz_node,

        robot_state_publisher_node,

        sim_node,

    ])
