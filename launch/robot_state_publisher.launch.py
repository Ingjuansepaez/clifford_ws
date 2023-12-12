# Libraries for file handling
import os
from ament_index_python import get_package_share_directory
import xacro
# Libraries for node launching
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument

pkg_filepath = get_package_share_directory("clifford_gazebo")

use_sim_time = LaunchConfiguration("use_sim_time")
urdf_filepath = os.path.join(pkg_filepath, "description", "clifford.urdf.xacro")
robot_description_file = xacro.process_file(urdf_filepath)

def generate_launch_description():
    robot_state_publisher_node = Node(
        package= "robot_state_publisher",
        executable= "robot_state_publisher",
        parameters=[
            {
                "robot_description": robot_description_file.toxml(),
                "use_sim_time": use_sim_time
            }
        ]
    )

    nodes_to_run = [
            DeclareLaunchArgument(
                "use_sim_time", 
                default_value= "false", 
                description= "Use sim time if true"
            ),
            robot_state_publisher_node
    ]
    return LaunchDescription(nodes_to_run)
