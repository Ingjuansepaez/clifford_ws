# Libraries for file handling
import os
from ament_index_python import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
# Libraries for node launching
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription

share_pkg_filepath = get_package_share_directory("clifford_gazebo")

gazebo_launch_filepath = os.path.join(share_pkg_filepath, "launch", "gazebo.launch.py")

controller_filepath = os.path.join(
    share_pkg_filepath, 
    "config", 
    "joint_group_position_controller.yaml"
)

def generate_launch_description():
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch_filepath])
    )

    controller_manager_node = Node(
        package= "controller_manager",
        executable= "ros2_control_node",
        parameters= [controller_filepath],
        arguments = [ 
            '--topic', '~/robot_description'
        ]
    )

    jsb_spawner_node = Node(
        package = "controller_manager",
        executable = "spawner",
        arguments = [
            "joint_state_broadcaster", 
            "--controller-manager", "/controller_manager"
        ]
    )
    
    jpc_spawner_node = Node(
        package = "controller_manager",
        executable ="spawner",
        arguments = [
            "joint_group_position_controller", 
            "--controller-manager", "/controller_manager"
        ]
    )

    nodes_to_run = [
        gazebo_launch, 
        controller_manager_node, 
        jsb_spawner_node, 
        jpc_spawner_node
    ]
    return LaunchDescription(nodes_to_run)