# Libraries for file handling
import os
from ament_index_python import get_package_share_directory
# Libraries for node launching
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch import LaunchDescription

pkg_filepath = get_package_share_directory("clifford_gazebo")
robot_state_publisher_launch_filepath = os.path.join(pkg_filepath, "launch", "robot_state_publisher.launch.py")
rviz_config_filepath = os.path.join(pkg_filepath, "config", "visualization.rviz")

def generate_launch_description():
    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_state_publisher_launch_filepath]),
        launch_arguments= {
            'use_sim_time': 'true'
        }.items()
    )

    rviz_cmd = ExecuteProcess(
        cmd= ["rviz2", "-d", rviz_config_filepath]
    )

    inverse_kinematics_publisher_node = Node(
        package= "clifford_gazebo",
        executable= "inverse_kinematics_publisher.py"
    )

    serial_port_writer_node = Node(
        package= "clifford_gazebo",
        executable= "serial_port_writer.py",
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        }
    )

    nodes_to_run = [
        robot_state_publisher_launch, 
        rviz_cmd, 
        inverse_kinematics_publisher_node,
        serial_port_writer_node
    ]
    return LaunchDescription(nodes_to_run)