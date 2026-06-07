import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    controller_config = os.path.join(
        get_package_share_directory('clifford'),
        'config',
        'joint_group_trajectory_controller.yaml'
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controller_config],
        output='screen'
    )

    return LaunchDescription([controller_manager])
