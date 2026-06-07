import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = 'clifford'

    # Ruta de los archivos necesarios
    gazebo_launch_file = os.path.join(
        get_package_share_directory(package_name), 'launch', 'sim_gazebo_clifford.launch.py'
    )
    controller_launch_file = os.path.join(
        get_package_share_directory(package_name), 'launch', 'controller_manager.launch.py'
    )

    # Lanzar Gazebo con el robot
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch_file])
    )

    # Iniciar los controladores
    controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([controller_launch_file])
    )

    # Iniciar el nodo que establece la pose inicial
    initial_pose = Node(
        package=package_name,
        executable='set_pose_node',
        output='screen'
    )
    """
    # Iniciar la locomoción
    locomotion = Node(
        package=package_name,
        executable='locomotion_node',
        output='screen'
    )
    """

    return LaunchDescription([
        gazebo,
        controllers,
        initial_pose,
        #locomotion
    ])
