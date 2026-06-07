import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    package_name = 'clifford'
    # Ruta del mundo en Gazebo
    world_file_path = os.path.join(
        os.getenv('HOME'),
        'clifford_ws', 'src', 'clifford', 'worlds', 'empty.world'
    )

    # Ruta del archivo SDF del robot
    sdf_path = os.path.join(
        os.getenv('HOME'), 
        'clifford_ws', 'src', 'clifford', 'description', 'clifford_core.sdf'
    )

    # Argumento para usar tiempo de simulación
    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')

    # Lanzar Gazebo con el mundo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={'world': world_file_path}.items()
    )

    # Cargar el robot desde el SDF
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-file', sdf_path, '-entity', 'clifford'],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time,
        gazebo,
        spawn_entity
    ])
