import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro
def generate_launch_description():
    # Ruta al archivo URDF
    pkg_name = 'clifford'
    file = 'description/clifford.urdf.xacro'

    xacro_file = os.path.join(get_package_share_directory(pkg_name),file)   
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    world_file_path = os.path.join(
        os.getenv('HOME'),
        'clifford_ws', 'src', 'clifford', 'my_robot_config', 'my_robot_config.rviz'
    )
     
    # Nodo para publicar el modelo URDF en el tópico /robot_description
    node_robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description':robot_description_raw, 
                         'use_sim_time': True}]
        )
    
    node_robot_state_publisher_gui = Node(
        package= "joint_state_publisher_gui",
        executable= "joint_state_publisher_gui",
        )
    
    # Nodo para lanzar RViz con la configuración predeterminada
    rviz = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', world_file_path]
            
        )
    
    return LaunchDescription([
        node_robot_state_publisher,
        rviz,
        node_robot_state_publisher_gui,

    ])
