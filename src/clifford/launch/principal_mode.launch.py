from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node 
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    param_declare_arg = DeclareLaunchArgument('modo',
                                              default_value='home',
                                              description='nombre del parametro a cambiar'
                                              )

    nodo_parametro = Node(
        package='clifford',
        executable='locomotion_node_2',
        name='locomotion_unique',
        output='screen',
        parameters=[{
            'modo': LaunchConfiguration(
                'modo'
            )
        }]
    )

    nodo_ros2arduino = Node(
        package='clifford',
        executable='ros2arduino_node',
        name='ros2arduino',
        output='screen',
    )

    nodo_trayectoria = Node(
        package='clifford',
        executable='trayectory_node',
        name='trayectory',
        output='screen',
    )

    """nodo_visual_trayectoria = Node(
        package='clifford',
        executable='visual_line_node',
        name='visual',
        output='screen',
    )"""

    """nodo_tf_change = Node(
        package='clifford',
        executable='change_tf_node',
        name='tf2',
        output='screen',
    )"""

    return LaunchDescription([
        LogInfo(msg="INICIANDO EL LAUNCH PARA LA LOCOMOCION"),
        LogInfo(msg="--------------------------------------"),
        LogInfo(msg="--------------------------------------"),
        param_declare_arg,
        nodo_parametro,
        nodo_trayectoria,
        nodo_ros2arduino, 
        #nodo_visual_trayectoria, 
        #nodo_tf_change, 

    ])
