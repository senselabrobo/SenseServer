from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    network_port = LaunchConfiguration('network_port')
    network_ip = LaunchConfiguration('network_ip')

    declare_network_port = DeclareLaunchArgument('network_port', default_value='12130', description='')
    declare_network_ip = DeclareLaunchArgument('network_ip', default_value='192.168.31.8', description='')

    multi_transform_node = Node(
        package='multi_transform',
        executable='multi_transform_node',
        name='multi_transform',
        output='screen',
        respawn=True,
        parameters=[{
            'network_port': network_port,
            'network_ip': network_ip,
        }]
    )

    ld = LaunchDescription()
    
    ld.add_action(declare_network_port)
    ld.add_action(declare_network_ip)
    
    ld.add_action(multi_transform_node)
    return ld