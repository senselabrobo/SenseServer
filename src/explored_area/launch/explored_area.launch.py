from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

def start_explored_area(context: LaunchContext, robot_id):
    id_str = context.perform_substitution(robot_id)
    start_explored_area = Node(
        package='explored_area',
        executable='explored_area_node',
        name='explored_area',
        namespace='robot_' + id_str,
        output='screen',
        parameters=[{
            'robot_id': robot_id
        }]
    )
    return [start_explored_area]

def generate_launch_description():
    robot_id = LaunchConfiguration('robot_id')
    declare_robot_id = DeclareLaunchArgument('robot_id', default_value='0', description='')

    

    ld = LaunchDescription()
    ld.add_action(declare_robot_id)

    ld.add_action(OpaqueFunction(function=start_explored_area, args=[robot_id]))
    
    return ld