import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def launch_robot_rviz(context: LaunchContext, robot_count):
    robot_count_int = int(context.perform_substitution(robot_count))
    start_list = []
    for idx in range(robot_count_int):
        # start_robot_rviz = Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     arguments=['-d', os.path.join(get_package_share_directory('visualization_bringup'), 'rviz', 'robot_' + str(idx) + '_visualization.rviz')],
        #     output='screen'
        # )
        # start_list.append(start_robot_rviz)
        start_explored_area = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('explored_area'), 'launch', 'explored_area.launch.py')
            ),
            launch_arguments={
                'robot_id': str(idx)
            }.items()
        )
        start_list.append(start_explored_area)
    return start_list

def generate_launch_description():
    robot_count = LaunchConfiguration('robot_count')
    declare_robot_count = DeclareLaunchArgument('robot_count', default_value='2', description='')

    rviz_config_file = os.path.join(get_package_share_directory('visualization_bringup'), 'rviz', 'multi_visualization.rviz')

    start_foxglove_bridge = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(os.path.join(
            get_package_share_directory('foxglove_bridge'), 'launch', 'foxglove_bridge_launch.xml')
        )
    )

    start_gicp_rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    start_multi_transform = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('multi_transform'), 'launch', 'multi_transform.launch.py')
        ),
    )

    ld = LaunchDescription()

    # Add the actions
    ld.add_action(declare_robot_count)
    
    ld.add_action(TimerAction(period=5.0, actions=[OpaqueFunction(function=launch_robot_rviz, args=[robot_count])]))

    ld.add_action(start_foxglove_bridge)
    ld.add_action(start_gicp_rviz)
    ld.add_action(start_multi_transform)

    return ld