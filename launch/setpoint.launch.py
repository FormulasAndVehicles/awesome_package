from ament_index_python.packages import get_package_share_path
from launch_ros.actions import Node, PushRosNamespace

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    launch_description = LaunchDescription()

    arg = DeclareLaunchArgument('vehicle_name')
    launch_description.add_action(arg)

    node = Node(executable='setpoint_publisher.py', package='awesome_package')
    group = GroupAction([
        PushRosNamespace(LaunchConfiguration('vehicle_name')),
        node,
    ])
    launch_description.add_action(group)

    package_path = get_package_share_path('fav')
    launch_path = str(package_path / 'launch/simulation.launch.py')
    source = PythonLaunchDescriptionSource(launch_path)
    launch_args = {'vehicle_name': LaunchConfiguration('vehicle_name')}
    action = IncludeLaunchDescription(source,
                                      launch_arguments=launch_args.items())
    launch_description.add_action(action)

    return launch_description
