from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description() -> LaunchDescription:
    launch_description = LaunchDescription()

    node = Node(executable='setpoint_publisher.py',
                package='awesome_package',
                namespace='my_namespace')
    launch_description.add_action(node)

    return launch_description
