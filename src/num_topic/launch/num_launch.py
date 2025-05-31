from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    talker_node = Node(
        package="num_topic",
        executable="number_publisher"
    )

    listener_node = Node(
        package="num_topic",
        executable="number_subscriber"
    )

    ld.add_action(talker_node)
    ld.add_action(listener_node)

    return ld


