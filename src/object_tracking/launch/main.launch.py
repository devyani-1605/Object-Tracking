from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    talker   = Node(
        package="object_tracking",
        executable="Publisher",
        name = "Publisher"
    )

    listner = Node(
        package="object_tracking",
        executable="Subscriber",
        name = "Subscriber",
        output="screen"
    )

    return LaunchDescription([
        talker,
        listner
    ])
