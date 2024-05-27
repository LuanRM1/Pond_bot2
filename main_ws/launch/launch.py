from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="image_publisher",
                executable="publisher",
                name="Image_Publisher",
            ),
            Node(
                package="new_bringup",
                executable="bringup_manager",
                name="bringup_manager",
            ),
        ]
    )

