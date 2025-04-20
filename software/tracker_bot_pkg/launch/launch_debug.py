from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():

    # Blob tracker node
    blob_tracker = Node(
        package='tracker_bot_pkg',
        executable='blob_tracker',
        name='blob_tracker',
        output='screen',
        parameters=[
            {'width': 1280},
            {'height': 720},
            {'fps': 30},
            {'flip': -1}
        ],
    )

    # Image streamer node
    image_streamer = Node(
        package='tracker_bot_pkg',
        executable='image_streamer',
        name='image_streamer',
        output='screen'
    )

    # Construct launch description
    ld = LaunchDescription([
        blob_tracker,
        image_streamer
    ])

    return ld