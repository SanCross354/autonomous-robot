# object_follower/launch/object_follower_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='object_follower',
            executable='object_follower_node',
            name='object_follower',
            parameters=[{'target_class': 'person'}]
        ),
        Node(
            package='object_follower',
            executable='object_follower_experiment',
            name='object_follower',
            parameters=[{'target_class': 'person'}]
        ),
        Node(
            package='object_follower',
            executable='object_follower_node_reconstructed',
            name='object_follower',
            parameters=[{'target_class': 'person'}]
        ),
        Node(
            package='object_follower',
            executable='object_tracker_csrt',
            name='object_follower',
            parameters=[{'target_class': 'person'}]
        ),
        Node(
            package='object_follower',
            executable='object_selector_gui',
            name='gui'
        ),
         Node(
            package='object_follower',
            executable='object_selector_node_pyqt5',
            name='gui'
        ),
    ])
