
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare arguments
    # use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # use_random_search = LaunchConfiguration('use_random_search', default='true')
    
    # declare_sim_time_cmd = DeclareLaunchArgument(
    #     'use_sim_time',
    #     default_value='true',
    #     description='Use simulation (Gazebo) clock if true'
    # )
    
    # declare_random_search_cmd = DeclareLaunchArgument(
    #     'use_random_search',
    #     default_value='true',
    #     description='Use random waypoint search if true, else systematic Boustrophedon'
    # )

    return LaunchDescription([
        # declare_sim_time_cmd,
        # declare_random_search_cmd,

        # ---------------------------------------------------------
        # OPTION 1: Original Node (Legacy)
        # ---------------------------------------------------------
        Node(
            package='object_follower',
            executable='object_follower_node',
            name='object_follower',
            parameters=[{'target_class': 'person'}]
        ),

        # ---------------------------------------------------------
        # OPTION 2: Experiment Node (Previous Logic)
        # ---------------------------------------------------------
        Node(
            package='object_follower',
            executable='object_follower_experiment',
            name='object_follower',
            output='screen',
            parameters=[{'target_class': 'person'}]
        ),

        # ---------------------------------------------------------
        # OPTION 3: Reconstructed Node
        # ---------------------------------------------------------
        Node(
            package='object_follower',
            executable='object_follower_node_reconstructed',
            name='object_follower',
            parameters=[{'target_class': 'person'}]
        ),

        # ---------------------------------------------------------
        # OPTION 4: CSRT Tracker
        # ---------------------------------------------------------
        Node(
            package='object_follower',
            executable='object_tracker_csrt',
            name='object_follower',
            parameters=[{'target_class': 'person'}]
        ),

        # ---------------------------------------------------------
        # OPTION 5: NEW MODULAR NODE (Recommended)
        # ---------------------------------------------------------
        Node(
            package='object_follower',
            executable='object_follower_modular',
            name='object_follower',
            output='screen',
            parameters=[{'target_class': 'person'}]
        ),
        
        # ---------------------------------------------------------
        # GUI (Always Required)
        # ---------------------------------------------------------
        Node(
            package='object_follower',
            executable='object_selector_gui',
            name='object_selector_gui',
            output='screen'
        ),
        
        # PyQT5 GUI (Alternative)
        Node(
           package='object_follower',
           executable='object_selector_node_pyqt5',
           name='gui'
        ),
    ])
