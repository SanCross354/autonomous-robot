import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node



def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='articubot_TA' #<--- CHANGE BASED ON OUR PROJECT'S NAME

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    spawn_yolo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('yolobot_recognition'),'launch', 'launch_yolov8.launch.py'
                )])
    )

    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
        )

    gazebo_params_file = os.path.join(get_package_share_directory(package_name),'config','gazebo_params.yaml')

    # Factory world file (can be overridden via command line: world:=/path/to/file.world)
    default_world = os.path.join(get_package_share_directory(package_name), 'worlds', 'factory.world')
    world = LaunchConfiguration('world', default=default_world)

    # Set GAZEBO_MODEL_PATH so gzserver can find factory world models
    gazebo_model_path = os.path.join(os.path.expanduser('~'), '.gazebo', 'models')
    existing_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    full_model_path = gazebo_model_path + ':' + existing_model_path if existing_model_path else gazebo_model_path
    set_gazebo_model_path = SetEnvironmentVariable('GAZEBO_MODEL_PATH', full_model_path)

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={
                        'world': world,
                        'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file
                    }.items()
             )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_bot'],
                        output='screen')

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["diff_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_broad"],
    )

    delayed_spawn_entity = TimerAction(
        period=5.0,  # delay 5 seconds to make sure Gazebo and clock are ready
        actions=[spawn_entity]
    )

    delayed_diff_drive_spawner = TimerAction(
        period=7.0,
        actions=[diff_drive_spawner]
    )

    delayed_joint_broad_spawner = TimerAction(
        period=7.5,
        actions=[joint_broad_spawner]
    )


    # Launch them all!
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=default_world, description='Path to Gazebo world file'),
        set_gazebo_model_path,    # must come first so gzserver inherits the env var
        rsp,
        spawn_yolo,
        gazebo,
        twist_mux,
        # spawn_entity,
        # diff_drive_spawner,
        # joint_broad_spawner,
        delayed_spawn_entity,     # spawns robot into factory world
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner,
    ])