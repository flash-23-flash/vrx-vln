from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import vrx_gz.bridges


def generate_launch_description():
    package_share = FindPackageShare('vrx_vln_bringup')

    world_default = PathJoinSubstitution(
        [package_share, 'worlds', 'vrx_vln_empty.sdf']
    )
    model_config_default = PathJoinSubstitution(
        [package_share, 'config', 'wamv_single.yaml']
    )
    rviz_config_default = PathJoinSubstitution(
        [package_share, 'config', 'vrx_vln_sensors.rviz']
    )

    vrx_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('vrx_gz'), 'launch', 'competition.launch.py']
            )
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'sim_mode': 'full',
            'bridge_competition_topics': 'False',
            'config_file': LaunchConfiguration('model_config'),
            'headless': LaunchConfiguration('headless'),
            'paused': LaunchConfiguration('paused'),
            'competition_mode': 'False',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'init_poses': '0,0,0',
            'goals': '0,0',
            'buoy_poses': '',
        }.items(),
    )

    lidar_processor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('lidar_processor'), 'launch', 'lidar_processor.launch.py']
            )
        ),
        launch_arguments={'robot_names': 'wamv'}.items(),
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[vrx_gz.bridges.clock().argument()],
        remappings=[vrx_gz.bridges.clock().remapping()],
    )

    sensor_markers = Node(
        package='vrx_vln_bringup',
        executable='sensor_marker_publisher',
        namespace='wamv',
        name='sensor_marker_publisher',
        parameters=[
            {
                'robot_name': 'wamv',
                'base_frame': 'wamv/wamv/base_link',
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=world_default,
            description='Absolute path to the VRX_VLN world SDF file.',
        ),
        DeclareLaunchArgument(
            'model_config',
            default_value=model_config_default,
            description='Absolute path to the single-USV model config YAML.',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=rviz_config_default,
            description='Absolute path to the RViz configuration file.',
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='False',
            description='Run Gazebo without its GUI.',
        ),
        DeclareLaunchArgument(
            'paused',
            default_value='False',
            description='Start Gazebo paused.',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='True',
            description='Launch RViz alongside Gazebo.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use Gazebo simulation time in visualization nodes.',
        ),
        vrx_launch,
        lidar_processor,
        clock_bridge,
        sensor_markers,
        rviz,
    ])
