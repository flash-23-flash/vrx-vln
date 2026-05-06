from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch(context):
    # Declare the argument to receive robot names
    robot_names_arg = DeclareLaunchArgument('robot_names', default_value='wamv1')
    
    robot_names = LaunchConfiguration('robot_names').perform(context)
    buoy_poses = LaunchConfiguration('buoy_poses').perform(context)

    node = Node(
        package='collision_detector',
        executable='collision_detector_node',
        name='collision_detector',
        arguments=[robot_names,buoy_poses]
    )

    return [node]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch)
    ])