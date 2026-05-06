from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch(context):
    # Declare the argument to receive robot names
    robot_names_arg = DeclareLaunchArgument('robot_names', default_value='wamv1')
    
    robot_names = LaunchConfiguration('robot_names').perform(context).split()
    print(robot_names)

    # Use the received robot names in your nodes
    nodes = []
    for robot_name in robot_names:
        node = Node(
            package='lidar_processor',
            executable='lidar_processor_node',
            name='lidar_processor_node_' + robot_name,
            namespace=robot_name,
            arguments=[robot_name]
        )
        nodes.append(node)

    return nodes

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch)
    ])
