from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch(context):
    # Declare the argument to receive robot names
    robot_names_arg = DeclareLaunchArgument('robot_names', default_value='wamv1')
    
    robot_names = LaunchConfiguration('robot_names').perform(context).split()
    robot_goals = LaunchConfiguration('robot_goals').perform(context).split()
    # print(robot_names)

    # Use the received robot names in your nodes
    nodes = []
    for i,robot_name in enumerate(robot_names):
        node = Node(
            package='state_processor',
            executable='state_processor_node',
            name='state_processor_node_' + robot_name,
            namespace=robot_name,
            arguments=[robot_name,robot_goals[i]]
        )
        nodes.append(node)

    return nodes

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch)
    ])
