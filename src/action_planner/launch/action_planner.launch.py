from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch(context):
    # Declare the argument to receive robot names
    robot_names_arg = DeclareLaunchArgument('robot_names', default_value='wamv1')
    
    method = LaunchConfiguration('method').perform(context)
    robot_names = LaunchConfiguration('robot_names').perform(context).split()
    model_path = LaunchConfiguration('model_path').perform(context)
    agent_type = LaunchConfiguration('agent_type').perform(context)
    print(robot_names)

    # Use the received robot names in your nodes
    nodes = []
    for i,robot_name in enumerate(robot_names):
        node = Node(
            package='action_planner',
            executable='action_planner_node',
            name='action_planner_node_' + robot_name,
            namespace=robot_name,
            arguments=[method,robot_name,model_path,agent_type]
        )
        nodes.append(node)

    return nodes

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch)
    ])