"""One-click launch for minimal Marine-VLN system on top of VRX."""

from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    package_share = FindPackageShare("marine_vln_vrx")
    vrx_share = FindPackageShare("vrx_vln_bringup")

    params_default = PathJoinSubstitution([package_share, "config", "marine_vln_params.yaml"])
    object_file_default = PathJoinSubstitution([package_share, "data", "scene_objects.yaml"])
    log_root_default = PathJoinSubstitution([package_share, "logs"])
    world_default = PathJoinSubstitution([vrx_share, "worlds", "vrx_vln_buoy_test.sdf"])

    run_id = datetime.now().strftime("%Y%m%d_%H%M%S")

    vrx_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([vrx_share, "launch", "empty_usv.launch.py"])
        ),
        condition=IfCondition(LaunchConfiguration("start_vrx")),
        launch_arguments={
            "world": LaunchConfiguration("world"),
            "headless": LaunchConfiguration("headless"),
            "paused": LaunchConfiguration("paused"),
            "rviz": LaunchConfiguration("rviz"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )

    common_params = [
        LaunchConfiguration("params_file"),
        {"log_root": ParameterValue(LaunchConfiguration("log_root"), value_type=str)},
        {"use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)},
    ]

    nodes = [
        Node(
            package="marine_vln_vrx",
            executable="instruction_manager_node",
            name="instruction_manager",
            output="screen",
            parameters=common_params
            + [
                {"parser_mode": ParameterValue(LaunchConfiguration("parser_mode"), value_type=str)},
                {"method_name": ParameterValue(LaunchConfiguration("method_name"), value_type=str)},
                {"benchmark_mode": ParameterValue(LaunchConfiguration("benchmark_mode"), value_type=bool)},
                {
                    "use_hybrid_fallback": ParameterValue(
                        LaunchConfiguration("use_hybrid_fallback"), value_type=bool
                    )
                },
                {
                    "use_grounding_verifier": ParameterValue(
                        LaunchConfiguration("use_grounding_verifier"), value_type=bool
                    )
                },
                {
                    "use_semantic_map_authority": ParameterValue(
                        LaunchConfiguration("use_semantic_map_authority"), value_type=bool
                    )
                },
                {
                    "use_topk_hypothesis": ParameterValue(
                        LaunchConfiguration("use_topk_hypothesis"), value_type=bool
                    )
                },
                {
                    "use_oracle_parser": ParameterValue(
                        LaunchConfiguration("use_oracle_parser"), value_type=bool
                    )
                },
                {"vlm_api_base": ParameterValue(LaunchConfiguration("vlm_api_base"), value_type=str)},
                {"vlm_model": ParameterValue(LaunchConfiguration("vlm_model"), value_type=str)},
            ],
        ),
        Node(
            package="marine_vln_vrx",
            executable="scene_parser_node",
            name="scene_parser",
            output="screen",
            parameters=common_params
            + [{"object_file": ParameterValue(LaunchConfiguration("object_file"), value_type=str)}],
        ),
        Node(
            package="marine_vln_vrx",
            executable="semantic_mapper_node",
            name="semantic_mapper",
            output="screen",
            parameters=common_params,
        ),
        Node(
            package="marine_vln_vrx",
            executable="subgoal_planner_node",
            name="subgoal_planner",
            output="screen",
            parameters=common_params
            + [
                {
                    "use_topk_hypothesis": ParameterValue(
                        LaunchConfiguration("use_topk_hypothesis"), value_type=bool
                    )
                },
                {
                    "hypothesis_switch_on_replan": ParameterValue(
                        LaunchConfiguration("use_replan"), value_type=bool
                    )
                },
            ],
        ),
        Node(
            package="marine_vln_vrx",
            executable="local_planner_node",
            name="local_planner",
            output="screen",
            parameters=common_params,
        ),
        Node(
            package="marine_vln_vrx",
            executable="controller_node",
            name="controller",
            output="screen",
            parameters=common_params,
        ),
        Node(
            package="marine_vln_vrx",
            executable="safety_monitor_node",
            name="safety_monitor",
            output="screen",
            parameters=common_params
            + [
                {
                    "use_encounter_aware_safety": ParameterValue(
                        LaunchConfiguration("use_encounter_aware_safety"), value_type=bool
                    )
                },
                {
                    "enable_emergency_stop": ParameterValue(
                        LaunchConfiguration("use_emergency_stop"), value_type=bool
                    )
                },
                {
                    "enable_replan_trigger": ParameterValue(
                        LaunchConfiguration("use_replan"), value_type=bool
                    )
                },
            ],
        ),
        Node(
            package="marine_vln_vrx",
            executable="dynamic_obstacle_driver_node",
            name="dynamic_obstacle_driver",
            output="screen",
            condition=IfCondition(LaunchConfiguration("enable_dynamic_obstacle_driver")),
            parameters=common_params
            + [
                {
                    "dynamic_set_pose_service": ParameterValue(
                        LaunchConfiguration("dynamic_set_pose_service"), value_type=str
                    )
                },
                {
                    "dynamic_object_file": ParameterValue(
                        LaunchConfiguration("dynamic_object_file"), value_type=str
                    )
                },
            ],
        ),
        Node(
            package="marine_vln_vrx",
            executable="vlm_visualizer_node",
            name="vlm_visualizer",
            output="screen",
            condition=IfCondition(LaunchConfiguration("enable_vlm_visualizer")),
            parameters=common_params,
        ),
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_vrx",
                default_value="True",
                description="If true, start VRX world (default includes buoy test scene).",
            ),
            DeclareLaunchArgument("world", default_value=world_default),
            DeclareLaunchArgument("headless", default_value="False"),
            DeclareLaunchArgument("paused", default_value="False"),
            DeclareLaunchArgument("rviz", default_value="True"),
            DeclareLaunchArgument("use_sim_time", default_value="True"),
            DeclareLaunchArgument("params_file", default_value=params_default),
            DeclareLaunchArgument("object_file", default_value=object_file_default),
            DeclareLaunchArgument(
                "enable_dynamic_obstacle_driver",
                default_value="True",
                description="Enable Gazebo set_pose driver for dynamic obstacle models.",
            ),
            DeclareLaunchArgument(
                "dynamic_set_pose_service",
                default_value="/world/vrx_vln_dynamic_challenge/set_pose",
                description="ROS service path for Gazebo SetEntityPose.",
            ),
            DeclareLaunchArgument(
                "dynamic_object_file",
                default_value=object_file_default,
                description="Object YAML used by dynamic obstacle driver.",
            ),
            DeclareLaunchArgument("log_root", default_value=log_root_default),
            DeclareLaunchArgument(
                "parser_mode",
                default_value="hybrid",
                description="instruction parser mode: rules | vlm | hybrid",
            ),
            DeclareLaunchArgument(
                "method_name",
                default_value="hybrid_current",
                description="Method profile name for benchmark logging.",
            ),
            DeclareLaunchArgument(
                "use_hybrid_fallback",
                default_value="True",
                description="Enable fallback to rules when VLM/grounding is unreliable.",
            ),
            DeclareLaunchArgument(
                "use_grounding_verifier",
                default_value="True",
                description="Enable map-constrained grounding verification.",
            ),
            DeclareLaunchArgument(
                "use_semantic_map_authority",
                default_value="True",
                description="Use semantic-map authority for VLM target validation.",
            ),
            DeclareLaunchArgument(
                "use_topk_hypothesis",
                default_value="True",
                description="Enable top-k hypothesis planning/recovery.",
            ),
            DeclareLaunchArgument(
                "use_oracle_parser",
                default_value="False",
                description="Use oracle parser for upper-bound experiments.",
            ),
            DeclareLaunchArgument(
                "use_encounter_aware_safety",
                default_value="True",
                description="Enable marine encounter-aware safety penalties.",
            ),
            DeclareLaunchArgument(
                "use_emergency_stop",
                default_value="True",
                description="Enable hard emergency stop in safety monitor.",
            ),
            DeclareLaunchArgument(
                "use_replan",
                default_value="True",
                description="Enable replan trigger and hypothesis switching.",
            ),
            DeclareLaunchArgument(
                "benchmark_mode",
                default_value="False",
                description="Enable benchmark-mode metadata/logging.",
            ),
            DeclareLaunchArgument(
                "enable_vlm_visualizer",
                default_value="True",
                description="Enable front camera debug overlay with VLM/task status.",
            ),
            DeclareLaunchArgument(
                "vlm_api_base",
                default_value="http://127.0.0.1:8000/v1",
                description="OpenAI-compatible base url for VLM server",
            ),
            DeclareLaunchArgument(
                "vlm_model",
                default_value="Qwen/Qwen2.5-VL-7B-Instruct-AWQ",
                description="VLM model id",
            ),
            SetEnvironmentVariable("MARINE_VLN_RUN_ID", run_id),
            vrx_launch,
            *nodes,
        ]
    )
