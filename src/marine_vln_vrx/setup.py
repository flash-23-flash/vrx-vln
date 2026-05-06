import os
from glob import glob

from setuptools import find_packages, setup


package_name = "marine_vln_vrx"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "config", "prompts"), glob("config/prompts/*.txt")),
        (os.path.join("share", package_name, "data"), glob("data/*.yaml")),
        (os.path.join("share", package_name, "data"), glob("data/*.jsonl")),
        (os.path.join("share", package_name, "data", "benchmark"), glob("data/benchmark/*.yaml")),
        (os.path.join("share", package_name, "docs"), glob("docs/*.md")),
        (os.path.join("share", package_name, "scripts"), glob("scripts/*.py")),
        (os.path.join("share", package_name, "logs"), glob("logs/.gitkeep")),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="pkuiflab",
    maintainer_email="pkuiflab@example.com",
    description="Minimal hierarchical Marine-VLN prototype for VRX (ROS 2).",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "instruction_manager_node = marine_vln_vrx.instruction_manager.node:main",
            "scene_parser_node = marine_vln_vrx.scene_parser.node:main",
            "semantic_mapper_node = marine_vln_vrx.semantic_mapper.node:main",
            "subgoal_planner_node = marine_vln_vrx.subgoal_planner.node:main",
            "local_planner_node = marine_vln_vrx.local_planner.node:main",
            "controller_node = marine_vln_vrx.controller.node:main",
            "safety_monitor_node = marine_vln_vrx.safety_monitor.node:main",
            "vlm_visualizer_node = marine_vln_vrx.vlm_visualizer.node:main",
            "dynamic_obstacle_driver_node = marine_vln_vrx.dynamic_obstacle_driver.node:main",
            "publish_instruction = marine_vln_vrx.scripts.publish_instruction:main",
            "test_vlm_parser = marine_vln_vrx.scripts.test_vlm_parser:main",
            "export_vlm_sft_data = marine_vln_vrx.scripts.export_vlm_sft_data:main",
        ]
    },
)
