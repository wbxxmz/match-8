#!/usr/bin/env python3
"""题二可选可视化：robot_state_publisher + RViz + 演示节点。

只用于观察 objects_link 与场景物体的关系，不产生答案；
答案仍由 `python3 src/question2_project/run_q2.py` 生成。
"""
import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.dirname(HERE)
BASE = os.path.dirname(os.path.dirname(SRC))
sys.path.insert(0, SRC)

from run_q2 import find_urdf_root  # noqa: E402
from urdf_chain import expand_urdf  # noqa: E402


def generate_launch_description():
    urdf_root = str(find_urdf_root())
    description = expand_urdf(urdf_root)

    rviz_config = LaunchConfiguration("rviz_config")
    input_path = LaunchConfiguration("input")
    period = LaunchConfiguration("period")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz_config",
                default_value=os.path.join(SRC, "config", "q2.rviz"),
            ),
            DeclareLaunchArgument(
                "input",
                default_value=os.path.join(BASE, "inputs", "q2_demo_input.jsonl"),
            ),
            DeclareLaunchArgument("period", default_value="8.0"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[
                    {"robot_description": ParameterValue(description, value_type=str)}
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
            ),
            ExecuteProcess(
                cmd=[
                    "python3",
                    os.path.join(SRC, "viz_q2.py"),
                    "--input", input_path,
                    "--urdf-root", urdf_root,
                    "--animate",
                    "--period", period,
                ],
                output="screen",
            ),
        ]
    )
