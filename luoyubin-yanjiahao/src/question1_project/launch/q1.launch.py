#!/usr/bin/env python3
"""一条命令看题一完整流程：robot_state_publisher + RViz + 驱动脚本。

默认同时启动驱动（driver:=true），预热 8 秒后开始动作；
只想手动跑驱动时传 driver:=false，再用 README 里的 python3 命令。
"""
import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.dirname(HERE)
BASE = os.path.dirname(os.path.dirname(SRC))
sys.path.insert(0, SRC)

from robot_urdf import expand_urdf  # noqa: E402


def generate_launch_description():
    urdf_root = os.environ.get("BAJIE_URDF_ROOT", "/home/yjh/bajie-competition/urdf")
    description = expand_urdf(urdf_root)

    rviz_config = LaunchConfiguration("rviz_config")
    input_path = LaunchConfiguration("input")
    output_path = LaunchConfiguration("output")
    driver = LaunchConfiguration("driver")
    warmup = LaunchConfiguration("warmup")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz_config",
                default_value=os.path.join(SRC, "config", "q1.rviz"),
            ),
            DeclareLaunchArgument(
                "input",
                default_value=os.path.join(BASE, "inputs", "q1_demo_input.jsonl"),
            ),
            DeclareLaunchArgument(
                "output",
                default_value=os.path.join(BASE, "answers", "q1_demo_output.jsonl"),
            ),
            DeclareLaunchArgument("driver", default_value="true"),
            DeclareLaunchArgument("warmup", default_value="8.0"),
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
                    os.path.join(SRC, "q1_driver.py"),
                    "--input", input_path,
                    "--output", output_path,
                    "--ros",
                    "--warmup", warmup,
                ],
                output="screen",
                condition=IfCondition(driver),
            ),
        ]
    )
