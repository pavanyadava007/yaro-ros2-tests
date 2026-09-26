"""ros2 launch yaro_check monitor.launch.py model:=yaro_1105"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    models = os.environ.get("YARO_MODELS_DIR", "/ws/models")
    model = LaunchConfiguration("model").perform(context)
    with open(os.path.join(models, model, "robot.urdf")) as f:
        description = f.read()
    return [
        Node(
            package="yaro_check",
            executable="joint_limit_monitor",
            parameters=[{
                "robot_description": description,
                "velocity_scale": float(LaunchConfiguration("velocity_scale").perform(context)),
            }],
            output="screen",
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("model", default_value="yaro_1105"),
        DeclareLaunchArgument("velocity_scale", default_value="1.0"),
        OpaqueFunction(function=_nodes),
    ])
