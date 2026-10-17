from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_config = str(Path(get_package_share_directory("robot_test")) / "config" / "robot.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=default_config),
        DeclareLaunchArgument("enable_motors", default_value="false"),
        Node(package="robot_test", executable="serial_bridge", output="screen",
             parameters=[LaunchConfiguration("config"), {
                 "enable_motors": ParameterValue(LaunchConfiguration("enable_motors"), value_type=bool),
             }]),
    ])
