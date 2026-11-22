"""실기 지도 만들기: bringup(EKF 켬, LiDAR) + slam_toolbox online async.

ros2 launch robot_description slam.launch.py
지도 저장: ros2 run nav2_map_server map_saver_cli -f ~/maps/<이름>
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = Path(get_package_share_directory("robot_description"))
    slam_config = package_dir / "config" / "slam_toolbox.yaml"
    bringup = package_dir / "launch" / "bringup.launch.py"

    return LaunchDescription([
        DeclareLaunchArgument("use_ekf", default_value="true"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(bringup)),
            launch_arguments={"use_ekf": LaunchConfiguration("use_ekf")}.items(),
        ),
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[str(slam_config)],
        ),
    ])
