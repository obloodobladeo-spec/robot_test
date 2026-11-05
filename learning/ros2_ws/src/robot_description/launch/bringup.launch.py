from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

# 런치 파일 경로 가져오기
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
	package_dir = Path(get_package_share_directory("robot_description"))
	urdf_path = package_dir / "urdf" / "robot.urdf" # 로봇 urdf 파일 경로

	robot_description = urdf_path.read_text(encoding="utf-8")

    #  c1 경로 만들기
	lidar_launch_path = (
		Path(get_package_share_directory("sllidar_ros2")) / "launch" / "sllidar_c1_launch.py"
    )

	return LaunchDescription([
		Node(
			package="robot_state_publisher",
			executable="robot_state_publisher",
			parameters=[{"robot_description" : robot_description}],
		),
		Node(
			package="joint_state_publisher",
			executable="joint_state_publisher",
		),
		Node(
			package="motor_bridge",
			executable="cmd_vel_receiver",
			output="screen",
		),
		IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(lidar_launch_path)),
        ),
	])