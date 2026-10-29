from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	package_dir = Path(get_package_share_directory("robot_description"))
	urdf_path = package_dir / "urdf" / "robot.urdf" # 로봇 urdf 파일 경로
	rviz_path = package_dir / "rviz" / "display.rviz" # rviz 화면설정이 저장된 파일 경로

	robot_description = urdf_path.read_text(encoding="utf-8")

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
			package="rviz2",
			executable="rviz2",
			arguments=["-d", str(rviz_path)],
		),
	])