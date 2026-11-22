from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

# 런치 파일 경로 가져오기
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
	package_dir = Path(get_package_share_directory("robot_description"))
	urdf_path = package_dir / "urdf" / "robot.urdf" # 로봇 urdf 파일 경로

	robot_description = urdf_path.read_text(encoding="utf-8")
	ekf_config_path = package_dir / "config" / "ekf.yaml" # robot_localization EKF 설정

	# use_ekf:=true 이면 EKF가 odom->base_link TF를 내고, motor bridge는 TF를 끈다.
	# (robot_localization 패키지 설치 필요: ros-humble-robot-localization)
	use_ekf = LaunchConfiguration("use_ekf")

    #  c1 경로 만들기
	lidar_launch_path = (
		Path(get_package_share_directory("sllidar_ros2")) / "launch" / "sllidar_c1_launch.py"
    )

	return LaunchDescription([
		DeclareLaunchArgument("use_ekf", default_value="false"),
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
			parameters=[{
				"publish_tf": PythonExpression(["'", use_ekf, "'.lower() != 'true'"]),
			}],
		),
		Node(
			package="robot_localization",
			executable="ekf_node",
			name="ekf_filter_node",
			output="screen",
			parameters=[str(ekf_config_path)],
			condition=IfCondition(use_ekf),
		),
		IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(lidar_launch_path)),
        ),
	])