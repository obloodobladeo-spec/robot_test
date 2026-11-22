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

	# 장치 경로: udev 규칙(tools/jetson/99-robot-usb.rules)이 설치돼 있으면 고정 이름을 기본으로 쓴다.
	stm32_default = "/dev/stm32" if Path("/dev/stm32").exists() else "/dev/ttyACM0"
	lidar_default = "/dev/lidar" if Path("/dev/lidar").exists() else "/dev/ttyUSB0"
	stm32_port = LaunchConfiguration("stm32_port")
	lidar_port = LaunchConfiguration("lidar_port")

    #  c1 경로 만들기
	lidar_launch_path = (
		Path(get_package_share_directory("sllidar_ros2")) / "launch" / "sllidar_c1_launch.py"
    )

	return LaunchDescription([
		DeclareLaunchArgument("use_ekf", default_value="false"),
		DeclareLaunchArgument("stm32_port", default_value=stm32_default),
		DeclareLaunchArgument("lidar_port", default_value=lidar_default),
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
				"serial_port": stm32_port,
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
            launch_arguments={"serial_port": lidar_port}.items(),
        ),
	])