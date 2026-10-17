from glob import glob
from setuptools import find_packages, setup

setup(
    name="robot_test",
    version="0.1.0",
    packages=find_packages(exclude=["tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/robot_test"]),
        ("share/robot_test", ["package.xml"]),
        ("share/robot_test/launch", glob("launch/*.launch.py")),
        ("share/robot_test/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    maintainer="robot_test",
    maintainer_email="robot@example.invalid",
    description="NUCLEO-F446RE USB serial differential-drive test",
    license="MIT",
    entry_points={"console_scripts": [
        "serial_bridge = robot_test.serial_bridge:main",
        "teleop_keyboard = robot_test.teleop_keyboard:main",
    ]},
)
