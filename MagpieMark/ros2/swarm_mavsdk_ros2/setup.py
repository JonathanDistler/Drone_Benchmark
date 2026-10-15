from setuptools import find_packages, setup

package_name = "swarm_mavsdk_ros2"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/swarm.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    description="Modular ROS 2 / PX4 drone and swarm benchmark.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "topic_recorder = swarm_mavsdk_ros2.topic_recorder:main",
            "drone_node = swarm_mavsdk_ros2.drone_node:main",
            "drone_command = swarm_mavsdk_ros2.command_cli:main",
            "swarm_command = swarm_mavsdk_ros2.swarm_command:main",
            "control_console = swarm_mavsdk_ros2.control_console:main",
            "swarm_controller = swarm_mavsdk_ros2.swarm_controller:main",
            "swarm_tests = swarm_mavsdk_ros2.swarm_node:main",
        ],
    },
)
