from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import math


def _start_drone_nodes(context):
    # Create one isolated DroneNode for every requested PX4 instance
    count = int(LaunchConfiguration("num_drones").perform(context))
    altitude = float(LaunchConfiguration("takeoff_altitude").perform(context))
    position_speed = float(context.launch_configurations.get("position_speed_mps", "0.4"))
    if not math.isfinite(position_speed) or position_speed <= 0:
        raise ValueError("position_speed_mps must be finite and positive.")
    if not 1 <= count <= 250:
        raise ValueError("num_drones must be between 1 and 250 (PX4 system IDs).")

    from swarm_mavsdk_ros2.architecture import leader_id
    architecture = context.launch_configurations.get("architecture", "central")
    leader_id(architecture, count)
    nodes = []
    for drone_id in range(count):
        # Instance zero uses the root DDS namespace
        px4_namespace = "" if drone_id == 0 else f"/px4_{drone_id}"
        nodes.append(
            Node(
                package="swarm_mavsdk_ros2",
                executable="drone_node",
                namespace=f"drone_{drone_id}",
                name="drone",
                parameters=[{
                    "drone_id": drone_id,
                    "system_id": drone_id + 1,
                    "px4_namespace": px4_namespace,
                    "takeoff_height": altitude,
                    "position_speed_mps": position_speed,
                    "swarm_size": count,
                    "architecture": architecture,
                    "world_name": LaunchConfiguration("world_name").perform(context),
                    "spawn_world_y": drone_id * float(LaunchConfiguration("spawn_spacing").perform(context)),
                }],
                output="screen",
                on_exit=[Shutdown(reason=f"Drone interface {drone_id} exited")],
            )
        )
    if architecture == "central":
        nodes.append(Node(
            package="swarm_mavsdk_ros2", executable="swarm_controller",
            name="swarm_controller", output="screen", arguments=["--n", str(count)],
            on_exit=[Shutdown(reason="Swarm controller exited")],
        ))
    return nodes


def generate_launch_description():
    count = LaunchConfiguration("num_drones")
    return LaunchDescription([
        DeclareLaunchArgument("architecture", default_value="central"),
        DeclareLaunchArgument("world_name", default_value=""),
        DeclareLaunchArgument("spawn_spacing", default_value="2.0"),
        DeclareLaunchArgument("num_drones", default_value="3"),
        DeclareLaunchArgument("takeoff_altitude", default_value="2.0"),
        DeclareLaunchArgument("position_speed_mps", default_value="0.4"),
        DeclareLaunchArgument("output_dir", default_value="results/ros2"),
        Node(
            package="swarm_mavsdk_ros2",
            executable="topic_recorder",
            name="topic_recorder",
            parameters=[{"output_dir": LaunchConfiguration("output_dir"),
                         "world_name": LaunchConfiguration("world_name")}],
            output="screen",
            on_exit=[Shutdown(reason="Topic recorder exited")],
        ),
        OpaqueFunction(function=_start_drone_nodes),
    ])
