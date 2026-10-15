import pytest
from swarm_mavsdk_ros2.architecture import leader_id


def test_f_string_selects_leader():
    assert leader_id("central", 3) is None
    for drone in range(3):
        assert leader_id(f"edge:drone_{drone}", 3) == drone


@pytest.mark.parametrize("value", ["edge", "edge:drone_-1", "edge:drone_3", "edge:drone_0/other"])
def test_invalid_architecture(value):
    with pytest.raises(ValueError):
        leader_id(value, 3)


def test_embedded_controller_ignores_drone_launch_remaps():
    import rclpy
    from swarm_mavsdk_ros2.drone_node import DroneNode
    from swarm_mavsdk_ros2.swarm_controller import SwarmController

    rclpy.init(args=["--ros-args", "-r", "__node:=drone", "-r",
                     "__ns:=/drone_0"])
    drone = controller = None
    try:
        drone = DroneNode()
        controller = SwarmController(3, namespace="/drone_0",
                                     use_global_arguments=False)
        assert drone.get_fully_qualified_name() == "/drone_0/drone"
        assert controller.get_fully_qualified_name() == "/drone_0/swarm_controller"
        assert drone.get_parameter("takeoff_height").value == 2.0
        assert controller.drones[0].parameters_client.srv_name == "/drone_0/drone/get_parameters"
    finally:
        if controller is not None:
            controller.destroy_node()
        if drone is not None:
            drone.destroy_node()
        rclpy.shutdown()
