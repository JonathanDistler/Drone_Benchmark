# Fleet send barriers and vector validation without a ROS graph
from types import SimpleNamespace
from unittest.mock import Mock
import pytest
from swarm_mavsdk_ros2.swarm_controller import SwarmController
from swarm_mavsdk_ros2.commands import DroneCommandClient
from swarm_mavsdk_ros2.behavior import Swarm


def fleet():
    controller = object.__new__(SwarmController)
    controller.drone_count = 3
    controller.drones = [Mock() for _ in range(3)]
    return controller


def test_services_dispatch_every_member_before_reading_results():
    controller = fleet()
    dispatched = []
    for i, drone in enumerate(controller.drones):
        future = Mock()
        def result():
            assert dispatched == [0, 1, 2]
            return SimpleNamespace(success=True, message="queued")
        future.result.side_effect = result
        def dispatch(request, i=i, future=future):
            dispatched.append(i)
            return future
        drone.land_client.call_async.side_effect = dispatch
    assert controller.command_all("land") == dict.fromkeys(range(3), "queued")


def test_missing_service_blocks_entire_fleet():
    controller = fleet()
    controller.drones[2].land_client.wait_for_service.return_value = False
    with pytest.raises(TimeoutError):
        controller.command_all("land")
    for drone in controller.drones:
        drone.land_client.call_async.assert_not_called()


def test_invalid_vector_never_dispatches():
    controller = fleet()
    with pytest.raises(ValueError):
        controller.command_all("set_velocity", 1, float("nan"), 0)
    for drone in controller.drones:
        drone.set_velocity.assert_not_called()


def test_swarm_exposes_position_and_velocity():
    swarm = Swarm(3)
    swarm.controller = Mock()
    swarm.move_to(1, 2, -1)
    swarm.set_velocity(0.5, 0, 0)
    assert [c.args for c in swarm.controller.command_all.call_args_list] == [
        ("move_to", 1, 2, -1), ("set_velocity", 0.5, 0, 0)]


def test_velocity_target_uses_atomic_topic_mode_switch():
    client = object.__new__(DroneCommandClient)
    client._publish_velocity = Mock()
    client.set_velocity_control = Mock()
    client.set_velocity(1, 2, 3)
    client._publish_velocity.assert_called_once_with(1, 2, 3)
    client.set_velocity_control.assert_not_called()

@pytest.mark.parametrize("module", ["command_cli", "swarm_command"])
def test_direct_target_cli(module):
    import importlib
    parser = importlib.import_module("swarm_mavsdk_ros2." + module)._arguments
    extra = ["--n", "3"] if module == "swarm_command" else []
    parsed = parser(["velocity", "--ned", "0.5", "0", "-0.2", *extra])
    assert parsed.ned == [0.5, 0, -0.2]


def test_recorder_subscribers_cannot_release_targets_without_drone_services():
    from unittest.mock import patch
    controller = fleet()
    for drone in controller.drones:
        drone.position_pub.get_subscription_count.return_value = 1
        drone.stop_client.service_is_ready.return_value = False
    with patch('swarm_mavsdk_ros2.swarm_controller.rclpy.spin_once'), \
         pytest.raises(TimeoutError, match='readiness'):
        controller._targets_all('move_to', 1., 2., -3., timeout_s=.001)
    for drone in controller.drones:
        drone.move_to.assert_not_called()
