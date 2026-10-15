# Do not let an immediate forward command cancel a slow member's climb

import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterValue, ParameterType

from swarm_mavsdk_ros2.commands import DroneCommandClient
from swarm_mavsdk_ros2.swarm_controller import SwarmController


def client_at(z):
    client = object.__new__(DroneCommandClient)
    client.node = object()
    client.namespace = "/drone_0"
    client.armed = client.offboard = True
    state = Odometry()
    state.pose.pose.position.z = z
    client._state_callback(state)
    return client


class TestTakeoffCompletion(unittest.TestCase):
    def test_armed_offboard_on_ground_is_not_completed_takeoff(self):
        client = client_at(0.01)
        self.assertFalse(client.takeoff_reached(-2.0, 0.0))
        client.state.pose.pose.position.z = -2.0
        self.assertTrue(client.takeoff_reached(-2.0, 0.0))
        client.state.twist.twist.linear.z = -0.8
        self.assertFalse(client.takeoff_reached(-2.0, 0.0))

    def test_stale_or_pre_request_altitude_cannot_complete_takeoff(self):
        client = client_at(-2.0)
        self.assertFalse(client.takeoff_reached(-2.0, time.monotonic()))
        client.state_received_at = time.monotonic() - 3.0
        self.assertFalse(client.takeoff_reached(-2.0, 0.0))

    def test_reads_configured_height_instead_of_assuming_two_metres(self):
        client = client_at(0.0)
        value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=3.5)
        future = Mock()
        future.done.return_value = True
        future.result.return_value = SimpleNamespace(values=[value])
        client.parameters_client = Mock()
        client.parameters_client.call_async.return_value = future
        self.assertEqual(client.takeoff_target_down(), -3.5)
        request = client.parameters_client.call_async.call_args.args[0]
        self.assertEqual(request.names, ["takeoff_height"])

    def test_single_takeoff_waits_for_climb_before_next_forward(self):
        client = client_at(0.01)
        client.wait_for_state = Mock()
        client.wait_for_armed = Mock()
        client.wait_for_offboard = Mock()
        client.takeoff_target_down = Mock(return_value=-2.0)
        client.takeoff_client = object()
        client._call = Mock(return_value="queued")
        client.set_position_control = Mock()
        client._publish_position = Mock()

        def climb(*args, **kwargs):
            state = Odometry()
            state.pose.pose.position.z = -2.0
            client._state_callback(state)

        with patch("swarm_mavsdk_ros2.commands.rclpy.spin_once", side_effect=climb) as spin:
            client.takeoff()
        spin.assert_called_once()
        def arrive(*args, **kwargs):
            target = client._publish_position.call_args.args
            state = Odometry()
            p = state.pose.pose.position
            p.x, p.y, p.z = target
            client._state_callback(state)

        with patch("swarm_mavsdk_ros2.commands.rclpy.spin_once", side_effect=arrive):
            client.move_forward(2.0)
            self.assertEqual(client.state.pose.pose.position.x, 2.0)
            client.move_backward(2.0)
            self.assertEqual(client.state.pose.pose.position.x, 0.0)
        client._publish_position.assert_called_with(0.0, 0.0, -2.0)

    def test_fleet_waits_for_drone_zero_even_when_everyone_is_offboard(self):
        controller = object.__new__(SwarmController)
        controller.drone_count = 2
        controller.states = {0: Odometry(), 1: Odometry()}
        controller.all_ready = Mock(return_value=True)
        controller.drones = [client_at(0.01), client_at(-2.0)]
        for client in controller.drones:
            client.takeoff_target_down = Mock(return_value=-2.0)
            client.takeoff_client = Mock()
            future = client.takeoff_client.call_async.return_value
            future.done.return_value = True
            future.result.return_value = SimpleNamespace(success=True, message="queued")
        heights = iter([-0.5, -1.0, -2.0])

        def climb(*args, **kwargs):
            # All takeoffs must be queued before waiting for any one member
            for client in controller.drones:
                client.takeoff_client.call_async.assert_called_once()
            controller.drones[0].state.pose.pose.position.z = next(heights)
            for client in controller.drones:
                client._state_callback(client.state)

        with patch("swarm_mavsdk_ros2.swarm_controller.rclpy.spin_once", side_effect=climb) as spin:
            self.assertEqual(controller.takeoff_all(), {0: "queued", 1: "queued"})
        self.assertEqual(spin.call_count, 3)

    def test_discovers_every_service_before_dispatch_and_failure_sends_nothing(self):
        controller = object.__new__(SwarmController)
        controller.drone_count = 2
        controller.states = {0: Odometry(), 1: Odometry()}
        controller.drones = [client_at(0.0), client_at(0.0)]
        for client in controller.drones:
            client.takeoff_target_down = Mock(return_value=-2.0)
            client.takeoff_client = Mock()

        def unavailable(**kwargs):
            for client in controller.drones:
                client.takeoff_client.call_async.assert_not_called()
            return False

        controller.drones[1].takeoff_client.wait_for_service.side_effect = unavailable
        with self.assertRaisesRegex(TimeoutError, "drone 1"):
            controller.takeoff_all()
        for client in controller.drones:
            client.takeoff_client.call_async.assert_not_called()

    def test_unarmed_third_member_blocks_every_takeoff(self):
        controller = object.__new__(SwarmController)
        controller.drone_count = 3
        controller.states = dict.fromkeys(range(3), Odometry())
        controller.drones = [client_at(0.0) for _ in range(3)]
        for client in controller.drones:
            client.takeoff_target_down = Mock(return_value=-2.0)
            client.takeoff_client = Mock()
        controller.drones[2].armed = False
        clock = [0.0]
        def wait(*args, **kwargs):
            clock[0] += 1.0
        with patch("time.monotonic", side_effect=lambda: clock[0]), patch(
            "swarm_mavsdk_ros2.swarm_controller.rclpy.spin_once", side_effect=wait
        ):
            with self.assertRaisesRegex(TimeoutError, r"drone\(s\): 2$"):
                controller.takeoff_all(timeout_s=3.0)
        for client in controller.drones:
            client.takeoff_client.call_async.assert_not_called()

    def test_fleet_timeout_identifies_drone_zero_if_it_never_climbs(self):
        controller = object.__new__(SwarmController)
        controller.drone_count = 2
        controller.states = {0: Odometry(), 1: Odometry()}
        controller.all_ready = Mock(return_value=True)
        controller.drones = [client_at(0.01), client_at(-2.0)]
        for client in controller.drones:
            client.takeoff_target_down = Mock(return_value=-2.0)
            client.takeoff_client = Mock()
            future = client.takeoff_client.call_async.return_value
            future.done.return_value = True
            future.result.return_value = SimpleNamespace(success=True, message="queued")
        clock = [0.0]

        def no_climb(*args, **kwargs):
            clock[0] += 1.0
            for client in controller.drones:
                client._state_callback(client.state)

        with patch("time.monotonic", side_effect=lambda: clock[0]), patch(
            "swarm_mavsdk_ros2.swarm_controller.rclpy.spin_once", side_effect=no_climb
        ):
            with self.assertRaisesRegex(TimeoutError, r"drone\(s\): 0\n") as caught:
                controller.takeoff_all(timeout_s=3.0)
            self.assertIn("drone 0: armed=True offboard=True down=0.01m target=-2.00m",
                          str(caught.exception))


if __name__ == "__main__":
    unittest.main()
