# Routing tests for numbered from zero individual and swarm commands

import unittest
from unittest.mock import Mock
from types import SimpleNamespace

from swarm_mavsdk_ros2.swarm_controller import SwarmController


class FakeDrone:
    def __init__(self, drone_id: int):
        self.drone_id = drone_id
        self.calls = []
        self.land_client = Mock()
        self.land_client.call_async.side_effect = self._land_future

    def _land_future(self, request):
        self.land()
        future = Mock()
        future.result.return_value = SimpleNamespace(success=True, message=self.drone_id)
        return future

    def move_right(self, distance, **kwargs):
        self.calls.append(("move_right", distance, kwargs))
        return self.drone_id

    def land(self):
        self.calls.append(("land",))
        return self.drone_id


def controller_with_fake_drones(count: int) -> SwarmController:
    # Create only the command-routing portion; no ROS graph is needed
    controller = object.__new__(SwarmController)
    controller.drone_count = count
    controller.drones = [FakeDrone(drone_id) for drone_id in range(count)]
    return controller


class TestSwarmCommandRouting(unittest.TestCase):
    def test_individual_command_targets_only_the_requested_zero_indexed_drone(self):
        controller = controller_with_fake_drones(3)

        result = controller.command(0, "move_right", 1.0, mode="position")

        self.assertEqual(0, result)
        self.assertEqual(
            [("move_right", 1.0, {"mode": "position"})],
            controller.drones[0].calls,
        )
        self.assertEqual([], controller.drones[1].calls)
        self.assertEqual([], controller.drones[2].calls)

    def test_fleet_command_targets_every_drone_in_zero_indexed_order(self):
        for count in (1, 3, 5, 12, 50, 250):
            with self.subTest(count=count):
                controller = controller_with_fake_drones(count)
                result = controller.command_all("land")
                self.assertEqual({i: i for i in range(count)}, result)
                for drone in controller.drones:
                    self.assertEqual([("land",)], drone.calls)
                # The highest member remains individually addressable
                self.assertEqual(count - 1, controller.command(count - 1, "land"))
                with self.assertRaises(IndexError):
                    controller.command(count, "land")

    def test_invalid_drone_id_is_rejected(self):
        controller = controller_with_fake_drones(3)

        with self.assertRaises(IndexError):
            controller.command(3, "land")

    def test_fleet_takeoff_uses_the_synchronized_path(self):
        controller = controller_with_fake_drones(3)
        controller.takeoff_all = Mock(return_value={0: "queued", 1: "queued", 2: "queued"})

        result = controller.command_all("takeoff")

        self.assertEqual({0: "queued", 1: "queued", 2: "queued"}, result)
        controller.takeoff_all.assert_called_once_with()


if __name__ == "__main__":
    unittest.main()
