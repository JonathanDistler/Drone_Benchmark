# Routing checks for the public, non-RL behavior helpers

import unittest
from unittest.mock import Mock

from swarm_mavsdk_ros2.behavior import Drone, Swarm


class TestDroneBehaviorApi(unittest.TestCase):
    def setUp(self):
        self.drone = Drone(0, node=object())
        self.drone.client = Mock()

    def test_movement_and_height_aliases_route_to_command_client(self):
        self.drone.forward(2.0, mode="velocity")
        self.drone.backward(1.0)
        self.drone.height_rise(0.5)
        self.drone.height_drop(0.25)

        self.drone.client.move_forward.assert_called_once_with(2.0, mode="velocity")
        self.drone.client.move_backward.assert_called_once_with(1.0)
        self.drone.client.move_up.assert_called_once_with(0.5)
        self.drone.client.move_down.assert_called_once_with(0.25)

    def test_yaw_and_lifecycle_commands_are_exposed(self):
        self.drone.yaw(1.2)
        self.drone.yaw_by(-0.4)
        self.drone.takeoff()
        self.drone.land()

        self.drone.client.set_yaw.assert_called_once_with(1.2)
        self.drone.client.yaw_by.assert_called_once_with(-0.4)
        self.drone.client.takeoff.assert_called_once_with()
        self.drone.client.land.assert_called_once_with()


class TestSwarmBehaviorApi(unittest.TestCase):
    def setUp(self):
        self.swarm = Swarm(3)
        self.swarm.controller = Mock()

    def test_movement_and_height_commands_broadcast(self):
        self.swarm.forward(2.0)
        self.swarm.backward(1.0)
        self.swarm.height_rise(0.5)
        self.swarm.height_drop(0.25)

        calls = self.swarm.controller.command_all.call_args_list
        self.assertEqual(
            [
                ("move_forward", (2.0,), {}),
                ("move_backward", (1.0,), {}),
                ("move_up", (0.5,), {}),
                ("move_down", (0.25,), {}),
            ],
            [(call.args[0], call.args[1:], call.kwargs) for call in calls],
        )

    def test_yaw_and_takeoff_broadcast(self):
        self.swarm.yaw(1.2)
        self.swarm.yaw_by(-0.4)
        self.swarm.takeoff()

        self.assertEqual(
            [("set_yaw", 1.2), ("yaw_by", -0.4), ("takeoff",)],
            [call.args for call in self.swarm.controller.command_all.call_args_list],
        )

    def test_formations_and_benchmarks_use_shared_controller(self):
        self.swarm.run_test(f"test {2}", seed=42)
        self.swarm.formation("circle", spacing_m=2.)
        self.swarm.translate(east=3.)
        self.swarm.randomize(seed=42)
        self.swarm.split("left")
        self.swarm.controller.run_test.assert_called_once_with("test 2", seed=42)
        self.swarm.controller.formation.assert_called_once_with("circle", 2.)
        self.swarm.controller.translate.assert_called_once_with(0., 3., 0.)
        self.swarm.controller.randomize.assert_called_once_with(3., seed=42)
        self.swarm.controller.split.assert_called_once_with("left", 1.5)


if __name__ == "__main__":
    unittest.main()
