# Fleet movement must include slow drone zero and send before waiting
import unittest
from unittest.mock import Mock, patch

from nav_msgs.msg import Odometry
from swarm_mavsdk_ros2.commands import DroneCommandClient
from swarm_mavsdk_ros2.swarm_controller import SwarmController


class TestCoordinatedMovement(unittest.TestCase):
    def fleet(self):
        controller = object.__new__(SwarmController)
        controller.drone_count = 2
        controller.drones = []
        for i in range(2):
            drone = object.__new__(DroneCommandClient)
            drone.state = Odometry()
            drone.state.pose.pose.position.z = -2.0
            drone.state.pose.pose.position.y = float(i)
            drone.state_received_at = 0.0
            drone.armed = drone.offboard = True
            drone.position_pub = Mock()
            drone.velocity_pub = Mock()
            drone.position_pub.get_subscription_count.return_value = 1
            drone.velocity_pub.get_subscription_count.return_value = 1
            drone._publish_position = Mock()
            drone._publish_velocity = Mock()
            controller.drones.append(drone)
        return controller

    def run_movement(self, mode, stuck=False, disconnected=False):
        controller = self.fleet()
        clock = [0.0]
        if disconnected:
            controller.drones[0].position_pub.get_subscription_count.return_value = 0

        def spin(*args, **kwargs):
            clock[0] += 0.1
            publications = [d._publish_position if mode == 'position' else d._publish_velocity
                            for d in controller.drones]
            # No spin/wait between the initial swarm publications
            self.assertEqual(bool(publications[0].call_count), bool(publications[1].call_count))
            for i, drone in enumerate(controller.drones):
                drone.state_received_at = clock[0]
                if publications[i].call_count and (i == 1 or (clock[0] > 1.5 and not stuck)):
                    drone.state.pose.pose.position.x = 0.5

        with patch('time.monotonic', side_effect=lambda: clock[0]), patch(
            'swarm_mavsdk_ros2.swarm_controller.rclpy.spin_once', side_effect=spin
        ):
            if stuck or disconnected:
                with self.assertRaisesRegex(TimeoutError, r'drone\(s\): 0'):
                    controller.move_all('forward', 0.5, mode=mode, timeout_s=3.0)
            else:
                self.assertEqual(controller.command_all('move_forward', 0.5, mode=mode), {0: None, 1: None})
                self.assertGreater(clock[0], 2.0)
                for i, drone in enumerate(controller.drones):
                    drone._publish_position.assert_called_with(0.5, float(i), -2.0)
        return controller

    def test_position_waits_for_slow_zero(self):
        self.run_movement('position')

    def test_velocity_dispatches_to_everyone_before_waiting(self):
        self.run_movement('velocity')

    def test_stuck_zero_is_reported(self):
        self.run_movement('position', stuck=True)

    def test_velocity_timeout_stops_unfinished_member(self):
        controller = self.run_movement('velocity', stuck=True)
        controller.drones[0]._publish_velocity.assert_called_with(0.0, 0.0, 0.0)

    def test_missing_command_subscriber_prevents_entire_fleet_dispatch(self):
        controller = self.run_movement('position', disconnected=True)
        for drone in controller.drones:
            drone._publish_position.assert_not_called()
