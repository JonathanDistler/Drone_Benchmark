# Check explicit startup and arming without sending commands to a vehicle
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, call, patch

import rclpy
from px4_msgs.msg import VehicleStatus, VehicleCommand
from swarm_mavsdk_ros2.drone_node import DroneNode
from swarm_mavsdk_ros2.behavior import Drone, Swarm


class TestStartup(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = DroneNode()
        self.node.publish_vehicle_command = Mock()

    def tearDown(self):
        self.node.destroy_node()

    def position(self):
        self.node.local_position = SimpleNamespace(x=1., y=2., z=-0.1)

    def test_construction_does_not_stream_or_arm(self):
        self.assertFalse(self.node.started)
        self.node.publish_offboard_control_mode()
        self.node.publish_setpoint()
        self.assertEqual(self.node.offboard_setpoint_count, 0)
        self.node.publish_vehicle_command.assert_not_called()

    def test_startup_requires_position_and_holds_without_arming(self):
        with self.assertRaises(RuntimeError):
            self.node.startup()
        self.position()
        self.node.startup()
        self.assertEqual(self.node.position_target, [1., 2., -0.1])
        for _ in range(self.node.OFFBOARD_WARMUP_SETPOINTS):
            self.node.publish_setpoint()
            self.node.publish_offboard_control_mode()
        self.node.publish_vehicle_command.assert_not_called()
        self.node.startup()
        self.assertEqual(self.node.offboard_setpoint_count, 20)
        self.node.arm()
        self.node.publish_vehicle_command.assert_called_once_with(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        status = VehicleStatus()
        status.nav_state = VehicleStatus.NAVIGATION_STATE_OFFBOARD
        status.pre_flight_checks_pass = True
        self.node.vehicle_status_callback(status)
        self.node.vehicle_status_callback(status)
        self.assertEqual(self.node.publish_vehicle_command.call_args_list, [
            call(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0),
            call(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0),
        ])

    def prepare_arm(self):
        self.position()
        self.node.startup()
        self.node.offboard_setpoint_count = self.node.OFFBOARD_WARMUP_SETPOINTS
        self.node.arm()

    def test_arm_waits_for_mode_and_health(self):
        self.prepare_arm()
        status = VehicleStatus()
        status.pre_flight_checks_pass = True
        self.node.vehicle_status_callback(status)
        status.nav_state = VehicleStatus.NAVIGATION_STATE_OFFBOARD
        status.pre_flight_checks_pass = False
        self.node.vehicle_status_callback(status)
        self.assertEqual(self.node.publish_vehicle_command.call_count, 1)

    def test_arm_retries_until_px4_confirms_then_stops(self):
        self.prepare_arm()
        status = VehicleStatus()
        status.nav_state = VehicleStatus.NAVIGATION_STATE_OFFBOARD
        status.pre_flight_checks_pass = True
        self.node.vehicle_status_callback(status)
        self.assertIsNotNone(self.node.arm_deadline)
        with patch('swarm_mavsdk_ros2.drone_node.time.monotonic',
                   return_value=self.node.arm_retry_at):
            self.node.vehicle_status_callback(status)
        self.assertEqual(self.node.publish_vehicle_command.call_args_list.count(
            call(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)), 2)
        status.arming_state = VehicleStatus.ARMING_STATE_ARMED
        self.node.vehicle_status_callback(status)
        self.assertIsNone(self.node.arm_deadline)
        self.node.publish_vehicle_command.reset_mock()
        self.node.vehicle_status_callback(status)
        self.node.publish_vehicle_command.assert_not_called()

    def test_mode_retry_and_expired_arm_never_arms_later(self):
        self.prepare_arm()
        with patch('swarm_mavsdk_ros2.drone_node.time.monotonic',
                   return_value=self.node.arm_mode_retry_at):
            self.node.publish_offboard_control_mode()
        self.assertEqual(self.node.publish_vehicle_command.call_count, 2)
        status = VehicleStatus()
        status.nav_state = VehicleStatus.NAVIGATION_STATE_OFFBOARD
        status.pre_flight_checks_pass = True
        with patch('swarm_mavsdk_ros2.drone_node.time.monotonic',
                   return_value=self.node.arm_deadline + 1):
            self.node.vehicle_status_callback(status)
        self.node.vehicle_status_callback(status)
        self.assertIsNone(self.node.arm_deadline)
        self.assertEqual(self.node.publish_vehicle_command.call_count, 2)

    def test_disarm_and_land_cancel_queued_arm(self):
        for cancel in (self.node.disarm,
                       lambda: self.node.land_callback(None, SimpleNamespace())):
            self.node.landing = False
            self.prepare_arm()
            cancel()
            self.assertIsNone(self.node.arm_deadline)
            self.node.publish_vehicle_command.reset_mock()
            status = VehicleStatus()
            status.nav_state = VehicleStatus.NAVIGATION_STATE_OFFBOARD
            status.pre_flight_checks_pass = True
            self.node.vehicle_status_callback(status)
            self.node.publish_vehicle_command.assert_not_called()

    def test_arm_requires_completed_startup(self):
        with self.assertRaises(RuntimeError):
            self.node.arm()
        self.position()
        self.node.startup()
        with self.assertRaises(RuntimeError):
            self.node.arm()
        self.node.publish_vehicle_command.assert_not_called()

    def test_takeoff_requires_explicit_arm_and_never_arms_itself(self):
        self.position()
        response = SimpleNamespace()
        self.node.takeoff_callback(None, response)
        self.assertFalse(response.success)
        self.node.startup()
        self.node.takeoff_callback(None, response)
        self.assertFalse(response.success)
        self.node.vehicle_status = SimpleNamespace(arming_state=VehicleStatus.ARMING_STATE_ARMED)
        self.node.takeoff_callback(None, response)
        self.assertTrue(response.success)
        self.node.request_offboard_mode = Mock()
        self.node.arm = Mock()
        for _ in range(20):
            self.node.publish_offboard_control_mode()
        self.node.request_offboard_mode.assert_called_once()
        self.node.arm.assert_not_called()

    def test_takeoff_preserves_completed_stream_warmup(self):
        self.position()
        self.node.startup()
        self.node.offboard_setpoint_count = 40
        self.node.vehicle_status = SimpleNamespace(
            arming_state=VehicleStatus.ARMING_STATE_ARMED,
            nav_state=VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        response = self.node.takeoff_callback(None, SimpleNamespace())
        self.assertTrue(response.success)
        self.assertEqual(self.node.offboard_setpoint_count, 40)
        self.node.request_offboard_mode = Mock()
        self.node.publish_offboard_control_mode()
        self.node.request_offboard_mode.assert_called_once()
        self.node.publish_offboard_control_mode()
        self.assertFalse(self.node.takeoff_pending)

    def test_takeoff_mode_request_retries_until_confirmed(self):
        self.position()
        self.node.startup()
        self.node.offboard_setpoint_count = 40
        status = VehicleStatus()
        status.arming_state = VehicleStatus.ARMING_STATE_ARMED
        self.node.vehicle_status = status
        self.node.takeoff_callback(None, SimpleNamespace())
        self.node.request_offboard_mode = Mock()
        self.node.publish_offboard_control_mode()
        self.assertTrue(self.node.takeoff_pending)
        with patch('swarm_mavsdk_ros2.drone_node.time.monotonic',
                   return_value=self.node.takeoff_mode_retry_at):
            self.node.publish_offboard_control_mode()
        self.assertEqual(self.node.request_offboard_mode.call_count, 2)
        status.nav_state = VehicleStatus.NAVIGATION_STATE_OFFBOARD
        self.node.publish_offboard_control_mode()
        self.assertFalse(self.node.takeoff_pending)
        self.node.publish_offboard_control_mode()
        self.assertEqual(self.node.request_offboard_mode.call_count, 2)

    def test_disarmed_takeoff_cancels_climb_target(self):
        self.position()
        self.node.startup()
        status = VehicleStatus()
        status.arming_state = VehicleStatus.ARMING_STATE_ARMED
        self.node.vehicle_status = status
        self.node.takeoff_callback(None, SimpleNamespace())
        status.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        self.node.publish_offboard_control_mode()
        self.assertFalse(self.node.takeoff_pending)
        self.assertEqual(self.node.position_target, [1., 2., -0.1])

    def test_behavior_startup_and_arm_are_separate(self):
        drone = Drone()
        drone.client = Mock()
        drone.startup()
        drone.client.startup.assert_called_once_with()
        drone.client.arm.assert_not_called()
        drone.arm()
        drone.client.arm.assert_called_once_with()
        swarm = Swarm(3)
        swarm.controller = Mock()
        swarm.startup()
        swarm.arm()
        self.assertEqual([('startup',), ('arm',)], [c.args for c in swarm.controller.command_all.call_args_list])

    def test_status_messages_follow_transitions_without_repeating(self):
        logger = Mock()
        self.node.get_logger = lambda: logger
        status = VehicleStatus()
        self.node.vehicle_status_callback(status)
        logger.info.assert_not_called()
        armed = VehicleStatus()
        armed.arming_state = VehicleStatus.ARMING_STATE_ARMED
        self.node.vehicle_status_callback(armed)
        self.node.vehicle_status_callback(armed)
        logger.info.assert_called_once()
        self.assertIn("armed; request takeoff", logger.info.call_args.args[0])
        self.node.vehicle_status_callback(status)
        self.assertEqual(logger.info.call_count, 2)
        self.assertIn("disarmed", logger.info.call_args.args[0])

    def test_startup_guidance_prints_once_after_warmup(self):
        logger = Mock()
        self.node.get_logger = lambda: logger
        self.position()
        self.node.startup()
        for _ in range(100):
            self.node.publish_offboard_control_mode()
        logger.info.assert_called_once()
        self.assertIn("startup complete", logger.info.call_args.args[0])
