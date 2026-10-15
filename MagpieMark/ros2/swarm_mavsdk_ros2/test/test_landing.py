# Prevent offboard targets from overwriting PX4 AUTO.LAND trajectories
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

from px4_msgs.msg import VehicleCommand, VehicleStatus
from swarm_mavsdk_ros2.drone_node import DroneNode


class TestLandingOwnership(unittest.TestCase):
    def test_land_cancels_pending_takeoff_and_stops_external_setpoints(self):
        node = SimpleNamespace(drone_id=0, landing=False, takeoff_pending=True,
                               get_logger=lambda: Mock(), publish_vehicle_command=Mock())
        response = SimpleNamespace()
        DroneNode.land_callback(node, None, response)
        self.assertTrue(response.success)
        self.assertTrue(node.landing)
        self.assertFalse(node.takeoff_pending)
        node.publish_vehicle_command.assert_called_once_with(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        # No PX4 publishers/clock exist on this double: these calls must not attempt
        # to publish anything while AUTO.LAND owns the setpoint
        DroneNode.publish_setpoint(node)
        DroneNode.publish_offboard_control_mode(node)

    def test_next_flight_requires_startup_then_arm(self):
        node = SimpleNamespace(started=True, landing=True, vehicle_status=None,
                               local_position=SimpleNamespace(x=1., y=2., z=0.),
                               position_target=[0.,0.,0.], takeoff_height=2.,
                               offboard_setpoint_count=100, offboard_requested=True,
                               get_logger=lambda: Mock())
        response = SimpleNamespace()
        DroneNode.takeoff_callback(node, None, response)
        self.assertFalse(response.success)
        DroneNode.startup(node)
        node.vehicle_status = SimpleNamespace(arming_state=VehicleStatus.ARMING_STATE_ARMED)
        DroneNode.takeoff_callback(node, None, response)
        self.assertTrue(response.success)
        self.assertFalse(node.landing)
        self.assertTrue(node.takeoff_pending)
        self.assertFalse(node.offboard_requested)
        self.assertEqual(node.offboard_setpoint_count, 0)
        self.assertEqual(node.position_target, [1.,2.,-2.])
