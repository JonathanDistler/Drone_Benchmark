# Fly-through arrivals and callback servicing under a busy swarm
import time
from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest
from nav_msgs.msg import Odometry

from swarm_mavsdk_ros2.commands import DroneCommandClient
from swarm_mavsdk_ros2.swarm_behaviors import SwarmBehaviors


def test_intermediate_waypoint_retains_speed_but_final_requires_settle():
    client = object.__new__(DroneCommandClient)
    client.node = Mock()
    client.state = Odometry()
    client.state.pose.pose.position.z = -2.
    client.state.twist.twist.linear.x = .4
    client.armed = client.offboard = True
    client.move_to = Mock()
    client.stop = Mock()
    def refresh(*args):
        client.state_received_at = time.monotonic()
    with patch('swarm_mavsdk_ros2.commands.spin_for', side_effect=refresh):
        client.move_to_and_wait(0, 0, -2, settle=False)
        client.stop.assert_not_called()
        with pytest.raises(TimeoutError):
            client.move_to_and_wait(0, 0, -2, timeout_s=.01)
    client.stop.assert_called_once()


def test_fleet_services_queued_telemetry_before_stale_check():
    drones = []
    for i in range(3):
        state = Odometry()
        state.pose.pose.position.x = i * 2.
        state.pose.pose.position.z = -2.
        drones.append(SimpleNamespace(state=state, state_received_at=0.,
                                     armed=True, offboard=True, move_to=Mock()))
    controller = SimpleNamespace(drones=drones, drone_count=3, command_all=Mock())
    behavior = SwarmBehaviors(controller)
    behavior.origins = [(0, 0, 0)] * 3
    points = [(i * 2., 0, -2) for i in range(3)]
    behavior.positions = Mock(return_value=points)
    def refresh(*args):
        for drone in drones:
            drone.state_received_at = time.monotonic()
    with patch('swarm_mavsdk_ros2.swarm_behaviors.spin_for', side_effect=refresh):
        behavior.move_to(points, timeout_s=1)
    controller.command_all.assert_not_called()
