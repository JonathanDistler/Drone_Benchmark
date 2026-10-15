from types import SimpleNamespace
from unittest.mock import Mock
import math

from swarm_mavsdk_ros2.drone_node import DroneNode


def test_position_reference_advances_at_point_four_and_reaches_goal():
    node = object.__new__(DroneNode)
    node.started, node.landing = True, False
    node.control_mode = 'position'
    node.local_position = SimpleNamespace(x=0., y=0., z=0.)
    node.position_target = [0.3, 0.4, 0.]
    node.position_reference = None
    node.reference_time_ns = None
    node.position_speed_mps = .4
    node.yaw_target = math.nan
    node.trajectory_setpoint_pub = Mock()
    clock = [0]
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=clock[0]))
    node.publish_setpoint()
    previous = [0., 0., 0.]
    for step in range(1, 31):
        clock[0] = step * 50_000_000
        node.publish_setpoint()
        target = list(node.trajectory_setpoint_pub.publish.call_args.args[0].position)
        assert math.dist(previous, target) <= .020001
        node.local_position = SimpleNamespace(x=target[0], y=target[1], z=target[2])
        previous = target
    assert math.dist(previous, node.position_target) < 1e-6
    node.control_mode = 'velocity'
    node.velocity_target = [.4, -.2, 0.]
    node.publish_setpoint()
    msg = node.trajectory_setpoint_pub.publish.call_args.args[0]
    assert all(math.isnan(x) for x in msg.position)
    assert list(msg.velocity) == [float(x) for x in msg.velocity]
    assert math.isclose(msg.velocity[0], .4, rel_tol=1e-6)


def test_takeoff_streams_climb_goal_then_resumes_slow_position_tracking():
    node = object.__new__(DroneNode)
    node.started, node.landing = True, False
    node.control_mode = 'position'
    node.local_position = SimpleNamespace(x=0., y=0., z=0.)
    node.position_target = [0., 0., -2.]
    node.position_reference = None
    node.reference_time_ns = None
    node.position_speed_mps = .4
    node.takeoff_climb_active = True
    node.yaw_target = math.nan
    node.trajectory_setpoint_pub = Mock()
    clock = [0]
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=clock[0]))
    node.publish_setpoint()
    assert list(node.trajectory_setpoint_pub.publish.call_args.args[0].position) == [0., 0., -2.]
    node.local_position.z = -2.
    node.publish_setpoint()
    assert not node.takeoff_climb_active
    node.position_target = [1., 0., -2.]
    clock[0] += 50_000_000
    node.publish_setpoint()
    assert math.isclose(node.trajectory_setpoint_pub.publish.call_args.args[0].position[0], .02, rel_tol=1e-6)
