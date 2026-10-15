from unittest.mock import Mock, patch

import pytest

from swarm_mavsdk_ros2.commands import DroneCommandClient


def client():
    value = object.__new__(DroneCommandClient)
    value.node = Mock()
    value.namespace = '/drone_0'
    value.armed = value.offboard = None
    value.move_to = Mock()
    return value


def test_odometry_before_status_waits_for_both_status_callbacks():
    value = client()
    statuses = iter(['armed', 'offboard'])
    def status(*args, **kwargs):
        setattr(value, next(statuses), True)
        value.move_to.assert_not_called()
    with patch('swarm_mavsdk_ros2.commands.rclpy.spin_once', side_effect=status) as spin:
        value.wait_for_movement_ready()
    assert spin.call_count == 2


@pytest.mark.parametrize('armed,offboard', [(False, True), (True, False), (False, False)])
def test_confirmed_unready_status_rejects_before_publishing(armed, offboard):
    value = client()
    value.armed, value.offboard = armed, offboard
    with pytest.raises(RuntimeError, match='startup --drone 0'):
        value.move_to_and_wait(1, 0, -2)
    value.move_to.assert_not_called()


def test_missing_status_times_out_without_publishing():
    value = client()
    with pytest.raises(TimeoutError, match='No armed/OFFBOARD status'):
        value.wait_for_movement_ready(timeout_s=0)
    value.move_to.assert_not_called()
