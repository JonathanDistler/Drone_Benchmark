import math
from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest
from nav_msgs.msg import Odometry

from swarm_mavsdk_ros2.rl_interface import DroneRlInterface
from swarm_mavsdk_ros2.swarm_rl import SwarmRlInterface
from swarm_mavsdk_ros2.swarm_behaviors import SwarmBehaviors


def test_rl_position_targets_are_metres_and_remain_active():
    client = Mock()
    adapter = DroneRlInterface(client)
    with patch.object(adapter, 'observation', return_value='state'), \
         patch('swarm_mavsdk_ros2.rl_interface.rclpy.spin_once'):
        assert adapter.step([10, 2, -3], duration_s=.001, mode='position') == 'state'
    client.move_to.assert_called_once_with(10., 2., -3.)
    client.set_velocity.assert_not_called()


@pytest.mark.parametrize('mode', ['position', 'velocity'])
def test_swarm_actions_validate_entire_fleet_before_dispatch(mode):
    controller = SimpleNamespace(drone_count=2, drones=[Mock(), Mock()])
    adapter = SwarmRlInterface(controller)
    apply = adapter.apply_position_actions if mode == 'position' else adapter.apply_actions
    with pytest.raises(ValueError):
        apply({0: [1, 2, 3], 1: [math.nan, 0, 0]})
    for drone in controller.drones:
        drone.move_to.assert_not_called()
        drone.set_velocity.assert_not_called()


def test_velocity_formation_caps_speed_brakes_and_holds_destination():
    state = Odometry()
    drone = SimpleNamespace(state=state, armed=True, offboard=True,
                            state_received_at=0., move_to=Mock(), set_velocity=Mock())
    controller = SimpleNamespace(drone_count=1, drones=[drone], command_all=Mock())
    behavior = SwarmBehaviors(controller)
    behavior.origins = [(0, 0, 0)]
    behavior.positions = Mock(return_value=[(0, 0, 0)])
    clock = [0.]
    def advance(*args):
        clock[0] += .1
        drone.state_received_at = clock[0]
        state.pose.pose.position.x = min(2., state.pose.pose.position.x + .1)
    with patch('swarm_mavsdk_ros2.swarm_behaviors.spin_for', side_effect=advance), \
         patch('swarm_mavsdk_ros2.swarm_behaviors.time.monotonic', side_effect=lambda: clock[0]):
        behavior.move_to([(2, 0, 0)], mode='velocity', speed_mps=.4)
    speeds = [math.hypot(*call.args) for call in drone.set_velocity.call_args_list]
    assert max(speeds) == pytest.approx(.4)
    assert min(speeds) == 0.
    drone.move_to.assert_not_called()
    controller.command_all.assert_called_once_with('stop')


def test_single_velocity_position_tracking_caps_vector_norm():
    from swarm_mavsdk_ros2.commands import DroneCommandClient
    client = object.__new__(DroneCommandClient)
    client.wait_for_state = Mock()
    client.state = Odometry()
    client.set_velocity = Mock()
    client.track_position(3, 4, 0, mode='velocity', speed_mps=2.)
    assert client.set_velocity.call_count == 1
    assert client.set_velocity.call_args.args == pytest.approx((1.2, 1.6, 0.))


@pytest.mark.parametrize('mode', ['position', 'velocity'])
def test_relative_move_tracks_destination_and_preserves_selected_mode(mode):
    from swarm_mavsdk_ros2.commands import DroneCommandClient
    client = object.__new__(DroneCommandClient)
    client.state = Odometry()
    client.state.pose.pose.position.y = 3.
    client.armed = client.offboard = True
    client.wait_for_state = Mock()
    client.move_to_and_wait = Mock()
    client.stop = Mock()
    client.move_forward(2., mode=mode, speed_mps=.4)
    client.move_to_and_wait.assert_called_once_with(
        2., 3., 0., mode=mode, speed_mps=.4, timeout_s=60.)
    client.move_to_and_wait.side_effect = RuntimeError('lost offboard')
    with pytest.raises(RuntimeError, match='lost offboard'):
        client.move_forward(2., mode=mode)
    client.stop.assert_called_once()
