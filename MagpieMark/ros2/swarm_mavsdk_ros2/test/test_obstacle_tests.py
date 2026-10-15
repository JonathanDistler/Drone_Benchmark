import csv
import math
from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest

from swarm_mavsdk_ros2 import obstacle_tests as tests


def test_formatted_string_selection():
    assert tests.parse_test(f'test {2}') == 2
    for command in ('test 0', 'test 4', 'test 2 extra', '2'):
        with pytest.raises(ValueError):
            tests.parse_test(command)


def test_logging_starts_after_transit_and_appends_all_runs(tmp_path):
    p = SimpleNamespace(x=0., y=0., z=-2.)
    state = SimpleNamespace(pose=SimpleNamespace(pose=SimpleNamespace(position=p)),
                            twist=SimpleNamespace(twist=SimpleNamespace(linear=SimpleNamespace(x=0., y=0., z=0.))))
    moves = []
    arrival_modes = []
    client = SimpleNamespace(state=state, armed=True, offboard=True,
                             wait_for_state=lambda: None, set_position_control=lambda: None,
                             startup=Mock(), arm=Mock(), takeoff=Mock(), wait_for_offboard=Mock())
    def move(*target, sample=None, settle=True, timeout_s=60., **controls):
        if sample is None:
            assert not (tmp_path / 'all_tests.csv').exists() or len(moves) >= 5
        p.x, p.y, p.z = target
        moves.append(target)
        if sample:
            arrival_modes.append(settle)
        if sample:
            sample(state)
    client.move_to_and_wait = move
    with patch.object(tests, 'world_configuration', return_value=0.), patch.object(tests, 'follow_sinusoid', side_effect=lambda c,s,r,n,sample,progress,**controls: [(progress(i,n(p)), c.move_to_and_wait(*n(p),sample=sample,settle=i==len(r))) for i,p in enumerate(r,1)]):
        tests.run_test(client, 'test 2', output_dir=tmp_path)
        tests.run_test(client, 'test 3', output_dir=tmp_path)
    with next(tmp_path.glob('20*.csv')).open() as file:
        rows = list(csv.DictReader(file))
    assert {r['test'] for r in rows} == {'2', '3'}
    assert [r['status'] for r in rows].count('completed') == 2
    assert arrival_modes == [False, True] + [False, False, False, True]
    assert rows[0]['waypoint'] == '0'
    assert float(rows[0]['north']) == -8
    assert float(rows[0]['east']) == -1


def test_blank_world_rejected_before_movement(tmp_path):
    client = SimpleNamespace()
    with patch.object(tests, 'world_configuration', side_effect=RuntimeError('blank')):
        with pytest.raises(RuntimeError):
            tests.run_test(client, 'test 1', output_dir=tmp_path)
    assert not (tmp_path / 'all_tests.csv').exists()


def test_refreshes_telemetry_after_blocking_world_check(tmp_path):
    clock = [10.]
    position = SimpleNamespace(x=0., y=0., z=-2.)
    state = SimpleNamespace(
        pose=SimpleNamespace(pose=SimpleNamespace(position=position)),
        twist=SimpleNamespace(twist=SimpleNamespace(
            linear=SimpleNamespace(x=0., y=0., z=0.))))
    client = SimpleNamespace(
        node=Mock(), state=state, state_received_at=clock[0],
        armed=True, offboard=True, startup=Mock(),
        wait_for_offboard=Mock(), set_position_control=Mock(), stop=Mock())
    def refresh():
        client.state_received_at = clock[0]
    client.wait_for_state = Mock(side_effect=refresh)
    def arrive(*target, **kwargs):
        position.x, position.y, position.z = target
        refresh()
    client.move_to_and_wait = Mock(side_effect=arrive)
    def inspect_world(_):
        clock[0] += 3.
        return 0.
    def follow(c, start, route, ned, sample, progress, **controls):
        assert clock[0] - c.state_received_at < 1.
        sample(c.state)
    with patch.object(tests.time, 'monotonic', side_effect=lambda: clock[0]), \
            patch.object(tests, 'world_configuration', side_effect=inspect_world), \
            patch.object(tests, 'follow_sinusoid', side_effect=follow):
        tests.run_test(client, 'test 1', output_dir=tmp_path)
    assert client.wait_for_state.call_count == 2
    with next(tmp_path.glob('20*.csv')).open() as file:
        rows = list(csv.DictReader(file))
    assert all(float(row['state_age_s']) < 1. for row in rows)
    assert rows[-1]['status'] == 'completed'


@pytest.mark.parametrize('signal', [0, 1])
def test_flag_output_and_complete_route(tmp_path, capsys, signal):
    p = SimpleNamespace(x=0., y=0., z=0.)
    state = SimpleNamespace(pose=SimpleNamespace(pose=SimpleNamespace(position=p)),
                            twist=SimpleNamespace(twist=SimpleNamespace(linear=SimpleNamespace(x=0., y=0., z=0.))))
    calls = []
    client = SimpleNamespace(state=state, armed=False, offboard=False,
                             wait_for_state=Mock(), set_position_control=Mock(),
                             startup=lambda: calls.append('startup'),
                             arm=lambda: calls.append('arm'),
                             takeoff=lambda: calls.append('takeoff'),
                             wait_for_offboard=lambda expected: calls.append('offboard'))
    def move(*target, sample=None, settle=True, timeout_s=60., **controls):
        calls.append(target)
        p.x, p.y, p.z = target
        if sample:
            sample(state)
    client.move_to_and_wait = move
    with patch.object(tests, 'world_configuration', return_value=2.), \
            patch.object(tests.random.Random, 'randint', return_value=signal), patch.object(tests, 'follow_sinusoid', side_effect=lambda c,s,r,n,sample,progress,**controls: [(progress(i,n(p)), c.move_to_and_wait(*n(p),sample=sample)) for i,p in enumerate(r,1)]):
        tests.run_test(client, 'test 2', output_dir=tmp_path)
    assert calls[:4] == ['startup', 'arm', 'takeoff', 'offboard']
    side_north = (-7.1 if signal == 0 else -8.9) - 2.
    assert calls[-2:] == [(side_north, 5, -2), (-10., 10, -2)]
    assert f'Test 2 output: {signal}' in capsys.readouterr().out


def test_cone_weave_crosses_between_cones_and_box_traverses_opening():
    _, cones = tests.COURSES[1]
    assert [point[1] for point in cones[:3]] == [-.68, .68, -.68]
    assert [point[0] for point in cones[:3]] == [2, 5, 8]
    start, box = tests.COURSES[3]
    assert start[0] < 5 and box[-1][0] > 7.438
    assert all(point[1:] == (8.5584, .5464) for point in box)


@pytest.mark.parametrize('spawn_y', [0., 2.])
@pytest.mark.parametrize('previous', [1, 2, 3])
def test_return_to_box_start_climbs_before_retreat_and_descends_after_lane_change(spawn_y, previous):
    ned = lambda point: (point[1] - spawn_y, point[0], -point[2])
    finish = ned(tests.COURSES[previous][1][-1])
    client = SimpleNamespace(
        state=SimpleNamespace(pose=SimpleNamespace(pose=SimpleNamespace(
            position=SimpleNamespace(x=finish[0], y=finish[1], z=finish[2])))),
        move_to_and_wait=Mock())
    tests.move_to_start(client, tests.COURSES[3][0], ned)
    targets = [call.args for call in client.move_to_and_wait.call_args_list]
    lane = 8.5584 - spawn_y
    assert targets == [(finish[0], 10., -3.), (finish[0], -2., -3.),
                       (lane, -2., -3.), (lane, -2., -.5464),
                       (lane, -1., -.5464)]
    assert all(call.kwargs['timeout_s'] >= 60.
               for call in client.move_to_and_wait.call_args_list)


def test_start_transit_timeout_identifies_failed_step():
    client = SimpleNamespace(
        state=SimpleNamespace(pose=SimpleNamespace(pose=SimpleNamespace(
            position=SimpleNamespace(x=-8., y=10., z=-2.)))),
        move_to_and_wait=Mock(side_effect=[None, TimeoutError('target unreachable')]))
    with pytest.raises(TimeoutError, match='Start transit step 2/5: target unreachable'):
        tests.move_to_start(client, tests.COURSES[3][0], lambda p: (p[1], p[0], -p[2]))
    assert client.move_to_and_wait.call_count == 2


def test_shorter_weave_preserves_clearance_and_finish():
    start, route = tests.COURSES[1]
    old = [start, (2, -.75, 2), (3.5, 0, 2), (5, .75, 2),
           (6.5, 0, 2), (8, -.75, 2), (10, 0, 2)]
    points = [start, *route]
    length = lambda path: sum(math.dist(a, b) for a, b in zip(path, path[1:]))
    assert length(points) < length(old)
    assert points[-1] == old[-1]
    # Check full segments against the footprint, rather than just waypoints
    # The rotated mesh occupies X=cone..cone+.0762, Y=-.0762..0
    for x in (2, 5, 8):
        center = (x + .0381, -.0381)
        for a, b in zip(points, points[1:]):
            dx, dy = b[0]-a[0], b[1]-a[1]
            t = max(0., min(1., ((center[0]-a[0])*dx + (center[1]-a[1])*dy)/(dx*dx+dy*dy)))
            distance = math.dist(center, (a[0]+t*dx, a[1]+t*dy))
            assert distance - math.hypot(.0381, .0381) > .50


def test_battery_power_and_invalid_or_stale_readings():
    import time
    state = SimpleNamespace(
        pose=SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(z=-2.))),
        twist=SimpleNamespace(twist=SimpleNamespace(linear=SimpleNamespace(x=3., y=4., z=0.))))
    battery = SimpleNamespace(timestamp=100, connected=True, warning=0,
                              voltage_v=12., current_a=2., remaining=.8,
                              discharged_mah=10., temperature=25., time_remaining_s=60.)
    client = SimpleNamespace(battery=battery, battery_received_at=time.monotonic())
    row = tests.telemetry_columns(client, state)
    assert row['power_w'] == 24.
    assert row['speed_mps'] == 5.
    assert row['altitude_m'] == 2.
    battery.current_a = -1.
    row = tests.telemetry_columns(client, state)
    assert row['battery_current_a'] == ''
    assert 'power_w' not in row
    battery.connected = False
    assert 'battery_voltage_v' not in tests.telemetry_columns(client, state)
    client.battery_received_at -= 10
    assert 'battery_connected' not in tests.telemetry_columns(client, state)


def test_csv_upgrade_preserves_history_and_adds_empty_cells(tmp_path):
    path = tmp_path / 'all_tests.csv'
    path.write_text('run_id,test,north\nold,1,2.5\n')
    tests.upgrade_csv(path)
    tests.upgrade_csv(path)
    with path.open() as file:
        reader = csv.DictReader(file)
        assert reader.fieldnames == tests.FIELDS
        rows = list(reader)
    assert len(rows) == 1
    assert rows[0]['north'] == '2.5'
    assert rows[0]['power_w'] == ''


def test_sinusoidal_peaks_and_continuous_forward_path():
    start, route = tests.COURSES[1]
    for point in route:
        assert tests.sinusoidal_point(start, route, point[0]) == pytest.approx(point)
    for x in (2, 5, 8):
        a = tests.sinusoidal_point(start, route, x-.0001)
        b = tests.sinusoidal_point(start, route, x+.0001)
        assert abs((b[1]-a[1])/.0002) < .001
    assert tests.sinusoidal_point(start, route, 3.5)[1] == pytest.approx(0)
    for sign in (-1, 1):
        flag = [(5, -8+sign*.9, 2), (10, -8, 2)]
        assert tests.sinusoidal_point((-1,-8,2), flag, 5)[1] == pytest.approx(-8+sign*.9)


@pytest.mark.parametrize('test', [1, 2])
def test_sinusoid_streams_without_intermediate_arrival_waits(test):
    start, route = tests.COURSES[test]
    clock = [0.]
    client = SimpleNamespace(node=Mock(), armed=True, offboard=True,
                             state=SimpleNamespace(pose=SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=start[0], y=start[1], z=start[2])))), state_received_at=0.,
                             move_to=Mock(), move_to_and_wait=Mock())
    def tick(*args):
        clock[0] += .05
        client.state_received_at = clock[0]
        p = client.state.pose.pose.position
        p.x, p.y, p.z = client.move_to.call_args.args
    sample, progress = Mock(), Mock()
    with patch.object(tests.time, 'monotonic', side_effect=lambda: clock[0]), \
            patch.object(tests, 'spin_for', side_effect=tick):
        tests.follow_sinusoid(client, start, route, lambda p: p, sample, progress, mode="position", speed_mps=.5)
    client.move_to_and_wait.assert_called_once_with(*route[-1], sample=sample)
    points = [call.args for call in client.move_to.call_args_list]
    assert len(points) > 100
    assert points[0] == start
    assert points[-1] == route[-1]
    assert all(0 <= b[0]-a[0] <= .5*.05+1e-8 for a,b in zip(points,points[1:]))
    assert all(b[0] > a[0] for a,b in zip(points,points[1:]) if start[0]+.01 < a[0] < route[-1][0]-.01)
    assert all(p[2] == start[2] for p in points)
    for peak in route[:-1]:
        nearest = min(points, key=lambda p: abs(p[0]-peak[0]))
        assert nearest[1] == pytest.approx(peak[1], abs=.001)


def test_sinusoid_rejects_nonforward_route_before_dispatch():
    client = SimpleNamespace(move_to=Mock())
    with pytest.raises(ValueError, match='strictly increasing'):
        tests.follow_sinusoid(client, (0,0,2), [(0,1,2)], lambda p:p, Mock(), Mock(), mode="position", speed_mps=.5)
    client.move_to.assert_not_called()


@pytest.mark.parametrize('recovers', [True, False])
def test_sinusoid_pauses_for_telemetry_gap_and_bounds_recovery(recovers):
    clock = [0.]
    client = SimpleNamespace(node=Mock(), armed=True, offboard=True,
                             state=SimpleNamespace(pose=SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=0., y=0., z=2.)))), state_received_at=-2.,
                             move_to=Mock(), move_to_and_wait=Mock())
    def recover(timeout_s):
        assert timeout_s == 2.
        clock[0] += 1.5
        if not recovers:
            raise TimeoutError('No state received')
        client.state_received_at = clock[0]
    client.wait_for_state = Mock(side_effect=recover)
    def tick(*args):
        clock[0] += .05
        if client.wait_for_state.called:
            client.state_received_at = clock[0]
        p = client.state.pose.pose.position
        p.x, p.y, p.z = client.move_to.call_args.args
    with patch.object(tests.time, 'monotonic', side_effect=lambda: clock[0]), \
            patch.object(tests, 'spin_for', side_effect=tick):
        if recovers:
            tests.follow_sinusoid(client, (0,0,2), [(1,0,2)], lambda p:p, Mock(), Mock(), mode="position", speed_mps=.5)
            points = [call.args for call in client.move_to.call_args_list]
            assert all(0 <= b[0]-a[0] <= .5*.05+1e-8
                       for a,b in zip(points, points[1:]))
            client.move_to_and_wait.assert_called_once()
        else:
            with pytest.raises(TimeoutError, match='Trajectory telemetry became stale'):
                tests.follow_sinusoid(client, (0,0,2), [(1,0,2)], lambda p:p, Mock(), Mock(), mode="position", speed_mps=.5)
            assert client.move_to.call_count == 1
            client.move_to_and_wait.assert_not_called()
    client.wait_for_state.assert_called_once_with(timeout_s=2.)


def test_start_corridor_uses_one_faster_settled_move():
    client = SimpleNamespace(
        state=SimpleNamespace(pose=SimpleNamespace(pose=SimpleNamespace(
            position=SimpleNamespace(x=0., y=0., z=-2.)))),
        move_to_and_wait=Mock())
    tests.move_to_start(client, tests.COURSES[1][0], lambda p: (p[1], p[0], -p[2]))
    client.move_to_and_wait.assert_called_once_with(
        0, -1, -2, timeout_s=60., settle=True, mode='velocity', speed_mps=2.)


@pytest.mark.parametrize('previous', [1, 2, 3])
@pytest.mark.parametrize('next_test', [1, 2, 3])
def test_sequential_tests_use_altitude_control_then_fast_horizontal_return(previous, next_test):
    ned = lambda p: (p[1], p[0], -p[2])
    position = SimpleNamespace(**dict(zip(('x', 'y', 'z'),
                                         ned(tests.COURSES[previous][1][-1]))))
    client = SimpleNamespace(state=SimpleNamespace(pose=SimpleNamespace(
        pose=SimpleNamespace(position=position))))
    def arrive(north, east, down, **controls):
        if down != position.z:
            assert controls['mode'] == 'position'
        else:
            assert controls['mode'] == 'velocity'
            assert controls['speed_mps'] == 2.
        position.x, position.y, position.z = north, east, down
    client.move_to_and_wait = Mock(side_effect=arrive)
    tests.move_to_start(client, tests.COURSES[next_test][0], ned)
    assert (position.x, position.y, position.z) == ned(tests.COURSES[next_test][0])
    assert client.move_to_and_wait.call_args.kwargs['settle'] is True


def test_return_transit_only_settles_at_start():
    client = SimpleNamespace(
        state=SimpleNamespace(pose=SimpleNamespace(pose=SimpleNamespace(
            position=SimpleNamespace(x=0., y=10., z=-2.)))),
        move_to_and_wait=Mock())
    tests.move_to_start(client, tests.COURSES[1][0], lambda p: (p[1], p[0], -p[2]))
    assert [call.kwargs['settle'] for call in client.move_to_and_wait.call_args_list] == [False]*4 + [True]


def test_stalled_drone_cannot_skip_weave_and_times_out():
    clock = [0.]
    start, route = tests.COURSES[1]
    p = SimpleNamespace(x=start[0], y=start[1], z=start[2])
    client = SimpleNamespace(
        node=Mock(), armed=True, offboard=True, state_received_at=0.,
        state=SimpleNamespace(pose=SimpleNamespace(pose=SimpleNamespace(position=p))),
        move_to=Mock(), move_to_and_wait=Mock())
    def tick(*args):
        clock[0] += .05
        client.state_received_at = clock[0]
    with patch.object(tests.time, 'monotonic', side_effect=lambda: clock[0]), \
            patch.object(tests, 'spin_for', side_effect=tick):
        with pytest.raises(TimeoutError, match='could not keep up'):
            tests.follow_sinusoid(client, start, route, lambda p: p, Mock(), Mock(), mode="position", speed_mps=.5)
    assert max(call.args[0] for call in client.move_to.call_args_list) < start[0] + .2
    client.move_to_and_wait.assert_not_called()


@pytest.mark.parametrize('test', [1, 2])
def test_default_velocity_route_completes_with_controller_lag(test):
    start, route = tests.COURSES[test]
    ned = lambda point: (point[1], point[0], -point[2])
    clock = [0.]
    position = SimpleNamespace(**dict(zip(('x', 'y', 'z'), ned(start))))
    velocity = [0., 0., 0.]
    client = SimpleNamespace(
        node=Mock(), armed=True, offboard=True, state_received_at=0.,
        state=SimpleNamespace(pose=SimpleNamespace(pose=SimpleNamespace(position=position))),
        set_velocity=Mock(), move_to_and_wait=Mock())
    errors = []
    def tick(*args):
        clock[0] += .05
        client.state_received_at = clock[0]
        for i, axis in enumerate(('x', 'y', 'z')):
            velocity[i] += (.05/.3)*(client.set_velocity.call_args.args[i]-velocity[i])
            setattr(position, axis, getattr(position, axis)+velocity[i]*.05)
    def progress(index, target):
        errors.append(math.dist((position.x, position.y, position.z), target))
    with patch.object(tests.time, 'monotonic', side_effect=lambda: clock[0]), \
            patch.object(tests, 'spin_for', side_effect=tick):
        tests.follow_sinusoid(client, start, route, ned, Mock(), progress)
    assert clock[0] < 40.
    assert max(errors) < .15
    assert max(math.hypot(*call.args) for call in client.set_velocity.call_args_list) <= 1.+1e-9
    client.move_to_and_wait.assert_called_once()
    assert client.move_to_and_wait.call_args.args == ned(route[-1])
    assert client.move_to_and_wait.call_args.kwargs['mode'] == 'velocity'
