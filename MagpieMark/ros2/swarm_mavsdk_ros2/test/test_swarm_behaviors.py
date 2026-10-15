# Geometry, common-frame movement, and start-gated swarm benchmark logging
import csv
import math
import time
from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest
from nav_msgs.msg import Odometry

from swarm_mavsdk_ros2.swarm_behaviors import SwarmBehaviors, formation_targets, validate_points
from swarm_mavsdk_ros2 import swarm_benchmarks as benchmarks


@pytest.mark.parametrize('world', ['drone_obstacle_course', 'blank', ''])
def test_benchmarks_reject_other_worlds(world):
    with patch.object(benchmarks, 'launch_configuration', return_value=(world, 0.)), \
         patch.object(benchmarks.subprocess, 'run') as scene:
        with pytest.raises(RuntimeError, match='--world swarm'):
            benchmarks.world_configuration(Mock())
    scene.assert_not_called()


@pytest.mark.parametrize('scene_text,code,valid', [
    ('name: "ground" name: "test_start"', 0, True),
    ('name: "ground"', 0, False),
    ('name: "ground" name: "test_start"', 1, False),
])
def test_benchmarks_verify_live_swarm_world(scene_text, code, valid):
    with patch.object(benchmarks, 'launch_configuration',
                      return_value=('drone_obstacle_course_swarm', 2.)), \
         patch.object(benchmarks.subprocess, 'run',
                      return_value=Mock(stdout=scene_text, returncode=code)) as scene:
        if valid:
            assert benchmarks.world_configuration(Mock()) == 2.
        else:
            with pytest.raises(RuntimeError, match='incompletely loaded'):
                benchmarks.world_configuration(Mock())
    assert '/world/drone_obstacle_course_swarm/scene/info' in scene.call_args.args[0]


@pytest.mark.parametrize('shape', ['line','grid','circle'])
@pytest.mark.parametrize('count', [1,3,4,8])
def test_formations_preserve_centroid_and_separation(shape,count):
    center = (4.,-10.,-2.)
    points = formation_targets(shape,count,center)
    validate_points(points,count)
    for k in range(3):
        assert sum(p[k] for p in points)/count == pytest.approx(center[k])


def test_validation_rejects_overlap_and_nonfinite():
    with pytest.raises(ValueError):
        validate_points([(0,0,-2),(0,.5,-2)],2)
    with pytest.raises(ValueError):
        validate_points([(math.nan,0,-2)],1)


def fake_controller(count=3):
    drones = []
    for i in range(count):
        state = Odometry()
        state.pose.pose.position.z = -2
        drone = SimpleNamespace(state=state, armed=True,offboard=True,state_received_at=0.,
                                move_to=Mock())
        drones.append(drone)
    return SimpleNamespace(drones=drones,drone_count=count,command_all=Mock())


def test_common_frame_preserves_distinct_spawn_origins():
    controller = fake_controller()
    behavior = SwarmBehaviors(controller)
    behavior.origins = [(0,0,0),(2,0,0),(4,0,0)]
    def refresh(*args,**kwargs):
        for drone in controller.drones:
            drone.state_received_at = time.monotonic()
    with patch('swarm_mavsdk_ros2.swarm_behaviors.rclpy.spin_once',side_effect=refresh):
        points = behavior.positions()
    assert points == [(0,0,-2),(2,0,-2),(4,0,-2)]
    assert benchmarks.pairwise_mse(points,'xy') == pytest.approx(8)
    assert benchmarks.pairwise_mse(points,'yz') == 0


def test_crossing_paths_are_rejected_before_dispatch():
    controller = fake_controller(2)
    behavior = SwarmBehaviors(controller)
    behavior.origins = [(0,0,0),(2,0,0)]
    behavior.positions = Mock(return_value=[(0,0,-2),(2,0,-2)])
    with pytest.raises(ValueError,match='Paths cross'):
        behavior.move_to([(2,0,-2),(0,0,-2)])
    for drone in controller.drones:
        drone.move_to.assert_not_called()


def test_stale_telemetry_holds_entire_fleet():
    controller = fake_controller(1)
    behavior = SwarmBehaviors(controller)
    behavior.origins = [(0,0,0)]
    behavior.positions = Mock(return_value=[(0,0,-2)])
    with patch('swarm_mavsdk_ros2.swarm_behaviors.spin_for'), \
            pytest.raises(TimeoutError,match='stale'):
        behavior.move_to([(0,1,-2)])
    controller.command_all.assert_called_once_with('stop')


def test_partition_moves_every_member_and_requires_majority():
    controller = fake_controller(5)
    behavior = SwarmBehaviors(controller)
    points = formation_targets('line',5,(0,-20,-2))
    behavior.positions = Mock(return_value=points)
    behavior.go_to = Mock()
    targets = behavior.split('left')
    assert all(p[0]<0 for p in targets[:3])
    assert all(p[0]>0 for p in targets[3:])
    validate_points(targets,5)
    controller.drone_count = 2
    behavior.positions.return_value = points[:2]
    with pytest.raises(ValueError):
        behavior.split()


@pytest.mark.parametrize('number', [1,2,3])
@pytest.mark.parametrize('mode', ['position', 'velocity'])
def test_setup_is_excluded_and_all_test_schemas_match(tmp_path,number,mode):
    controller = fake_controller()
    calls = []
    class FakeBehaviors:
        def __init__(self,_):
            self.points = [(0,-15+i*1.5,-2) for i in range(3)]
        def configure(self): pass
        def positions(self): return list(self.points)
        def formation(self,*args,**kwargs):
            calls.append('line')
            self.emit('line',kwargs)
        def randomize(self,*args,**kwargs):
            calls.append('random')
            assert not (tmp_path/'all_tests.csv').exists()
        def split(self,*args,**kwargs):
            calls.append('partition')
            assert not (tmp_path/'all_tests.csv').exists()
        def translate(self,**kwargs):
            calls.append('translation')
            self.points = [(n,e+kwargs['east'],d) for n,e,d in self.points]
            self.emit(kwargs['phase'],kwargs)
        def emit(self,phase,kwargs):
            if kwargs.get('sample'):
                kwargs['sample'](phase,self.points,self.points)
    with patch.object(benchmarks,'SwarmBehaviors',FakeBehaviors), patch.object(benchmarks,'world_configuration'):
        path = benchmarks.run_test(controller,f'test {number}',output_dir=tmp_path,seed=42,
                                   mode=mode, speed_mps=.4)
    with open(path) as file:
        rows = list(csv.DictReader(file))
    assert set(rows[0]) == set(benchmarks.SAMPLE_FIELDS)
    assert all(r['phase'] not in ('partition','random') for r in rows)
    assert rows[-1]['status'] == 'completed'
    assert {r['drone_id'] for r in rows} == {'0', '1', '2'}
    assert all(float(r['swarm_min_separation_m']) == pytest.approx(1.5) for r in rows)
    assert all(float(r['swarm_target_rmse_m']) == 0 for r in rows)
    assert all(r['armed'] == 'True' and r['control_mode'] == mode for r in rows)
    assert all(float(r['requested_speed_mps']) == .4 for r in rows)
    with (tmp_path/'summary.csv').open() as file:
        summary = list(csv.DictReader(file))[0]
    assert float(summary['xy_mse_after_m2']) == pytest.approx(float(summary['xy_mse_before_m2']))
    assert calls == {1:['line','translation'],2:['line','random','line'],3:['line','partition','translation']}[number]


def test_failure_preserves_samples_and_summary(tmp_path):
    controller = fake_controller()
    behavior = Mock()
    behavior.positions.return_value = [(0,-15+i*1.5,-2) for i in range(3)]
    behavior.translate.side_effect = TimeoutError('failed movement')
    with patch.object(benchmarks,'SwarmBehaviors',return_value=behavior), patch.object(benchmarks,'world_configuration'):
        with pytest.raises(TimeoutError):
            benchmarks.run_test(controller,'test 1',output_dir=tmp_path)
    with (tmp_path/'summary.csv').open() as file:
        row = list(csv.DictReader(file))[0]
    assert row['status'] == 'failed'
    assert 'failed movement' in row['error']
    controller.command_all.assert_called_with('stop')


@pytest.mark.parametrize('targets', [
    [(2,4,-2),(2,2,-2),(2,0,-2)],
    [(0,4,-2),(0,2,-2),(0,0,-2)],
])
def test_staged_formation_stays_on_one_plane_and_avoids_stationary_drones(targets):
    controller = fake_controller(3)
    behavior = SwarmBehaviors(controller)
    starts = [(0,0,-2),(0,2,-2),(0,4,-2)]
    behavior.positions = Mock(return_value=list(starts))
    dispatched = []
    def move(points, **kwargs):
        dispatched.append(list(points))
        validate_points(points,3)
    behavior.move_to = move
    behavior.go_to(targets)
    previous = starts
    for points in dispatched:
        assert all(p[2] == -2 for p in points)
        assert sum(a != b for a,b in zip(previous,points)) <= 1
        for u in range(101):
            interpolated = [tuple(a[k]+u/100*(b[k]-a[k]) for k in range(3))
                            for a,b in zip(previous,points)]
            validate_points(interpolated,3)
        previous = points
    assert dispatched[-1] == targets


@pytest.mark.parametrize('shape', ['line', 'grid', 'circle'])
def test_shape_moves_horizontally_to_minimum_travel_slots(shape):
    import itertools
    from swarm_mavsdk_ros2.swarm_behaviors import optimal_slots
    starts = [(4, 0, -2.1), (0, 0, -2), (2, 0, -1.9)]
    controller = fake_controller(3)
    behavior = SwarmBehaviors(controller)
    behavior.origins = [(0, 0, 0), (2, 0, 0), (4, 0, 0)]
    behavior.positions = Mock(return_value=starts)
    behavior.move_to = Mock()
    behavior.go_to = Mock()
    targets = behavior.formation(shape, staged=True)
    assert all(p[2] == pytest.approx(-2.) for p in targets)
    behavior.go_to.assert_called_once_with(targets, staged=True)
    behavior.move_to.assert_not_called()
    def cost(points):
        return sum(sum((p[k]-q[k])**2 for k in (0, 1)) for p,q in zip(starts,points))
    assert cost(targets) == pytest.approx(min(cost(order) for order in itertools.permutations(targets)))
    assert optimal_slots(starts, targets) == targets
    if shape == 'line':
        assert all(p[0] == pytest.approx(2) for p in targets)
        assert len({p[1] for p in targets}) == 3


def test_line_is_perpendicular_to_launch_axis():
    behavior = SwarmBehaviors(fake_controller(3))
    behavior.origins = [(0, 0, 0), (0, 2, 0), (0, 4, 0)]
    behavior.positions = Mock(return_value=[(0, 0, -2), (0, 2, -2), (0, 4, -2)])
    behavior.move_to = Mock()
    targets = behavior.formation('line')
    assert all(p[1] == pytest.approx(2) for p in targets)
    assert len({p[0] for p in targets}) == 3


def test_vision_world_positions_do_not_add_spawn_displacement_twice():
    controller = fake_controller()
    behavior = SwarmBehaviors(controller)
    with patch('swarm_mavsdk_ros2.swarm_behaviors.launch_configuration',
               side_effect=[('blank', 0.), ('blank', 2.), ('blank', 4.)]):
        behavior.configure()
    assert behavior.origins == [(0., 0., 0.)] * 3
    assert behavior.spawn_positions == [(0., 0., 0.), (2., 0., 0.), (4., 0., 0.)]


@pytest.mark.parametrize('stuck', [False, True])
def test_progress_watchdog_allows_slow_flight_but_detects_stuck_member(stuck):
    controller = fake_controller(2)
    behavior = SwarmBehaviors(controller)
    behavior.origins = [(0, 0, 0)] * 2
    controller.drones[1].state.pose.pose.position.y = 2.
    behavior.positions = Mock(return_value=[(0, 0, -2), (0, 2, -2)])
    clock = [0.]

    def advance(*args):
        clock[0] += .1
        for i, drone in enumerate(controller.drones):
            drone.state_received_at = clock[0]
            if not (stuck and i == 0):
                drone.state.pose.pose.position.x = min(1., clock[0] * .1)

    with patch('swarm_mavsdk_ros2.swarm_behaviors.spin_for', side_effect=advance), \
         patch('swarm_mavsdk_ros2.swarm_behaviors.time.monotonic', side_effect=lambda: clock[0]):
        if stuck:
            with pytest.raises(TimeoutError, match='drone 0:.*error=1.000 m'):
                behavior.move_to([(1, 0, -2), (1, 2, -2)], timeout_s=2.)
            assert clock[0] < 3.
            controller.command_all.assert_called_once_with('stop')
        else:
            behavior.move_to([(1, 0, -2), (1, 2, -2)], timeout_s=2.)
            assert clock[0] > 9.
            controller.command_all.assert_not_called()


def test_hover_noise_does_not_extend_progress_timeout():
    controller = fake_controller(1)
    behavior = SwarmBehaviors(controller)
    behavior.origins = [(0, 0, 0)]
    behavior.positions = Mock(return_value=[(0, 0, -2)])
    clock = [0.]

    def advance(*args):
        clock[0] += .1
        drone = controller.drones[0]
        drone.state_received_at = clock[0]
        drone.state.pose.pose.position.x = .01 * (round(clock[0] * 10) % 2)

    with patch('swarm_mavsdk_ros2.swarm_behaviors.spin_for', side_effect=advance), \
         patch('swarm_mavsdk_ros2.swarm_behaviors.time.monotonic', side_effect=lambda: clock[0]):
        with pytest.raises(TimeoutError, match='no arrival or 0.05 m progress'):
            behavior.move_to([(1, 0, -2)], timeout_s=2.)
    assert clock[0] < 3.
