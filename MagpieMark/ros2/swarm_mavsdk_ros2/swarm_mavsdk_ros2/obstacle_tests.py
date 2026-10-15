from .test_session import recorded_test, session_path
# Continuous sinusoidal obstacle tests; recording excludes transit to the start
import csv
import math
from datetime import datetime, timezone
from pathlib import Path
import re
import random
import subprocess
import time
import os
import tempfile

import rclpy
from rcl_interfaces.srv import GetParameters
from .spinning import spin_for


def sinusoidal_point(start, route, x):
    # Continuous forward path with zero lateral slope at every waypoint
    x = max(start[0], min(route[-1][0], x))
    points = [start, *route]
    for a, b in zip(points, points[1:]):
        if x <= b[0]:
            fraction = max(0., min(1., (x-a[0])/(b[0]-a[0])))
            blend = .5-.5*math.cos(math.pi*fraction)
            return (x, a[1]+(b[1]-a[1])*blend, a[2]+(b[2]-a[2])*blend)
    return points[-1]


def follow_sinusoid(client, start, route, ned, sample, progress, *, mode="velocity", speed_mps=1.):
    if mode not in ("position", "velocity") or not math.isfinite(speed_mps) or speed_mps <= 0:
        raise ValueError("Use position or velocity mode and a finite positive speed.")
    # Track the curve continuously, with velocity feed-forward in velocity mode.
    # Ease forward speed only at the start/end, never at interior waypoints
    points = [start, *route]
    if not route or any(b[0] <= a[0] for a,b in zip(points, points[1:])):
        raise ValueError('Sinusoidal routes require strictly increasing forward positions.')
    # Reserve speed for lateral motion and feedback; easing peaks at twice average.
    duration = 2*(route[-1][0]-start[0])/(speed_mps * (.7 if mode == "velocity" else 1.))
    began = time.monotonic()
    last_tick = began
    flight_time = 0.
    target = ned(start)
    deadline = began + max(60., duration * 4)
    progress_time = 0.
    while True:
        now = time.monotonic()
        if now > deadline:
            raise TimeoutError('Trajectory could not keep up with the obstacle route.')
        p = client.state.pose.pose.position
        # Advance the path clock only while the vehicle tracks the reference.
        # Otherwise a slow controller cuts across the weave as its goal runs away.
        tracking = math.dist((p.x, p.y, p.z), target) <= .15
        if tracking:
            flight_time += now - last_tick
        # A slow but advancing vehicle must not fail near the finish.
        if flight_time - progress_time >= .01:
            deadline = now + 60.
            progress_time = flight_time
        last_tick = now
        elapsed = flight_time
        u = min(1., elapsed/duration)
        distance_fraction = u-math.sin(2*math.pi*u)/(2*math.pi)
        x = start[0]+(route[-1][0]-start[0])*distance_fraction
        target = ned(sinusoidal_point(start, route, x))
        index = next((i for i,p in enumerate(route, 1) if x <= p[0]), len(route))
        progress(index, target)
        if mode == "position":
            client.move_to(*target)
        else:
            # Differentiate the path in NED, including the easing profile.
            dx_dt = ((route[-1][0]-start[0])/duration
                     * (1-math.cos(2*math.pi*u))) if tracking else 0.
            lo = max(start[0], x-.001)
            hi = min(route[-1][0], x+.001)
            a, b = ned(sinusoidal_point(start, route, lo)), ned(sinusoidal_point(start, route, hi))
            velocity = [dx_dt*(q-r)/(hi-lo) + goal-actual
                        for q, r, goal, actual in zip(b, a, target, (p.x, p.y, p.z))]
            scale = min(1., speed_mps/max(math.hypot(*velocity), 1e-9))
            client.set_velocity(*(scale*v for v in velocity))
        spin_for(client.node, .05)
        if client.armed is not True or client.offboard is not True:
            raise RuntimeError('Trajectory requires armed OFFBOARD mode.')
        if client.state is None or time.monotonic()-client.state_received_at > 1:
            # Hold the last reference while servicing callbacks through a brief
            # flight data gap. Exclude that wait from the trajectory clock so the
            # next reference cannot jump ahead when flight data resumes
            paused = time.monotonic()
            try:
                client.wait_for_state(timeout_s=2.)
            except TimeoutError as exc:
                raise TimeoutError('Trajectory telemetry became stale.') from exc
            began += time.monotonic() - paused
            last_tick = time.monotonic()
            deadline += last_tick - paused
            if client.armed is not True or client.offboard is not True:
                raise RuntimeError('Trajectory requires armed OFFBOARD mode.')
        sample(client.state)
        if u >= 1:
            break
    client.move_to_and_wait(*ned(route[-1]), sample=sample,
                            **({} if mode == "position" else dict(mode=mode, speed_mps=speed_mps)))

# Gazebo XYZ. Box mesh opening after its SDF rotation is at Z=.013..1.080,
# Y=8.025..9.092, and extends along X=5..7.438
# Keep alternating sides while eliminating intermediate straight-line targets
# At altitude 2 m the rotated flag meshes are below the vehicle. A 0.68 m
# offset also keeps each segment >0.50 m from the XY pole footprint, allowing
# for a 0.25 m vehicle radius and the 0.15 m waypoint arrival tolerance
CONE_OFFSET_M = .68
COURSES = {
    1: ((-1, 0, 2), [(2, -CONE_OFFSET_M, 2), (5, CONE_OFFSET_M, 2),
                         (8, -CONE_OFFSET_M, 2), (10, 0, 2)]),
    2: ((-1, -8, 2), [(5, -7.1, 2), (10, -8, 2)]),
    3: ((-1, 8.5584, .5464), [(4, 8.5584, .5464), (5.5, 8.5584, .5464),
                              (7.9, 8.5584, .5464), (10, 8.5584, .5464)]),
}
#for CSV output on drone metrics 
FIELDS = ['run_id', 'test', 'elapsed_s', 'wall_time', 'waypoint', 'status',
          'target_n', 'target_e', 'target_d', 'north', 'east', 'down',
          'vn', 've', 'vd', 'error_m', 'distance_m']
FIELDS += ['drone_namespace', 'speed_mps', 'horizontal_speed_mps', 'altitude_m',
           'armed', 'offboard', 'state_age_s', 'local_telemetry_age_s', 'px4_timestamp_us',
           'accel_n_mps2', 'accel_e_mps2', 'accel_d_mps2', 'heading_rad',
           'xy_valid', 'z_valid', 'v_xy_valid', 'v_z_valid',
           'battery_age_s', 'battery_timestamp_us', 'battery_connected',
           'battery_voltage_v', 'battery_current_a', 'power_w',
           'battery_remaining_fraction', 'battery_discharged_mah',
           'battery_temperature_c', 'battery_time_remaining_s', 'battery_warning',
           'control_mode', 'requested_speed_mps']


def telemetry_columns(client, state):
    # Missing, stale, and invalid sensor values are empty CSV cells
    now = time.monotonic()
    v = state.twist.twist.linear
    result = dict(control_mode=getattr(client, 'test_control_mode', ''),
                  requested_speed_mps=getattr(client, 'test_speed_mps', ''),
                  drone_namespace=getattr(client, 'namespace', ''),
                  speed_mps=math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z),
                  horizontal_speed_mps=math.hypot(v.x, v.y),
                  altitude_m=-state.pose.pose.position.z,
                  armed=getattr(client, 'armed', ''), offboard=getattr(client, 'offboard', ''))
    def age(name):
        value = now - getattr(client, name + '_received_at', float('-inf'))
        return value if math.isfinite(value) else ''
    result['state_age_s'] = age('state')
    local = getattr(client, 'local_telemetry', None)
    local_age = age('local_telemetry')
    result['local_telemetry_age_s'] = local_age
    if local is not None and local_age != '' and local_age <= 2:
        for field, attr in [('px4_timestamp_us', 'timestamp'),
                            ('accel_n_mps2', 'ax'), ('accel_e_mps2', 'ay'),
                            ('accel_d_mps2', 'az'), ('heading_rad', 'heading'),
                            *[(name, name) for name in ('xy_valid', 'z_valid', 'v_xy_valid', 'v_z_valid')]]:
            value = getattr(local, attr, '')
            result[field] = value if value == '' or math.isfinite(value) else ''
    battery = getattr(client, 'battery', None)
    battery_age = age('battery')
    result['battery_age_s'] = battery_age
    if battery is not None and battery_age != '' and battery_age <= 5:
        result.update(battery_timestamp_us=battery.timestamp,
                      battery_connected=battery.connected, battery_warning=battery.warning)
        if battery.connected:
            for field, attr, minimum, maximum in [
                ('battery_voltage_v', 'voltage_v', 0.000001, math.inf),
                ('battery_current_a', 'current_a', 0, math.inf),
                ('battery_remaining_fraction', 'remaining', 0, 1),
                ('battery_discharged_mah', 'discharged_mah', 0, math.inf),
                ('battery_temperature_c', 'temperature', -math.inf, math.inf),
                ('battery_time_remaining_s', 'time_remaining_s', 0, math.inf),
            ]:
                value = getattr(battery, attr, math.nan)
                result[field] = value if math.isfinite(value) and minimum <= value <= maximum else ''
            voltage, current = result['battery_voltage_v'], result['battery_current_a']
            if voltage != '' and current != '':
                result['power_w'] = voltage * current
    return result


def upgrade_csv(path):
    # Preserve historical rows when extending the shared CSV schema
    if not path.exists() or path.stat().st_size == 0:
        return
    with path.open(newline='', encoding='utf-8') as source:
        reader = csv.DictReader(source)
        if reader.fieldnames == FIELDS:
            return
        if not reader.fieldnames or not set(reader.fieldnames) <= set(FIELDS):
            raise ValueError(f'Unrecognized CSV schema: {path}')
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(mode='w', newline='', encoding='utf-8',
                                             dir=path.parent, delete=False) as target:
                temporary = target.name
                writer = csv.DictWriter(target, fieldnames=FIELDS)
                writer.writeheader()
                writer.writerows(reader)
            os.replace(temporary, path)
        finally:
            if temporary and os.path.exists(temporary):
                os.unlink(temporary)


def parse_test(command):
    match = re.fullmatch(r'test\s+([123])', str(command).strip(), re.IGNORECASE)
    if not match:
        raise ValueError("Use 'test 1', 'test 2', or 'test 3'.")
    return int(match[1])


def launch_configuration(client):
    service = client.parameters_client
    if not service.wait_for_service(timeout_sec=5):
        raise TimeoutError('Drone configuration unavailable.')
    request = GetParameters.Request()
    request.names = ['world_name', 'spawn_world_y']
    future = service.call_async(request)
    deadline = time.monotonic() + 5
    while not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(client.node, timeout_sec=.05)
    if not future.done():
        raise TimeoutError('Drone configuration timed out.')
    values = future.result().values
    if len(values) != 2 or not math.isfinite(values[1].double_value):
        raise RuntimeError('Invalid simulation configuration.')
    return values[0].string_value, values[1].double_value


def world_configuration(client):
    world, spawn_y = launch_configuration(client)
    if world != 'drone_obstacle_course':
        raise RuntimeError('Tests require the launcher with --world obstacles.')
    # Inspect the live world rather than trusting launch parameters alone
    result = subprocess.run(['gz', 'service', '-s', '/world/drone_obstacle_course/scene/info',
        '--reqtype', 'gz.msgs.Empty', '--reptype', 'gz.msgs.Scene',
        '--timeout', '3000', '--req', ''], capture_output=True, text=True, timeout=5)
    expected = ['cone_1', 'cone_2', 'cone_3', 'test_flag', 'test_box',
                'test1_start', 'test2_start', 'test3_start']
    if result.returncode or any(f'name: "{name}"' not in result.stdout for name in expected):
        raise RuntimeError('Obstacle world is unavailable or incompletely loaded.')
    return spawn_y


def move_to_start(client, start, ned):
    # Return above the obstacles, then descend in the clear start corridor
    p = client.state.pose.pose.position
    north, east, down = p.x, p.y, p.z
    transit_down = min(down, -3.)
    start_north, start_east, start_down = ned(start)
    # Already in the clear start corridor: go directly to the start.
    if east <= 0.:
        targets = []
        if abs(down-start_down) > .1:
            targets.append((north, east, start_down))
        targets.append((start_north, start_east, start_down))
    else:
        targets = [(north, east, transit_down),
                   (north, -2., transit_down),
                   (start_north, -2., transit_down),
                   (start_north, -2., start_down),
                   (start_north, start_east, start_down)]
    for step, target in enumerate(targets, 1):
        # Use altitude position control for climbs/descents. Horizontal returns
        # use a faster velocity controller, independently of the test settings.
        controls = (dict(mode='position') if abs(target[2]-down) > .1
                    else dict(mode='velocity', speed_mps=2.))
        # Allow slow SITL vehicles time to cross the full 16 m lane spacing
        timeout = max(60., math.dist((north, east, down), target) / .3 + 15.)
        try:
            client.move_to_and_wait(*target, timeout_s=timeout,
                                    settle=step == len(targets), **controls)
        except TimeoutError as exc:
            raise TimeoutError(f'Start transit step {step}/{len(targets)}: {exc}') from exc
        north, east, down = target


@recorded_test
def run_test(client, command, *, output_dir='results/tests', flag_direction=None, seed=None,
             mode='velocity', speed_mps=1.):
    if mode not in ('position', 'velocity') or not math.isfinite(speed_mps) or speed_mps <= 0:
        raise ValueError('Use position or velocity mode and a finite positive speed.')
    client.test_control_mode, client.test_speed_mps = mode, speed_mps
    controls = {} if mode == 'position' else dict(mode=mode, speed_mps=speed_mps)
    test = parse_test(command)
    if flag_direction not in (None, 'left', 'right'):
        raise ValueError("flag_direction must be left or right.")
    spawn_y = world_configuration(client)
    client.wait_for_state()
    # Test commands are complete simulation actions, including preparation
    # Startup confirms flight data/control streaming before arming
    client.startup()
    if client.armed is not True:
        print('Preparing drone: arm and takeoff.', flush=True)
        client.arm()
        client.takeoff()
    elif client.state.pose.pose.position.z > -.3:
        client.takeoff()
    client.wait_for_offboard(True)
    start, route = COURSES[test]
    if test == 2:
        # Near = toward the central cone lane (Y=0); far = away from it
        # Preserve the existing left/right override for Python callers
        signal = (0 if flag_direction == 'right' else 1) if flag_direction else random.Random(seed).randint(0, 1)
        sign = 1 if signal == 0 else -1
        # Approach, respond to the simulated flag signal, then pass forward
        route = [(5, -8 + sign * .9, 2), (10, -8, 2)]
        print(f"Test 2 output: {signal} ({'near' if signal == 0 else 'far'} side of flag)", flush=True)
    # PX4 local NED origin is the launcher spawn (Gazebo X=0,Y=spawn_y)
    def ned(point):
        x, y, z = point
        return (y - spawn_y, x, -z)
    client.set_position_control()
    print(f'Test {test}: moving to start.', flush=True)
    move_to_start(client, start, ned)
    world_configuration(client)
    # Gazebo's blocking scene query can outlast the flight data freshness limit
    # Service queued ROS callbacks before recording or starting the trajectory
    client.wait_for_state()
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    path = session_path(client, output)
    upgrade_csv(path)
    run_id = datetime.now(timezone.utc).isoformat()
    began = time.monotonic()
    previous = None
    distance = 0.0
    with path.open('a', newline='', encoding='utf-8') as file:
        writer = csv.DictWriter(file, fieldnames=FIELDS)
        if file.tell() == 0:
            writer.writeheader()
        def record(state, status='running'):
            nonlocal previous, distance
            p = state.pose.pose.position
            v = state.twist.twist.linear
            point = (p.x, p.y, p.z)
            if previous is not None:
                distance += math.dist(previous, point)
            previous = point
            row = dict(zip(FIELDS, [run_id, test, time.monotonic()-began,
                datetime.now(timezone.utc).isoformat(), index, status, *target,
                *point, v.x, v.y, v.z, math.dist(point, target), distance]))
            row.update(telemetry_columns(client, state))
            writer.writerow(row)
            file.flush()
        index, target = 0, ned(start)
        record(client.state)
        try:
            if test in (1, 2):
                def progress(new_index, new_target):
                    nonlocal index, target
                    index, target = new_index, new_target
                follow_sinusoid(client, start, route, ned, record, progress,
                                mode=mode, speed_mps=speed_mps)
            else:
                for index, point in enumerate(route, 1):
                    target = ned(point)
                    print(f'Test {test}: waypoint {index}/{len(route)} -> Gazebo {point}', flush=True)
                    client.move_to_and_wait(*target, sample=record, settle=index == len(route), **controls)
            record(client.state, 'completed')
            print(f'Test {test} completed.', flush=True)
        except BaseException:
            record(client.state, 'failed')
            client.stop()
            raise
    return str(path.resolve())
