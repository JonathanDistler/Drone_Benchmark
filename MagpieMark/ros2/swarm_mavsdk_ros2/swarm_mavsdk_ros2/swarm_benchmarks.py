from .test_session import recorded_test, session_path
# The three recorded swarm benchmarks, using the public drone interfaces
import csv
from datetime import datetime, timezone
import math
from pathlib import Path
import random
import subprocess
import time

from .obstacle_tests import launch_configuration, parse_test, telemetry_columns, FIELDS
from .swarm_behaviors import SwarmBehaviors

# for CSV output in regard to comparision with real-data
SAMPLE_FIELDS = ['run_id','test','drone_id','elapsed_s','wall_time','phase','status',
                 'flag_direction','target_n','target_e','target_d','north','east','down',
                 'vn','ve','vd','error_m','distance_m']
SAMPLE_FIELDS += FIELDS[17:] + [
    'swarm_centroid_n', 'swarm_centroid_e', 'swarm_centroid_d',
    'swarm_xy_mse_m2', 'swarm_yz_mse_m2', 'swarm_target_rmse_m',
    'swarm_min_separation_m']
SUMMARY_FIELDS = ['run_id','test','status','duration_s','drone_count','seed','flag_direction',
                  'majority_count','minority_count','xy_mse_before_m2','yz_mse_before_m2',
                  'xy_mse_after_m2','yz_mse_after_m2','target_rmse_m','error']


def pairwise_mse(points, plane):
    axes = {'xy': (0,1), 'yz': (1,2)}
    if plane not in axes:
        raise ValueError('Plane must be xy or yz.')
    values = [sum((a[k]-b[k])**2 for k in axes[plane])
              for i,a in enumerate(points) for b in points[i+1:]]
    return sum(values)/len(values) if values else 0.


def append_row(path, fields, row):
    with path.open('a', newline='', encoding='utf-8') as file:
        writer = csv.DictWriter(file, fieldnames=fields)
        if file.tell() == 0:
            writer.writeheader()
        writer.writerow(row)


def world_configuration(drone):
    world, spawn_y = launch_configuration(drone)
    if world != 'drone_obstacle_course_swarm':
        raise RuntimeError(
            f'Swarm benchmarks require the launcher with --world swarm '
            f'(current world: {world or "unspecified"}). Stop the simulation and restart with: '
            '/usr/bin/python3 scripts/start_simulation.py --world swarm --n <drone-count>')
    result = subprocess.run(['gz', 'service', '-s', f'/world/{world}/scene/info',
        '--reqtype', 'gz.msgs.Empty', '--reptype', 'gz.msgs.Scene',
        '--timeout', '3000', '--req', ''], capture_output=True, text=True, timeout=5)
    if result.returncode or any(f'name: "{name}"' not in result.stdout
                                for name in ('ground', 'test_start')):
        raise RuntimeError('Swarm world is unavailable or incompletely loaded.')
    return spawn_y


@recorded_test
def run_test(controller, command, *, output_dir='results/swarm_tests', seed=None,
             flag_direction=None, spacing_m=1.5, signal=None, mode="position", speed_mps=1.0):
    if mode not in ("position", "velocity") or not math.isfinite(speed_mps) or speed_mps <= 0:
        raise ValueError("Use position or velocity mode and a finite positive speed.")
    controls = {} if mode == "position" else dict(mode=mode, speed_mps=speed_mps)
    test = parse_test(command)
    if not math.isfinite(spacing_m) or spacing_m < .8:
        raise ValueError('Spacing must be finite and at least 0.8 m.')
    if flag_direction not in (None,'left','right'):
        raise ValueError('Flag direction must be left or right.')
    if test == 3 and controller.drone_count < 3:
        raise ValueError('Swarm test 3 requires at least three drones.')
    for drone in controller.drones:
        world_configuration(drone)
    behavior = SwarmBehaviors(controller)
    behavior.configure()
    # Use the center of the dedicated open swarm arena
    behavior.formation('line', spacing_m, center=(0.,0.,-2.), **controls)
    if test == 2:
        behavior.randomize(3., seed=seed, staged=False, **controls)
    direction = (flag_direction or random.Random(seed).choice(('left','right'))) if test == 3 else ''
    if test == 3:
        print(f'Partition direction: {direction}', flush=True)
        behavior.split(direction, spacing_m, staged=False, **controls)
    for drone in controller.drones:
        world_configuration(drone)
    behavior.positions()
    if signal:
        signal()  # Optional external/operator start signal, after setup has settled
    before = behavior.positions()
    run_id = datetime.now(timezone.utc).isoformat()
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    path = session_path(controller, output)
    began = time.monotonic()
    previous = list(before)
    distances = [0.]*controller.drone_count
    last_targets = list(before)
    last_points = list(before)
    last_phase = 'start'
    status = 'running'
    error = ''
    with path.open('a', newline='', encoding='utf-8') as file:
        writer = csv.DictWriter(file, fieldnames=SAMPLE_FIELDS)
        if file.tell() == 0:
            writer.writeheader()
        def sample(phase, targets, points):
            nonlocal last_targets, last_points, last_phase
            last_targets, last_points, last_phase = list(targets), list(points), phase
            elapsed = time.monotonic()-began
            wall = datetime.now(timezone.utc).isoformat()
            fleet = dict(control_mode=mode, requested_speed_mps=speed_mps,
                swarm_xy_mse_m2=pairwise_mse(points, 'xy'),
                swarm_yz_mse_m2=pairwise_mse(points, 'yz'),
                swarm_target_rmse_m=math.sqrt(sum(math.dist(p,t)**2
                    for p,t in zip(points,targets))/len(points)),
                swarm_min_separation_m=min((math.dist(p,q)
                    for i,p in enumerate(points) for q in points[i+1:]), default=''))
            fleet.update(zip(('swarm_centroid_n','swarm_centroid_e','swarm_centroid_d'),
                             (sum(p[k] for p in points)/len(points) for k in range(3))))
            for i,(drone,target,point) in enumerate(zip(controller.drones,targets,points)):
                distances[i] += math.dist(previous[i],point)
                previous[i] = point
                v = drone.state.twist.twist.linear
                row = dict(zip(SAMPLE_FIELDS,[run_id,test,i,elapsed,wall,phase,status,
                    direction,*target,*point,v.x,v.y,v.z,math.dist(point,target),distances[i]]))
                row.update(telemetry_columns(drone, drone.state))
                row.update(fleet)
                writer.writerow(row)
            file.flush()
        sample('start',before,before)
        try:
            if test == 1:
                behavior.translate(east=3., sample=sample, phase='translate_3m', **controls)
            elif test == 2:
                behavior.formation('line',spacing_m,sample=sample,phase='form_line', **controls)
            else:
                behavior.translate(east=1.,sample=sample,phase='translate_1m', **controls)
            status = 'completed'
        except BaseException as exc:
            status = 'failed'
            error = f'{type(exc).__name__}: {exc}'
            try:
                controller.command_all('stop')
            except Exception:
                pass
            raise
        finally:
            sample(last_phase,last_targets,last_points)
            after = last_points
            summary = dict(zip(SUMMARY_FIELDS,[run_id,test,status,time.monotonic()-began,
                controller.drone_count,seed,direction,
                controller.drone_count//2+1 if test==3 else '',
                controller.drone_count-(controller.drone_count//2+1) if test==3 else '',
                pairwise_mse(before,'xy'),pairwise_mse(before,'yz'),
                pairwise_mse(after,'xy'),pairwise_mse(after,'yz'),
                math.sqrt(sum(math.dist(p,t)**2 for p,t in zip(after,last_targets))/len(after)),error]))
            append_row(output/'summary.csv',SUMMARY_FIELDS,summary)
    return str(path.resolve())
