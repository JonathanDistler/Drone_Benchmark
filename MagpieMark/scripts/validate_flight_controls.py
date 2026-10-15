#!/usr/bin/env python3
"""Verify single-drone and swarm control in an already running swarm SITL world.

Commands startup, arm, takeoff, moves, all three benchmarks in both modes,
then landing and disarming. Use --controls-only for a smaller movement check.
"""
import csv
import math
import time
from pathlib import Path
from swarm_mavsdk_ros2 import Swarm
from swarm_mavsdk_ros2.obstacle_tests import telemetry_columns, FIELDS
from swarm_mavsdk_ros2.test_session import recorded_test
from swarm_validity_test import wait_until
from px4_msgs.msg import VehicleLandDetected
from rclpy.qos import qos_profile_sensor_data

@recorded_test
def single(controller, label, mode):
    drone = controller.drones[0]
    drone.test_control_mode, drone.test_speed_mps = mode, .4
    drone.wait_for_state()
    p = drone.state.pose.pose.position
    target = (p.x + .5, p.y, p.z)
    path = controller.validation_output / (label + '.csv')
    fields = ['test', 'mode', 'elapsed_s', 'status', 'north', 'east', 'down',
              'vn', 've', 'vd', 'target_n', 'target_e', 'target_d', 'error_m', *FIELDS[17:]]
    count, peak = 0, 0.
    began = time.monotonic()
    with path.open('w', newline='', encoding='utf-8') as handle:
        writer = csv.DictWriter(handle, fields)
        writer.writeheader()
        def sample(state, status='running'):
            nonlocal count, peak
            p, v = state.pose.pose.position, state.twist.twist.linear
            telemetry = telemetry_columns(drone, state)
            count += 1
            peak = max(peak, telemetry['speed_mps'])
            writer.writerow(dict(test=label, mode=mode, elapsed_s=time.monotonic()-began,
                status=status, north=p.x, east=p.y, down=p.z, vn=v.x, ve=v.y, vd=v.z,
                target_n=target[0], target_e=target[1], target_d=target[2],
                error_m=math.dist((p.x,p.y,p.z), target), **telemetry))
            handle.flush()
        try:
            drone.move_to_and_wait(*target, mode=mode, speed_mps=.4, sample=sample)
        except BaseException:
            sample(drone.state, 'failed')
            drone.stop()
            raise
        sample(drone.state, 'completed')
    drone.move_forward(.2, mode=mode, speed_mps=.4)
    drone.move_backward(.2, mode=mode, speed_mps=.4)
    print('PASS', label, 'samples', count, 'peak speed', peak, flush=True)
    return str(path)


def main():
    import argparse
    from datetime import datetime, timezone
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--n', type=int, default=3)
    parser.add_argument('--architecture', default='central')
    parser.add_argument('--controls-only', action='store_true')
    parser.add_argument('--output-dir', type=Path, default=Path('results/control_validation'))
    args = parser.parse_args()
    if args.n < 1 or (args.n < 3 and not args.controls_only):
        parser.error('All three benchmarks require at least three drones.')
    output = args.output_dir / datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S_%f')
    output.mkdir(parents=True)
    with Swarm(args.n, architecture=args.architecture) as swarm:
        from swarm_mavsdk_ros2.swarm_controller import SwarmController
        observer = None
        if args.architecture != 'central':
            observer = SwarmController(args.n, namespace='/validation', use_global_arguments=False)
        c = observer or swarm.controller
        c.validation_output = output
        landed = {}
        for i in range(args.n):
            c.create_subscription(VehicleLandDetected, ('' if i == 0 else f'/px4_{i}') + '/fmu/out/vehicle_land_detected', lambda msg, i=i: landed.update({i: (time.monotonic(), msg.landed)}), qos_profile_sensor_data)
        try:
            swarm.startup(); swarm.arm(); swarm.takeoff()
            for mode in ('position', 'velocity'):
                single(c, 'single_' + mode, mode)
            for mode in ('position', 'velocity'):
                swarm.forward(.5, mode=mode, speed_mps=.4)
                swarm.backward(.5, mode=mode, speed_mps=.4)
                print('PASS fleet directional', args.architecture, mode, flush=True)
            for mode in (() if args.controls_only else ('position', 'velocity')):
                for test in (1, 2, 3):
                    path = swarm.run_test(f'test {test}', mode=mode, speed_mps=.4, seed=42,
                                         output_dir=output / 'swarm')
                    print('PASS swarm', mode, test, path, flush=True)
        finally:
            try:
                requested_at = time.monotonic()
                try:
                    swarm.stop()
                finally:
                    swarm.land()
                wait_until(c, lambda: len(landed) == args.n and all(
                    stamp > requested_at and time.monotonic()-stamp < 2 and on_ground
                    for stamp,on_ground in landed.values()), 'all drones landed', timeout=90)
                swarm.disarm()
            finally:
                if observer is not None:
                    observer.destroy_node()


if __name__ == '__main__':
    main()
