#!/usr/bin/env python3
# Wait for usable estimator data and a GCS heartbeat from each local SITL
import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger


def interfaces_ready(node, count, timeout):
    # Require live state publishers and command services for every member
    states = {}
    clients = {}
    for i in range(count):
        node.create_subscription(
            Odometry, f'/drone_{i}/state',
            lambda message, i=i: states.update({i: time.monotonic()}), 10)
        clients[i] = [node.create_client(Trigger, f'/drone_{i}/{command}')
                      for command in ('startup', 'arm', 'takeoff', 'land', 'stop')]

    def ready(i):
        return (i in states and time.monotonic() - states[i] < 2.0
                and all(client.service_is_ready() for client in clients[i]))

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if all(ready(i) for i in range(count)):
            print(f'All {count} drone interfaces have live state and command services.', flush=True)
            return
    missing = [i for i in range(count) if not ready(i)]
    raise SystemExit(f'Drone interfaces unavailable for drones {missing}; inspect ROS launch log ros.log (or /tmp/drone_swarm_ros_nodes.log).')


def main():
    parser = argparse.ArgumentParser(description='Wait for usable estimator data and a GCS heartbeat from each local SITL')
    parser.add_argument('count', type=int)
    parser.add_argument('--timeout', type=float, default=60.0)
    parser.add_argument('--interfaces', action='store_true')
    args = parser.parse_args()
    if args.count < 1:
        parser.error('count must be positive')
    rclpy.init()
    node = Node('sitl_readiness_check')
    if args.interfaces:
        try:
            interfaces_ready(node, args.count, args.timeout)
        finally:
            node.destroy_node()
            rclpy.shutdown()
        return
    positions, statuses = {}, {}

    def receive(table, i, message):
        table[i] = (time.monotonic(), message)

    for i in range(args.count):
        prefix = '' if i == 0 else f'/px4_{i}'
        node.create_subscription(
            VehicleLocalPosition, prefix+'/fmu/out/vehicle_local_position_v1',
            lambda m,i=i: receive(positions,i,m), qos_profile_sensor_data)
        node.create_subscription(
            VehicleStatus, prefix+'/fmu/out/vehicle_status_v1',
            lambda m,i=i: receive(statuses,i,m), qos_profile_sensor_data)

    def ready(i):
        if i not in positions or i not in statuses:
            return False
        tp, p = positions[i]
        ts, s = statuses[i]
        return (time.monotonic()-min(tp,ts) < 2.0 and p.xy_valid and p.z_valid
                and p.v_xy_valid and p.v_z_valid and p.heading_good_for_control
                and not s.gcs_connection_lost)

    deadline = time.monotonic()+args.timeout
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if all(ready(i) for i in range(args.count)):
                print(f'All {args.count} SITL estimators valid; all GCS links connected.', flush=True)
                return
        missing = [i for i in range(args.count) if not ready(i)]
        raise SystemExit(f'SITL readiness timed out for drones {missing}; inspect PX4/MAVProxy logs.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
