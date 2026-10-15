#!/usr/bin/env python3
# Start, arm, and optionally flight-test the local ROS 2 SITL swarm

import argparse
import math
import time


COMMANDS = {
    "startup()": "Start hold streaming; wait for startup readiness.",
    "arm()": "Arm and verify armed telemetry for every member.",
    "takeoff()": "Wait for every member to reach its configured altitude and settle in OFFBOARD.",
    "land()": "Request landing; this call does not wait for touchdown.",
    "disarm()": "Disarm and verify telemetry after touchdown.",
    "stop() / hold_position()": "Hold each drone's current position.",
    "forward(d) / backward(d)": "Move North/South by d metres.",
    "right(d) / left(d)": "Move East/West by d metres.",
    "up(d) / down(d)": "Rise/descend by d metres.",
    "height_rise(d) / height_drop(d)": "Aliases for up/down.",
    "set_yaw(rad) / yaw(rad)": "Set an absolute NED yaw angle.",
    "yaw_by(rad)": "Turn relative to reported orientation; see the topic reference limitation.",
    "drone(i).move_to(n, e, d)": "Set one member's absolute local NED position.",
    "drone(i).set_velocity(vn, ve, vd)": "Set persistent NED velocity in m/s; stop explicitly.",
    "drone(i).set_position_control()": "Select position control.",
    "drone(i).set_velocity_control()": "Select velocity control.",
    "drone(i).wait_for_state(timeout_s=10)": "Wait for an estimator sample.",
    "drone(i).wait_for_armed(True)": "Wait for reported arming state.",
    "drone(i).wait_for_offboard(True)": "Wait for reported OFFBOARD state.",
}


def wait_until(node, predicate, description, timeout=45.0):
    # Keep receiving flight data until a condition succeeds or times out
    import rclpy

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            print(f"PASS: {description}", flush=True)
            return
    raise TimeoutError(description)


def position(drone):
    p = drone.state.pose.pose.position
    return (p.x, p.y, p.z)


def wait_for_targets(swarm, targets):
    # Check position error and speed for every member
    def reached():
        for i, target in enumerate(targets):
            drone = swarm.drone(i)
            if drone.state is None:
                return False
            stamp = drone.state.header.stamp
            age = (swarm.controller.get_clock().now().nanoseconds - stamp.sec * 10**9 - stamp.nanosec) / 1e9
            velocity = drone.state.twist.twist.linear
            speed = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
            if not (0 <= age < 2.0 and math.dist(position(drone), target) < 0.20 and speed < 0.25):
                return False
        return True

    wait_until(swarm.controller, reached, "all drones reached their position targets")


def run(count, fly, height):
    from rclpy.qos import qos_profile_sensor_data
    from px4_msgs.msg import VehicleLandDetected
    from swarm_mavsdk_ros2 import Swarm

    with Swarm(count) as swarm:
        landed = {}

        def receive_landed(i, message):
            landed[i] = (time.monotonic(), message.landed)

        for i in range(count):
            prefix = "" if i == 0 else f"/px4_{i}"
            swarm.controller.create_subscription(
                VehicleLandDetected, f"{prefix}/fmu/out/vehicle_land_detected",
                lambda message, i=i: receive_landed(i, message), qos_profile_sensor_data,
            )

        try:
            print("Starting every drone...", flush=True)
            swarm.startup()
            print("Arming every drone...", flush=True)
            swarm.arm()
            print(f"PASS: all {count} drones report armed", flush=True)
            if fly:
                targets = [(position(swarm.drone(i))[0], position(swarm.drone(i))[1], -height) for i in range(count)]
                swarm.takeoff()
                wait_for_targets(swarm, targets)
                steps = [
                    ("forward", (0.5, 0., 0.)),
                    ("backward", (-0.5, 0., 0.)),
                    ("right", (0., 0.5, 0.)),
                    ("left", (0., -0.5, 0.)),
                    ("up", (0., 0., -0.5)),
                    ("down", (0., 0., 0.5)),
                ]
                for name, offset in steps:
                    targets = [tuple(a + b for a, b in zip(position(swarm.drone(i)), offset)) for i in range(count)]
                    print(f"Testing {name}(0.5)...", flush=True)
                    getattr(swarm, name)(0.5)
                    wait_for_targets(swarm, targets)
                swarm.stop()
                swarm.hold_position()
                print("PASS: stop and hold services accepted", flush=True)
        finally:
            # Attempt cleanup for every member even if another member rejects a command
            failures = []
            for i in range(count):
                try:
                    drone = swarm.drone(i)
                    drone.land()
                    requested_at = time.monotonic()
                    wait_until(
                        swarm.controller,
                        lambda i=i, start=requested_at: i in landed and landed[i][0] > start and time.monotonic() - landed[i][0] < 2.0 and landed[i][1],
                        f"drone {i} reports touchdown",
                    )
                    drone.disarm()
                except Exception as exc:
                    failures.append(f"drone {i}: {exc}")
            if failures:
                raise RuntimeError("Cleanup incomplete: " + "; ".join(failures))
        print("PASS: all drones landed and disarmed", flush=True)


def main():
    parser = argparse.ArgumentParser(description='Start, arm, and optionally flight-test the local ROS 2 SITL swarm')
    parser.add_argument("--n", type=int, default=3, help="Number of launched drones.")
    parser.add_argument("--fly", action="store_true", help="Also take off and test six 0.5 m position moves.")
    parser.add_argument("--height", type=float, default=2.0, help="Expected takeoff height; must match the ROS launch configuration.")
    parser.add_argument("--list-commands", action="store_true", help="Print the command catalog without connecting to ROS.")
    args = parser.parse_args()
    if args.n < 1 or not math.isfinite(args.height) or args.height <= 0:
        parser.error("--n and --height must be positive and finite")
    if args.list_commands:
        for command, description in COMMANDS.items():
            print(f"{command}: {description}")
        print("Directional calls accept mode='position' or mode='velocity', speed_mps=0.5.")
        print("Individual clients use move_forward/move_backward/move_left/move_right/move_up/move_down.")
        return
    run(args.n, args.fly, args.height)


if __name__ == "__main__":
    main()
