#!/usr/bin/env python3
# One-shot command-line controls for a single DroneNode. The commands use local NED
# coordinates. In the course convention, forward is North (+X), right is East (+Y), and up is
# negative Down (-Z)

import argparse
import math
import sys
import time

import rclpy
from rclpy.utilities import remove_ros_args

from .commands import DroneCommandClient


def _arguments(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Send a modular command to one PX4 ROS 2 drone."
    )
    parser.add_argument("command", choices=(
        "test", "startup", "arm", "disarm", "takeoff", "land", "stop", "position", "velocity",
        "forward", "backward", "left", "right", "up", "down",
    ))
    parser.add_argument(
        "distance",
        nargs="?",
        type=float,
        help="Distance in metres (required for movement commands).",
    )
    parser.add_argument("--drone", type=int, default=0, help="Drone ID.")
    parser.add_argument(
        "--mode",
        choices=("position", "velocity"),
        default="position",
        help="Position target (default) or velocity-controlled movement.",
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=0.5,
        help="Velocity-mode speed in metres per second (default: 0.5).",
    )
    parser.add_argument("--ned", nargs=3, type=float, metavar=("NORTH", "EAST", "DOWN"),
                        help="Local NED position (m) or velocity (m/s).")
    return parser.parse_args(argv)


def main(args=None) -> None:
    argv = remove_ros_args(args=sys.argv if args is None else args)[1:]
    parsed = _arguments(argv)
    movement_commands = {"forward", "backward", "left", "right", "up", "down"}
    if parsed.command in movement_commands and (
        parsed.distance is None or not math.isfinite(parsed.distance) or parsed.distance <= 0.0
    ):
        raise SystemExit("Movement commands require a positive distance in metres.")
    if not math.isfinite(parsed.speed) or parsed.speed <= 0.0:
        raise SystemExit("--speed must be positive.")

    if parsed.command in {"position", "velocity"} and (
        parsed.ned is None or not all(math.isfinite(v) for v in parsed.ned)
    ):
        raise SystemExit("Position and velocity commands require three finite --ned values.")

    rclpy.init(args=args)
    node = rclpy.create_node("drone_command")
    client = DroneCommandClient(node, f"/drone_{parsed.drone}")

    try:
        if parsed.command == "test":
            if parsed.distance not in (1, 2, 3):
                raise ValueError("Use test 1, test 2, or test 3.")
            print(client.run_test(f"test {int(parsed.distance)}", mode=parsed.mode, speed_mps=parsed.speed))
        elif parsed.command in {"position", "velocity"}:
            method = client.move_to if parsed.command == "position" else client.set_velocity
            publisher = client.position_pub if parsed.command == "position" else client.velocity_pub
            deadline = time.monotonic() + 10.0
            # A recorder subscription alone must not release the command.
            while not (client.stop_client.service_is_ready() and
                       publisher.get_subscription_count() > 0):
                if time.monotonic() >= deadline:
                    raise TimeoutError("Drone command interface is unavailable.")
                rclpy.spin_once(node, timeout_sec=.05)
            method(*parsed.ned)
            from .spinning import spin_for
            spin_for(node, .2)
            print(f"Drone {parsed.drone}: {parsed.command} {parsed.ned}")
        elif parsed.command in movement_commands:
            method = getattr(client, f"move_{parsed.command}")
            method(parsed.distance, mode=parsed.mode, speed_mps=parsed.speed)
            print(
                f"Drone {parsed.drone}: {parsed.command} "
                f"{parsed.distance:.2f} m ({parsed.mode} control)"
            )
        else:
            if parsed.command == "takeoff":
                print("Waiting for the drone to reach takeoff altitude...", flush=True)
            message = getattr(client, parsed.command)()
            print(f"Drone {parsed.drone}: {message}")
    except (RuntimeError, TimeoutError, ValueError) as exc:
        print(f"Drone {parsed.drone}: {exc}", file=sys.stderr)
        raise SystemExit(1) from None
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
