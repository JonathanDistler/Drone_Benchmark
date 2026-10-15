#!/usr/bin/env python3
# One-shot, shared commands for every DroneNode in a running swarm

import argparse
import math
import sys

import rclpy
from rclpy.utilities import remove_ros_args

from .swarm_controller import SwarmController


MOVEMENT_COMMANDS = {"forward", "backward", "left", "right", "up", "down"}
COMMANDS = ("test", "formation", "randomize", "split", "translate", "yaw", "yaw_by", "startup", "arm", "disarm", "takeoff", "land", "stop", "position", "velocity", *sorted(MOVEMENT_COMMANDS))


def _arguments(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Send the same command to every zero-indexed drone in a swarm."
    )
    parser.add_argument("--architecture", default="central")
    parser.add_argument("command", choices=COMMANDS)
    parser.add_argument(
        "distance", nargs="?", type=float,
        help="Distance in metres (required for movement commands).",
    )
    parser.add_argument("--n", type=int, required=True, help="Number of drones.")
    parser.add_argument(
        "--mode", choices=("position", "velocity"), default="position",
        help="Position target (default) or velocity-controlled movement.",
    )
    parser.add_argument(
        "--speed", type=float, default=1.0,
        help="Velocity-mode speed in metres per second (default: 1.0).",
    )
    parser.add_argument("--ned", nargs=3, type=float, metavar=("NORTH", "EAST", "DOWN"),
                        help="Local NED position/velocity; translate uses a common-frame NED offset (m).")
    parser.add_argument("--shape", choices=("line", "grid", "circle"), default="line")
    parser.add_argument("--spacing", type=float, default=1.5)
    parser.add_argument("--direction", choices=("left", "right"))
    parser.add_argument("--seed", type=int)
    parser.add_argument("--output-dir", default="results/swarm_tests")
    parser.add_argument("--radians", type=float)
    return parser.parse_args(argv)


def main(args=None) -> None:
    argv = remove_ros_args(args=sys.argv if args is None else args)[1:]
    parsed = _arguments(argv)
    if parsed.n < 1:
        raise SystemExit("--n must be at least 1.")
    if parsed.command in MOVEMENT_COMMANDS and (
        parsed.distance is None or not math.isfinite(parsed.distance) or parsed.distance <= 0.0
    ):
        raise SystemExit("Movement commands require a positive distance in metres.")
    if not math.isfinite(parsed.speed) or parsed.speed <= 0.0:
        raise SystemExit("--speed must be positive.")

    if parsed.command in {"position", "velocity", "translate"} and (
        parsed.ned is None or not all(math.isfinite(v) for v in parsed.ned)
    ):
        raise SystemExit("Position and velocity commands require three finite --ned values.")

    from .architecture import leader_id
    from .edge import EdgeCommandClient
    selected = leader_id(parsed.architecture, parsed.n)
    rclpy.init(args=args)
    controller = SwarmController(parsed.n) if selected is None else EdgeCommandClient(selected)
    try:
        if parsed.command == "test":
            if parsed.distance not in (1, 2, 3):
                raise ValueError("Use test 1, test 2, or test 3.")
            print(controller.run_test(f"test {int(parsed.distance)}", seed=parsed.seed,
                spacing_m=parsed.spacing, flag_direction=parsed.direction, output_dir=parsed.output_dir,
                mode=parsed.mode, speed_mps=parsed.speed))
        elif parsed.command == "formation":
            print(controller.formation(parsed.shape, parsed.spacing, mode=parsed.mode, speed_mps=parsed.speed))
        elif parsed.command == "randomize":
            print(controller.randomize(parsed.distance if parsed.distance is not None else 3., seed=parsed.seed, mode=parsed.mode, speed_mps=parsed.speed))
        elif parsed.command == "split":
            print(controller.split(parsed.direction or "left", parsed.spacing, mode=parsed.mode, speed_mps=parsed.speed))
        elif parsed.command == "translate":
            print(controller.translate(*parsed.ned, mode=parsed.mode, speed_mps=parsed.speed))
        elif parsed.command in {"yaw", "yaw_by"}:
            if parsed.radians is None or not math.isfinite(parsed.radians):
                raise ValueError("Yaw commands require finite --radians.")
            print(controller.command_all("set_yaw" if parsed.command == "yaw" else "yaw_by", parsed.radians))
        elif parsed.command in {"position", "velocity"}:
            command = "move_to" if parsed.command == "position" else "set_velocity"
            controller.command_all(command, *parsed.ned)
            print(f"Drones 0-{parsed.n - 1}: {parsed.command} {parsed.ned}")
        elif parsed.command in MOVEMENT_COMMANDS:
            print("Waiting for fleet readiness, movement, and settling...", flush=True)
            controller.command_all(
                f"move_{parsed.command}", parsed.distance,
                mode=parsed.mode, speed_mps=parsed.speed,
            )
            print(
                f"Drones 0-{parsed.n - 1}: {parsed.command} "
                f"{parsed.distance:.2f} m ({parsed.mode} control)"
            )
        else:
            if parsed.command == "takeoff":
                print("Waiting for every drone to reach takeoff altitude...", flush=True)
            controller.command_all(parsed.command)
            print(f"Drones 0-{parsed.n - 1}: {parsed.command}")
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
