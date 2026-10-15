#!/usr/bin/env python3
# Interactive, terminal-friendly controls for a running DroneNode

import argparse
import math
import shlex
import sys

import rclpy

from .commands import DroneCommandClient


HELP = """Commands:
  startup | arm | takeoff | land | stop | disarm
  forward <m> [position|velocity] [speed_mps]
  backward <m> [position|velocity] [speed_mps]
  left <m> [position|velocity] [speed_mps]
  right <m> [position|velocity] [speed_mps]
  up <m> [position|velocity] [speed_mps]
  down <m> [position|velocity] [speed_mps]
  position <north_m> <east_m> <down_m>
  velocity <north_mps> <east_mps> <down_mps>
  test <1|2|3>
  help | quit

Position control is the default. Directions use local NED: forward is North,
right is East, and up is negative Down.
"""


def _parse_args(args):
    parser = argparse.ArgumentParser(description="Interactive PX4 control console.")
    parser.add_argument("--drone", type=int, default=0)
    return parser.parse_args(args)


def _movement(client: DroneCommandClient, words: list[str]) -> str:
    if len(words) not in (2, 3, 4):
        raise ValueError("Use: direction <metres> [position|velocity] [speed_mps]")
    distance = float(words[1])
    if not math.isfinite(distance) or distance <= 0.0:
        raise ValueError("Distance must be positive.")
    mode = words[2] if len(words) >= 3 else "position"
    speed = float(words[3]) if len(words) == 4 else 0.5
    if mode not in ("position", "velocity"):
        raise ValueError("Control mode must be position or velocity.")
    if not math.isfinite(speed) or speed <= 0.0:
        raise ValueError("Speed must be positive.")
    getattr(client, f"move_{words[0]}")(distance, mode=mode, speed_mps=speed)
    return f"{words[0]} complete."


def main(args=None) -> None:
    parsed = _parse_args(sys.argv[1:] if args is None else args)
    rclpy.init(args=args)
    node = rclpy.create_node("drone_control_console")
    client = DroneCommandClient(node, f"/drone_{parsed.drone}")
    movement = {"forward", "backward", "left", "right", "up", "down"}

    print(f"Connected to /drone_{parsed.drone}. Type 'help' for commands.")
    try:
        while rclpy.ok():
            try:
                words = shlex.split(input("drone> "))
                if not words:
                    continue
                command = words[0].lower()
                if command in ("quit", "exit"):
                    return
                if command == "help":
                    print(HELP)
                elif command == "test":
                    print(f"CSV: {client.run_test(' '.join(words))}")
                elif command in ("position", "velocity"):
                    if len(words) != 4:
                        raise ValueError("Use: position|velocity <north> <east> <down>")
                    target = [float(value) for value in words[1:]]
                    method = client.move_to if command == "position" else client.set_velocity
                    method(*target)
                    print(f"{command} target set.")
                elif command in movement:
                    print(_movement(client, words))
                elif command in ("startup", "arm", "disarm", "takeoff", "land", "stop"):
                    if len(words) != 1:
                        raise ValueError(f"{command} does not take arguments.")
                    print(getattr(client, command)())
                else:
                    print("Unknown command. Type 'help' for commands.")
            except (RuntimeError, TimeoutError, ValueError) as exc:
                print(f"Command failed: {exc}")
    except (EOFError, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
