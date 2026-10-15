#!/usr/bin/env python3
# Interactive or single entry point for the three swarm benchmarks
import argparse
import sys

import rclpy
from rclpy.utilities import remove_ros_args

from .swarm_controller import SwarmController


def main(args=None):
    parser = argparse.ArgumentParser(description='Interactive or one-shot entry point for the three swarm benchmarks')
    parser.add_argument('--n', type=int, default=3)
    parser.add_argument('--test', choices=('1','2','3','all'))
    parser.add_argument('--seed', type=int)
    parser.add_argument('--output-dir', default='results/swarm_tests')
    parser.add_argument('--spacing', type=float, default=1.5)
    parser.add_argument('--direction', choices=('left','right'))
    parser.add_argument('--wait-for-signal', action='store_true')
    parsed = parser.parse_args(remove_ros_args(sys.argv if args is None else args)[1:])
    rclpy.init(args=args)
    controller = SwarmController(parsed.n)
    options = dict(output_dir=parsed.output_dir, seed=parsed.seed,
                   spacing_m=parsed.spacing, flag_direction=parsed.direction,
                   signal=(lambda: input('Setup reached. Press Enter to start timing: '))
                           if parsed.wait_for_signal else None)
    try:
        if parsed.test:
            tests = (1,2,3) if parsed.test == 'all' else (int(parsed.test),)
            for number in tests:
                print(controller.run_test(f'test {number}', **options))
        else:
            print('Swarm tests: 1 = translation MSE; 2 = random → line timing; '
                  '3 = partition → translation MSE.\n'
                  'Lifecycle: startup, arm, takeoff, land, stop. q = quit.')
            while rclpy.ok():
                command = input('swarm> ').strip().lower()
                if command in ('q','quit','exit'):
                    break
                try:
                    if command in ('startup','arm','takeoff','land','stop'):
                        print(controller.command_all(command))
                    else:
                        text = f'test {command}' if command in ('1','2','3') else command
                        print(controller.run_test(text, **options))
                except (ValueError, RuntimeError, TimeoutError) as exc:
                    print(f'Command failed: {exc}')
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
