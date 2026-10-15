# Service ROS callbacks for a bounded control period, including busy swarms
import time

import rclpy


def spin_for(node, duration_s=.05):
    # spin_once handles only one callback. Returning after that callback can
    # starve state subscriptions behind status/command traffic on a busy node
    deadline = time.monotonic() + duration_s
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return
        rclpy.spin_once(node, executor=getattr(node, "_command_executor", None), timeout_sec=min(.005, remaining))
