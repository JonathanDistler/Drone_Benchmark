#!/usr/bin/env python3
# Fleet commands and shared local NED state

import argparse
import math
from dataclasses import dataclass
import sys
import time
from typing import Callable

import rclpy
from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from .commands import DroneCommandClient


@dataclass(frozen=True)
class DroneSnapshot:
    # A drone's latest position and flight mode, in local NED coordinates

    drone_id: int
    state: Odometry | None
    armed: bool | None
    offboard: bool | None


class SwarmController(Node):
    # One controller that coordinates any number of independent DroneNodes

    def __init__(self, drone_count: int, *, namespace: str | None = None,
                 use_global_arguments: bool = True):
        if drone_count < 1:
            raise ValueError("drone_count must be at least one.")
        super().__init__("swarm_controller", namespace=namespace,
                         use_global_arguments=use_global_arguments)
        self.drone_count = drone_count
        self.states: dict[int, Odometry] = {}
        self.armed: dict[int, bool] = {}
        self.offboard: dict[int, bool] = {}
        self.state_listeners: list[Callable[[dict[int, DroneSnapshot]], None]] = []
        self.drones = [
            DroneCommandClient(self, f"/drone_{drone_id}")
            for drone_id in range(drone_count)
        ]

        for drone_id in range(drone_count):
            prefix = f"/drone_{drone_id}"
            self.create_subscription(
                Odometry, f"{prefix}/state",
                lambda message, i=drone_id: self._state_callback(i, message), 10,
            )
            self.create_subscription(
                Bool, f"{prefix}/armed",
                lambda message, i=drone_id: self._armed_callback(i, message), 10,
            )
            self.create_subscription(
                Bool, f"{prefix}/offboard",
                lambda message, i=drone_id: self._offboard_callback(i, message), 10,
            )

        # Ordered by drone ID. Other nodes can subscribe once instead of wiring a
        # separate subscription for every vehicle
        self.positions_pub = self.create_publisher(
            PoseArray, "swarm/positions_ned", 10
        )
        self.create_timer(0.1, self._publish_positions)
        self.get_logger().info(f"Managing {drone_count} drone nodes.")

    def _state_callback(self, drone_id: int, message: Odometry) -> None:
        self.states[drone_id] = message
        self._notify_state_listeners()

    def _armed_callback(self, drone_id: int, message: Bool) -> None:
        self.armed[drone_id] = message.data
        self._notify_state_listeners()

    def _offboard_callback(self, drone_id: int, message: Bool) -> None:
        self.offboard[drone_id] = message.data
        self._notify_state_listeners()

    def snapshot(self, drone_id: int) -> DroneSnapshot:
        # Return the latest data received from one member
        if drone_id < 0 or drone_id >= self.drone_count:
            raise IndexError(f"Unknown drone ID {drone_id}.")
        return DroneSnapshot(
            drone_id, self.states.get(drone_id), self.armed.get(drone_id),
            self.offboard.get(drone_id),
        )

    def snapshots(self) -> dict[int, DroneSnapshot]:
        return {drone_id: self.snapshot(drone_id) for drone_id in range(self.drone_count)}

    def add_state_listener(
        self, listener: Callable[[dict[int, DroneSnapshot]], None]
    ) -> None:
        # Register a planner or RL observer without exposing PX4 topics
        self.state_listeners.append(listener)

    def all_ready(self) -> bool:
        # True when every member has an position data and is in OFFBOARD
        return all(
            self.snapshot(drone_id).state is not None
            and self.snapshot(drone_id).armed is True
            and self.snapshot(drone_id).offboard is True
            for drone_id in range(self.drone_count)
        )

    def takeoff_all(self, timeout_s: float = 45.0) -> dict[int, str]:
        # Send takeoff together and wait for every drone to settle
        deadline = time.monotonic() + timeout_s
        # Service discovery alone is not readiness: DroneNode advertises its service
        # during createion, before the first PX4 position sample
        while time.monotonic() < deadline and not all(
            drone_id in self.states for drone_id in range(self.drone_count)
        ):
            rclpy.spin_once(self, executor=getattr(self, "_command_executor", None), timeout_sec=0.05)
        missing = [i for i in range(self.drone_count) if i not in self.states]
        if missing:
            raise TimeoutError(
                "No estimator state received from drone(s): "
                + ", ".join(map(str, missing))
            )
        targets = {
            i: drone.takeoff_target_down(timeout_s=max(0.01, deadline - time.monotonic()))
            for i, drone in enumerate(self.drones)
        }
        # Discover every service before releasing any member. Discovery can block
        for drone_id, drone in enumerate(self.drones):
            remaining = deadline - time.monotonic()
            if remaining <= 0.0 or not drone.takeoff_client.wait_for_service(
                timeout_sec=remaining
            ):
                raise TimeoutError(
                    f"Takeoff service for drone {drone_id} is unavailable."
                )
        # An advertised service can still reject an unarmed member. Hold the
        # whole swarm until every member reports armed OFFBOARD readiness
        while not all(d.armed is True and d.offboard is True for d in self.drones):
            if time.monotonic() >= deadline:
                pending = [str(i) for i, d in enumerate(self.drones)
                           if d.armed is not True or d.offboard is not True]
                raise TimeoutError(
                    "Takeoff requires armed OFFBOARD readiness for drone(s): "
                    + ", ".join(pending))
            rclpy.spin_once(self, executor=getattr(self, "_command_executor", None), timeout_sec=0.02)
        requested_at = time.monotonic()
        futures = {
            i: drone.takeoff_client.call_async(Trigger.Request())
            for i, drone in enumerate(self.drones)
        }

        while time.monotonic() < deadline and not all(
            future.done() for future in futures.values()
        ):
            rclpy.spin_once(self, executor=getattr(self, "_command_executor", None), timeout_sec=0.05)
        if not all(future.done() for future in futures.values()):
            raise TimeoutError("Timed out queuing the coordinated takeoff.")

        responses = {}
        for drone_id, future in futures.items():
            response = future.result()
            if response is None or not response.success:
                detail = "no response" if response is None else response.message
                raise RuntimeError(f"Drone {drone_id} rejected takeoff: {detail}")
            responses[drone_id] = response.message

        while time.monotonic() < deadline:
            if self.all_ready() and all(
                drone.takeoff_reached(targets[i], requested_at)
                for i, drone in enumerate(self.drones)
            ):
                print("Swarm takeoff complete; movement commands are available.", flush=True)
                return responses
            rclpy.spin_once(self, executor=getattr(self, "_command_executor", None), timeout_sec=0.05)
        pending = [
            str(i) for i, drone in enumerate(self.drones)
            if not drone.takeoff_reached(targets[i], requested_at)
        ]
        diagnostics = []
        for drone_id in pending:
            i = int(drone_id)
            drone = self.drones[i]
            if drone.state is None:
                telemetry = "no state"
            else:
                p = drone.state.pose.pose.position
                v = drone.state.twist.twist.linear
                telemetry = (
                    f"down={p.z:.2f}m target={targets[i]:.2f}m "
                    f"speed={math.hypot(v.x, v.y, v.z):.2f}m/s "
                    f"state_age={time.monotonic() - drone.state_received_at:.2f}s"
                )
            diagnostics.append(
                f"drone {i}: armed={drone.armed} offboard={drone.offboard} {telemetry}"
            )
        raise TimeoutError(
            "Takeoff altitude not reached and settled for drone(s): " + ", ".join(pending)
            + "\n" + "\n".join(diagnostics)
        )

    def move_all(
        self, direction: str, distance_m: float = 1.0, *,
        mode: str = "position", speed_mps: float = 1.0,
        timeout_s: float = 45.0,
    ) -> dict[int, object]:
        # Send relative targets together and wait for the entire swarm
        axes = {
            "forward": (1, 0, 0), "backward": (-1, 0, 0),
            "right": (0, 1, 0), "left": (0, -1, 0),
            "up": (0, 0, -1), "down": (0, 0, 1),
        }
        if mode not in ("position", "velocity"):
            raise ValueError("mode must be 'position' or 'velocity'.")
        if not math.isfinite(distance_m) or not math.isfinite(speed_mps) or speed_mps <= 0:
            raise ValueError("Distance must be finite and speed must be finite and positive.")
        offset = tuple(abs(distance_m) * value for value in axes[direction])
        deadline = time.monotonic() + timeout_s
        requested_at = time.monotonic()
        # Wait for command subscribers before sending single targets
        def ready(drone):
            return (drone.state is not None and drone.armed is True
                    and drone.offboard is True
                    and drone.state_received_at > requested_at
                    and time.monotonic() - drone.state_received_at < 1.0
                    and drone.position_pub.get_subscription_count() > 0
                    and (mode == "position" or drone.velocity_pub.get_subscription_count() > 0)
                    and math.hypot(drone.state.twist.twist.linear.x,
                                   drone.state.twist.twist.linear.y,
                                   drone.state.twist.twist.linear.z) < 0.25)

        ready_since = None
        while True:
            if all(ready(drone) for drone in self.drones):
                if ready_since is None:
                    ready_since = time.monotonic()
                elif time.monotonic() - ready_since >= 0.5:
                    break
            else:
                ready_since = None
            if time.monotonic() >= deadline:
                pending = [str(i) for i, drone in enumerate(self.drones) if not ready(drone)]
                raise TimeoutError("Movement readiness failed for drone(s): " + ", ".join(pending))
            rclpy.spin_once(self, executor=getattr(self, "_command_executor", None), timeout_sec=0.02)
        starts = [(d.state.pose.pose.position.x, d.state.pose.pose.position.y,
                   d.state.pose.pose.position.z) for d in self.drones]
        targets = [tuple(p + delta for p, delta in zip(start, offset)) for start in starts]
        sent_at = time.monotonic()
        settled_since = None
        pending = list(range(self.drone_count))
        velocity_done = set()
        try:
            while time.monotonic() < deadline:
                pending = []
                for i, drone in enumerate(self.drones):
                    p = drone.state.pose.pose.position
                    v = drone.state.twist.twist.linear
                    fresh = (drone.state_received_at > sent_at
                             and time.monotonic() - drone.state_received_at < 1.0)
                    if drone.armed is not True or drone.offboard is not True:
                        raise RuntimeError(f"Drone {i} lost armed OFFBOARD state during movement.")
                    if mode == "velocity" and i not in velocity_done:
                        progress = sum((value - origin) * axis for value, origin, axis
                                       in zip((p.x, p.y, p.z), starts[i], axes[direction]))
                        if fresh and progress >= abs(distance_m):
                            velocity_done.add(i)
                        else:
                            drone._publish_velocity(*(speed_mps * a for a in axes[direction]))
                    if mode == "position" or i in velocity_done:
                        drone._publish_position(*targets[i])
                    error = math.dist((p.x, p.y, p.z), targets[i])
                    if (not fresh or error >= min(0.10, max(0.01, abs(distance_m) * 0.2))
                            or math.hypot(v.x, v.y, v.z) >= 0.15
                            or (mode == "velocity" and i not in velocity_done)):
                        pending.append(i)
                if pending:
                    settled_since = None
                elif settled_since is None:
                    settled_since = time.monotonic()
                elif time.monotonic() - settled_since >= 0.5:
                    return {i: None for i in range(self.drone_count)}
                rclpy.spin_once(self, executor=getattr(self, "_command_executor", None), timeout_sec=0.02)
            raise TimeoutError("Movement did not settle for drone(s): " + ", ".join(map(str, pending)))
        finally:
            # Never leave a continuous velocity target running after an error
            if mode == "velocity":
                for i, drone in enumerate(self.drones):
                    if i not in velocity_done:
                        drone._publish_velocity(0.0, 0.0, 0.0)

    def _services_all(self, command: str, timeout_s: float = 10.0):
        # Discover the whole swarm, send all requests, then await results
        aliases = {"hold_position": "stop", "set_position_control": "position_mode",
                   "set_velocity_control": "velocity_mode"}
        clients = [getattr(d, aliases.get(command, command) + "_client") for d in self.drones]
        deadline = time.monotonic() + timeout_s
        if command == "startup":
            while not all(d.state is not None for d in self.drones):
                if time.monotonic() >= deadline:
                    raise TimeoutError("Swarm startup requires telemetry from every drone.")
                rclpy.spin_once(self, executor=getattr(self, "_command_executor", None), timeout_sec=0.02)
        for i, client in enumerate(clients):
            remaining = deadline - time.monotonic()
            if remaining <= 0 or not client.wait_for_service(timeout_sec=remaining):
                raise TimeoutError(f"Drone {i} {command} service unavailable.")
        futures = [client.call_async(Trigger.Request()) for client in clients]
        while not all(f.done() for f in futures):
            if time.monotonic() >= deadline:
                raise TimeoutError(f"Timed out dispatching swarm {command}.")
            rclpy.spin_once(self, executor=getattr(self, "_command_executor", None), timeout_sec=0.02)
        results = {}
        errors = []
        for i, future in enumerate(futures):
            response = future.result()
            if response is None or not response.success:
                errors.append(f"Drone {i}: {getattr(response, 'message', 'no response')}")
            else:
                results[i] = response.message
        if errors:
            raise RuntimeError("; ".join(errors))
        if command in ("arm", "disarm", "startup"):
            def confirmed(d):
                return d.started if command == "startup" else d.armed is (command == "arm")
            while not all(confirmed(d) for d in self.drones):
                if time.monotonic() >= deadline:
                    pending = [str(i) for i, d in enumerate(self.drones) if not confirmed(d)]
                    raise TimeoutError(f"Swarm {command} not confirmed for drone(s): " + ", ".join(pending))
                rclpy.spin_once(self, executor=getattr(self, "_command_executor", None), timeout_sec=0.02)
        return results

    def _targets_all(self, command, *args, timeout_s=10.0):
        # Prepare every target before releasing any member
        if command in ("move_to", "set_velocity"):
            DroneCommandClient._validate_vector(*args)
            if len(args) != 3:
                raise ValueError("Provide north, east, and down.")
        elif len(args) != 1 or not math.isfinite(args[0]):
            raise ValueError("Provide one finite yaw in radians.")
        publisher = {"move_to": "position_pub", "set_velocity": "velocity_pub",
                     "set_yaw": "yaw_pub", "yaw_by": "yaw_pub"}[command]
        deadline = time.monotonic() + timeout_s
        requested_at = time.monotonic()
        while not all(d.stop_client.service_is_ready()
                      and getattr(d, publisher).get_subscription_count() > 0
                      and (command != "yaw_by" or
                           (d.state is not None and d.state_received_at > requested_at))
                      for d in self.drones):
            if time.monotonic() >= deadline:
                raise TimeoutError(f"Swarm {command} readiness failed.")
            rclpy.spin_once(self, executor=getattr(self, "_command_executor", None), timeout_sec=0.02)
        targets = [args for d in self.drones]
        if command == "yaw_by":
            targets = []
            for d in self.drones:
                q = d.state.pose.pose.orientation
                yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
                targets.append((yaw + args[0],))
            command = "set_yaw"
        results = {i: getattr(d, command)(*targets[i]) for i, d in enumerate(self.drones)}
        # Service DDS before a one-shot CLI destroys its publishers.
        from .spinning import spin_for
        spin_for(self, 0.2)
        return results

    def startup_all(self) -> dict[int, object]:
        # Start each member without arming it
        return self.command_all("startup")

    def arm_all(self) -> dict[int, object]:
        # Explicitly arm each member and confirm its reported state
        return self.command_all("arm")

    def land_all(self) -> None:
        self.command_all("land")

    def command(self, drone_id: int, command: str, *args, **kwargs):
        # Send a command to one drone, using IDs that start at zero
        if drone_id < 0 or drone_id >= self.drone_count:
            raise IndexError(f"Unknown drone ID {drone_id}.")
        return getattr(self.drones[drone_id], command)(*args, **kwargs)

    def command_all(self, command: str, *args, **kwargs) -> dict[int, object]:
        # Coordinate takeoff and relative movement; route other commands by ID
        if command == "takeoff" and not args and not kwargs:
            return self.takeoff_all()
        if command in {"move_forward", "move_backward", "move_left", "move_right", "move_up", "move_down"}:
            return self.move_all(command.removeprefix("move_"), *args, **kwargs)
        if command in {"startup", "arm", "disarm", "land", "stop", "hold_position",
                       "set_position_control", "set_velocity_control"}:
            return self._services_all(command, *args, **kwargs)
        if command in {"move_to", "set_velocity", "set_yaw", "yaw_by"}:
            return self._targets_all(command, *args, **kwargs)
        return {
            drone_id: self.command(drone_id, command, *args, **kwargs)
            for drone_id in range(self.drone_count)
        }

    def publish_topic(self, topic: str, message_type: str, fields: dict,
                      drone_ids=None, timeout_s: float = 10.0):
        # Send a typed ROS message to each selected drone, including the leader
        from rosidl_runtime_py.utilities import get_message
        from rosidl_runtime_py.set_message import set_message_fields
        ids = list(range(self.drone_count)) if drone_ids is None else list(drone_ids)
        if not topic or topic.startswith("/") or ".." in topic.split("/"):
            raise ValueError("Use a relative topic inside each drone namespace")
        for drone_id in ids:
            self.snapshot(drone_id)
        message_class = get_message(message_type)
        message = message_class()
        set_message_fields(message, fields)
        if not hasattr(self, "_topic_publishers"):
            self._topic_publishers = {}
        publishers = []
        for drone_id in ids:
            key = (drone_id, topic, message_type)
            if key not in self._topic_publishers:
                self._topic_publishers[key] = self.create_publisher(
                    message_class, f"/drone_{drone_id}/{topic}", 10
                )
            publishers.append(self._topic_publishers[key])
        deadline = time.monotonic() + timeout_s
        while not all(pub.get_subscription_count() > 0 for pub in publishers):
            if time.monotonic() >= deadline:
                raise TimeoutError("Topic subscribers are unavailable")
            rclpy.spin_once(self, executor=getattr(self, "_command_executor", None), timeout_sec=0.05)
        for publisher in publishers:
            publisher.publish(message)
        return ids

    def behaviors(self):
        from .swarm_behaviors import SwarmBehaviors
        if not hasattr(self, "_behaviors"):
            self._behaviors = SwarmBehaviors(self)
        return self._behaviors

    def run_test(self, command, **kwargs):
        from .swarm_benchmarks import run_test
        return run_test(self, command, **kwargs)

    def formation(self, shape="line", spacing_m=1.5, **kwargs):
        return self.behaviors().formation(shape, spacing_m, **kwargs)

    def go_to(self, targets, **kwargs):
        return self.behaviors().go_to(targets, **kwargs)

    def translate(self, north=0., east=0., down=0., **kwargs):
        return self.behaviors().translate(north, east, down, **kwargs)

    def randomize(self, radius_m=3., **kwargs):
        return self.behaviors().randomize(radius_m, **kwargs)

    def split(self, direction="left", spacing_m=1.5, **kwargs):
        return self.behaviors().split(direction, spacing_m, **kwargs)

    def centroid(self):
        return self.behaviors().centroid()

    def _notify_state_listeners(self) -> None:
        if not self.state_listeners:
            return
        snapshots = self.snapshots()
        for listener in self.state_listeners:
            listener(snapshots)

    def _publish_positions(self) -> None:
        message = PoseArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "local_ned"
        for drone_id in range(self.drone_count):
            state = self.states.get(drone_id)
            pose = Pose()
            if state is not None:
                pose.position = state.pose.pose.position
                pose.orientation = state.pose.pose.orientation
            message.poses.append(pose)
        self.positions_pub.publish(message)


def main(args=None) -> None:
    parser = argparse.ArgumentParser(description="Run the shared swarm controller.")
    parser.add_argument("--n", type=int, default=1, help="Number of drone nodes.")
    raw_args = sys.argv if args is None else args
    parsed = parser.parse_args(remove_ros_args(raw_args)[1:])
    rclpy.init(args=args)
    node = SwarmController(parsed.n)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
