# Commands through the public DroneNode ROS interface

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import BatteryStatus, VehicleLocalPosition
from std_msgs.msg import Bool, Float64
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from std_srvs.srv import Trigger
from .spinning import spin_for


class DroneCommandClient:
    # Position steps default to 1 m; velocity steps use 0.5 m/s

    DEFAULT_DISTANCE_M = 1.0
    DEFAULT_SPEED_MPS = 0.5

    def __init__(self, node: Node, namespace: str) -> None:
        self.node = node
        self.namespace = namespace.rstrip("/")

        self.position_pub = node.create_publisher(
            PoseStamped,
            f"{self.namespace}/command/position_ned",
            10,
        )
        self.velocity_pub = node.create_publisher(
            TwistStamped,
            f"{self.namespace}/command/velocity_ned",
            10,
        )
        self.yaw_pub = node.create_publisher(
            Float64, f"{self.namespace}/command/yaw_rad", 10
        )

        self.takeoff_client = node.create_client(
            Trigger, f"{self.namespace}/takeoff"
        )
        self.parameters_client = node.create_client(
            GetParameters, f"{self.namespace}/drone/get_parameters"
        )
        self.startup_client = node.create_client(Trigger, f"{self.namespace}/startup")
        self.started = False
        self.started_sub = node.create_subscription(Bool, f"{self.namespace}/started", self._started_callback, 10)
        self.arm_client = node.create_client(Trigger, f"{self.namespace}/arm")
        self.disarm_client = node.create_client(
            Trigger, f"{self.namespace}/disarm"
        )
        self.land_client = node.create_client(
            Trigger, f"{self.namespace}/land"
        )
        self.stop_client = node.create_client(
            Trigger, f"{self.namespace}/stop"
        )
        self.position_mode_client = node.create_client(
            Trigger, f"{self.namespace}/set_position_mode"
        )
        self.velocity_mode_client = node.create_client(
            Trigger, f"{self.namespace}/set_velocity_mode"
        )

        self.state = None
        self.battery = None
        self.battery_received_at = float('-inf')
        self.local_telemetry = None
        self.local_telemetry_received_at = float('-inf')
        self.battery_sub = node.create_subscription(
            BatteryStatus, f'{self.namespace}/battery', self._battery_callback, 10)
        self.local_telemetry_sub = node.create_subscription(
            VehicleLocalPosition, f'{self.namespace}/local_telemetry', self._local_telemetry_callback, 10)
        self.state_received_at = float("-inf")
        self.armed = None
        self.offboard = None
        self.state_sub = node.create_subscription(
            Odometry,
            f"{self.namespace}/state",
            self._state_callback,
            10,
        )
        self.armed_sub = node.create_subscription(
            Bool, f"{self.namespace}/armed", self._armed_callback, 10
        )
        self.offboard_sub = node.create_subscription(
            Bool, f"{self.namespace}/offboard", self._offboard_callback, 10
        )

    def _started_callback(self, msg: Bool) -> None:
        self.started = msg.data

    def _battery_callback(self, msg) -> None:
        self.battery = msg
        self.battery_received_at = time.monotonic()

    def _local_telemetry_callback(self, msg) -> None:
        self.local_telemetry = msg
        self.local_telemetry_received_at = time.monotonic()

    def _state_callback(self, msg) -> None:
        self.state = msg
        self.state_received_at = time.monotonic()

    def _armed_callback(self, msg: Bool) -> None:
        self.armed = msg.data

    def _offboard_callback(self, msg: Bool) -> None:
        self.offboard = msg.data

    def _call(self, client, timeout_s: float = 5.0):
        if not client.wait_for_service(timeout_sec=timeout_s):
            raise TimeoutError(f"ROS 2 service {client.srv_name} is unavailable.")

        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + timeout_s

        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, executor=getattr(self.node, "_command_executor", None), timeout_sec=0.05)

        if not future.done():
            raise TimeoutError(f"Timed out calling {client.srv_name}.")

        response = future.result()
        if not response.success:
            raise RuntimeError(response.message)
        return response.message

    def startup(self) -> str:
        # Wait for flight data and start control streaming without arming
        self.wait_for_state()
        message = self._call(self.startup_client)
        deadline = time.monotonic() + 5.0
        while not self.started and time.monotonic() < deadline:
            rclpy.spin_once(self.node, executor=getattr(self.node, "_command_executor", None), timeout_sec=0.05)
        if not self.started:
            raise TimeoutError(f"Startup did not complete for {self.namespace}.")
        return message

    def takeoff(self) -> str:
        # Wait for the configured altitude, not just armed OFFBOARD mode
        # Wait for flight data before requesting takeoff
        self.wait_for_state()
        target_down = self.takeoff_target_down()
        requested_at = time.monotonic()
        message = self._call(self.takeoff_client)
        self.wait_for_armed(True)
        self.wait_for_offboard(True)
        deadline = time.monotonic() + 45.0
        while not self.takeoff_reached(target_down, requested_at):
            if time.monotonic() >= deadline:
                raise TimeoutError(
                    f"{self.namespace} did not reach takeoff altitude "
                    f"{-target_down:.2f} m."
                )
            rclpy.spin_once(self.node, executor=getattr(self.node, "_command_executor", None), timeout_sec=0.05)
        print(f"{self.namespace}: takeoff complete; movement commands are available.", flush=True)
        return message

    def takeoff_target_down(self, timeout_s: float = 5.0) -> float:
        # Read the vehicle's actual configuration, including launch overrides
        client = self.parameters_client
        if not client.wait_for_service(timeout_sec=timeout_s):
            raise TimeoutError(f"ROS 2 service {client.srv_name} is unavailable.")
        request = GetParameters.Request()
        request.names = ["takeoff_height"]
        future = client.call_async(request)
        deadline = time.monotonic() + timeout_s
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, executor=getattr(self.node, "_command_executor", None), timeout_sec=0.05)
        if not future.done():
            raise TimeoutError(f"Timed out reading {self.namespace} takeoff height.")
        response = future.result()
        if response is None or len(response.values) != 1:
            raise RuntimeError(f"No takeoff height returned by {self.namespace}.")
        height = response.values[0].double_value
        if not math.isfinite(height) or height <= 0:
            raise ValueError(f"Invalid takeoff height for {self.namespace}: {height}.")
        return -height

    def takeoff_reached(self, target_down: float, requested_at: float) -> bool:
        # Require fresh, settled altitude flight data after the takeoff request
        if (self.state is None or self.armed is not True or self.offboard is not True
                or self.state_received_at <= requested_at
                or time.monotonic() - self.state_received_at >= 2.0):
            return False
        p = self.state.pose.pose.position
        v = self.state.twist.twist.linear
        return (abs(p.z - target_down) < 0.20
                and math.sqrt(v.x**2 + v.y**2 + v.z**2) < 0.25)

    def arm(self) -> str:
        # Arm and confirm PX4 reports the vehicle armed
        message = self._call(self.arm_client)
        self.wait_for_armed(True)
        return message

    def disarm(self) -> str:
        message = self._call(self.disarm_client)
        self.wait_for_armed(False)
        return message

    def land(self) -> str:
        return self._call(self.land_client)

    def stop(self) -> str:
        return self._call(self.stop_client)

    def set_position_control(self) -> str:
        return self._call(self.position_mode_client)

    def set_velocity_control(self) -> str:
        return self._call(self.velocity_mode_client)

    def wait_for_state(self, timeout_s: float = 10.0) -> None:
        deadline = time.monotonic() + timeout_s
        while (self.state is None or time.monotonic() - self.state_received_at >= 1.0) and time.monotonic() < deadline:
            rclpy.spin_once(self.node, executor=getattr(self.node, "_command_executor", None), timeout_sec=0.05)

        if self.state is None or time.monotonic() - self.state_received_at >= 1.0:
            raise TimeoutError(f"No state received from {self.namespace}.")

    def wait_for_armed(self, expected: bool, timeout_s: float = 5.0) -> None:
        # Wait until PX4's reported arming state matches ``expected``
        deadline = time.monotonic() + timeout_s
        while self.armed is not expected and time.monotonic() < deadline:
            rclpy.spin_once(self.node, executor=getattr(self.node, "_command_executor", None), timeout_sec=0.05)
        if self.armed is not expected:
            state = "armed" if expected else "disarmed"
            raise TimeoutError(f"PX4 did not report {state} within {timeout_s:.1f}s.")

    def wait_for_offboard(self, expected: bool, timeout_s: float = 5.0) -> None:
        # Wait until PX4's reported navigation mode matches ``expected``
        deadline = time.monotonic() + timeout_s
        while self.offboard is not expected and time.monotonic() < deadline:
            rclpy.spin_once(self.node, executor=getattr(self.node, "_command_executor", None), timeout_sec=0.05)
        if self.offboard is not expected:
            state = "OFFBOARD" if expected else "a non-OFFBOARD mode"
            raise TimeoutError(f"PX4 did not report {state} within {timeout_s:.1f}s.")

    def wait_for_movement_ready(self, timeout_s: float = 5.0) -> None:
        # Odometry and status arrive independently in a new one-shot client.
        # Unknown status must not be treated as a confirmed disarmed vehicle.
        deadline = time.monotonic() + timeout_s
        while self.armed is None or self.offboard is None:
            if time.monotonic() >= deadline:
                raise TimeoutError("No armed/OFFBOARD status received from the drone.")
            rclpy.spin_once(self.node, executor=getattr(self.node, "_command_executor", None), timeout_sec=0.05)
        if self.armed is not True or self.offboard is not True:
            drone_id = self.namespace.rsplit('_', 1)[-1]
            raise RuntimeError(
                "Movement requires an armed drone in OFFBOARD mode. "
                f"Run drone_command startup --drone {drone_id}, then arm, then takeoff; "
                "wait for each command to complete."
            )

    def _publish_position(self, north: float, east: float, down: float) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.position.x = float(north)
        msg.pose.position.y = float(east)
        msg.pose.position.z = float(down)
        self.position_pub.publish(msg)

    def _publish_velocity(self, north: float, east: float, down: float) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.twist.linear.x = float(north)
        msg.twist.linear.y = float(east)
        msg.twist.linear.z = float(down)
        self.velocity_pub.publish(msg)

    def set_velocity(self, north: float, east: float, down: float) -> None:
        # Set a continuous velocity target in metres per second, in NED
        self._validate_vector(north, east, down)
        self._publish_velocity(north, east, down)

    def hold_position(self) -> None:
        # Hold the current local position
        self.stop()

    def move_to(
        self, north: float, east: float, down: float
    ) -> None:
        # Absolute position command in PX4 local NED
        self._validate_vector(north, east, down)
        self._publish_position(north, east, down)

    def track_position(self, north, east, down, *, mode="position", speed_mps=.5):
        # Follow a position reference through either PX4 control interface.
        self._validate_vector(north, east, down, speed_mps)
        if mode not in ("position", "velocity") or speed_mps <= 0:
            raise ValueError("Use position or velocity mode and a positive speed.")
        if mode == "position":
            return self.move_to(north, east, down)
        self.wait_for_state()
        p = self.state.pose.pose.position
        error = [north-p.x, east-p.y, down-p.z]
        scale = min(1., speed_mps / max(math.hypot(*error), 1e-9))
        self.set_velocity(*(scale*x for x in error))

    def move_to_and_wait(self, north, east, down, *, sample=None, timeout_s=60.0,
                         settle=True, mode="position", speed_mps=.5):
        # Reach a waypoint; intermediate route points retain motion
        #
        # ``settle=False`` advances within 15 cm at at most 0.6 m/s, allowing
        # PX4's position controller to brake into a turn without stopping
        # Final destinations retain the tighter, settled arrival requirement
        self._validate_vector(north, east, down)
        self.wait_for_movement_ready()
        requested = time.monotonic()
        deadline = requested + timeout_s
        while time.monotonic() < deadline:
            self.track_position(north, east, down, mode=mode, speed_mps=speed_mps)
            spin_for(self.node)
            if self.state is None or self.state_received_at <= requested:
                continue
            if time.monotonic() - self.state_received_at > 1.0:
                continue
            if self.armed is not True or self.offboard is not True:
                raise RuntimeError("Movement requires an armed drone in OFFBOARD mode.")
            p = self.state.pose.pose.position
            v = self.state.twist.twist.linear
            if sample:
                sample(self.state)
            if (math.dist((p.x, p.y, p.z), (north, east, down)) <= (0.10 if settle else 0.15)
                    and math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z) <= (0.15 if settle else 0.6)):
                if mode == "velocity" and settle:
                    self.stop()
                return
        self.stop()
        details = f"target NED=({north:.3f}, {east:.3f}, {down:.3f})"
        if self.state is not None:
            p = self.state.pose.pose.position
            v = self.state.twist.twist.linear
            details += (f"; actual NED=({p.x:.3f}, {p.y:.3f}, {p.z:.3f})"
                        f"; error={math.dist((p.x, p.y, p.z), (north, east, down)):.3f} m"
                        f"; speed={math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z):.3f} m/s"
                        f"; telemetry age={time.monotonic() - self.state_received_at:.2f} s")
        raise TimeoutError(f"Drone did not settle at its destination within {timeout_s:.1f}s: {details}.")

    def run_test(self, command, *, output_dir="results/tests", **kwargs):
        from .obstacle_tests import run_test
        return run_test(self, command, output_dir=output_dir, **kwargs)

    @staticmethod
    def _validate_vector(*values):
        if not all(math.isfinite(value) for value in values):
            raise ValueError("NED coordinates must be finite.")

    def set_yaw(self, yaw_rad: float) -> None:
        # Set an absolute local-NED yaw target, in radians
        if not math.isfinite(yaw_rad):
            raise ValueError("yaw_rad must be finite.")
        message = Float64()
        message.data = math.atan2(math.sin(yaw_rad), math.cos(yaw_rad))
        self.yaw_pub.publish(message)

    def yaw_by(self, delta_rad: float) -> None:
        # Turn relative to the latest estimated heading, in radians
        self.wait_for_state()
        q = self.state.pose.pose.orientation
        current = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.set_yaw(math.atan2(math.sin(current + delta_rad), math.cos(current + delta_rad)))

    def move_right(
        self, distance_m: float = DEFAULT_DISTANCE_M, *, mode: str = "position",
        speed_mps: float = DEFAULT_SPEED_MPS,
    ) -> None:
        # Move +East in NED, i.e. right in the course convention
        self._move_relative(0.0, abs(distance_m), 0.0, mode, speed_mps)

    def move_left(
        self, distance_m: float = DEFAULT_DISTANCE_M, *, mode: str = "position",
        speed_mps: float = DEFAULT_SPEED_MPS,
    ) -> None:
        self._move_relative(0.0, -abs(distance_m), 0.0, mode, speed_mps)

    def move_forward(
        self, distance_m: float = DEFAULT_DISTANCE_M, *, mode: str = "position",
        speed_mps: float = DEFAULT_SPEED_MPS,
    ) -> None:
        self._move_relative(abs(distance_m), 0.0, 0.0, mode, speed_mps)

    def move_backward(
        self, distance_m: float = DEFAULT_DISTANCE_M, *, mode: str = "position",
        speed_mps: float = DEFAULT_SPEED_MPS,
    ) -> None:
        self._move_relative(-abs(distance_m), 0.0, 0.0, mode, speed_mps)

    def move_up(
        self, distance_m: float = DEFAULT_DISTANCE_M, *, mode: str = "position",
        speed_mps: float = DEFAULT_SPEED_MPS,
    ) -> None:
        self._move_relative(0.0, 0.0, -abs(distance_m), mode, speed_mps)

    def move_down(
        self, distance_m: float = DEFAULT_DISTANCE_M, *, mode: str = "position",
        speed_mps: float = DEFAULT_SPEED_MPS,
    ) -> None:
        self._move_relative(0.0, 0.0, abs(distance_m), mode, speed_mps)

    def _move_relative(
        self,
        dn: float,
        de: float,
        dd: float,
        mode: str,
        speed_mps: float,
    ) -> None:
        self._validate_vector(dn, de, dd, speed_mps)
        if mode not in ("position", "velocity") or speed_mps <= 0:
            raise ValueError("Use position or velocity mode and a positive speed.")
        self.wait_for_state()
        self.wait_for_movement_ready()
        current = self.state.pose.pose.position
        target = (current.x + dn, current.y + de, current.z + dd)
        # Track the destination so lateral drift cannot finish a forward move.
        # The shared waypoint helper checks fresh armed OFFBOARD telemetry,
        # brakes near arrival, and captures a hold after velocity movement.
        try:
            self.move_to_and_wait(
                *target, mode=mode, speed_mps=speed_mps,
                timeout_s=max(60.0, 3.0 * math.hypot(dn, de, dd) / speed_mps + 10.0),
            )
        except BaseException:
            self.stop()
            raise
