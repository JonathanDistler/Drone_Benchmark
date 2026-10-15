#!/usr/bin/env python3

import math
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Pose, PoseArray, PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64, String
from std_srvs.srv import Trigger

from px4_msgs.msg import (
    BatteryStatus,
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)


class DroneNode(Node):
    # ROS interface for one PX4 vehicle; positions use North, East, Down

    # Warm up the OFFBOARD stream for one second at 20 Hz
    OFFBOARD_WARMUP_SETPOINTS = 20

    def __init__(self):
        super().__init__("drone")


        self.declare_parameter("world_name", "")
        self.declare_parameter("spawn_world_y", 0.0)
        self.declare_parameter("drone_id", 0)
        self.declare_parameter("system_id", 1)
        self.declare_parameter("takeoff_height", 2.0)
        self.declare_parameter("px4_namespace", "")
        self.declare_parameter("swarm_size", 1)
        self.declare_parameter("architecture", "central")
        self.declare_parameter("position_speed_mps", 0.4)

        self.drone_id = self.get_parameter("drone_id").value
        self.system_id = self.get_parameter("system_id").value
        self.takeoff_height = self.get_parameter("takeoff_height").value
        self.swarm_size = self.get_parameter("swarm_size").value
        if self.swarm_size < 1:
            raise ValueError("swarm_size must be at least 1.")
        if self.drone_id < 0 or self.drone_id >= self.swarm_size:
            raise ValueError("drone_id must be within swarm_size.")
        self.px4_namespace = self._normalise_namespace(
            self.get_parameter("px4_namespace").value
        )

        self.drone_namespace = f"/drone_{self.drone_id}"


        self.local_position = None
        self.vehicle_status = None
        # Keep other drones’ positions through the public ROS interface
        self.peer_states: dict[int, Odometry] = {}

        self.position_speed_mps = float(self.get_parameter("position_speed_mps").value)
        if not math.isfinite(self.position_speed_mps) or self.position_speed_mps <= 0:
            raise ValueError("position_speed_mps must be finite and positive.")
        self.position_reference = None
        self.reference_time_ns = None
        self.takeoff_climb_active = False
        self.position_target = [0.0, 0.0, 0.0]
        self.velocity_target = [0.0, 0.0, 0.0]
        self.yaw_target = math.nan

        self.control_mode = "position"

        # Count streamed messages before requesting OFFBOARD
        self.offboard_setpoint_count = 0
        self.takeoff_pending = False
        self.started = False
        self.offboard_requested = False
        self.landing = False
        self.arm_deadline = None
        self.arm_mode_retry_at = 0.0
        self.arm_retry_at = 0.0
        self.takeoff_mode_retry_at = 0.0


        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode,
            self._px4_topic("fmu/in/offboard_control_mode"),
            10,
        )

        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            self._px4_topic("fmu/in/trajectory_setpoint"),
            10,
        )

        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            self._px4_topic("fmu/in/vehicle_command"),
            10,
        )


        self.create_subscription(
            VehicleLocalPosition,
            self._px4_topic("fmu/out/vehicle_local_position_v1"),
            self.local_position_callback,
            qos_profile_sensor_data,
        )

        self.create_subscription(
            VehicleStatus,
            self._px4_topic("fmu/out/vehicle_status_v1"),
            self.vehicle_status_callback,
            qos_profile_sensor_data,
        )


        self.state_pub = self.create_publisher(
            Odometry,
            "state",
            10,
        )
        self.battery_pub = self.create_publisher(BatteryStatus, 'battery', 10)
        self.local_telemetry_pub = self.create_publisher(VehicleLocalPosition, 'local_telemetry', 10)
        self.create_subscription(
            BatteryStatus, self._px4_topic('fmu/out/battery_status_v1'),
            self.battery_pub.publish, qos_profile_sensor_data,
        )
        self.control_state_pub = self.create_publisher(String, "control_state", 10)
        self.started_pub = self.create_publisher(Bool, "started", 10)
        self.armed_pub = self.create_publisher(Bool, "armed", 10)
        self.offboard_pub = self.create_publisher(Bool, "offboard", 10)
        # Index poses by drone ID; NaN means missing state
        self.peer_states_pub = self.create_publisher(
            PoseArray, "peer_states_ned", 10
        )

        # Subscribe to every swarm member, including this drone
        for peer_id in range(self.swarm_size):
            self.create_subscription(
                Odometry,
                f"/drone_{peer_id}/state",
                lambda message, i=peer_id: self.peer_state_callback(i, message),
                10,
            )


        self.create_subscription(
            PoseStamped,
            "command/position_ned",
            self.position_command_callback,
            10,
        )

        self.create_subscription(
            TwistStamped,
            "command/velocity_ned",
            self.velocity_command_callback,
            10,
        )
        self.create_subscription(Float64, "command/yaw_rad", self.yaw_command_callback, 10)


        self.create_service(Trigger, "startup", self.startup_callback)

        self.create_service(
            Trigger,
            "arm",
            self.arm_callback,
        )

        self.create_service(
            Trigger,
            "disarm",
            self.disarm_callback,
        )

        self.create_service(
            Trigger,
            "takeoff",
            self.takeoff_callback,
        )

        self.create_service(
            Trigger,
            "land",
            self.land_callback,
        )

        self.create_service(
            Trigger,
            "stop",
            self.stop_callback,
        )

        self.create_service(
            Trigger,
            "set_position_mode",
            self.set_position_mode_callback,
        )

        self.create_service(
            Trigger,
            "set_velocity_mode",
            self.set_velocity_mode_callback,
        )


        # Keep OFFBOARD control and setpoints streaming at 20 Hz
        self.offboard_timer = self.create_timer(
            0.05,
            self.publish_offboard_control_mode,
        )

        self.setpoint_timer = self.create_timer(
            0.05,
            self.publish_setpoint,
        )

        self.state_timer = self.create_timer(
            0.05,
            self.publish_state,
        )
        self.peer_state_timer = self.create_timer(0.1, self.publish_peer_states)

        self.get_logger().info(
            f"Drone {self.drone_id} ready: "
            f"PX4 namespace={self.px4_namespace or '/'} system_id={self.system_id}"
        )

    @staticmethod
    def _normalise_namespace(namespace: str) -> str:
        # Return an absolute namespace without a trailing slash
        namespace = namespace.strip()
        if not namespace or namespace == "/":
            return ""
        return "/" + namespace.strip("/")

    def _px4_topic(self, suffix: str) -> str:
        # Build a PX4 DDS topic for this vehicle's DDS namespace
        return f"{self.px4_namespace}/{suffix}"


    def local_position_callback(self, msg):
        self.local_position = msg

    def vehicle_status_callback(self, msg):
        previous = self.vehicle_status
        self.vehicle_status = msg
        was_armed = previous is not None and previous.arming_state == VehicleStatus.ARMING_STATE_ARMED
        is_armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        if is_armed != was_armed:
            guidance = "armed; request takeoff and wait for completion" if is_armed else "disarmed"
            self.get_logger().info(f"Drone {self.drone_id} {guidance}.")
        armed = Bool()
        armed.data = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        self.armed_pub.publish(armed)
        offboard = Bool()
        offboard.data = msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        self.offboard_pub.publish(offboard)
        # A mode command being published does not mean PX4 has applied it
        # Use a subsequent status sample before sending the arm command
        if self.arm_deadline is not None:
            if time.monotonic() >= self.arm_deadline:
                self.arm_deadline = None
            elif is_armed:
                self.arm_deadline = None
            elif offboard.data and msg.pre_flight_checks_pass:
                now = time.monotonic()
                if now >= self.arm_retry_at:
                    self.arm_retry_at = now + 0.5
                    self.get_logger().info(f"Requesting arm for drone {self.drone_id}")
                    self.publish_vehicle_command(
                        VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def peer_state_callback(self, drone_id: int, msg: Odometry) -> None:
        # Cache another DroneNode's network-visible position data
        self.peer_states[drone_id] = msg


    def position_command_callback(self, msg):
        if not all(math.isfinite(v) for v in (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)):
            self.get_logger().warning("Ignoring non-finite NED command.")
            return
        if self.control_mode != "position":
            self.position_reference = None
        self.control_mode = "position"
        self.takeoff_climb_active = False

        self.position_target = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ]

    def velocity_command_callback(self, msg):
        if not all(math.isfinite(v) for v in (msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z)):
            self.get_logger().warning("Ignoring non-finite NED command.")
            return
        self.control_mode = "velocity"
        self.takeoff_climb_active = False

        self.velocity_target = [
            msg.twist.linear.x,
            msg.twist.linear.y,
            msg.twist.linear.z,
        ]

    def yaw_command_callback(self, msg: Float64):
        if not math.isfinite(msg.data):
            return
        self.yaw_target = float(msg.data)


    def publish_vehicle_command(
        self,
        command,
        param1=0.0,
        param2=0.0,
    ):
        msg = VehicleCommand()

        msg.timestamp = self.get_clock().now().nanoseconds // 1000

        msg.param1 = float(param1)
        msg.param2 = float(param2)

        msg.command = command

        msg.target_system = self.system_id
        msg.target_component = 1

        msg.source_system = self.system_id
        msg.source_component = 1

        msg.from_external = True

        self.vehicle_command_pub.publish(msg)


    def request_offboard_mode(self):
        # Request PX4 OFFBOARD mode

        self.get_logger().info(
            f"Requesting OFFBOARD mode for drone {self.drone_id}"
        )

        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            1.0,
            6.0,
        )


    def startup(self):
        # Start streaming a ground hold target without arming or taking off
        if self.started and not self.landing:
            return
        if self.local_position is None:
            raise RuntimeError("No PX4 local-position data received yet.")
        if self.vehicle_status is not None and self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED:
            raise RuntimeError("Disarm before restarting after landing.")
        self.position_target = [self.local_position.x, self.local_position.y, self.local_position.z]
        self.velocity_target = [0.0, 0.0, 0.0]
        self.control_mode = "position"
        self.takeoff_climb_active = False
        self.position_reference = None
        self.reference_time_ns = None
        self.offboard_setpoint_count = 0
        self.offboard_requested = False
        self.takeoff_pending = False
        self.arm_deadline = None
        self.landing = False
        self.started = True

    def startup_callback(self, request, response):
        del request
        try:
            self.startup()
        except RuntimeError as exc:
            response.success = False
            response.message = str(exc)
        else:
            response.success = True
            response.message = "Startup queued; streaming the current position without arming."
        return response

    def arm(self):
        if not self.started or self.landing or self.offboard_setpoint_count < self.OFFBOARD_WARMUP_SETPOINTS:
            raise RuntimeError("Call startup() and wait for completion before arm().")
        if self.arm_deadline is not None:
            return
        # Keep streaming while waiting: PX4's warmup uses simulation time,
        # which can advance more slowly than our ROS wall-clock timer
        now = time.monotonic()
        self.arm_deadline = now + 4.0
        self.arm_mode_retry_at = now + 0.5
        self.arm_retry_at = now
        self.request_offboard_mode()

    def disarm(self):
        self.arm_deadline = None
        self.takeoff_climb_active = False
        self.get_logger().info(
            f"Requesting disarm for drone {self.drone_id}"
        )

        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            0.0,
        )

    def arm_callback(self, request, response):
        del request

        try:
            self.arm()
        except RuntimeError as exc:
            response.success = False
            response.message = str(exc)
            return response

        response.success = True
        response.message = "Arming queued; waiting for PX4 OFFBOARD and preflight readiness."

        return response

    def disarm_callback(self, request, response):
        del request

        self.disarm()

        response.success = True
        response.message = "PX4 disarm command sent."

        return response


    def takeoff_callback(self, request, response):
        del request

        if not self.started or self.landing:
            response.success = False
            response.message = "Call startup() before takeoff()."
            return response
        if self.vehicle_status is None or self.vehicle_status.arming_state != VehicleStatus.ARMING_STATE_ARMED:
            response.success = False
            response.message = "Call arm() before takeoff()."
            return response

        if self.local_position is None:
            response.success = False
            response.message = "No PX4 local-position data received yet."
            return response


        self.landing = False

        self.position_target[0] = self.local_position.x
        self.position_target[1] = self.local_position.y

        # PX4 uses NED coordinates: negative Z means upward
        self.position_target[2] = -self.takeoff_height
        self.takeoff_climb_active = True


        self.control_mode = "position"

        # Startup already warmed the continuously running stream. Preserve it
        # so takeoff does not introduce another per-drone mode-entry delay
        self.takeoff_pending = True
        self.offboard_requested = False
        self.takeoff_mode_retry_at = 0.0

        response.success = True
        response.message = (
            f"Takeoff queued to {self.takeoff_height:.2f} m; "
            "waiting for the PX4 OFFBOARD setpoint stream."
        )

        self.get_logger().info(
            f"Takeoff: target = "
            f"[{self.position_target[0]:.2f}, "
            f"{self.position_target[1]:.2f}, "
            f"{self.position_target[2]:.2f}]"
        )

        return response

    def land_callback(self, request, response):
        del request


        self.get_logger().info(
            f"Requesting land for drone {self.drone_id}"
        )

        self.landing = True
        self.takeoff_climb_active = False
        self.arm_deadline = None
        self.takeoff_pending = False
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_NAV_LAND
        )

        response.success = True
        response.message = "PX4 land command sent."

        return response


    def stop_callback(self, request, response):
        del request
        self.takeoff_climb_active = False
        self.position_reference = None
        self.reference_time_ns = None

        if self.local_position is not None:
            self.control_mode = "position"

            self.position_target = [
                self.local_position.x,
                self.local_position.y,
                self.local_position.z,
            ]

        response.success = True
        response.message = "Drone commanded to hold current position."

        return response


    def set_position_mode_callback(self, request, response):
        del request
        self.takeoff_climb_active = False
        self.position_reference = None
        self.reference_time_ns = None

        if self.local_position is None:
            response.success = False
            response.message = "Position control requires local-position telemetry."
            return response
        self.position_target = [self.local_position.x, self.local_position.y, self.local_position.z]
        self.control_mode = "position"

        response.success = True
        response.message = "Position control enabled; holding current position."

        return response

    def set_velocity_mode_callback(self, request, response):
        del request
        self.takeoff_climb_active = False

        self.velocity_target = [0.0, 0.0, 0.0]
        self.control_mode = "velocity"

        response.success = True
        response.message = "Velocity control enabled with zero velocity."

        return response


    def publish_offboard_control_mode(self):
        if self.landing or not self.started:
            return
        msg = OffboardControlMode()

        msg.timestamp = self.get_clock().now().nanoseconds // 1000

        msg.position = self.control_mode == "position"
        msg.velocity = self.control_mode == "velocity"

        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False

        self.offboard_control_mode_pub.publish(msg)

        self.offboard_setpoint_count += 1
        if self.arm_deadline is not None:
            now = time.monotonic()
            if now >= self.arm_deadline:
                self.arm_deadline = None
                self.get_logger().error(
                    "Arming cancelled: PX4 did not confirm OFFBOARD and preflight readiness. "
                    "Inspect PX4 preflight messages, then retry arm().")
            elif now >= self.arm_mode_retry_at:
                self.request_offboard_mode()
                self.arm_mode_retry_at = now + 0.5
        if self.offboard_setpoint_count == self.OFFBOARD_WARMUP_SETPOINTS and not self.takeoff_pending:
            self.get_logger().info(f"Drone {self.drone_id} startup complete; arm() is available.")
        if self.takeoff_pending:
            status = self.vehicle_status
            if status is not None and status.arming_state != VehicleStatus.ARMING_STATE_ARMED:
                self.takeoff_pending = False
                self.takeoff_climb_active = False
                # Cancel the climb target if PX4 disarms before takeoff
                # A later explicit arm must not release an old takeoff
                if self.local_position is not None:
                    self.position_target = [self.local_position.x, self.local_position.y,
                                            self.local_position.z]
                self.get_logger().error("Takeoff cancelled: PX4 reports disarmed; retry arm() and takeoff().")
            elif (self.offboard_requested and status is not None
                  and getattr(status, "nav_state", None) == VehicleStatus.NAVIGATION_STATE_OFFBOARD):
                self.takeoff_pending = False
            elif (self.offboard_setpoint_count >= self.OFFBOARD_WARMUP_SETPOINTS
                  and time.monotonic() >= self.takeoff_mode_retry_at):
                self.request_offboard_mode()
                self.offboard_requested = True
                self.takeoff_mode_retry_at = time.monotonic() + 0.5

    def publish_setpoint(self):
        # Let AUTO.LAND own the trajectory during descent
        if self.landing or not self.started:
            return
        msg = TrajectorySetpoint()

        msg.timestamp = self.get_clock().now().nanoseconds // 1000

        if self.control_mode == "position":
            now_ns = self.get_clock().now().nanoseconds
            current = self.local_position
            if current is None:
                return
            # PX4 needs a sustained climb target to complete its takeoff ramp.
            # A reference capped near the ground can keep it in the landed state.
            climbing = getattr(self, 'takeoff_climb_active', False)
            if climbing and abs(current.z - self.position_target[2]) <= 0.1:
                self.takeoff_climb_active = False
                climbing = False
                self.position_reference = [current.x, current.y, current.z]
                self.reference_time_ns = now_ns
            if self.position_reference is None or self.reference_time_ns is None:
                self.position_reference = [current.x, current.y, current.z]
                dt = 0.0
            else:
                dt = max(0.0, min(0.1, (now_ns - self.reference_time_ns) / 1e9))
            self.reference_time_ns = now_ns
            # Keep the reference close to the measured vehicle, so a blocked
            # drone cannot accumulate a distant target and accelerate on release.
            if math.dist(self.position_reference, [current.x, current.y, current.z]) > 0.2:
                self.position_reference = [current.x, current.y, current.z]
            error = [goal - ref for goal, ref in zip(self.position_target, self.position_reference)]
            distance = math.hypot(*error)
            scale = min(1.0, self.position_speed_mps * dt / distance) if distance else 0.0
            self.position_reference = [ref + scale * delta
                                       for ref, delta in zip(self.position_reference, error)]
            msg.position = list(self.position_reference)
            if climbing:
                msg.position = list(self.position_target)
                self.position_reference = None
                self.reference_time_ns = None

            msg.velocity = [
                math.nan,
                math.nan,
                math.nan,
            ]

        elif self.control_mode == "velocity":
            self.position_reference = None
            self.reference_time_ns = None
            msg.position = [
                math.nan,
                math.nan,
                math.nan,
            ]

            msg.velocity = [
                float(self.velocity_target[0]),
                float(self.velocity_target[1]),
                float(self.velocity_target[2]),
            ]

        msg.acceleration = [
            math.nan,
            math.nan,
            math.nan,
        ]

        msg.jerk = [
            math.nan,
            math.nan,
            math.nan,
        ]

        msg.yaw = float(self.yaw_target)
        msg.yawspeed = math.nan

        self.trajectory_setpoint_pub.publish(msg)


    def publish_state(self):
        self.control_state_pub.publish(String(data=json.dumps({
            "drone_id": self.drone_id, "system_id": self.system_id,
            "architecture": self.get_parameter("architecture").value,
            "mode": self.control_mode, "started": self.started,
            "landing": self.landing, "takeoff_pending": self.takeoff_pending,
            "takeoff_climb_active": self.takeoff_climb_active,
            "offboard_requested": self.offboard_requested,
            "offboard_setpoint_count": self.offboard_setpoint_count,
            "position_target_ned_m": self.position_target,
            "position_reference_ned_m": self.position_reference,
            "velocity_target_ned_mps": self.velocity_target,
            "yaw_target_rad": self.yaw_target,
            "position_speed_mps": self.position_speed_mps,
            "takeoff_height_m": self.takeoff_height,
        })))
        started = Bool()
        started.data = self.started and not self.landing and self.offboard_setpoint_count >= self.OFFBOARD_WARMUP_SETPOINTS
        self.started_pub.publish(started)
        if self.local_position is None:
            return

        self.local_telemetry_pub.publish(self.local_position)

        msg = Odometry()

        now = self.get_clock().now().to_msg()

        msg.header.stamp = now
        msg.header.frame_id = f"drone_{self.drone_id}_ned"
        msg.child_frame_id = f"drone_{self.drone_id}_base"

        msg.pose.pose.position.x = float(
            self.local_position.x
        )

        msg.pose.pose.position.y = float(
            self.local_position.y
        )

        msg.pose.pose.position.z = float(
            self.local_position.z
        )

        msg.pose.pose.orientation.w = 1.0

        msg.twist.twist.linear.x = float(
            self.local_position.vx
        )

        msg.twist.twist.linear.y = float(
            self.local_position.vy
        )

        msg.twist.twist.linear.z = float(
            self.local_position.vz
        )

        self.state_pub.publish(msg)

    def publish_peer_states(self) -> None:
        # Publish local NED poses by drone ID; missing poses contain NaN
        message = PoseArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "local_ned"
        for drone_id in range(self.swarm_size):
            pose = Pose()
            state = self.peer_states.get(drone_id)
            if state is None:
                pose.position.x = math.nan
                pose.position.y = math.nan
                pose.position.z = math.nan
            else:
                pose.position = state.pose.pose.position
                pose.orientation = state.pose.pose.orientation
            message.poses.append(pose)
        self.peer_states_pub.publish(message)


def main(args=None):
    rclpy.init(args=args)

    node = DroneNode()

    from .architecture import leader_id
    from .edge import EdgeLeader
    selected = leader_id(node.get_parameter("architecture").value, node.swarm_size)
    leader = EdgeLeader(node) if selected == node.drone_id else None
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()

    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass

    finally:
        if leader is not None:
            leader.close()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
