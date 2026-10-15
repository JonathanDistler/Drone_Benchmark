#!/usr/bin/env python3

# PX4 / ROS 2 MULTI-DRONE SWARM CONTROLLER ROS 2 / PX4 / uXRCE-DDS Three-vehicle controller
# for: PX4 instance 0 -> / PX4 instance 1 -> /px4_1 PX4 instance 2 -> /px4_2 The controller:
# 1. Subscribes to VehicleLocalPosition. 2. Subscribes to VehicleStatus. 3. Subscribes to
# VehicleCommandAck. 4. Continuously publishes OffboardControlMode. 5. Continuously publishes
# TrajectorySetpoint. 6. Sends correctly routed VehicleCommand messages. Initialization:
# position-control Offboard takeoff/hold Experiments: TEST 1: Direct 3 m East translation
# XY/YZ pairwise MSE TEST 2: Random positions Form a straight line TEST 3: Form a line Split
# around a flag direction Translate both groups 1 m East

import argparse
import csv
import math
import random
import time

from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import numpy as np

import rclpy

from rclpy.node import Node

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleCommandAck,
    VehicleLocalPosition,
    VehicleStatus,
)


# CONFIGURATION

DEFAULT_DRONE_COUNT = 3

CONTROL_RATE_HZ = 20.0

CONTROL_PERIOD_S = (
    1.0 / CONTROL_RATE_HZ
)

# Initial Offboard takeoff

TAKEOFF_ALTITUDE_M = 2.0

TAKEOFF_TOLERANCE_M = 0.20

TAKEOFF_HOLD_TIME_S = 2.0

TAKEOFF_TIMEOUT_S = 20.0

# Velocity limits

MAX_HORIZONTAL_SPEED_MPS = 1.50

MAX_VERTICAL_SPEED_MPS = 0.70

# Position controller

POSITION_KP = 1.0

POSITION_TOLERANCE_M = 0.25

POSITION_SPEED_TOLERANCE_MPS = 0.30

POSITION_TIMEOUT_S = 30.0

# Test 1

TEST1_TRANSLATE_M = 3.0

TEST1_SPEED_MPS = 1.0

TEST1_TIMEOUT_S = 10.0

# Test 2

TEST2_RANDOM_RANGE_M = 2.0

TEST2_LINE_SPACING_M = 1.5

TEST2_SPEED_MPS = 1.0

TEST2_TIMEOUT_S = 30.0

# Test 3

TEST3_GROUP_SPACING_M = 1.5

TEST3_TRANSLATE_M = 1.0

TEST3_SPEED_MPS = 1.0

TEST3_TIMEOUT_S = 30.0

# Startup

TELEMETRY_TIMEOUT_S = 15.0

INITIAL_SETPOINT_COUNT = 20

OFFBOARD_RETRY_COUNT = 5

OFFBOARD_RETRY_INTERVAL_S = 0.40

ARM_RETRY_COUNT = 5

ARM_RETRY_INTERVAL_S = 0.40

# Randomization

DEFAULT_RANDOM_SEED = None

# Output

DEFAULT_RESULTS_CSV = (
    str(
        Path.home()
        / "px4_ros2_ws"
        / "swarm_test_results.csv"
    )
)


# PX4 NAMESPACE CONFIGURATION

# PX4's documented multi-vehicle simulation behavior: instance 0 -> no namespace instance 1 ->
# /px4_1 instance 2 -> /px4_2 Do not replace these with /uav_0 etc. for this simulation

PX4_NAMESPACES = [
    "",
    "/px4_1",
    "/px4_2",
]


# PX4 SYSTEM IDS

# PX4 instance 0 -> MAV_SYS_ID 1 PX4 instance 1 -> MAV_SYS_ID 2 PX4 instance 2 -> MAV_SYS_ID 3

PX4_SYS_ID_START = 1


# PX4 COMMAND CONSTANTS

VEHICLE_CMD_COMPONENT_ARM_DISARM = 400

VEHICLE_CMD_DO_SET_MODE = 176

VEHICLE_CMD_NAV_LAND = 21


# PX4 NAVIGATION CONSTANTS

NAVIGATION_STATE_MANUAL = 0

NAVIGATION_STATE_ALTCTL = 1

NAVIGATION_STATE_POSCTL = 2

NAVIGATION_STATE_AUTO_MISSION = 3

NAVIGATION_STATE_AUTO_LOITER = 4

NAVIGATION_STATE_AUTO_RTL = 5

NAVIGATION_STATE_POSITION_SLOW = 6

NAVIGATION_STATE_ACRO = 7

NAVIGATION_STATE_DESCEND = 9

NAVIGATION_STATE_TERMINATION = 11

NAVIGATION_STATE_OFFBOARD = 14

NAVIGATION_STATE_STAB = 15

NAVIGATION_STATE_AUTO_TAKEOFF = 22


# ROS 2 QoS

PX4_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


# DRONE STATE

@dataclass
class DroneState:

    index: int

    namespace: str

    system_id: int

    north_m: float = 0.0

    east_m: float = 0.0

    down_m: float = 0.0

    north_velocity_mps: float = 0.0

    east_velocity_mps: float = 0.0

    down_velocity_mps: float = 0.0

    xy_valid: bool = False

    z_valid: bool = False

    v_xy_valid: bool = False

    v_z_valid: bool = False

    armed: bool = False

    nav_state: int = 0

    pre_flight_checks_pass: bool = False

    last_position_time: float = 0.0

    last_status_time: float = 0.0

    last_ack_time: float = 0.0

    last_ack_command: int = -1

    last_ack_result: int = -1

    target_north_m: float = 0.0

    target_east_m: float = 0.0

    target_down_m: float = -TAKEOFF_ALTITUDE_M

    velocity_north_mps: float = 0.0

    velocity_east_mps: float = 0.0

    velocity_down_mps: float = 0.0

    position_control_mode: bool = True


# CONTROLLER

class SwarmController(Node):

    def __init__(
        self,
        drone_count,
        random_seed,
        results_csv,
    ):

        super().__init__(
            "swarm_mavsdk_ros2_controller"
        )

        self.drone_count = (
            drone_count
        )

        self.random_seed = (
            random_seed
        )

        self.results_csv = (
            results_csv
        )

        self.drones = []

        self.offboard_publishers = []

        self.trajectory_publishers = []

        self.vehicle_command_publishers = []

        self.position_subscribers = []

        self.status_subscribers = []

        self.ack_subscribers = []

        # Create one interface set per vehicle

        for i in range(
            drone_count
        ):

            namespace = (
                PX4_NAMESPACES[i]
            )

            system_id = (
                PX4_SYS_ID_START
                + i
            )

            state = DroneState(
                index=i,
                namespace=namespace,
                system_id=system_id,
            )

            self.drones.append(
                state
            )

            # Topics

            prefix = (
                namespace
                if namespace
                else ""
            )

            offboard_topic = (
                f"{prefix}/fmu/in/"
                "offboard_control_mode"
            )

            trajectory_topic = (
                f"{prefix}/fmu/in/"
                "trajectory_setpoint"
            )

            command_topic = (
                f"{prefix}/fmu/in/"
                "vehicle_command"
            )

            position_topic = (
                f"{prefix}/fmu/out/"
                "vehicle_local_position_v1"
            )

            status_topic = (
                f"{prefix}/fmu/out/"
                "vehicle_status_v1"
            )

            ack_topic = (
                f"{prefix}/fmu/out/"
                "vehicle_command_ack"
            )

            # Publishers

            self.offboard_publishers.append(
                self.create_publisher(
                    OffboardControlMode,
                    offboard_topic,
                    PX4_QOS,
                )
            )

            self.trajectory_publishers.append(
                self.create_publisher(
                    TrajectorySetpoint,
                    trajectory_topic,
                    PX4_QOS,
                )
            )

            self.vehicle_command_publishers.append(
                self.create_publisher(
                    VehicleCommand,
                    command_topic,
                    PX4_QOS,
                )
            )

            # Subscribers

            self.position_subscribers.append(
                self.create_subscription(
                    VehicleLocalPosition,
                    position_topic,
                    lambda msg,
                    idx=i:
                    self.position_callback(
                        idx,
                        msg,
                    ),
                    PX4_QOS,
                )
            )

            self.status_subscribers.append(
                self.create_subscription(
                    VehicleStatus,
                    status_topic,
                    lambda msg,
                    idx=i:
                    self.status_callback(
                        idx,
                        msg,
                    ),
                    PX4_QOS,
                )
            )

            self.ack_subscribers.append(
                self.create_subscription(
                    VehicleCommandAck,
                    ack_topic,
                    lambda msg,
                    idx=i:
                    self.ack_callback(
                        idx,
                        msg,
                    ),
                    PX4_QOS,
                )
            )

            self.get_logger().info(
                f"UAV {i}: "
                f"namespace="
                f"{namespace if namespace else '/'} "
                f"system_id={system_id}"
            )

        # Continuous 20 Hz control loop

        self.control_timer = (
            self.create_timer(
                CONTROL_PERIOD_S,
                self.control_timer_callback,
            )
        )

        self.get_logger().info(
            f"Created swarm controller "
            f"for {drone_count} drones."
        )

    # POSITION CALLBACK

    def position_callback(
        self,
        index,
        msg,
    ):

        state = (
            self.drones[index]
        )

        state.north_m = float(
            msg.x
        )

        state.east_m = float(
            msg.y
        )

        state.down_m = float(
            msg.z
        )

        state.north_velocity_mps = float(
            msg.vx
        )

        state.east_velocity_mps = float(
            msg.vy
        )

        state.down_velocity_mps = float(
            msg.vz
        )

        state.xy_valid = bool(
            msg.xy_valid
        )

        state.z_valid = bool(
            msg.z_valid
        )

        state.v_xy_valid = bool(
            msg.v_xy_valid
        )

        state.v_z_valid = bool(
            msg.v_z_valid
        )

        state.last_position_time = (
            time.monotonic()
        )

    # STATUS CALLBACK

    def status_callback(
        self,
        index,
        msg,
    ):

        state = (
            self.drones[index]
        )

        state.armed = (
            msg.arming_state
            == VehicleStatus.ARMING_STATE_ARMED
        )

        state.nav_state = (
            int(msg.nav_state)
        )

        state.pre_flight_checks_pass = bool(
            msg.pre_flight_checks_pass
        )

        state.last_status_time = (
            time.monotonic()
        )

    # ACK CALLBACK

    def ack_callback(
        self,
        index,
        msg,
    ):

        state = (
            self.drones[index]
        )

        state.last_ack_command = int(
            msg.command
        )

        state.last_ack_result = int(
            msg.result
        )

        state.last_ack_time = (
            time.monotonic()
        )

        self.get_logger().info(
            f"UAV {index}: "
            f"VehicleCommandAck "
            f"command={msg.command} "
            f"result={msg.result}"
        )

    # CONTROL TIMER

    def control_timer_callback(
        self,
    ):

        timestamp = int(
            self.get_clock()
            .now()
            .nanoseconds
            // 1000
        )

        for i in range(
            self.drone_count
        ):

            state = (
                self.drones[i]
            )

            # OFFBOARD HEARTBEAT

            offboard_msg = (
                OffboardControlMode()
            )

            offboard_msg.position = (
                bool(
                    state.position_control_mode
                )
            )

            offboard_msg.velocity = (
                not state.position_control_mode
            )

            offboard_msg.acceleration = False

            offboard_msg.attitude = False

            offboard_msg.body_rate = False

            offboard_msg.thrust_and_torque = False

            offboard_msg.direct_actuator = False

            offboard_msg.timestamp = (
                timestamp
            )

            self.offboard_publishers[i].publish(
                offboard_msg
            )

            # TRAJECTORY SETPOINT

            trajectory_msg = (
                TrajectorySetpoint()
            )

            if (
                state.position_control_mode
            ):

                trajectory_msg.position = [
                    float(
                        state.target_north_m
                    ),
                    float(
                        state.target_east_m
                    ),
                    float(
                        state.target_down_m
                    ),
                ]

                trajectory_msg.velocity = [
                    math.nan,
                    math.nan,
                    math.nan,
                ]

            else:

                trajectory_msg.position = [
                    math.nan,
                    math.nan,
                    math.nan,
                ]

                trajectory_msg.velocity = [
                    float(
                        state.velocity_north_mps
                    ),
                    float(
                        state.velocity_east_mps
                    ),
                    float(
                        state.velocity_down_mps
                    ),
                ]

            trajectory_msg.acceleration = [
                math.nan,
                math.nan,
                math.nan,
            ]

            trajectory_msg.jerk = [
                math.nan,
                math.nan,
                math.nan,
            ]

            trajectory_msg.yaw = (
                math.radians(
                    90.0
                )
            )

            trajectory_msg.yawspeed = 0.0

            trajectory_msg.timestamp = (
                timestamp
            )

            self.trajectory_publishers[i].publish(
                trajectory_msg
            )

    # VEHICLE COMMAND

    def send_vehicle_command(
        self,
        index,
        command,
        param1=0.0,
        param2=0.0,
        param3=0.0,
        param4=0.0,
        param5=0.0,
        param6=0.0,
        param7=0.0,
    ):

        state = (
            self.drones[index]
        )

        msg = (
            VehicleCommand()
        )

        msg.command = (
            int(command)
        )

        msg.param1 = (
            float(param1)
        )

        msg.param2 = (
            float(param2)
        )

        msg.param3 = (
            float(param3)
        )

        msg.param4 = (
            float(param4)
        )

        msg.param5 = (
            float(param5)
        )

        msg.param6 = (
            float(param6)
        )

        msg.param7 = (
            float(param7)
        )

        # CRITICAL MULTI-VEHICLE ROUTING

        msg.target_system = (
            state.system_id
        )

        msg.target_component = 1

        msg.source_system = 1

        msg.source_component = 1

        msg.from_external = True

        msg.timestamp = int(
            self.get_clock()
            .now()
            .nanoseconds
            // 1000
        )

        self.vehicle_command_publishers[index].publish(
            msg
        )

    # OFFBOARD

    def request_offboard(
        self,
        index,
    ):

        self.send_vehicle_command(
            index=index,
            command=(
                VEHICLE_CMD_DO_SET_MODE
            ),
            param1=1.0,
            param2=6.0,
        )

    # ARM

    def arm(
        self,
        index,
    ):

        self.send_vehicle_command(
            index=index,
            command=(
                VEHICLE_CMD_COMPONENT_ARM_DISARM
            ),
            param1=1.0,
        )

    # DISARM

    def disarm(
        self,
        index,
    ):

        self.send_vehicle_command(
            index=index,
            command=(
                VEHICLE_CMD_COMPONENT_ARM_DISARM
            ),
            param1=0.0,
        )

    # LAND

    def land(
        self,
        index,
    ):

        self.send_vehicle_command(
            index=index,
            command=(
                VEHICLE_CMD_NAV_LAND
            )
        )

    # SET POSITION TARGET

    def set_position_target(
        self,
        index,
        north_m,
        east_m,
        down_m,
    ):

        state = (
            self.drones[index]
        )

        state.target_north_m = (
            float(north_m)
        )

        state.target_east_m = (
            float(east_m)
        )

        state.target_down_m = (
            float(down_m)
        )

    # SET VELOCITY TARGET

    def set_velocity_target(
        self,
        index,
        north_mps,
        east_mps,
        down_mps,
    ):

        horizontal_speed = math.sqrt(
            north_mps ** 2
            + east_mps ** 2
        )

        if (
            horizontal_speed
            > MAX_HORIZONTAL_SPEED_MPS
        ):

            scale = (
                MAX_HORIZONTAL_SPEED_MPS
                / horizontal_speed
            )

            north_mps *= scale

            east_mps *= scale

        down_mps = max(
            -MAX_VERTICAL_SPEED_MPS,
            min(
                MAX_VERTICAL_SPEED_MPS,
                down_mps,
            ),
        )

        state = (
            self.drones[index]
        )

        state.velocity_north_mps = (
            float(north_mps)
        )

        state.velocity_east_mps = (
            float(east_mps)
        )

        state.velocity_down_mps = (
            float(down_mps)
        )

    # STOP DRONE

    def stop_drone(
        self,
        index,
    ):

        self.set_velocity_target(
            index,
            0.0,
            0.0,
            0.0,
        )

    # STOP ALL

    def stop_all(
        self,
    ):

        for i in range(
            self.drone_count
        ):

            self.stop_drone(i)

    # SPIN

    def spin_for(
        self,
        duration_s,
    ):

        end_time = (
            time.monotonic()
            + duration_s
        )

        while (
            time.monotonic()
            < end_time
        ):

            rclpy.spin_once(
                self,
                timeout_sec=0.02,
            )

    # WAIT FOR POSITION TELEMETRY

    def wait_for_positions(
        self,
    ):

        print()

        print(
            "-" * 60
        )

        print(
            "[ROS 2] Waiting for PX4 local position telemetry..."
        )

        print(
            "-" * 60
        )

        start = (
            time.monotonic()
        )

        while (
            time.monotonic()
            - start
            < TELEMETRY_TIMEOUT_S
        ):

            rclpy.spin_once(
                self,
                timeout_sec=0.05,
            )

            connected = sum(
                state.last_position_time > 0.0
                for state in self.drones
            )

            print(
                f"\r[ROS 2] Position telemetry "
                f"{connected}/{self.drone_count}",
                end="",
                flush=True,
            )

            if (
                connected
                == self.drone_count
            ):

                print()

                print(
                    "[ROS 2] All PX4 position streams connected."
                )

                return True

        print()

        return False

    # WAIT FOR VALID POSITION

    def wait_for_valid_position(
        self,
    ):

        print()

        print(
            "-" * 60
        )

        print(
            "[HEALTH] Waiting for valid local positions..."
        )

        print(
            "-" * 60
        )

        start = (
            time.monotonic()
        )

        while (
            time.monotonic()
            - start
            < TELEMETRY_TIMEOUT_S
        ):

            rclpy.spin_once(
                self,
                timeout_sec=0.05,
            )

            valid = all(
                state.xy_valid
                and state.z_valid
                for state in self.drones
            )

            if valid:

                print(
                    "[HEALTH] All local positions valid."
                )

                return True

        print(
            "[HEALTH] Position validity timeout."
        )

        self.print_positions()

        return False

    # PRINT POSITIONS

    def print_positions(
        self,
    ):

        for state in self.drones:

            print(
                f"Drone {state.index}: "
                f"N={state.north_m:+.3f} m, "
                f"E={state.east_m:+.3f} m, "
                f"D={state.down_m:+.3f} m"
            )

    # PRINT STATUS

    def print_status(
        self,
    ):

        for state in self.drones:

            speed = math.sqrt(
                state.north_velocity_mps ** 2
                + state.east_velocity_mps ** 2
                + state.down_velocity_mps ** 2
            )

            print(
                f"Drone {state.index}: "
                f"armed={state.armed} | "
                f"nav_state={state.nav_state} | "
                f"preflight={state.pre_flight_checks_pass} | "
                f"speed={speed:.2f} m/s | "
                f"N={state.north_m:+.2f} | "
                f"E={state.east_m:+.2f} | "
                f"D={state.down_m:+.2f}"
            )

    # PREPARE POSITION MODE

    def prepare_position_mode(
        self,
    ):

        for state in self.drones:

            state.position_control_mode = True

            self.set_position_target(
                state.index,
                state.north_m,
                state.east_m,
                -TAKEOFF_ALTITUDE_M,
            )

    # INITIAL SETPOINT STREAM

    def stream_initial_setpoints(
        self,
    ):

        print()

        print(
            "[OFFBOARD] Streaming initial position setpoints..."
        )

        for _ in range(
            INITIAL_SETPOINT_COUNT
        ):

            rclpy.spin_once(
                self,
                timeout_sec=0.05,
            )

    # REQUEST OFFBOARD

    def start_offboard(
        self,
    ):

        print()

        print(
            "[OFFBOARD] Requesting OFFBOARD..."
        )

        for attempt in range(
            OFFBOARD_RETRY_COUNT
        ):

            print(
                f"[OFFBOARD] Attempt "
                f"{attempt + 1}/"
                f"{OFFBOARD_RETRY_COUNT}"
            )

            for i in range(
                self.drone_count
            ):

                self.request_offboard(i)

            self.spin_for(
                OFFBOARD_RETRY_INTERVAL_S
            )

    # ARM ALL

    def arm_all(
        self,
    ):

        print()

        print(
            "[ARM] Arming all drones..."
        )

        for attempt in range(
            ARM_RETRY_COUNT
        ):

            print(
                f"[ARM] Attempt "
                f"{attempt + 1}/"
                f"{ARM_RETRY_COUNT}"
            )

            for i in range(
                self.drone_count
            ):

                self.arm(i)

            self.spin_for(
                ARM_RETRY_INTERVAL_S
            )

            if all(
                state.armed
                for state in self.drones
            ):

                print(
                    "[ARM] All drones report ARMED."
                )

                return True

        print(
            "[ARM] Not all drones report ARMED."
        )

        self.print_status()

        return False

    # WAIT FOR TAKEOFF

    def wait_for_takeoff(
        self,
    ):

        print()

        print(
            "=" * 60
        )

        print(
            "[TAKEOFF] Position-control Offboard takeoff"
        )

        print(
            "=" * 60
        )

        start = (
            time.monotonic()
        )

        next_print = start

        while True:

            now = (
                time.monotonic()
            )

            elapsed = (
                now - start
            )

            all_at_altitude = all(
                (
                    -state.down_m
                    >=
                    TAKEOFF_ALTITUDE_M
                    - TAKEOFF_TOLERANCE_M
                )
                for state in self.drones
            )

            if all_at_altitude:

                print()

                print(
                    "[TAKEOFF] All drones reached target altitude."
                )

                return True

            if (
                elapsed
                >= TAKEOFF_TIMEOUT_S
            ):

                print()

                print(
                    "[TAKEOFF] Timeout."
                )

                self.print_positions()

                self.print_status()

                return False

            if (
                now
                >= next_print
            ):

                print(
                    f"[TAKEOFF] "
                    f"t={elapsed:.1f} s | "
                    + " | ".join(
                        (
                            f"UAV {state.index}: "
                            f"alt={-state.down_m:.2f} m"
                        )
                        for state in self.drones
                    )
                )

                next_print = (
                    now + 1.0
                )

            rclpy.spin_once(
                self,
                timeout_sec=0.02,
            )

    # HOLD

    def hold_position(
        self,
        duration_s,
    ):

        print()

        print(
            f"[HOLD] Holding position "
            f"for {duration_s:.1f} s..."
        )

        start = (
            time.monotonic()
        )

        while (
            time.monotonic()
            - start
            < duration_s
        ):

            rclpy.spin_once(
                self,
                timeout_sec=0.02,
            )

    # POSITION TARGET MANEUVER

    def move_to_targets(
        self,
        targets,
        timeout_s,
        label,
    ):

        print()

        print(
            "-" * 60
        )

        print(
            f"[POSITION] {label}"
        )

        print(
            "-" * 60
        )

        for i, target in enumerate(
            targets
        ):

            self.set_position_target(
                i,
                target[0],
                target[1],
                target[2],
            )

        start = (
            time.monotonic()
        )

        next_print = start

        while True:

            now = (
                time.monotonic()
            )

            elapsed = (
                now - start
            )

            distances = []

            for i, target in enumerate(
                targets
            ):

                state = (
                    self.drones[i]
                )

                error = np.array(
                    [
                        target[0]
                        - state.north_m,

                        target[1]
                        - state.east_m,

                        target[2]
                        - state.down_m,
                    ]
                )

                distances.append(
                    np.linalg.norm(
                        error
                    )
                )

            if all(
                distance
                <= POSITION_TOLERANCE_M
                for distance in distances
            ):

                self.stop_all()

                self.spin_for(
                    0.5
                )

                return True

            if (
                elapsed
                >= timeout_s
            ):

                self.stop_all()

                print(
                    f"[POSITION] "
                    f"{label} timed out."
                )

                self.print_positions()

                return False

            # Velocity estimate toward target. Used only when a drone is already
            # in velocity Offboard mode

            if (
                not self.drones[0]
                .position_control_mode
            ):

                for i, target in enumerate(
                    targets
                ):

                    state = (
                        self.drones[i]
                    )

                    error_n = (
                        target[0]
                        - state.north_m
                    )

                    error_e = (
                        target[1]
                        - state.east_m
                    )

                    error_d = (
                        target[2]
                        - state.down_m
                    )

                    self.set_velocity_target(
                        i,
                        POSITION_KP * error_n,
                        POSITION_KP * error_e,
                        POSITION_KP * error_d,
                    )

            if (
                now
                >= next_print
            ):

                print(
                    f"[POSITION] "
                    f"t={elapsed:.1f} s | "
                    + " | ".join(
                        (
                            f"UAV {i}: "
                            f"err={distances[i]:.2f} m"
                        )
                        for i in range(
                            self.drone_count
                        )
                    )
                )

                next_print = (
                    now + 1.0
                )

            rclpy.spin_once(
                self,
                timeout_sec=0.02,
            )

    # TEST 1

    def test_1(
        self,
    ):

        print()

        print(
            "=" * 60
        )

        print(
            "TEST 1 — 3 M TRANSLATION: XY/YZ MSE"
        )

        print(
            "=" * 60
        )

        start_positions = np.array(
            [
                [
                    state.north_m,
                    state.east_m,
                    state.down_m,
                ]
                for state in self.drones
            ],
            dtype=float,
        )

        xy_before = (
            self.pairwise_mse(
                "xy"
            )
        )

        yz_before = (
            self.pairwise_mse(
                "yz"
            )
        )

        print()

        print(
            "[TEST 1] Initial positions:"
        )

        self.print_positions()

        # Switch to velocity control

        for state in self.drones:

            state.position_control_mode = False

        print()

        print(
            "[TEST 1] Commanding "
            f"+{TEST1_SPEED_MPS:.2f} m/s East "
            f"for {TEST1_TRANSLATE_M:.2f} m."
        )

        duration = (
            TEST1_TRANSLATE_M
            / TEST1_SPEED_MPS
        )

        start = (
            time.monotonic()
        )

        next_print = start

        while True:

            now = (
                time.monotonic()
            )

            elapsed = (
                now - start
            )

            if (
                elapsed
                >= duration
            ):

                break

            for i in range(
                self.drone_count
            ):

                self.set_velocity_target(
                    i,
                    0.0,
                    TEST1_SPEED_MPS,
                    0.0,
                )

            if (
                now
                >= next_print
            ):

                print(
                    f"[TEST 1] "
                    f"t={elapsed:.1f} s | "
                    + " | ".join(
                        (
                            f"UAV {state.index}: "
                            f"E={state.east_m:+.2f}"
                        )
                        for state in self.drones
                    )
                )

                next_print = (
                    now + 1.0
                )

            rclpy.spin_once(
                self,
                timeout_sec=0.02,
            )

        self.stop_all()

        self.spin_for(
            1.0
        )

        final_positions = np.array(
            [
                [
                    state.north_m,
                    state.east_m,
                    state.down_m,
                ]
                for state in self.drones
            ],
            dtype=float,
        )

        xy_after = (
            self.pairwise_mse(
                "xy"
            )
        )

        yz_after = (
            self.pairwise_mse(
                "yz"
            )
        )

        displacement = (
            np.linalg.norm(
                final_positions
                - start_positions,
                axis=1,
            )
        )

        print()

        print(
            "[TEST 1] RESULTS"
        )

        print(
            f"[TEST 1] Commanded distance = "
            f"{TEST1_TRANSLATE_M:.3f} m"
        )

        print(
            f"[TEST 1] XY MSE before = "
            f"{xy_before:.6f} m²"
        )

        print(
            f"[TEST 1] YZ MSE before = "
            f"{yz_before:.6f} m²"
        )

        print(
            f"[TEST 1] XY MSE after = "
            f"{xy_after:.6f} m²"
        )

        print(
            f"[TEST 1] YZ MSE after = "
            f"{yz_after:.6f} m²"
        )

        for i in range(
            self.drone_count
        ):

            print(
                f"[TEST 1] UAV {i}: "
                f"displacement="
                f"{displacement[i]:.3f} m"
            )

        self.save_result(
            test_name=(
                "TEST 1 — 3 M TRANSLATION"
            ),
            start_positions=(
                start_positions
            ),
            final_positions=(
                final_positions
            ),
            duration=duration,
            xy_mse_before=xy_before,
            yz_mse_before=yz_before,
            xy_mse_after=xy_after,
            yz_mse_after=yz_after,
        )

    # TEST 2

    def test_2(
        self,
    ):

        print()

        print(
            "=" * 60
        )

        print(
            "TEST 2 — RANDOM POSITIONS -> STRAIGHT LINE"
        )

        print(
            "=" * 60
        )

        rng = random.Random(
            self.random_seed
        )

        # Velocity mode

        for state in self.drones:

            state.position_control_mode = False

        start_positions = np.array(
            [
                [
                    state.north_m,
                    state.east_m,
                    state.down_m,
                ]
                for state in self.drones
            ],
            dtype=float,
        )

        random_targets = []

        for state in self.drones:

            random_targets.append(
                np.array(
                    [
                        state.north_m
                        + rng.uniform(
                            -TEST2_RANDOM_RANGE_M,
                            TEST2_RANDOM_RANGE_M,
                        ),

                        state.east_m
                        + rng.uniform(
                            -TEST2_RANDOM_RANGE_M,
                            TEST2_RANDOM_RANGE_M,
                        ),

                        state.down_m,
                    ],
                    dtype=float,
                )
            )

        random_targets = np.array(
            random_targets
        )

        print()

        print(
            "[TEST 2] Random targets:"
        )

        for i, target in enumerate(
            random_targets
        ):

            print(
                f"UAV {i}: "
                f"N={target[0]:+.2f} "
                f"E={target[1]:+.2f} "
                f"D={target[2]:+.2f}"
            )

        # Move to random targets

        if not self.velocity_move(
            random_targets,
            TEST2_TIMEOUT_S,
            "RANDOM POSITIONING",
        ):

            raise RuntimeError(
                "TEST 2 random positioning timed out."
            )

        self.stop_all()

        self.spin_for(
            0.5
        )

        # Create line

        current = np.array(
            [
                [
                    state.north_m,
                    state.east_m,
                    state.down_m,
                ]
                for state in self.drones
            ],
            dtype=float,
        )

        center_n = np.mean(
            current[:, 0]
        )

        center_e = np.mean(
            current[:, 1]
        )

        center_d = np.mean(
            current[:, 2]
        )

        line_targets = []

        for i in range(
            self.drone_count
        ):

            offset = (
                i
                - (
                    self.drone_count
                    - 1
                )
                / 2.0
            ) * TEST2_LINE_SPACING_M

            line_targets.append(
                np.array(
                    [
                        center_n,
                        center_e
                        + offset,
                        center_d,
                    ],
                    dtype=float,
                )
            )

        line_targets = np.array(
            line_targets
        )

        print()

        print(
            "[TEST 2] Forming straight line."
        )

        if not self.velocity_move(
            line_targets,
            TEST2_TIMEOUT_S,
            "LINE FORMATION",
        ):

            raise RuntimeError(
                "TEST 2 line formation timed out."
            )

        self.stop_all()

        self.spin_for(
            0.5
        )

        final_positions = np.array(
            [
                [
                    state.north_m,
                    state.east_m,
                    state.down_m,
                ]
                for state in self.drones
            ],
            dtype=float,
        )

        line_errors = np.linalg.norm(
            final_positions
            - line_targets,
            axis=1,
        )

        line_rmse = math.sqrt(
            np.mean(
                line_errors ** 2
            )
        )

        print()

        print(
            "[TEST 2] RESULTS"
        )

        print(
            f"[TEST 2] Line spacing = "
            f"{TEST2_LINE_SPACING_M:.3f} m"
        )

        print(
            f"[TEST 2] Line RMSE = "
            f"{line_rmse:.3f} m"
        )

        self.print_positions()

        self.save_result(
            test_name=(
                "TEST 2 — RANDOM "
                "POSITIONS -> LINE"
            ),
            start_positions=(
                start_positions
            ),
            final_positions=(
                final_positions
            ),
            duration=0.0,
            line_rmse=line_rmse,
        )

    # VELOCITY POSITION MANEUVER

    def velocity_move(
        self,
        targets,
        timeout_s,
        label,
    ):

        start = (
            time.monotonic()
        )

        next_print = start

        while True:

            now = (
                time.monotonic()
            )

            elapsed = (
                now - start
            )

            current_errors = []

            all_reached = True

            for i, target in enumerate(
                targets
            ):

                state = (
                    self.drones[i]
                )

                error = np.array(
                    [
                        target[0]
                        - state.north_m,

                        target[1]
                        - state.east_m,

                        target[2]
                        - state.down_m,
                    ]
                )

                distance = np.linalg.norm(
                    error
                )

                current_errors.append(
                    distance
                )

                if (
                    distance
                    > POSITION_TOLERANCE_M
                ):

                    all_reached = False

                    self.set_velocity_target(
                        i,
                        POSITION_KP * error[0],
                        POSITION_KP * error[1],
                        POSITION_KP * error[2],
                    )

                else:

                    self.stop_drone(i)

            if all_reached:

                self.stop_all()

                return True

            if (
                elapsed
                >= timeout_s
            ):

                self.stop_all()

                print(
                    f"[VELOCITY] "
                    f"{label} timed out."
                )

                self.print_positions()

                return False

            if (
                now
                >= next_print
            ):

                print(
                    f"[{label}] "
                    f"t={elapsed:.1f} s | "
                    + " | ".join(
                        (
                            f"UAV {i}: "
                            f"err="
                            f"{current_errors[i]:.2f}"
                        )
                        for i in range(
                            self.drone_count
                        )
                    )
                )

                next_print = (
                    now + 1.0
                )

            rclpy.spin_once(
                self,
                timeout_sec=0.02,
            )

    # TEST 3

    def test_3(
        self,
    ):

        print()

        print(
            "=" * 60
        )

        print(
            "TEST 3 — FLAG TURN / SPLIT / 1 M TRANSLATION"
        )

        print(
            "=" * 60
        )

        # Random flag

        rng = random.Random(
            self.random_seed
        )

        direction = rng.choice(
            [
                "LEFT",
                "RIGHT",
            ]
        )

        print()

        print(
            f"[FLAG] Direction = {direction}"
        )

        # Velocity mode

        for state in self.drones:

            state.position_control_mode = False

        start_positions = np.array(
            [
                [
                    state.north_m,
                    state.east_m,
                    state.down_m,
                ]
                for state in self.drones
            ],
            dtype=float,
        )

        # Initial line

        center_n = np.mean(
            start_positions[:, 0]
        )

        center_e = np.mean(
            start_positions[:, 1]
        )

        center_d = np.mean(
            start_positions[:, 2]
        )

        initial_line = []

        for i in range(
            self.drone_count
        ):

            offset = (
                i
                - (
                    self.drone_count
                    - 1
                )
                / 2.0
            ) * TEST3_GROUP_SPACING_M

            initial_line.append(
                np.array(
                    [
                        center_n
                        + offset,
                        center_e,
                        center_d,
                    ],
                    dtype=float,
                )
            )

        initial_line = np.array(
            initial_line
        )

        print()

        print(
            "[TEST 3] Forming initial line."
        )

        if not self.velocity_move(
            initial_line,
            TEST3_TIMEOUT_S,
            "INITIAL LINE",
        ):

            raise RuntimeError(
                "TEST 3 initial line timed out."
            )

        # Split

        majority_count = (
            self.drone_count // 2
            + 1
        )

        minority_count = (
            self.drone_count
            - majority_count
        )

        split_targets = (
            initial_line.copy()
        )

        if direction == "LEFT":

            majority_sign = +1.0

            minority_sign = -1.0

        else:

            majority_sign = -1.0

            minority_sign = +1.0

        for i in range(
            self.drone_count
        ):

            if i < majority_count:

                split_targets[i, 0] += (
                    majority_sign
                    * TEST3_GROUP_SPACING_M
                )

            else:

                split_targets[i, 0] += (
                    minority_sign
                    * TEST3_GROUP_SPACING_M
                )

        print()

        print(
            "[TEST 3] Splitting groups."
        )

        if not self.velocity_move(
            split_targets,
            TEST3_TIMEOUT_S,
            "GROUP SPLIT",
        ):

            raise RuntimeError(
                "TEST 3 group split timed out."
            )

        # Translate East 1 m

        translation_targets = (
            split_targets.copy()
        )

        translation_targets[:, 1] += (
            TEST3_TRANSLATE_M
        )

        print()

        print(
            "[TEST 3] Translating both "
            f"groups +{TEST3_TRANSLATE_M:.2f} m East."
        )

        translation_start = (
            time.monotonic()
        )

        if not self.velocity_move(
            translation_targets,
            TEST3_TIMEOUT_S,
            "EAST TRANSLATION",
        ):

            raise RuntimeError(
                "TEST 3 east translation timed out."
            )

        translation_time = (
            time.monotonic()
            - translation_start
        )

        self.stop_all()

        self.spin_for(
            0.5
        )

        final_positions = np.array(
            [
                [
                    state.north_m,
                    state.east_m,
                    state.down_m,
                ]
                for state in self.drones
            ],
            dtype=float,
        )

        print()

        print(
            "[TEST 3] RESULTS"
        )

        print(
            f"[TEST 3] Direction = {direction}"
        )

        print(
            f"[TEST 3] Majority = "
            f"{majority_count}"
        )

        print(
            f"[TEST 3] Minority = "
            f"{minority_count}"
        )

        print(
            f"[TEST 3] Translation time = "
            f"{translation_time:.3f} s"
        )

        self.print_positions()

        self.save_result(
            test_name=(
                "TEST 3 — FLAG / SPLIT / "
                "TRANSLATE"
            ),
            start_positions=(
                start_positions
            ),
            final_positions=(
                final_positions
            ),
            duration=translation_time,
            flag_direction=direction,
            majority_count=majority_count,
            minority_count=minority_count,
        )

    # PAIRWISE MSE

    def pairwise_mse(
        self,
        plane,
    ):

        positions = np.array(
            [
                [
                    state.north_m,
                    state.east_m,
                    state.down_m,
                ]
                for state in self.drones
            ],
            dtype=float,
        )

        values = []

        for i in range(
            len(positions)
        ):

            for j in range(
                i + 1,
                len(positions),
            ):

                if plane == "xy":

                    dn = (
                        positions[i, 0]
                        - positions[j, 0]
                    )

                    de = (
                        positions[i, 1]
                        - positions[j, 1]
                    )

                    values.append(
                        dn ** 2
                        + de ** 2
                    )

                else:

                    de = (
                        positions[i, 1]
                        - positions[j, 1]
                    )

                    dd = (
                        positions[i, 2]
                        - positions[j, 2]
                    )

                    values.append(
                        de ** 2
                        + dd ** 2
                    )

        if not values:

            return 0.0

        return (
            sum(values)
            / len(values)
        )

    # SAVE RESULT

    def save_result(
        self,
        test_name,
        start_positions,
        final_positions,
        duration,
        xy_mse_before=None,
        yz_mse_before=None,
        xy_mse_after=None,
        yz_mse_after=None,
        line_rmse=None,
        flag_direction=None,
        majority_count=None,
        minority_count=None,
    ):

        path = Path(
            self.results_csv
        )

        path.parent.mkdir(
            parents=True,
            exist_ok=True,
        )

        fields = [
            "timestamp",
            "test",
            "n_drones",
            "control_rate_hz",
            "max_horizontal_speed_mps",
            "max_vertical_speed_mps",
            "takeoff_altitude_m",
            "position_tolerance_m",
            "test_duration_s",
            "xy_mse_before_m2",
            "yz_mse_before_m2",
            "xy_mse_after_m2",
            "yz_mse_after_m2",
            "line_rmse_m",
            "flag_direction",
            "majority_count",
            "minority_count",
            "start_positions",
            "final_positions",
        ]

        exists = path.exists()

        row = {
            field: ""
            for field in fields
        }

        row["timestamp"] = (
            datetime.now().isoformat(
                timespec="seconds"
            )
        )

        row["test"] = test_name

        row["n_drones"] = (
            self.drone_count
        )

        row["control_rate_hz"] = (
            CONTROL_RATE_HZ
        )

        row["max_horizontal_speed_mps"] = (
            MAX_HORIZONTAL_SPEED_MPS
        )

        row["max_vertical_speed_mps"] = (
            MAX_VERTICAL_SPEED_MPS
        )

        row["takeoff_altitude_m"] = (
            TAKEOFF_ALTITUDE_M
        )

        row["position_tolerance_m"] = (
            POSITION_TOLERANCE_M
        )

        row["test_duration_s"] = (
            duration
        )

        row["xy_mse_before_m2"] = (
            xy_mse_before
        )

        row["yz_mse_before_m2"] = (
            yz_mse_before
        )

        row["xy_mse_after_m2"] = (
            xy_mse_after
        )

        row["yz_mse_after_m2"] = (
            yz_mse_after
        )

        row["line_rmse_m"] = (
            line_rmse
        )

        row["flag_direction"] = (
            flag_direction
        )

        row["majority_count"] = (
            majority_count
        )

        row["minority_count"] = (
            minority_count
        )

        row["start_positions"] = repr(
            np.round(
                start_positions,
                4,
            ).tolist()
        )

        row["final_positions"] = repr(
            np.round(
                final_positions,
                4,
            ).tolist()
        )

        with open(
            path,
            "a",
            newline="",
        ) as csv_file:

            writer = csv.DictWriter(
                csv_file,
                fieldnames=fields,
            )

            if (
                not exists
                or path.stat().st_size == 0
            ):

                writer.writeheader()

            writer.writerow(
                row
            )

        print()

        print(
            "[CSV] Results saved to:"
        )

        print(
            f"      {path}"
        )

    # INITIALIZE

    def initialize(
        self,
    ):

        print()

        print(
            "=" * 60
        )

        print(
            "INITIALIZING 3-DRONE PX4 SWARM"
        )

        print(
            "=" * 60
        )

        if not self.wait_for_positions():

            raise RuntimeError(
                "PX4 position telemetry did not connect."
            )

        if not self.wait_for_valid_position():

            raise RuntimeError(
                "PX4 local position is not valid."
            )

        print()

        print(
            "[INITIAL] Current state:"
        )

        self.print_positions()

        self.prepare_position_mode()

        self.stream_initial_setpoints()

        # OFFBOARD

        self.start_offboard()

        self.spin_for(
            0.5
        )

        # ARM

        if not self.arm_all():

            raise RuntimeError(
                "Not all drones entered the ARMED state."
            )

        # Takeoff

        if not self.wait_for_takeoff():

            raise RuntimeError(
                "Offboard position takeoff failed."
            )

        self.hold_position(
            TAKEOFF_HOLD_TIME_S
        )

        print()

        print(
            "=" * 60
        )

        print(
            "[READY] ALL DRONES ARE AIRBORNE."
        )

        print(
            "=" * 60
        )

        self.print_positions()

    # LAND ALL

    def land_all(
        self,
    ):

        print()

        print(
            "=" * 60
        )

        print(
            "[LAND] Landing all drones..."
        )

        print(
            "=" * 60
        )

        self.stop_all()

        self.spin_for(
            0.5
        )

        for i in range(
            self.drone_count
        ):

            self.land(i)

        self.spin_for(
            5.0
        )

    # RUN TEST

    def run_test(
        self,
        test_number,
    ):

        if test_number == 1:

            self.test_1()

        elif test_number == 2:

            self.test_2()

        elif test_number == 3:

            self.test_3()

        else:

            raise ValueError(
                "Test must be 1, 2, or 3."
            )


# MENU

def print_menu():

    print()

    print(
        "=" * 60
    )

    print(
        "DRONE SWARM TEST MENU"
    )

    print(
        "=" * 60
    )

    print(
        "1 = Test 1: Translate 3 m + XY/YZ MSE"
    )

    print(
        "2 = Test 2: Random positions -> straight line"
    )

    print(
        "3 = Test 3: Flag turn -> split -> translate 1 m"
    )

    print(
        "q = Quit"
    )

    print(
        "=" * 60
    )


# MAIN

def main(
    args=None,
):

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--n",
        type=int,
        default=DEFAULT_DRONE_COUNT,
    )

    parser.add_argument(
        "--seed",
        type=int,
        default=DEFAULT_RANDOM_SEED,
    )

    parser.add_argument(
        "--results-csv",
        type=str,
        default=DEFAULT_RESULTS_CSV,
    )

    parsed = parser.parse_args(
        args
    )

    if (
        parsed.n < 1
        or parsed.n > 3
    ):

        raise ValueError(
            "This controller currently supports "
            "1 to 3 drones."
        )

    print()

    print(
        "=" * 60
    )

    print(
        "PX4 / ROS 2 DRONE SWARM CONTROLLER"
    )

    print(
        "=" * 60
    )

    print(
        f"[CONFIG] Number of drones = "
        f"{parsed.n}"
    )

    print(
        f"[CONFIG] Control rate = "
        f"{CONTROL_RATE_HZ:.1f} Hz"
    )

    print(
        f"[CONFIG] Maximum horizontal velocity = "
        f"{MAX_HORIZONTAL_SPEED_MPS:.2f} m/s"
    )

    print(
        f"[CONFIG] Takeoff altitude = "
        f"{TAKEOFF_ALTITUDE_M:.2f} m"
    )

    print(
        "[CONFIG] Control interface = "
        "PX4 uXRCE-DDS"
    )

    print(
        "[CONFIG] Offboard initialization = "
        "POSITION"
    )

    print(
        "[CONFIG] Test control = "
        "VELOCITY"
    )

    print(
        "[CONFIG] PX4 namespaces:"
    )

    for i in range(
        parsed.n
    ):

        namespace = (
            PX4_NAMESPACES[i]
            if PX4_NAMESPACES[i]
            else "/"
        )

        print(
            f"           UAV {i}: "
            f"{namespace}"
        )

    print(
        "[CONFIG] VehicleStatus subscribed."
    )

    print(
        "[CONFIG] VehicleCommandAck subscribed."
    )

    print(
        "=" * 60
    )

    rclpy.init(
        args=args
    )

    node = SwarmController(
        drone_count=parsed.n,
        random_seed=parsed.seed,
        results_csv=parsed.results_csv,
    )

    try:

        node.initialize()

        while rclpy.ok():

            print_menu()

            selection = input(
                "Select a test: "
            ).strip().lower()

            if selection == "1":

                try:

                    node.run_test(
                        1
                    )

                except Exception as exc:

                    print()

                    print(
                        "=" * 60
                    )

                    print(
                        "[TEST ERROR]"
                    )

                    print(
                        "=" * 60
                    )

                    print(
                        str(exc)
                    )

                    node.stop_all()

            elif selection == "2":

                try:

                    node.run_test(
                        2
                    )

                except Exception as exc:

                    print()

                    print(
                        "=" * 60
                    )

                    print(
                        "[TEST ERROR]"
                    )

                    print(
                        "=" * 60
                    )

                    print(
                        str(exc)
                    )

                    node.stop_all()

            elif selection == "3":

                try:

                    node.run_test(
                        3
                    )

                except Exception as exc:

                    print()

                    print(
                        "=" * 60
                    )

                    print(
                        "[TEST ERROR]"
                    )

                    print(
                        "=" * 60
                    )

                    print(
                        str(exc)
                    )

                    node.stop_all()

            elif selection == "q":

                print()

                print(
                    "[QUIT] Leaving swarm controller."
                )

                break

            else:

                print()

                print(
                    "[ERROR] Invalid selection."
                )

    except KeyboardInterrupt:

        print()

        print(
            "[STOP] Ctrl+C detected."
        )

    finally:

        try:

            node.stop_all()

            node.spin_for(
                0.5
            )

            node.land_all()

        except Exception:

            pass

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


# ENTRY POINT

if __name__ == "__main__":

    main()
