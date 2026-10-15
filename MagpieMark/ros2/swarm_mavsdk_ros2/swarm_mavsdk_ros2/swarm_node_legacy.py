#!/usr/bin/env python3

# PX4 / ROS 2 3-DRONE SWARM CONTROLLER Direct ROS 2 / uXRCE-DDS control. PX4 instances: UAV 0
# -> / UAV 1 -> /px4_1 UAV 2 -> /px4_2 PX4 system IDs: UAV 0 -> 1 UAV 1 -> 2 UAV 2 -> 3
# INITIALIZATION: 1. Wait for all PX4 flight data. 2. Wait for valid local NED
# position/velocity. 3. Wait for estimator stability. 4. Capture each vehicle's actual local
# NED position. 5. Stream stationary position setpoints. 6. Request OFFBOARD. 7. Wait for
# OFFBOARD confirmation. 8. ARM all vehicles. 9. Wait for ARMED confirmation. 10. Command 2 m
# altitude in NED. 11. Wait until all vehicles are airborne. TEST 1: Translate all drones +3 m
# East. TEST 2: Move to randomized positions. Form a straight line. TEST 3: Form a line. Split
# around the flag direction. Translate both groups +1 m East

import argparse
import csv
import math
import random
import select
import sys
import time

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

DEFAULT_DRONES = 3

CONTROL_RATE_HZ = 20.0

CONTROL_PERIOD_S = (
    1.0 / CONTROL_RATE_HZ
)

# EKF / ESTIMATOR INITIALIZATION

TELEMETRY_TIMEOUT_S = 30.0

ESTIMATOR_STABILITY_S = 5.0

STABILITY_POSITION_TOLERANCE_M = 0.05

STABILITY_VELOCITY_TOLERANCE_MPS = 0.10

# Offboard startup

INITIAL_SETPOINT_COUNT = 60

OFFBOARD_CONFIRM_TIMEOUT_S = 5.0

ARM_CONFIRM_TIMEOUT_S = 10.0

# Takeoff

TAKEOFF_ALTITUDE_M = 2.0

TAKEOFF_TOLERANCE_M = 0.25

TAKEOFF_TIMEOUT_S = 30.0

TAKEOFF_HOLD_S = 3.0

# Control

MAX_HORIZONTAL_SPEED_MPS = 1.0

MAX_VERTICAL_SPEED_MPS = 0.6

POSITION_KP = 0.8

POSITION_TOLERANCE_M = 0.30

VELOCITY_SETTLE_MPS = 0.20

# TEST 1

TEST1_DISTANCE_M = 3.0

TEST1_SPEED_MPS = 0.75

TEST1_TIMEOUT_S = 10.0

# TEST 2

TEST2_RANDOM_RANGE_M = 2.0

TEST2_LINE_SPACING_M = 1.5

TEST2_TIMEOUT_S = 45.0

# TEST 3

TEST3_LINE_SPACING_M = 1.5

TEST3_TRANSLATE_M = 1.0

TEST3_TIMEOUT_S = 45.0

# Output

DEFAULT_SEED = None

DEFAULT_CSV = str(
    Path.home()
    / "px4_ros2_ws"
    / "swarm_test_results.csv"
)

# PX4 NAMESPACE / SYSTEM ID

PX4_NAMESPACES = [
    "",
    "/px4_1",
    "/px4_2",
]

PX4_SYSTEM_IDS = [
    1,
    2,
    3,
]


# PX4 COMMANDS

VEHICLE_CMD_COMPONENT_ARM_DISARM = 400

VEHICLE_CMD_DO_SET_MODE = 176

VEHICLE_CMD_NAV_LAND = 21


# PX4 NAVIGATION STATE

NAVIGATION_STATE_OFFBOARD = 14


# PX4 QoS

PX4_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


# DRONE STATE

class DroneState:

    def __init__(
        self,
        index,
        namespace,
        system_id,
    ):

        self.index = index

        self.namespace = namespace

        self.system_id = system_id

        # Position

        self.north = 0.0

        self.east = 0.0

        self.down = 0.0

        # Velocity

        self.vn = 0.0

        self.ve = 0.0

        self.vd = 0.0

        # Validity

        self.xy_valid = False

        self.z_valid = False

        self.vxy_valid = False

        self.vz_valid = False

        # Vehicle state

        self.armed = False

        self.nav_state = 0

        self.preflight_pass = False

        # Telemetry timing

        self.last_position_time = 0.0

        self.last_status_time = 0.0

        # Command acknowledgement

        self.last_ack_command = -1

        self.last_ack_result = -1

        self.last_ack_time = 0.0

        # Control mode

        self.position_mode = True

        # Stable reference

        self.reference_north = 0.0

        self.reference_east = 0.0

        self.reference_down = 0.0

        # Active targets

        self.target_north = 0.0

        self.target_east = 0.0

        self.target_down = 0.0

        self.target_vn = 0.0

        self.target_ve = 0.0

        self.target_vd = 0.0


# SWARM NODE

class SwarmNode(Node):

    def __init__(
        self,
        drone_count,
        seed,
        csv_path,
    ):

        super().__init__(
            "swarm_ros2_controller"
        )

        self.drone_count = (
            drone_count
        )

        self.seed = seed

        self.csv_path = Path(
            csv_path
        )

        self.drones = []

        self.offboard_publishers = []

        self.trajectory_publishers = []

        self.command_publishers = []

        self.position_subscribers = []

        self.status_subscribers = []

        self.ack_subscribers = []

        # Build all per-vehicle ROS interfaces

        for i in range(
            drone_count
        ):

            namespace = (
                PX4_NAMESPACES[i]
            )

            system_id = (
                PX4_SYSTEM_IDS[i]
            )

            state = DroneState(
                i,
                namespace,
                system_id,
            )

            self.drones.append(
                state
            )

            prefix = namespace

            # Topic names

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

            self.command_publishers.append(
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
                    lambda msg, idx=i:
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
                    lambda msg, idx=i:
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
                    lambda msg, idx=i:
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

        # Continuous 20 Hz Offboard stream

        self.timer = (
            self.create_timer(
                CONTROL_PERIOD_S,
                self.control_callback,
            )
        )


    # VEHICLE LOCAL POSITION

    def position_callback(
        self,
        index,
        msg,
    ):

        state = (
            self.drones[index]
        )

        state.north = float(
            msg.x
        )

        state.east = float(
            msg.y
        )

        state.down = float(
            msg.z
        )

        state.vn = float(
            msg.vx
        )

        state.ve = float(
            msg.vy
        )

        state.vd = float(
            msg.vz
        )

        state.xy_valid = bool(
            msg.xy_valid
        )

        state.z_valid = bool(
            msg.z_valid
        )

        state.vxy_valid = bool(
            msg.v_xy_valid
        )

        state.vz_valid = bool(
            msg.v_z_valid
        )

        state.last_position_time = (
            time.monotonic()
        )


    # VEHICLE STATUS

    def status_callback(
        self,
        index,
        msg,
    ):

        state = (
            self.drones[index]
        )

        state.armed = (
            int(msg.arming_state)
            == int(
                VehicleStatus
                .ARMING_STATE_ARMED
            )
        )

        state.nav_state = int(
            msg.nav_state
        )

        state.preflight_pass = bool(
            msg.pre_flight_checks_pass
        )

        state.last_status_time = (
            time.monotonic()
        )


    # COMMAND ACK

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
            f"ACK command={msg.command} "
            f"result={msg.result}"
            f"result_param1={msg.result_param1}"
            f"result_param2={msg.result_param2}"
        )


    # CONTINUOUS OFFBOARD PUBLISHER

    def control_callback(
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

            mode = (
                OffboardControlMode()
            )

            mode.position = bool(
                state.position_mode
            )

            mode.velocity = bool(
                not state.position_mode
            )

            mode.acceleration = False

            mode.attitude = False

            mode.body_rate = False

            mode.thrust_and_torque = False

            mode.direct_actuator = False

            mode.timestamp = timestamp

            self.offboard_publishers[i].publish(
                mode
            )

            # TRAJECTORY SETPOINT

            setpoint = (
                TrajectorySetpoint()
            )

            if state.position_mode:

                setpoint.position = [
                    float(
                        state.target_north
                    ),
                    float(
                        state.target_east
                    ),
                    float(
                        state.target_down
                    ),
                ]

                setpoint.velocity = [
                    math.nan,
                    math.nan,
                    math.nan,
                ]

            else:

                setpoint.position = [
                    math.nan,
                    math.nan,
                    math.nan,
                ]

                setpoint.velocity = [
                    float(
                        state.target_vn
                    ),
                    float(
                        state.target_ve
                    ),
                    float(
                        state.target_vd
                    ),
                ]

            setpoint.acceleration = [
                math.nan,
                math.nan,
                math.nan,
            ]

            setpoint.jerk = [
                math.nan,
                math.nan,
                math.nan,
            ]

            setpoint.yaw = 0.0

            setpoint.yawspeed = 0.0

            setpoint.timestamp = (
                timestamp
            )

            self.trajectory_publishers[i].publish(
                setpoint
            )


    # VEHICLE COMMAND

    def send_command(
        self,
        index,
        command,
        param1=0.0,
        param2=0.0,
    ):

        state = (
            self.drones[index]
        )

        msg = (
            VehicleCommand()
        )

        msg.timestamp = int(
            self.get_clock()
            .now()
            .nanoseconds
            // 1000
        )

        msg.command = int(
            command
        )

        msg.param1 = float(
            param1
        )

        msg.param2 = float(
            param2
        )

        # MULTI-VEHICLE ROUTING

        msg.target_system = int(
            state.system_id
        )

        msg.target_component = 1

        msg.source_system = 1

        msg.source_component = 1

        msg.from_external = True

        self.command_publishers[index].publish(
            msg
        )
        if command == VEHICLE_CMD_COMPONENT_ARM_DISARM:
            self.get_logger().info(
            f"UAV {index}: "
            f"ARM command "
            f"param1={msg.param1} "
            f"param2={msg.param2} "
            f"target_system={msg.target_system}")


    # REQUEST OFFBOARD

    def request_offboard(
        self,
        index,
    ):

        self.send_command(
            index,
            VEHICLE_CMD_DO_SET_MODE,
            1.0,
            6.0,
        )


    # ARM

    def arm(
        self,
        index,
    ):

        self.send_command(
            index,
            VEHICLE_CMD_COMPONENT_ARM_DISARM,
            1.0,
            0.0,
        )


    # LAND

    def land(
        self,
        index,
    ):

        self.send_command(
            index,
            VEHICLE_CMD_NAV_LAND,
        )


    # SET POSITION TARGET

    def set_position_target(
        self,
        index,
        north,
        east,
        down,
    ):

        state = (
            self.drones[index]
        )

        state.position_mode = True

        state.target_north = float(
            north
        )

        state.target_east = float(
            east
        )

        state.target_down = float(
            down
        )


    # SET VELOCITY TARGET

    def set_velocity_target(
        self,
        index,
        vn,
        ve,
        vd,
    ):

        horizontal_speed = math.sqrt(
            vn ** 2
            + ve ** 2
        )

        if (
            horizontal_speed
            > MAX_HORIZONTAL_SPEED_MPS
        ):

            scale = (
                MAX_HORIZONTAL_SPEED_MPS
                / horizontal_speed
            )

            vn *= scale

            ve *= scale

        vd = max(
            -MAX_VERTICAL_SPEED_MPS,
            min(
                MAX_VERTICAL_SPEED_MPS,
                vd,
            ),
        )

        state = (
            self.drones[index]
        )

        state.position_mode = False

        state.target_vn = float(
            vn
        )

        state.target_ve = float(
            ve
        )

        state.target_vd = float(
            vd
        )


    # STOP

    def stop_all(
        self,
    ):

        for state in self.drones:

            state.target_vn = 0.0

            state.target_ve = 0.0

            state.target_vd = 0.0


    # SPIN

    def spin_for(
        self,
        duration_s,
    ):

        end = (
            time.monotonic()
            + duration_s
        )

        while (
            time.monotonic()
            < end
        ):

            rclpy.spin_once(
                self,
                timeout_sec=0.02,
            )


    # TELEMETRY WAIT

    def wait_for_telemetry(
        self,
    ):

        print()

        print(
            "------------------------------------------------------------"
        )

        print(
            "[INIT] Waiting for all PX4 position streams..."
        )

        print(
            "------------------------------------------------------------"
        )

        start = time.monotonic()

        while (
            time.monotonic()
            - start
            < TELEMETRY_TIMEOUT_S
        ):

            rclpy.spin_once(
                self,
                timeout_sec=0.05,
            )

            count = sum(
                state.last_position_time > 0.0
                for state in self.drones
            )

            print(
                f"\r[INIT] Telemetry "
                f"{count}/{self.drone_count}",
                end="",
                flush=True,
            )

            if (
                count
                == self.drone_count
            ):

                print()

                print(
                    "[INIT] All position streams connected."
                )

                return True

        print()

        return False


    # VALID POSITION / VELOCITY

    def wait_for_valid_estimates(
        self,
    ):

        print()

        print(
            "[INIT] Waiting for valid "
            "position AND velocity estimates..."
        )

        start = time.monotonic()

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
                (
                    state.xy_valid
                    and state.z_valid
                    and state.vxy_valid
                    and state.vz_valid
                )
                for state in self.drones
            )

            if valid:

                print(
                    "[INIT] All position and "
                    "velocity estimates are valid."
                )

                return True

        print(
            "[INIT] Estimate validity timeout."
        )

        self.print_state()

        return False


    # ESTIMATOR STABILITY

    def wait_for_estimator_stability(
        self,
    ):

        print()

        print(
            "[INIT] Waiting for estimator stability..."
        )

        stable_start = None

        reference = None

        while True:

            rclpy.spin_once(
                self,
                timeout_sec=0.02,
            )

            valid = all(
                (
                    state.xy_valid
                    and state.z_valid
                    and state.vxy_valid
                    and state.vz_valid
                )
                for state in self.drones
            )

            if not valid:

                stable_start = None

                reference = None

                continue

            current = np.array(
                [
                    [
                        state.north,
                        state.east,
                        state.down,
                    ]
                    for state in self.drones
                ],
                dtype=float,
            )

            velocity = np.array(
                [
                    [
                        state.vn,
                        state.ve,
                        state.vd,
                    ]
                    for state in self.drones
                ],
                dtype=float,
            )

            if reference is None:

                reference = current.copy()

                stable_start = time.monotonic()

                print(
                    "[INIT] Stable reference "
                    "candidate captured."
                )

                continue

            position_change = (
                np.linalg.norm(
                    current
                    - reference,
                    axis=1,
                )
            )

            speed = (
                np.linalg.norm(
                    velocity,
                    axis=1,
                )
            )

            position_stable = np.all(
                position_change
                <= STABILITY_POSITION_TOLERANCE_M
            )

            velocity_stable = np.all(
                speed
                <= STABILITY_VELOCITY_TOLERANCE_MPS
            )

            if (
                position_stable
                and velocity_stable
            ):

                stable_duration = (
                    time.monotonic()
                    - stable_start
                )

                if (
                    stable_duration
                    >= ESTIMATOR_STABILITY_S
                ):

                    print(
                        "[INIT] Estimator stable for "
                        f"{ESTIMATOR_STABILITY_S:.1f} s."
                    )

                    return True

            else:

                reference = current.copy()

                stable_start = time.monotonic()


    # CAPTURE REFERENCE

    def capture_reference(
        self,
    ):

        print()

        print(
            "[INIT] Capturing local NED references..."
        )

        for state in self.drones:

            state.reference_north = (
                state.north
            )

            state.reference_east = (
                state.east
            )

            state.reference_down = (
                state.down
            )

            state.target_north = (
                state.reference_north
            )

            state.target_east = (
                state.reference_east
            )

            state.target_down = (
                state.reference_down
                - TAKEOFF_ALTITUDE_M
            )

        self.print_state()


    # INITIAL OFFBOARD STREAM

    def stream_initial_setpoints(
        self,
    ):

        print()

        print(
            "[INIT] Streaming stationary NED "
            "position setpoints..."
        )

        for state in self.drones:

            state.position_mode = True

            state.target_north = (
                state.reference_north
            )

            state.target_east = (
                state.reference_east
            )

            state.target_down = (
                state.reference_down
            )

        self.spin_for(
            INITIAL_SETPOINT_COUNT
            * CONTROL_PERIOD_S
        )


    # REQUEST OFFBOARD

    def enter_offboard(
        self,
    ):

        print()

        print(
            "[OFFBOARD] Requesting OFFBOARD..."
        )

        for i in range(
            self.drone_count
        ):

            self.request_offboard(
                i
            )

        start = time.monotonic()

        while (
            time.monotonic()
            - start
            < OFFBOARD_CONFIRM_TIMEOUT_S
        ):

            rclpy.spin_once(
                self,
                timeout_sec=0.02,
            )

            if all(
                state.nav_state
                == NAVIGATION_STATE_OFFBOARD
                for state in self.drones
            ):

                print(
                    "[OFFBOARD] "
                    "All drones report OFFBOARD."
                )

                return True

        print(
            "[OFFBOARD] OFFBOARD confirmation timeout."
        )

        self.print_state()

        return False


    # ARM ALL

    def arm_all(
        self,
    ):

        print()

        print(
            "[ARM] Arming all drones..."
        )

        for i in range(
            self.drone_count
        ):

            self.arm(i)

        start = time.monotonic()

        while (
            time.monotonic()
            - start
            < ARM_CONFIRM_TIMEOUT_S
        ):

            rclpy.spin_once(
                self,
                timeout_sec=0.02,
            )

            armed = sum(
                state.armed
                for state in self.drones
            )

            print(
                f"\r[ARM] Armed "
                f"{armed}/{self.drone_count}",
                end="",
                flush=True,
            )

            if (
                armed
                == self.drone_count
            ):

                print()

                print(
                    "[ARM] All drones report ARMED."
                )

                return True

        print()

        print(
            "[ARM] ARM confirmation timeout."
        )

        self.print_state()

        return False


    # TAKEOFF

    def takeoff(
        self,
    ):

        print()

        print(
            "=" * 60
        )

        print(
            "[TAKEOFF] Commanding "
            f"{TAKEOFF_ALTITUDE_M:.1f} m altitude"
        )

        print(
            "=" * 60
        )

        for state in self.drones:

            state.position_mode = True

            state.target_north = (
                state.reference_north
            )

            state.target_east = (
                state.reference_east
            )

            state.target_down = (
                state.reference_down
                - TAKEOFF_ALTITUDE_M
            )

        start = time.monotonic()

        next_print = start

        while (
            time.monotonic()
            - start
            < TAKEOFF_TIMEOUT_S
        ):

            rclpy.spin_once(
                self,
                timeout_sec=0.02,
            )

            now = time.monotonic()

            all_reached = all(
                (
                    -state.down
                    >=
                    -state.reference_down
                    + TAKEOFF_ALTITUDE_M
                    - TAKEOFF_TOLERANCE_M
                )
                for state in self.drones
            )

            # More robust altitude test

            all_reached = all(
                abs(
                    state.down
                    - state.target_down
                )
                <= TAKEOFF_TOLERANCE_M
                for state in self.drones
            )

            if all_reached:

                print()

                print(
                    "[TAKEOFF] All drones reached "
                    "the commanded altitude."
                )

                return True

            if now >= next_print:

                print(
                    f"[TAKEOFF] "
                    f"t={now - start:.1f} s | "
                    + " | ".join(
                        (
                            f"UAV {state.index}: "
                            f"alt="
                            f"{-state.down:.2f}"
                        )
                        for state in self.drones
                    )
                )

                next_print = (
                    now + 1.0
                )

        print()

        print(
            "[TAKEOFF] TIMEOUT."
        )

        self.print_state()

        return False


    # HOLD

    def hold(
        self,
        duration_s,
    ):

        print()

        print(
            f"[HOLD] Holding for "
            f"{duration_s:.1f} s..."
        )

        self.spin_for(
            duration_s
        )


    # ENTER VELOCITY MODE

    def enter_velocity_mode(
        self,
    ):

        for state in self.drones:

            state.position_mode = False

            state.target_vn = 0.0

            state.target_ve = 0.0

            state.target_vd = 0.0

        self.spin_for(
            1.0
        )

        print(
            "[CONTROL] Velocity Offboard active."
        )


    # MOVE TO POSITION USING VELOCITY CONTROL

    def move_to_targets(
        self,
        targets,
        timeout_s,
        label,
    ):

        print()

        print(
            f"[{label}] Moving to target positions..."
        )

        start = time.monotonic()

        while (
            time.monotonic()
            - start
            < timeout_s
        ):

            all_reached = True

            for i, target in enumerate(
                targets
            ):

                state = (
                    self.drones[i]
                )

                error_n = (
                    target[0]
                    - state.north
                )

                error_e = (
                    target[1]
                    - state.east
                )

                error_d = (
                    target[2]
                    - state.down
                )

                distance = math.sqrt(
                    error_n ** 2
                    + error_e ** 2
                    + error_d ** 2
                )

                if (
                    distance
                    > POSITION_TOLERANCE_M
                ):

                    all_reached = False

                    self.set_velocity_target(
                        i,
                        POSITION_KP
                        * error_n,
                        POSITION_KP
                        * error_e,
                        POSITION_KP
                        * error_d,
                    )

                else:

                    self.set_velocity_target(
                        i,
                        0.0,
                        0.0,
                        0.0,
                    )

            if all_reached:

                self.stop_all()

                self.spin_for(
                    0.5
                )

                return True

            rclpy.spin_once(
                self,
                timeout_sec=0.02,
            )

        self.stop_all()

        print(
            f"[{label}] TIMEOUT."
        )

        self.print_state()

        return False


    # TEST 1

    def test_1(
        self,
    ):

        print()

        print(
            "=" * 60
        )

        print(
            "TEST 1 — 3 M EAST TRANSLATION"
        )

        print(
            "=" * 60
        )

        start_positions = (
            self.positions()
        )

        xy_before = (
            self.pairwise_mse("xy")
        )

        yz_before = (
            self.pairwise_mse("yz")
        )

        # PX4 NED convention: +X = North +Y = East +Z = Down Therefore +3 m East is +3 m in Y

        targets = (
            start_positions.copy()
        )

        targets[:, 1] += (
            TEST1_DISTANCE_M
        )

        test_start = time.monotonic()

        success = self.move_to_targets(
            targets,
            TEST1_TIMEOUT_S,
            "TEST 1",
        )

        duration = (
            time.monotonic()
            - test_start
        )

        if not success:

            raise RuntimeError(
                "TEST 1 failed to reach "
                "the requested East target."
            )

        # Give the vehicles a short settling interval while continuing the 20 Hz Offboard stream

        self.spin_for(
            1.0
        )

        final_positions = (
            self.positions()
        )

        xy_after = (
            self.pairwise_mse("xy")
        )

        yz_after = (
            self.pairwise_mse("yz")
        )

        displacement = (
            np.linalg.norm(
                final_positions
                - start_positions,
                axis=1,
            )
        )

        east_displacement = (
            final_positions[:, 1]
            - start_positions[:, 1]
        )

        north_drift = (
            final_positions[:, 0]
            - start_positions[:, 0]
        )

        down_drift = (
            final_positions[:, 2]
            - start_positions[:, 2]
        )

        target_error = (
            np.linalg.norm(
                final_positions
                - targets,
                axis=1,
            )
        )

        print()

        print(
            "[TEST 1] RESULTS"
        )

        print(
            f"Requested East translation = "
            f"{TEST1_DISTANCE_M:.3f} m"
        )

        print(
            f"Test duration = "
            f"{duration:.3f} s"
        )

        print(
            f"XY MSE before = "
            f"{xy_before:.6f}"
        )

        print(
            f"YZ MSE before = "
            f"{yz_before:.6f}"
        )

        print(
            f"XY MSE after = "
            f"{xy_after:.6f}"
        )

        print(
            f"YZ MSE after = "
            f"{yz_after:.6f}"
        )

        for i in range(
            self.drone_count
        ):

            print(
                f"UAV {i}: "
                f"East={east_displacement[i]:+.3f} m | "
                f"North drift={north_drift[i]:+.3f} m | "
                f"Down drift={down_drift[i]:+.3f} m | "
                f"displacement={displacement[i]:.3f} m | "
                f"target error={target_error[i]:.3f} m"
            )

        self.save_result(
            "TEST 1 — 3 M EAST TRANSLATION",
            start_positions,
            final_positions,
            duration,
            xy_before,
            yz_before,
            xy_after,
            yz_after,
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
            self.seed
        )

        start_positions = (
            self.positions()
        )

        random_targets = []

        for state in self.drones:

            random_targets.append(
                [
                    state.north
                    + rng.uniform(
                        -TEST2_RANDOM_RANGE_M,
                        TEST2_RANDOM_RANGE_M,
                    ),
                    state.east
                    + rng.uniform(
                        -TEST2_RANDOM_RANGE_M,
                        TEST2_RANDOM_RANGE_M,
                    ),
                    state.down,
                ]
            )

        random_targets = np.array(
            random_targets,
            dtype=float,
        )

        random_start = time.monotonic()

        if not self.move_to_targets(
            random_targets,
            TEST2_TIMEOUT_S,
            "TEST 2 RANDOM",
        ):

            raise RuntimeError(
                "TEST 2 random positioning failed."
            )

        random_duration = (
            time.monotonic()
            - random_start
        )

        self.spin_for(
            0.5
        )

        current = (
            self.positions()
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
                    self.drone_count - 1
                )
                / 2.0
            ) * TEST2_LINE_SPACING_M

            line_targets.append(
                [
                    center_n,
                    center_e + offset,
                    center_d,
                ]
            )

        line_targets = np.array(
            line_targets,
            dtype=float,
        )

        line_start = time.monotonic()

        if not self.move_to_targets(
            line_targets,
            TEST2_TIMEOUT_S,
            "TEST 2 LINE",
        ):

            raise RuntimeError(
                "TEST 2 line formation failed."
            )

        line_duration = (
            time.monotonic()
            - line_start
        )

        final_positions = (
            self.positions()
        )

        errors = np.linalg.norm(
            final_positions
            - line_targets,
            axis=1,
        )

        rmse = math.sqrt(
            np.mean(
                errors ** 2
            )
        )

        print()

        print(
            f"[TEST 2] Random-position duration = "
            f"{random_duration:.3f} s"
        )

        print(
            f"[TEST 2] Line-formation duration = "
            f"{line_duration:.3f} s"
        )

        print(
            f"[TEST 2] Line RMSE = "
            f"{rmse:.3f} m"
        )

        self.save_result(
            "TEST 2 — RANDOM POSITIONS -> LINE",
            start_positions,
            final_positions,
            0.0,
            line_rmse=rmse,
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
            "TEST 3 — FLAG / SPLIT / TRANSLATE"
        )

        print(
            "=" * 60
        )

        rng = random.Random(
            self.seed
        )

        direction = rng.choice(
            [
                "LEFT",
                "RIGHT",
            ]
        )

        start_positions = (
            self.positions()
        )

        center_n = np.mean(
            start_positions[:, 0]
        )

        center_e = np.mean(
            start_positions[:, 1]
        )

        center_d = np.mean(
            start_positions[:, 2]
        )

        line_targets = []

        for i in range(
            self.drone_count
        ):

            offset = (
                i
                - (
                    self.drone_count - 1
                )
                / 2.0
            ) * TEST3_LINE_SPACING_M

            line_targets.append(
                [
                    center_n + offset,
                    center_e,
                    center_d,
                ]
            )

        line_targets = np.array(
            line_targets,
            dtype=float,
        )

        line_start = time.monotonic()

        if not self.move_to_targets(
            line_targets,
            TEST3_TIMEOUT_S,
            "TEST 3 LINE",
        ):

            raise RuntimeError(
                "TEST 3 line formation failed."
            )

        line_duration = (
            time.monotonic()
            - line_start
        )

        majority_count = (
            self.drone_count // 2
            + 1
        )

        minority_count = (
            self.drone_count
            - majority_count
        )

        split_targets = (
            line_targets.copy()
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
                    * TEST3_LINE_SPACING_M
                )

            else:

                split_targets[i, 0] += (
                    minority_sign
                    * TEST3_LINE_SPACING_M
                )

        split_start = time.monotonic()

        if not self.move_to_targets(
            split_targets,
            TEST3_TIMEOUT_S,
            "TEST 3 SPLIT",
        ):

            raise RuntimeError(
                "TEST 3 split failed."
            )

        split_duration = (
            time.monotonic()
            - split_start
        )

        translation_targets = (
            split_targets.copy()
        )

        translation_targets[:, 1] += (
            TEST3_TRANSLATE_M
        )

        translation_start = (
            time.monotonic()
        )

        if not self.move_to_targets(
            translation_targets,
            TEST3_TIMEOUT_S,
            "TEST 3 TRANSLATE",
        ):

            raise RuntimeError(
                "TEST 3 translation failed."
            )

        translation_time = (
            time.monotonic()
            - translation_start
        )

        final_positions = (
            self.positions()
        )

        self.save_result(
            "TEST 3 — FLAG / SPLIT / TRANSLATE",
            start_positions,
            final_positions,
            translation_time,
            flag_direction=direction,
            majority_count=majority_count,
            minority_count=minority_count,
        )

        print()

        final_error = np.linalg.norm(
            final_positions
            - translation_targets,
            axis=1,
        )

        print(
            f"[TEST 3] Flag direction = "
            f"{direction}"
        )

        print(
            f"[TEST 3] Line duration = "
            f"{line_duration:.3f} s"
        )

        print(
            f"[TEST 3] Split duration = "
            f"{split_duration:.3f} s"
        )

        print(
            f"[TEST 3] Translation time = "
            f"{translation_time:.3f} s"
        )

        print(
            f"[TEST 3] Final target RMSE = "
            f"{math.sqrt(np.mean(final_error ** 2)):.3f} m"
        )


    # POSITIONS

    def positions(
        self,
    ):

        return np.array(
            [
                [
                    state.north,
                    state.east,
                    state.down,
                ]
                for state in self.drones
            ],
            dtype=float,
        )


    # PAIRWISE MSE

    def pairwise_mse(
        self,
        plane,
    ):

        positions = (
            self.positions()
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

                    values.append(
                        (
                            positions[i, 0]
                            - positions[j, 0]
                        ) ** 2
                        +
                        (
                            positions[i, 1]
                            - positions[j, 1]
                        ) ** 2
                    )

                else:

                    values.append(
                        (
                            positions[i, 1]
                            - positions[j, 1]
                        ) ** 2
                        +
                        (
                            positions[i, 2]
                            - positions[j, 2]
                        ) ** 2
                    )

        if not values:

            return 0.0

        return (
            sum(values)
            / len(values)
        )


    # PRINT STATE

    def print_state(
        self,
    ):

        for state in self.drones:

            speed = math.sqrt(
                state.vn ** 2
                + state.ve ** 2
                + state.vd ** 2
            )

            print(
                f"UAV {state.index}: "
                f"armed={state.armed} | "
                f"nav={state.nav_state} | "
                f"preflight={state.preflight_pass} | "
                f"N={state.north:+.3f} | "
                f"E={state.east:+.3f} | "
                f"D={state.down:+.3f} | "
                f"V={speed:.3f}"
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

        self.csv_path.parent.mkdir(
            parents=True,
            exist_ok=True,
        )

        fields = [
            "timestamp",
            "test",
            "n_drones",
            "duration_s",
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

        exists = (
            self.csv_path.exists()
        )

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

        row["duration_s"] = (
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
            self.csv_path,
            "a",
            newline="",
        ) as file:

            writer = csv.DictWriter(
                file,
                fieldnames=fields,
            )

            if not exists:

                writer.writeheader()

            writer.writerow(
                row
            )


    # INITIALIZATION STATE MACHINE

    def initialize(
        self,
    ):

        print()

        print(
            "=" * 60
        )

        print(
            "INITIALIZING 3-DRONE ROS 2 SWARM"
        )

        print(
            "=" * 60
        )

        # STATE 1: TELEMETRY

        if not self.wait_for_telemetry():

            raise RuntimeError(
                "Telemetry initialization failed."
            )

        # STATE 2: ESTIMATOR VALIDITY

        if not self.wait_for_valid_estimates():

            raise RuntimeError(
                "Estimator validity initialization failed."
            )

        # STATE 3: ESTIMATOR STABILITY

        if not self.wait_for_estimator_stability():

            raise RuntimeError(
                "Estimator stability failed."
            )

        # STATE 4: CAPTURE NED REFERENCE

        self.capture_reference()

        # STATE 5: STATIONARY OFFBOARD PREFLIGHT STREAM

        self.stream_initial_setpoints()

        # STATE 6: OFFBOARD

        if not self.enter_offboard():

            raise RuntimeError(
                "Not all drones entered OFFBOARD."
            )

        # STATE 7: ARM

        if not self.arm_all():

            raise RuntimeError(
                "Not all drones armed."
            )

        # STATE 8: TAKEOFF

        if not self.takeoff():

            raise RuntimeError(
                "Takeoff failed."
            )

        # STATE 9: HOLD

        self.hold(
            TAKEOFF_HOLD_S
        )

        # STATE 10: VELOCITY MODE

        self.enter_velocity_mode()

        print()

        print(
            "=" * 60
        )

        print(
            "[READY] ALL THREE DRONES AIRBORNE"
        )

        print(
            "=" * 60
        )

        self.print_state()


    # LAND ALL

    def land_all(
        self,
    ):

        print()

        print(
            "[LAND] Sending LAND commands..."
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
            8.0
        )


# MAIN

def main():

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--n",
        type=int,
        default=DEFAULT_DRONES,
    )

    parser.add_argument(
        "--seed",
        type=int,
        default=DEFAULT_SEED,
    )

    parser.add_argument(
        "--results-csv",
        type=str,
        default=DEFAULT_CSV,
    )

    args = parser.parse_args()

    if args.n < 1 or args.n > 3:

        raise ValueError(
            "This controller supports 1-3 drones."
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
        f"[CONFIG] Drones = {args.n}"
    )

    print(
        f"[CONFIG] Control rate = "
        f"{CONTROL_RATE_HZ:.1f} Hz"
    )

    print(
        "[CONFIG] Initialization = "
        "EKF stability + local NED reference"
    )

    print(
        "[CONFIG] Takeoff = "
        "position Offboard"
    )

    print(
        "[CONFIG] Experiments = "
        "velocity Offboard"
    )

    rclpy.init()

    node = SwarmNode(
        drone_count=args.n,
        seed=args.seed,
        csv_path=args.results_csv,
    )

    try:

        node.initialize()

        # MENU

        while rclpy.ok():

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
                "1 = 3 m East translation"
            )

            print(
                "2 = Random positions -> straight line"
            )

            print(
                "3 = Flag -> split -> translate"
            )

            print(
                "q = Quit"
            )

            print(
                "=" * 60
            )

            print(
                "Select a test: ",
                end="",
                flush=True,
            )

            selection = None

            while (
                selection is None
                and rclpy.ok()
            ):

                # Keep the ROS 2 executor spinning while waiting for keyboard
                # input so the 20 Hz Offboard heartbeat continues
                # uninterrupted

                readable, _, _ = select.select(
                    [sys.stdin],
                    [],
                    [],
                    0.10,
                )

                rclpy.spin_once(
                    node,
                    timeout_sec=0.02,
                )

                if readable:

                    selection = (
                        sys.stdin.readline()
                        .strip()
                        .lower()
                    )

            if selection == "1":

                try:

                    node.test_1()

                except Exception as exc:

                    print()

                    print(
                        f"[TEST 1 ERROR] {exc}"
                    )

                    node.stop_all()

            elif selection == "2":

                try:

                    node.test_2()

                except Exception as exc:

                    print()

                    print(
                        f"[TEST 2 ERROR] {exc}"
                    )

                    node.stop_all()

            elif selection == "3":

                try:

                    node.test_3()

                except Exception as exc:

                    print()

                    print(
                        f"[TEST 3 ERROR] {exc}"
                    )

                    node.stop_all()

            elif selection == "q":

                break

            else:

                print(
                    "[ERROR] Invalid selection."
                )

    except KeyboardInterrupt:

        print(
            "[STOP] Ctrl+C."
        )

    except Exception as exc:

        print()

        print(
            "=" * 60
        )

        print(
            "[CONTROLLER ERROR]"
        )

        print(
            "=" * 60
        )

        print(
            str(exc)
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


# RUN

if __name__ == "__main__":

    main()
