#!/usr/bin/env python3

# PX4 / ROS 2 3-DRONE SWARM CONTROLLER Direct ROS 2 / uXRCE-DDS control. PX4 instances: UAV 0
# -> / UAV 1 -> /px4_1 UAV 2 -> /px4_2 PX4 system IDs: UAV 0 -> 1 UAV 1 -> 2 UAV 2 -> 3
# CONTROL: 1. Stream position Offboard setpoints. 2. Request Offboard. 3. Arm. 4. Command a 2
# m NED position. 5. Verify all drones are airborne. 6. Switch to velocity Offboard control
# TEST 1: +3 m East translation. TEST 2: Random positions -> straight line. TEST 3: Straight
# line -> split -> +1 m East

import argparse
import csv
import math
import random
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

CONTROL_PERIOD_S = 1.0 / CONTROL_RATE_HZ

TAKEOFF_ALTITUDE_M = 2.0

TAKEOFF_TOLERANCE_M = 0.25

TAKEOFF_TIMEOUT_S = 25.0

TAKEOFF_HOLD_S = 2.0

POSITION_TOLERANCE_M = 0.30

POSITION_TIMEOUT_S = 30.0

MAX_HORIZONTAL_SPEED_MPS = 1.50

MAX_VERTICAL_SPEED_MPS = 0.70

POSITION_KP = 0.80

TEST1_DISTANCE_M = 3.0

TEST1_SPEED_MPS = 1.0

TEST1_DURATION_S = (
    TEST1_DISTANCE_M
    / TEST1_SPEED_MPS
)

TEST2_RANDOM_RANGE_M = 2.0

TEST2_LINE_SPACING_M = 1.5

TEST2_TIMEOUT_S = 45.0

TEST3_GROUP_SPACING_M = 1.5

TEST3_TRANSLATE_M = 1.0

TEST3_TIMEOUT_S = 45.0

INITIAL_SETPOINT_COUNT = 30

OFFBOARD_ATTEMPTS = 5

ARM_ATTEMPTS = 10

COMMAND_INTERVAL_S = 0.25

TELEMETRY_TIMEOUT_S = 15.0

DEFAULT_SEED = None

DEFAULT_CSV = str(
    Path.home()
    / "px4_ros2_ws"
    / "swarm_test_results.csv"
)


# PX4 NAMESPACE

PX4_NAMESPACES = [
    "",
    "/px4_1",
    "/px4_2",
]


# PX4 SYSTEM IDs

PX4_SYSTEM_IDS = [
    1,
    2,
    3,
]


# PX4 COMMANDS

VEHICLE_CMD_COMPONENT_ARM_DISARM = 400

VEHICLE_CMD_DO_SET_MODE = 176

VEHICLE_CMD_NAV_LAND = 21


# ROS 2 QoS

PX4_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
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

        # Vehicle status

        self.armed = False

        self.nav_state = 0

        self.preflight_pass = False

        # ACK

        self.last_ack_command = -1

        self.last_ack_result = -1

        self.last_ack_time = 0.0

        # Telemetry time

        self.last_position_time = 0.0

        self.last_status_time = 0.0

        # Control

        self.position_mode = True

        self.target_north = 0.0

        self.target_east = 0.0

        self.target_down = -TAKEOFF_ALTITUDE_M

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

        self.drone_count = drone_count

        self.seed = seed

        self.csv_path = Path(csv_path)

        self.drones = []

        self.offboard_publishers = []

        self.trajectory_publishers = []

        self.command_publishers = []

        self.position_subscribers = []

        self.status_subscribers = []

        self.ack_subscribers = []

        # Build interfaces

        for i in range(drone_count):

            namespace = PX4_NAMESPACES[i]

            system_id = PX4_SYSTEM_IDS[i]

            state = DroneState(
                i,
                namespace,
                system_id,
            )

            self.drones.append(state)

            prefix = namespace

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

        # 20 Hz continuous control loop

        self.timer = self.create_timer(
            CONTROL_PERIOD_S,
            self.control_callback,
        )

    # POSITION CALLBACK

    def position_callback(
        self,
        index,
        msg,
    ):

        state = self.drones[index]

        state.north = float(msg.x)

        state.east = float(msg.y)

        state.down = float(msg.z)

        state.vn = float(msg.vx)

        state.ve = float(msg.vy)

        state.vd = float(msg.vz)

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

    # STATUS CALLBACK

    def status_callback(
        self,
        index,
        msg,
    ):

        state = self.drones[index]

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

    # ACK CALLBACK

    def ack_callback(
        self,
        index,
        msg,
    ):

        state = self.drones[index]

        state.last_ack_command = (
            int(msg.command)
        )

        state.last_ack_result = (
            int(msg.result)
        )

        state.last_ack_time = (
            time.monotonic()
        )

        self.get_logger().info(
            f"UAV {index}: "
            f"ACK command={msg.command} "
            f"result={msg.result}"
        )

    # CONTINUOUS OFFBOARD STREAM

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

            state = self.drones[i]

            # OffboardControlMode

            mode = (
                OffboardControlMode()
            )

            mode.position = (
                state.position_mode
            )

            mode.velocity = (
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

            # TrajectorySetpoint

            setpoint = (
                TrajectorySetpoint()
            )

            if state.position_mode:

                setpoint.position = [
                    state.target_north,
                    state.target_east,
                    state.target_down,
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
                    state.target_vn,
                    state.target_ve,
                    state.target_vd,
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

            setpoint.yaw = (
                math.radians(90.0)
            )

            setpoint.yawspeed = 0.0

            setpoint.timestamp = timestamp

            self.trajectory_publishers[i].publish(
                setpoint
            )

    # SEND VEHICLE COMMAND

    def send_command(
        self,
        index,
        command,
        param1=0.0,
        param2=0.0,
    ):

        state = self.drones[index]

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

        # CRITICAL MULTI-VEHICLE ROUTING

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

    # STOP VELOCITY

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

            altitude = -state.down

            print(
                f"UAV {state.index}: "
                f"armed={state.armed} | "
                f"nav={state.nav_state} | "
                f"preflight={state.preflight_pass} | "
                f"N={state.north:+.2f} | "
                f"E={state.east:+.2f} | "
                f"alt={altitude:+.2f} | "
                f"speed={speed:.2f}"
            )

    # WAIT FOR TELEMETRY

    def wait_for_telemetry(
        self,
    ):

        print()

        print(
            "------------------------------------------------------------"
        )

        print(
            "[ROS 2] Waiting for telemetry..."
        )

        print(
            "------------------------------------------------------------"
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

            count = sum(
                state.last_position_time > 0.0
                for state in self.drones
            )

            print(
                f"\r[ROS 2] Telemetry "
                f"{count}/{self.drone_count}",
                end="",
                flush=True,
            )

            if count == self.drone_count:

                print()

                print(
                    "[ROS 2] All telemetry streams connected."
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
            "[HEALTH] Waiting for valid position estimates..."
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
                    "[HEALTH] All position estimates valid."
                )

                return True

        print(
            "[HEALTH] Position validation timed out."
        )

        self.print_state()

        return False

    # STREAM SETPOINTS

    def stream_initial_setpoints(
        self,
    ):

        print()

        print(
            "[OFFBOARD] Streaming "
            f"{INITIAL_SETPOINT_COUNT} position setpoints..."
        )

        # Current horizontal position. Command target altitude in NED

        for state in self.drones:

            state.position_mode = True

            state.target_north = state.north

            state.target_east = state.east

            state.target_down = (
                -TAKEOFF_ALTITUDE_M
            )

        self.spin_for(
            INITIAL_SETPOINT_COUNT
            / CONTROL_RATE_HZ
        )

    # START OFFBOARD

    def start_offboard(
        self,
    ):

        print()

        print(
            "[OFFBOARD] Requesting OFFBOARD..."
        )

        for attempt in range(
            OFFBOARD_ATTEMPTS
        ):

            print(
                f"[OFFBOARD] Attempt "
                f"{attempt + 1}/"
                f"{OFFBOARD_ATTEMPTS}"
            )

            for i in range(
                self.drone_count
            ):

                self.request_offboard(
                    i
                )

            self.spin_for(
                COMMAND_INTERVAL_S
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
            ARM_ATTEMPTS
        ):

            for i in range(
                self.drone_count
            ):

                self.arm(i)

            self.spin_for(
                COMMAND_INTERVAL_S
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
            "[TAKEOFF] Position Offboard takeoff"
        )

        print(
            "=" * 60
        )

        # IMPORTANT: The actual takeoff command is the POSITION setpoint streamed
        # through TrajectorySetpoint. We are not using MAVSDK takeoff

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

            all_reached = all(
                (
                    -state.down
                    >=
                    TAKEOFF_ALTITUDE_M
                    - TAKEOFF_TOLERANCE_M
                )
                for state in self.drones
            )

            if all_reached:

                print()

                print(
                    "[TAKEOFF] "
                    "All drones reached altitude."
                )

                return True

            if (
                elapsed
                >= TAKEOFF_TIMEOUT_S
            ):

                print()

                print(
                    "[TAKEOFF] TIMEOUT."
                )

                self.print_state()

                return False

            if now >= next_print:

                print(
                    f"[TAKEOFF] "
                    f"t={elapsed:.1f} s | "
                    + " | ".join(
                        (
                            f"UAV {state.index}: "
                            f"alt={-state.down:.2f} m"
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

    def hold(
        self,
        duration_s,
    ):

        print()

        print(
            f"[HOLD] Holding at altitude "
            f"for {duration_s:.1f} s..."
        )

        self.spin_for(
            duration_s
        )

    # SWITCH TO VELOCITY MODE

    def enter_velocity_mode(
        self,
    ):

        print()

        print(
            "[CONTROL] Switching to "
            "VELOCITY Offboard control."
        )

        for state in self.drones:

            state.position_mode = False

    # VELOCITY MOVE

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

            reached = []

            for i, target in enumerate(
                targets
            ):

                state = self.drones[i]

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

                reached.append(
                    distance
                    <= POSITION_TOLERANCE_M
                )

                if reached[-1]:

                    state.target_vn = 0.0

                    state.target_ve = 0.0

                    state.target_vd = 0.0

                    continue

                self.set_velocity_target(
                    i,
                    POSITION_KP * error_n,
                    POSITION_KP * error_e,
                    POSITION_KP * error_d,
                )

            if all(reached):

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

                print()

                print(
                    f"[{label}] TIMEOUT."
                )

                self.print_state()

                return False

            if now >= next_print:

                print(
                    f"[{label}] "
                    f"t={elapsed:.1f} s"
                )

                next_print = (
                    now + 1.0
                )

            rclpy.spin_once(
                self,
                timeout_sec=0.02,
            )

    # SET VELOCITY

    def set_velocity_target(
        self,
        index,
        vn,
        ve,
        vd,
    ):

        horizontal = math.sqrt(
            vn ** 2
            + ve ** 2
        )

        if (
            horizontal
            > MAX_HORIZONTAL_SPEED_MPS
        ):

            scale = (
                MAX_HORIZONTAL_SPEED_MPS
                / horizontal
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

        state = self.drones[index]

        state.target_vn = float(vn)

        state.target_ve = float(ve)

        state.target_vd = float(vd)

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

        start_positions = self.positions()

        xy_before = self.pairwise_mse(
            "xy"
        )

        yz_before = self.pairwise_mse(
            "yz"
        )

        start = (
            time.monotonic()
        )

        while (
            time.monotonic()
            - start
            < TEST1_DURATION_S
        ):

            for i in range(
                self.drone_count
            ):

                self.set_velocity_target(
                    i,
                    0.0,
                    TEST1_SPEED_MPS,
                    0.0,
                )

            rclpy.spin_once(
                self,
                timeout_sec=0.02,
            )

        self.stop_all()

        self.spin_for(
            1.0
        )

        final_positions = (
            self.positions()
        )

        xy_after = self.pairwise_mse(
            "xy"
        )

        yz_after = self.pairwise_mse(
            "yz"
        )

        displacement = np.linalg.norm(
            final_positions
            - start_positions,
            axis=1,
        )

        print()

        print(
            "[TEST 1] RESULTS"
        )

        print(
            f"[TEST 1] Distance = "
            f"{TEST1_DISTANCE_M:.3f} m"
        )

        print(
            f"[TEST 1] XY MSE before = "
            f"{xy_before:.6f}"
        )

        print(
            f"[TEST 1] YZ MSE before = "
            f"{yz_before:.6f}"
        )

        print(
            f"[TEST 1] XY MSE after = "
            f"{xy_after:.6f}"
        )

        print(
            f"[TEST 1] YZ MSE after = "
            f"{yz_after:.6f}"
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
            "TEST 1 — 3 M TRANSLATION",
            start_positions,
            final_positions,
            TEST1_DURATION_S,
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

        start_positions = self.positions()

        random_targets = []

        for state in self.drones:

            random_targets.append(
                np.array(
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
            )

        random_targets = np.array(
            random_targets
        )

        print()

        print(
            "[TEST 2] Moving to randomized positions..."
        )

        if not self.velocity_move(
            random_targets,
            TEST2_TIMEOUT_S,
            "TEST 2 RANDOM",
        ):

            raise RuntimeError(
                "TEST 2 random positioning failed."
            )

        self.stop_all()

        self.spin_for(
            0.5
        )

        center_n = np.mean(
            [
                state.north
                for state in self.drones
            ]
        )

        center_e = np.mean(
            [
                state.east
                for state in self.drones
            ]
        )

        center_d = np.mean(
            [
                state.down
                for state in self.drones
            ]
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
                        center_e + offset,
                        center_d,
                    ]
                )
            )

        line_targets = np.array(
            line_targets
        )

        print()

        print(
            "[TEST 2] Forming straight line..."
        )

        if not self.velocity_move(
            line_targets,
            TEST2_TIMEOUT_S,
            "TEST 2 LINE",
        ):

            raise RuntimeError(
                "TEST 2 line formation failed."
            )

        self.stop_all()

        self.spin_for(
            0.5
        )

        final_positions = self.positions()

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

        start_positions = self.positions()

        center_n = np.mean(
            start_positions[:, 0]
        )

        center_e = np.mean(
            start_positions[:, 1]
        )

        center_d = np.mean(
            start_positions[:, 2]
        )

        # Initial line along North

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
            ) * TEST3_GROUP_SPACING_M

            line_targets.append(
                np.array(
                    [
                        center_n + offset,
                        center_e,
                        center_d,
                    ]
                )
            )

        line_targets = np.array(
            line_targets
        )

        print(
            "[TEST 3] Forming initial line..."
        )

        if not self.velocity_move(
            line_targets,
            TEST3_TIMEOUT_S,
            "TEST 3 LINE",
        ):

            raise RuntimeError(
                "TEST 3 initial line failed."
            )

        # Split along North

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
                    * TEST3_GROUP_SPACING_M
                )

            else:

                split_targets[i, 0] += (
                    minority_sign
                    * TEST3_GROUP_SPACING_M
                )

        print(
            f"[TEST 3] Flag direction = "
            f"{direction}"
        )

        print(
            "[TEST 3] Splitting groups..."
        )

        if not self.velocity_move(
            split_targets,
            TEST3_TIMEOUT_S,
            "TEST 3 SPLIT",
        ):

            raise RuntimeError(
                "TEST 3 split failed."
            )

        # Translate East

        translation_targets = (
            split_targets.copy()
        )

        translation_targets[:, 1] += (
            TEST3_TRANSLATE_M
        )

        print(
            "[TEST 3] Translating "
            f"+{TEST3_TRANSLATE_M:.2f} m East..."
        )

        translation_start = (
            time.monotonic()
        )

        if not self.velocity_move(
            translation_targets,
            TEST3_TIMEOUT_S,
            "TEST 3 TRANSLATION",
        ):

            raise RuntimeError(
                "TEST 3 translation failed."
            )

        translation_time = (
            time.monotonic()
            - translation_start
        )

        self.stop_all()

        self.spin_for(
            0.5
        )

        final_positions = self.positions()

        print()

        print(
            "[TEST 3] RESULTS"
        )

        print(
            f"[TEST 3] Direction = "
            f"{direction}"
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

        self.save_result(
            "TEST 3 — FLAG / SPLIT / TRANSLATE",
            start_positions,
            final_positions,
            translation_time,
            flag_direction=direction,
            majority_count=majority_count,
            minority_count=minority_count,
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

        positions = self.positions()

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

    # SAVE RESULTS

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

        exists = self.csv_path.exists()

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

        row["duration_s"] = duration

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

        row["line_rmse_m"] = line_rmse

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

    # INITIALIZE

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

        if not self.wait_for_telemetry():

            raise RuntimeError(
                "Telemetry failed."
            )

        if not self.wait_for_valid_position():

            raise RuntimeError(
                "Position validity failed."
            )

        self.print_state()

        # Position Offboard

        self.stream_initial_setpoints()

        self.start_offboard()

        self.spin_for(
            1.0
        )

        # Arm

        if not self.arm_all():

            raise RuntimeError(
                "Not all drones armed."
            )

        # Takeoff

        if not self.takeoff():

            raise RuntimeError(
                "Takeoff failed."
            )

        self.hold(
            TAKEOFF_HOLD_S
        )

        print()

        print(
            "=" * 60
        )

        print(
            "[READY] ALL DRONES AIRBORNE"
        )

        print(
            "=" * 60
        )

        self.print_state()

        # Switch to velocity control

        self.enter_velocity_mode()

        self.spin_for(
            1.0
        )

    # LAND

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
            5.0
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
        "1 = Test 1: 3 m East translation"
    )

    print(
        "2 = Test 2: random positions -> line"
    )

    print(
        "3 = Test 3: flag -> split -> translate"
    )

    print(
        "q = Quit"
    )

    print(
        "=" * 60
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
        "[CONFIG] ROS 2 / uXRCE-DDS control"
    )

    print(
        "[CONFIG] Position Offboard takeoff"
    )

    print(
        "[CONFIG] Velocity Offboard experiments"
    )

    rclpy.init()

    node = SwarmNode(
        args.n,
        args.seed,
        args.results_csv,
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

                    node.test_1()

                except Exception as exc:

                    print(
                        f"[TEST ERROR] {exc}"
                    )

                    node.stop_all()

            elif selection == "2":

                try:

                    node.test_2()

                except Exception as exc:

                    print(
                        f"[TEST ERROR] {exc}"
                    )

                    node.stop_all()

            elif selection == "3":

                try:

                    node.test_3()

                except Exception as exc:

                    print(
                        f"[TEST ERROR] {exc}"
                    )

                    node.stop_all()

            elif selection == "q":

                print(
                    "[QUIT] Exiting."
                )

                break

            else:

                print(
                    "[ERROR] Invalid selection."
                )

    except KeyboardInterrupt:

        print(
            "[STOP] Ctrl+C."
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
