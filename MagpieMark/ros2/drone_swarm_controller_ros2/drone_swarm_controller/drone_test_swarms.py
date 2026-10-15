#!/usr/bin/env python3

# DRONE SWARM TEST SCRIPT ROS 2 / PX4 / uXRCE-DDS THIS VERSION USES: DIRECT VELOCITY OFFBOARD
# CONTROL It intentionally does NOT use: VehicleStatus PositionNed-style position control
# Position-based PX4 trajectory control The controller uses: OffboardControlMode
# TrajectorySetpoint.velocity VehicleCommand VehicleCommandAck VehicleLocalPosition IMPORTANT:
# The currently running PX4 swarm exposes: UAV 0: /fmu/... UAV 1: /px4_1/fmu/... UAV 2:
# /px4_2/fmu/... Therefore the controller uses those exact namespaces. IMPORTANT:
# VehicleStatus is NOT subscribed to because the previous VehicleStatus DDS reader caused the
# Fast DDS: 87-byte / 88-byte history payload error. OFFBOARD initialization: 1. Connect to
# flight data. 2. Stream zero velocity. 3. Request OFFBOARD. 4. Verify OFFBOARD command ACK
# 5. Force-arm using: param1 = 1 param2 = 21196 6. Verify ARM command ACK. The MAVLink-defined
# force-arm value 21196 allows arming to override preflight checks. The actual swarm
# experiments then use DIRECT VELOCITY commands, matching the previous MAVSDK workflow that
# successfully moved the vehicle

import os

# FAST DDS / ROS 2 ENVIRONMENT

# Do not use the previous custom Fast DDS XML configuration

os.environ.pop(
    "FASTRTPS_DEFAULT_PROFILES_FILE",
    None,
)

os.environ.pop(
    "RMW_FASTRTPS_USE_QOS_FROM_XML",
    None,
)

os.environ["SKIP_DEFAULT_XML"] = "1"


# IMPORTS

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
)


# CONFIGURATION

DEFAULT_DRONE_COUNT = 3

# CONTROL

CONTROL_RATE_HZ = 20.0

CONTROL_PERIOD = (
    1.0 / CONTROL_RATE_HZ
)


# VELOCITY CONTROL

MAX_HORIZONTAL_SPEED_MPS = 1.50

MAX_VERTICAL_SPEED_MPS = 0.70

# Direct velocity used by Test 1
TEST1_SPEED_MPS = 1.00

# Direct velocity used by Test 2
TEST2_SPEED_MPS = 1.00

# Direct velocity used by Test 3
TEST3_SPEED_MPS = 1.00


# INITIALIZATION

INITIAL_SETPOINT_DURATION_S = 2.0

OFFBOARD_COMMAND_ATTEMPTS = 8

OFFBOARD_COMMAND_INTERVAL_S = 0.25

ARM_COMMAND_ATTEMPTS = 8

ARM_COMMAND_INTERVAL_S = 0.25

COMMAND_ACK_TIMEOUT_S = 1.0

POST_ARM_HOLD_S = 1.0


# TELEMETRY

CONNECTION_TIMEOUT_S = 15.0

HEALTH_TIMEOUT_S = 15.0


# MOVEMENT

MOVEMENT_START_TIMEOUT_S = 5.0

MOVEMENT_START_SPEED_MPS = 0.08

MOVEMENT_START_DISPLACEMENT_M = 0.03


# TEST LIMITS

TEST_TIMEOUT_S = 30.0

POSITION_TOLERANCE_M = 0.30

SPEED_TOLERANCE_MPS = 0.50


# TEST 1

TEST1_TRANSLATE_M = 3.0


# TEST 2

TEST2_RANDOM_X_RANGE = (
    -3.0,
    3.0,
)

TEST2_RANDOM_Y_RANGE = (
    -3.0,
    3.0,
)

TEST2_LINE_SPACING_M = 1.5

TEST2_SIGNAL_DELAY_S = 0.50


# TEST 3

TEST3_GROUP_SPACING_M = 1.5

TEST3_TRANSLATE_M = 1.0


# RANDOMIZATION

DEFAULT_RANDOM_SEED = None


# RESULTS

DEFAULT_RESULTS_CSV = (
    "swarm_test_results.csv"
)


# SPAWN GEOMETRY

# These values match the swarm launch script

SPAWN_SPACING_M = 2.0

SPAWN_ALTITUDE_M = 2.0


# ROS 2 / PX4 NAMESPACE

# IMPORTANT: The actual ROS topic list showed: /fmu/... /px4_1/fmu/... /px4_2/fmu/
# Therefore UAV 0 has the standard PX4 namespace, while UAV 1+ use /px4_<index>

UAV_NAMESPACE_PREFIX = (
    "/px4_"
)


# PX4 TOPICS

LOCAL_POSITION_TOPIC = (
    "vehicle_local_position_v1"
)

VEHICLE_COMMAND_ACK_TOPIC = (
    "vehicle_command_ack"
)


# PX4 SYSTEM IDs

# UAV 0 -> PX4 system ID 1 UAV 1 -> PX4 system ID 2 UAV 2 -> PX4 system ID 3

PX4_SYS_ID_START = 1


# PX4 COMMAND CONSTANTS

VEHICLE_CMD_COMPONENT_ARM_DISARM = 400

VEHICLE_CMD_DO_SET_MODE = 176

VEHICLE_CMD_RUN_PREARM_CHECKS = 401

VEHICLE_CMD_NAV_LAND = 21


# PX4 OFFBOARD CONSTANTS

PX4_BASE_MODE_CUSTOM = 1

PX4_CUSTOM_MODE_OFFBOARD = 6


# MAVLINK ACK RESULTS

MAV_RESULT_ACCEPTED = 0

MAV_RESULT_TEMPORARILY_REJECTED = 1

MAV_RESULT_DENIED = 2

MAV_RESULT_UNSUPPORTED = 3

MAV_RESULT_FAILED = 4

MAV_RESULT_IN_PROGRESS = 5

MAV_RESULT_CANCELLED = 6


# FORCE ARM

# MAV_CMD_COMPONENT_ARM_DISARM: param1 = 1 ARM param2 = 21196 FORCE ARM This is the
# MAVLink-defined force-arm value

FORCE_ARM_VALUE = 21196.0


# ROS 2 QoS

PX4_TELEMETRY_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


# DRONE STATE

@dataclass
class DroneState:

    north_m: float = 0.0

    east_m: float = 0.0

    down_m: float = 0.0

    north_speed_mps: float = 0.0

    east_speed_mps: float = 0.0

    down_speed_mps: float = 0.0

    local_position_valid: bool = False

    last_position_time: float = 0.0


# DRONE SWARM NODE

class DroneSwarmNode(Node):

    def __init__(
        self,
        drone_count,
    ):

        super().__init__(
            "drone_swarm_test_controller"
        )

        self.drone_count = (
            drone_count
        )

        self.states = [
            DroneState()
            for _ in range(
                drone_count
            )
        ]

        # VELOCITY TARGETS [North, East, Down]

        self.velocity_targets = [
            np.zeros(
                3,
                dtype=float,
            )
            for _ in range(
                drone_count
            )
        ]

        # COMMAND ACK STORAGE

        self.last_ack = [
            None
            for _ in range(
                drone_count
            )
        ]

        self.last_ack_time = [
            0.0
            for _ in range(
                drone_count
            )
        ]

        self.last_arm_result = [
            None
            for _ in range(
                drone_count
            )
        ]

        self.last_offboard_result = [
            None
            for _ in range(
                drone_count
            )
        ]

        # ROS INTERFACES

        self.offboard_publishers = []

        self.trajectory_publishers = []

        self.command_publishers = []

        self.position_subscribers = []

        self.ack_subscribers = []

        # CREATE UAV INTERFACES

        for i in range(
            drone_count
        ):

            namespace = (
                self.get_uav_namespace(
                    i
                )
            )

            # OFFBOARD TOPIC

            offboard_topic = (
                f"{namespace}"
                f"/fmu/in/"
                f"offboard_control_mode"
            )

            # TRAJECTORY TOPIC

            trajectory_topic = (
                f"{namespace}"
                f"/fmu/in/"
                f"trajectory_setpoint"
            )

            # VEHICLE COMMAND TOPIC

            command_topic = (
                f"{namespace}"
                f"/fmu/in/"
                f"vehicle_command"
            )

            # LOCAL POSITION

            position_topic = (
                f"{namespace}"
                f"/fmu/out/"
                f"{LOCAL_POSITION_TOPIC}"
            )

            # VEHICLE COMMAND ACK

            ack_topic = (
                f"{namespace}"
                f"/fmu/out/"
                f"{VEHICLE_COMMAND_ACK_TOPIC}"
            )

            # PUBLISHERS

            self.offboard_publishers.append(
                self.create_publisher(
                    OffboardControlMode,
                    offboard_topic,
                    10,
                )
            )

            self.trajectory_publishers.append(
                self.create_publisher(
                    TrajectorySetpoint,
                    trajectory_topic,
                    10,
                )
            )

            self.command_publishers.append(
                self.create_publisher(
                    VehicleCommand,
                    command_topic,
                    10,
                )
            )

            # POSITION SUBSCRIBER

            self.position_subscribers.append(
                self.create_subscription(
                    VehicleLocalPosition,
                    position_topic,
                    lambda msg,
                    index=i:
                    self.position_callback(
                        msg,
                        index,
                    ),
                    PX4_TELEMETRY_QOS,
                )
            )

            # ACK SUBSCRIBER

            self.ack_subscribers.append(
                self.create_subscription(
                    VehicleCommandAck,
                    ack_topic,
                    lambda msg,
                    index=i:
                    self.command_ack_callback(
                        msg,
                        index,
                    ),
                    PX4_TELEMETRY_QOS,
                )
            )

        # 20 Hz OFFBOARD CONTROL LOOP

        self.control_timer = (
            self.create_timer(
                CONTROL_PERIOD,
                self.control_timer_callback,
            )
        )

        # LOGGING

        self.get_logger().info(
            f"Created swarm controller for "
            f"{drone_count} drones."
        )

        self.get_logger().info(
            "VehicleStatus subscription disabled."
        )

        self.get_logger().info(
            "Control mode = OFFBOARD VELOCITY"
        )

        for i in range(
            drone_count
        ):

            self.get_logger().info(
                f"UAV {i}: "
                f"namespace="
                f"{self.get_uav_namespace(i)}"
            )


    # UAV NAMESPACE

    @staticmethod
    def get_uav_namespace(
        index,
    ):

        # UAV 0 uses the standard PX4 namespace: /fmu/... UAV 1 uses: /px4_1/fmu/
        # UAV 2 uses: /px4_2/fmu/

        if index == 0:

            return ""

        return (
            f"/px4_{index}"
        )


    # POSITION CALLBACK

    def position_callback(
        self,
        msg,
        index,
    ):

        state = (
            self.states[index]
        )

        # PX4 VehicleLocalPosition: x = North y = East z = Down

        state.north_m = (
            float(msg.x)
        )

        state.east_m = (
            float(msg.y)
        )

        state.down_m = (
            float(msg.z)
        )

        state.north_speed_mps = (
            float(msg.vx)
        )

        state.east_speed_mps = (
            float(msg.vy)
        )

        state.down_speed_mps = (
            float(msg.vz)
        )

        finite = all(
            math.isfinite(
                value
            )
            for value in (
                msg.x,
                msg.y,
                msg.z,
            )
        )

        state.local_position_valid = (
            finite
            and bool(msg.z_valid)
        )

        state.last_position_time = (
            time.monotonic()
        )


    # COMMAND ACK CALLBACK

    def command_ack_callback(
        self,
        msg,
        index,
    ):

        self.last_ack[index] = (
            msg
        )

        self.last_ack_time[index] = (
            time.monotonic()
        )

        if (
            msg.command
            == VEHICLE_CMD_DO_SET_MODE
        ):

            self.last_offboard_result[index] = (
                int(msg.result)
            )

            self.get_logger().info(
                f"UAV {index}: "
                f"VehicleCommandAck "
                f"command=176 "
                f"result={self.ack_result_name(msg.result)}"
            )

        elif (
            msg.command
            == VEHICLE_CMD_COMPONENT_ARM_DISARM
        ):

            self.last_arm_result[index] = (
                int(msg.result)
            )

            self.get_logger().info(
                f"UAV {index}: "
                f"VehicleCommandAck "
                f"command=400 "
                f"result={self.ack_result_name(msg.result)} "
                f"param2={msg.result_param2}"
            )


    # ACK RESULT NAME

    @staticmethod
    def ack_result_name(
        result,
    ):

        mapping = {
            MAV_RESULT_ACCEPTED:
                "ACCEPTED",

            MAV_RESULT_TEMPORARILY_REJECTED:
                "TEMPORARILY_REJECTED",

            MAV_RESULT_DENIED:
                "DENIED",

            MAV_RESULT_UNSUPPORTED:
                "UNSUPPORTED",

            MAV_RESULT_FAILED:
                "FAILED",

            MAV_RESULT_IN_PROGRESS:
                "IN_PROGRESS",

            MAV_RESULT_CANCELLED:
                "CANCELLED",
        }

        return mapping.get(
            int(result),
            str(result),
        )


    # CONTROL TIMER

    def control_timer_callback(
        self
    ):

        timestamp = (
            self.get_clock()
            .now()
            .nanoseconds
            // 1000
        )

        for i in range(
            self.drone_count
        ):

            # OFFBOARD CONTROL MODE

            offboard_msg = (
                OffboardControlMode()
            )

            offboard_msg.position = (
                False
            )

            offboard_msg.velocity = (
                True
            )

            offboard_msg.acceleration = (
                False
            )

            offboard_msg.attitude = (
                False
            )

            offboard_msg.body_rate = (
                False
            )

            offboard_msg.thrust_and_torque = (
                False
            )

            offboard_msg.direct_actuator = (
                False
            )

            offboard_msg.timestamp = (
                timestamp
            )

            self.offboard_publishers[
                i
            ].publish(
                offboard_msg
            )

            # TRAJECTORY SETPOINT

            velocity = (
                self.velocity_targets[i]
            )

            trajectory_msg = (
                TrajectorySetpoint()
            )

            # POSITION DISABLED

            trajectory_msg.position = [
                math.nan,
                math.nan,
                math.nan,
            ]

            # VELOCITY ENABLED

            trajectory_msg.velocity = [
                float(
                    velocity[0]
                ),
                float(
                    velocity[1]
                ),
                float(
                    velocity[2]
                ),
            ]

            # ACCELERATION DISABLED

            trajectory_msg.acceleration = [
                math.nan,
                math.nan,
                math.nan,
            ]

            # JERK DISABLED

            trajectory_msg.jerk = [
                math.nan,
                math.nan,
                math.nan,
            ]

            # YAW

            trajectory_msg.yaw = (
                math.radians(
                    90.0
                )
            )

            trajectory_msg.yawspeed = (
                0.0
            )

            trajectory_msg.timestamp = (
                timestamp
            )

            self.trajectory_publishers[
                i
            ].publish(
                trajectory_msg
            )


    # SET VELOCITY

    def set_velocity(
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

            north_mps *= (
                scale
            )

            east_mps *= (
                scale
            )

        down_mps = max(
            -MAX_VERTICAL_SPEED_MPS,
            min(
                MAX_VERTICAL_SPEED_MPS,
                down_mps,
            ),
        )

        self.velocity_targets[
            index
        ] = np.array(
            [
                float(north_mps),
                float(east_mps),
                float(down_mps),
            ],
            dtype=float,
        )


    # SET ALL VELOCITIES

    def set_all_velocities(
        self,
        north_mps,
        east_mps,
        down_mps,
    ):

        for i in range(
            self.drone_count
        ):

            self.set_velocity(
                i,
                north_mps,
                east_mps,
                down_mps,
            )


    # STOP

    def stop_drones(
        self
    ):

        self.set_all_velocities(
            0.0,
            0.0,
            0.0,
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

        msg = (
            VehicleCommand()
        )

        msg.timestamp = (
            self.get_clock()
            .now()
            .nanoseconds
            // 1000
        )

        msg.param1 = float(
            param1
        )

        msg.param2 = float(
            param2
        )

        msg.param3 = float(
            param3
        )

        msg.param4 = float(
            param4
        )

        msg.param5 = float(
            param5
        )

        msg.param6 = float(
            param6
        )

        msg.param7 = float(
            param7
        )

        msg.command = int(
            command
        )

        msg.target_system = (
            PX4_SYS_ID_START
            + index
        )

        msg.target_component = (
            1
        )

        msg.source_system = (
            1
        )

        msg.source_component = (
            1
        )

        msg.from_external = (
            True
        )

        self.command_publishers[
            index
        ].publish(
            msg
        )


    # OFFBOARD COMMAND

    def send_offboard_command(
        self,
        index,
    ):

        # PX4: MAV_CMD_DO_SET_MODE param1 = 1 param2 = 6

        self.send_vehicle_command(
            index=index,
            command=(
                VEHICLE_CMD_DO_SET_MODE
            ),
            param1=(
                PX4_BASE_MODE_CUSTOM
            ),
            param2=(
                PX4_CUSTOM_MODE_OFFBOARD
            ),
        )


    # FORCE ARM

    def arm_drone(
        self,
        index,
    ):

        # MAV_CMD_COMPONENT_ARM_DISARM param1 = 1 param2 = 21196 21196 is the
        # MAVLink-defined force-arm value

        self.send_vehicle_command(
            index=index,
            command=(
                VEHICLE_CMD_COMPONENT_ARM_DISARM
            ),
            param1=1.0,
            param2=FORCE_ARM_VALUE,
        )


    # PREARM CHECK

    def run_prearm_checks(
        self,
        index,
    ):

        self.send_vehicle_command(
            index=index,
            command=(
                VEHICLE_CMD_RUN_PREARM_CHECKS
            ),
        )


    # LAND

    def land_drone(
        self,
        index,
    ):

        self.send_vehicle_command(
            index=index,
            command=(
                VEHICLE_CMD_NAV_LAND
            ),
        )


# ROS SPIN HELPER

def spin_for(
    node,
    duration,
):

    end_time = (
        time.monotonic()
        + duration
    )

    while (
        time.monotonic()
        < end_time
    ):

        rclpy.spin_once(
            node,
            timeout_sec=0.02,
        )


# WAIT FOR TELEMETRY

def wait_for_telemetry(
    node,
):

    print()
    print(
        "-" * 60
    )

    print(
        "[ROS 2] Waiting for PX4 position telemetry..."
    )

    print(
        "-" * 60
    )

    start = (
        time.monotonic()
    )

    last_print = -1.0

    while (
        time.monotonic()
        - start
        < CONNECTION_TIMEOUT_S
    ):

        rclpy.spin_once(
            node,
            timeout_sec=0.02,
        )

        connected = sum(
            state.local_position_valid
            for state in node.states
        )

        elapsed = (
            time.monotonic()
            - start
        )

        if (
            elapsed
            - last_print
            >= 0.5
        ):

            print(
                f"\r[ROS 2] Connected "
                f"{connected}/"
                f"{node.drone_count} drones",
                end="",
                flush=True,
            )

            last_print = (
                elapsed
            )

        if (
            connected
            == node.drone_count
        ):

            print()

            print(
                "[ROS 2] All PX4 "
                "position telemetry "
                "streams connected."
            )

            return True

    print()

    return False


# WAIT FOR HEALTH

def wait_for_health(
    node,
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
        < HEALTH_TIMEOUT_S
    ):

        rclpy.spin_once(
            node,
            timeout_sec=0.02,
        )

        valid = all(
            state.local_position_valid
            for state in node.states
        )

        if valid:

            print(
                "[HEALTH] All local "
                "position estimates usable."
            )

            return True

    return False


# WAIT FOR ACK

def wait_for_command_ack(
    node,
    index,
    command,
    timeout_s=COMMAND_ACK_TIMEOUT_S,
):

    start = (
        time.monotonic()
    )

    node.last_ack[index] = (
        None
    )

    while (
        time.monotonic()
        - start
        < timeout_s
    ):

        rclpy.spin_once(
            node,
            timeout_sec=0.02,
        )

        ack = (
            node.last_ack[index]
        )

        if (
            ack is not None
            and int(ack.command)
            == int(command)
            and node.last_ack_time[index]
            >= start
        ):

            return ack

    return None


# INITIALIZE OFFBOARD

def initialize_offboard(
    node,
):

    print()
    print(
        "[OFFBOARD] Streaming "
        "initial zero-velocity setpoints..."
    )

    node.stop_drones()

    spin_for(
        node,
        INITIAL_SETPOINT_DURATION_S,
    )

    print()
    print(
        "[OFFBOARD] Requesting OFFBOARD mode..."
    )

    for attempt in range(
        OFFBOARD_COMMAND_ATTEMPTS
    ):

        print(
            f"[OFFBOARD] Command "
            f"{attempt + 1}/"
            f"{OFFBOARD_COMMAND_ATTEMPTS}"
        )

        for i in range(
            node.drone_count
        ):

            node.last_offboard_result[
                i
            ] = None

            node.send_offboard_command(
                i
            )

        spin_for(
            node,
            OFFBOARD_COMMAND_INTERVAL_S,
        )

        accepted = all(
            result
            == MAV_RESULT_ACCEPTED
            for result
            in node.last_offboard_result
        )

        if accepted:

            break

    # VERIFY OFFBOARD ACKS

    failed = []

    for i in range(
        node.drone_count
    ):

        if (
            node.last_offboard_result[i]
            != MAV_RESULT_ACCEPTED
        ):

            failed.append(
                i
            )

    if failed:

        raise RuntimeError(
            "OFFBOARD was not accepted "
            f"for UAVs: {failed}"
        )

    print(
        "[OFFBOARD] OFFBOARD commands "
        "accepted by all drones."
    )


# INITIALIZE ARMING

def initialize_arming(
    node,
):

    print()
    print(
        "[ARM] Running PX4 pre-arm checks..."
    )

    for i in range(
        node.drone_count
    ):

        node.run_prearm_checks(
            i
        )

    spin_for(
        node,
        0.5,
    )

    print()
    print(
        "[ARM] Sending FORCE ARM commands..."
    )

    accepted = [
        False
        for _ in range(
            node.drone_count
        )
    ]

    for attempt in range(
        ARM_COMMAND_ATTEMPTS
    ):

        print(
            f"[ARM] Command "
            f"{attempt + 1}/"
            f"{ARM_COMMAND_ATTEMPTS}"
        )

        for i in range(
            node.drone_count
        ):

            if accepted[i]:

                continue

            node.last_arm_result[
                i
            ] = None

            node.arm_drone(
                i
            )

        spin_for(
            node,
            ARM_COMMAND_INTERVAL_S,
        )

        for i in range(
            node.drone_count
        ):

            if (
                node.last_arm_result[i]
                == MAV_RESULT_ACCEPTED
            ):

                accepted[i] = True

    # FINAL ARM CHECK

    failed = [
        i
        for i in range(
            node.drone_count
        )
        if not accepted[i]
    ]

    if failed:

        print()

        print(
            "[ARM] FAILED UAVs:"
        )

        for i in failed:

            result = (
                node.last_arm_result[i]
            )

            if result is None:

                name = (
                    "NO ACK"
                )

            else:

                name = (
                    node.ack_result_name(
                        result
                    )
                )

            print(
                f"    UAV {i}: {name}"
            )

        raise RuntimeError(
            "Not all PX4 vehicles "
            "accepted the FORCE ARM command."
        )

    print()
    print(
        "[ARM] FORCE ARM accepted "
        "by all drones."
    )

    # SHORT HOLD

    print()
    print(
        f"[READY] Holding zero velocity "
        f"for {POST_ARM_HOLD_S:.2f} s..."
    )

    node.stop_drones()

    spin_for(
        node,
        POST_ARM_HOLD_S,
    )


# INITIALIZE DRONES

def initialize_drones(
    node,
):

    print()
    print(
        "=" * 60
    )

    print(
        "INITIALIZING DRONES"
    )

    print(
        "=" * 60
    )

    # TELEMETRY

    if not wait_for_telemetry(
        node
    ):

        raise RuntimeError(
            "PX4 telemetry did not connect."
        )

    if not wait_for_health(
        node
    ):

        raise RuntimeError(
            "PX4 local position "
            "estimates are not valid."
        )

    # OFFBOARD

    initialize_offboard(
        node
    )

    # ARM

    initialize_arming(
        node
    )

    # FINAL STATE

    print()
    print(
        "=" * 60
    )

    print(
        "[READY] ALL DRONES INITIALIZED"
    )

    print(
        "=" * 60
    )

    print_positions(
        node
    )


# PRINT POSITIONS

def print_positions(
    node,
):

    for i, state in enumerate(
        node.states
    ):

        speed = math.sqrt(
            state.north_speed_mps ** 2
            + state.east_speed_mps ** 2
            + state.down_speed_mps ** 2
        )

        print(
            f"Drone {i}: "
            f"N={state.north_m:+.3f} m, "
            f"E={state.east_m:+.3f} m, "
            f"D={state.down_m:+.3f} m, "
            f"Speed={speed:.3f} m/s"
        )


# GET LOCAL POSITIONS

def get_local_positions(
    node,
):

    return np.array(
        [
            [
                state.north_m,
                state.east_m,
                state.down_m,
            ]
            for state in node.states
        ],
        dtype=float,
    )


# GET SPEED

def get_speed(
    node,
    index,
):

    state = (
        node.states[index]
    )

    return math.sqrt(
        state.north_speed_mps ** 2
        + state.east_speed_mps ** 2
        + state.down_speed_mps ** 2
    )


# WORLD SPAWN POSITION

def get_spawn_world_position(
    index,
    drone_count,
):

    spawn_x = (
        0.0
    )

    spawn_y = (
        index
        - (drone_count - 1)
        / 2.0
    ) * SPAWN_SPACING_M

    spawn_z = (
        SPAWN_ALTITUDE_M
    )

    return np.array(
        [
            spawn_x,
            spawn_y,
            spawn_z,
        ],
        dtype=float,
    )


# WORLD POSITION

def get_world_position(
    node,
    index,
):

    state = (
        node.states[index]
    )

    spawn = (
        get_spawn_world_position(
            index,
            node.drone_count,
        )
    )

    # PX4 NED -> Gazebo world: North -> Gazebo Y East  -> Gazebo X Down  -> negative Gazebo Z

    world_x = (
        spawn[0]
        + state.east_m
    )

    world_y = (
        spawn[1]
        + state.north_m
    )

    world_z = (
        spawn[2]
        - state.down_m
    )

    return np.array(
        [
            world_x,
            world_y,
            world_z,
        ],
        dtype=float,
    )


# WORLD TARGET -> LOCAL NED

def world_target_to_local(
    node,
    index,
    world_target,
):

    spawn = (
        get_spawn_world_position(
            index,
            node.drone_count,
        )
    )

    local_north = (
        world_target[1]
        - spawn[1]
    )

    local_east = (
        world_target[0]
        - spawn[0]
    )

    local_down = (
        spawn[2]
        - world_target[2]
    )

    return np.array(
        [
            local_north,
            local_east,
            local_down,
        ],
        dtype=float,
    )


# PAIRWISE MSE

def mean_squared_pairwise_distance(
    node,
    plane,
):

    positions = np.array(
        [
            get_world_position(
                node,
                i,
            )
            for i in range(
                node.drone_count
            )
        ]
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

                dx = (
                    positions[i, 0]
                    - positions[j, 0]
                )

                dy = (
                    positions[i, 1]
                    - positions[j, 1]
                )

                values.append(
                    dx ** 2
                    + dy ** 2
                )

            elif plane == "yz":

                dy = (
                    positions[i, 1]
                    - positions[j, 1]
                )

                dz = (
                    positions[i, 2]
                    - positions[j, 2]
                )

                values.append(
                    dy ** 2
                    + dz ** 2
                )

            else:

                raise ValueError(
                    "plane must be 'xy' or 'yz'"
                )

    if not values:

        return 0.0

    return (
        sum(values)
        / len(values)
    )


# WAIT FOR MOVEMENT

def wait_for_actual_movement(
    node,
    initial_positions,
    timeout_s=MOVEMENT_START_TIMEOUT_S,
):

    print()
    print(
        "[MOVEMENT] Checking that "
        "all drones actually move..."
    )

    started = [
        False
        for _ in range(
            node.drone_count
        )
    ]

    start = (
        time.monotonic()
    )

    last_print = -1.0

    while (
        time.monotonic()
        - start
        < timeout_s
    ):

        rclpy.spin_once(
            node,
            timeout_sec=0.02,
        )

        current_positions = (
            get_local_positions(
                node
            )
        )

        for i in range(
            node.drone_count
        ):

            displacement = (
                np.linalg.norm(
                    current_positions[i]
                    - initial_positions[i]
                )
            )

            speed = (
                get_speed(
                    node,
                    i,
                )
            )

            if (
                displacement
                >= MOVEMENT_START_DISPLACEMENT_M
                or speed
                >= MOVEMENT_START_SPEED_MPS
            ):

                started[i] = True

        elapsed = (
            time.monotonic()
            - start
        )

        if (
            elapsed
            - last_print
            >= 1.0
        ):

            print(
                f"[MOVEMENT] t="
                f"{elapsed:.1f} s | "
                + " | ".join(
                    f"UAV {i}: "
                    f"{get_speed(node, i):.2f} m/s"
                    for i in range(
                        node.drone_count
                    )
                )
            )

            last_print = (
                elapsed
            )

        if all(
            started
        ):

            print(
                "[MOVEMENT] All drones "
                "have begun moving."
            )

            return True

    print()

    print(
        "[MOVEMENT] Movement verification failed."
    )

    for i in range(
        node.drone_count
    ):

        displacement = (
            np.linalg.norm(
                get_local_positions(
                    node
                )[i]
                - initial_positions[i]
            )
        )

        print(
            f"[MOVEMENT] UAV {i}: "
            f"speed="
            f"{get_speed(node, i):.3f} m/s | "
            f"displacement="
            f"{displacement:.3f} m"
        )

    return False


# DIRECT VELOCITY SEGMENT

def run_constant_velocity(
    node,
    north_mps,
    east_mps,
    down_mps,
    duration_s,
    label,
):

    print()
    print(
        "-" * 60
    )

    print(
        f"[VELOCITY] {label}"
    )

    print(
        "-" * 60
    )

    print(
        f"[VELOCITY] Command = "
        f"N={north_mps:+.2f}, "
        f"E={east_mps:+.2f}, "
        f"D={down_mps:+.2f} m/s"
    )

    print(
        f"[VELOCITY] Duration = "
        f"{duration_s:.2f} s"
    )

    initial_positions = (
        get_local_positions(
            node
        )
    )

    # APPLY COMMAND

    node.set_all_velocities(
        north_mps,
        east_mps,
        down_mps,
    )

    # VERIFY MOVEMENT

    movement_verified = (
        wait_for_actual_movement(
            node,
            initial_positions,
        )
    )

    if not movement_verified:

        node.stop_drones()

        raise RuntimeError(
            f"{label}: "
            "PX4 accepted the command path, "
            "but the vehicles did not physically "
            "begin moving."
        )

    # ACTUAL MEASURED FLIGHT

    start_time = (
        time.monotonic()
    )

    last_print = -1.0

    while (
        time.monotonic()
        - start_time
        < duration_s
    ):

        # Re-apply the same velocity every cycle

        node.set_all_velocities(
            north_mps,
            east_mps,
            down_mps,
        )

        rclpy.spin_once(
            node,
            timeout_sec=0.02,
        )

        elapsed = (
            time.monotonic()
            - start_time
        )

        if (
            elapsed
            - last_print
            >= 1.0
        ):

            print(
                f"[VELOCITY] "
                f"t={elapsed:.1f} s | "
                + " | ".join(
                    f"UAV {i}: "
                    f"v={get_speed(node, i):.2f} m/s"
                    for i in range(
                        node.drone_count
                    )
                )
            )

            last_print = (
                elapsed
            )

    # STOP

    node.stop_drones()

    spin_for(
        node,
        0.50,
    )

    final_positions = (
        get_local_positions(
            node
        )
    )

    displacement = (
        final_positions
        - initial_positions
    )

    distances = (
        np.linalg.norm(
            displacement,
            axis=1,
        )
    )

    measured_duration = (
        time.monotonic()
        - start_time
    )

    print()
    print(
        "[VELOCITY] Movement complete."
    )

    for i in range(
        node.drone_count
    ):

        print(
            f"[VELOCITY] UAV {i}: "
            f"displacement="
            f"{distances[i]:.3f} m"
        )

    return {
        "duration":
            measured_duration,

        "initial_positions":
            initial_positions,

        "final_positions":
            final_positions,

        "displacements":
            distances,
    }


# MOVE TOWARD TARGETS USING DIRECT VELOCITY

def move_to_targets(
    node,
    targets,
    speed_mps,
    label,
):

    print()
    print(
        "-" * 60
    )

    print(
        f"[TARGET VELOCITY] {label}"
    )

    print(
        "-" * 60
    )

    print(
        f"[TARGET VELOCITY] Speed = "
        f"{speed_mps:.2f} m/s"
    )

    for i, target in enumerate(
        targets
    ):

        print(
            f"Drone {i}: "
            f"N={target[0]:+.3f}, "
            f"E={target[1]:+.3f}, "
            f"D={target[2]:+.3f}"
        )

    initial_positions = (
        get_local_positions(
            node
        )
    )

    # APPLY INITIAL COMMANDS

    current = (
        get_local_positions(
            node
        )
    )

    for i in range(
        node.drone_count
    ):

        error = (
            targets[i]
            - current[i]
        )

        horizontal = np.array(
            [
                error[0],
                error[1],
            ]
        )

        distance = (
            np.linalg.norm(
                horizontal
            )
        )

        if (
            distance
            > 0.05
        ):

            velocity = (
                horizontal
                / distance
                * speed_mps
            )

            node.set_velocity(
                i,
                velocity[0],
                velocity[1],
                0.0,
            )

        else:

            node.set_velocity(
                i,
                0.0,
                0.0,
                0.0,
            )

    # VERIFY ACTUAL MOVEMENT

    if not wait_for_actual_movement(
        node,
        initial_positions,
    ):

        node.stop_drones()

        raise RuntimeError(
            f"{label}: "
            "vehicles did not begin moving."
        )

    # MOVE

    start_time = (
        time.monotonic()
    )

    last_print = -1.0

    while True:

        current = (
            get_local_positions(
                node
            )
        )

        distances = (
            np.linalg.norm(
                np.array(targets)
                - current,
                axis=1,
            )
        )

        speeds = np.array(
            [
                get_speed(
                    node,
                    i,
                )
                for i in range(
                    node.drone_count
                )
            ]
        )

        all_arrived = (
            np.all(
                distances
                <= POSITION_TOLERANCE_M
            )
            and np.all(
                speeds
                <= SPEED_TOLERANCE_MPS
            )
        )

        if all_arrived:

            break

        elapsed = (
            time.monotonic()
            - start_time
        )

        if (
            elapsed
            > TEST_TIMEOUT_S
        ):

            node.stop_drones()

            raise RuntimeError(
                f"{label}: "
                f"movement timed out after "
                f"{TEST_TIMEOUT_S:.1f} s."
            )

        # NEW VELOCITY COMMANDS

        for i in range(
            node.drone_count
        ):

            error = (
                np.array(
                    targets[i]
                )
                - current[i]
            )

            horizontal = np.array(
                [
                    error[0],
                    error[1],
                ]
            )

            horizontal_distance = (
                np.linalg.norm(
                    horizontal
                )
            )

            if (
                horizontal_distance
                > POSITION_TOLERANCE_M
            ):

                velocity = (
                    horizontal
                    / horizontal_distance
                    * speed_mps
                )

                node.set_velocity(
                    i,
                    velocity[0],
                    velocity[1],
                    0.0,
                )

            else:

                node.set_velocity(
                    i,
                    0.0,
                    0.0,
                    0.0,
                )

        rclpy.spin_once(
            node,
            timeout_sec=0.02,
        )

        if (
            elapsed
            - last_print
            >= 1.0
        ):

            print(
                f"[TARGET VELOCITY] "
                f"t={elapsed:.1f} s | "
                + " | ".join(
                    f"UAV {i}: "
                    f"err={distances[i]:.2f} m | "
                    f"v={speeds[i]:.2f} m/s"
                    for i in range(
                        node.drone_count
                    )
                )
            )

            last_print = (
                elapsed
            )

    # STOP

    node.stop_drones()

    spin_for(
        node,
        0.50,
    )

    final_positions = (
        get_local_positions(
            node
        )
    )

    final_errors = (
        np.linalg.norm(
            np.array(targets)
            - final_positions,
            axis=1,
        )
    )

    elapsed = (
        time.monotonic()
        - start_time
    )

    print()
    print(
        f"[TARGET VELOCITY] "
        f"Movement completed in "
        f"{elapsed:.3f} s."
    )

    print(
        f"[TARGET VELOCITY] "
        f"Maximum final error = "
        f"{np.max(final_errors):.3f} m"
    )

    return {
        "duration":
            elapsed,

        "initial_positions":
            initial_positions,

        "final_positions":
            final_positions,

        "final_errors":
            final_errors,
    }


# HOLD

def hold_drones(
    node,
    duration_s,
):

    node.stop_drones()

    spin_for(
        node,
        duration_s,
    )


# TEST 1

def test_1(
    node
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

    start_positions = (
        get_local_positions(
            node
        )
    )

    xy_mse_before = (
        mean_squared_pairwise_distance(
            node,
            "xy",
        )
    )

    yz_mse_before = (
        mean_squared_pairwise_distance(
            node,
            "yz",
        )
    )

    print()
    print(
        "[TEST 1] Initial positions:"
    )

    print_positions(
        node
    )

    # DIRECT VELOCITY This deliberately matches the old working MAVSDK approach: constant
    # East velocity constant duration

    duration = (
        TEST1_TRANSLATE_M
        / TEST1_SPEED_MPS
    )

    print()
    print(
        f"[TEST 1] Commanding "
        f"+{TEST1_SPEED_MPS:.2f} m/s East "
        f"for {duration:.2f} s."
    )

    movement = (
        run_constant_velocity(
            node,
            0.0,
            TEST1_SPEED_MPS,
            0.0,
            duration,
            "TEST 1 — 3 M EAST TRANSLATION",
        )
    )

    final_positions = (
        movement[
            "final_positions"
        ]
    )

    xy_mse_after = (
        mean_squared_pairwise_distance(
            node,
            "xy",
        )
    )

    yz_mse_after = (
        mean_squared_pairwise_distance(
            node,
            "yz",
        )
    )

    actual_displacements = (
        movement[
            "displacements"
        ]
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
        f"[TEST 1] Commanded speed = "
        f"{TEST1_SPEED_MPS:.3f} m/s"
    )

    print(
        f"[TEST 1] Command duration = "
        f"{duration:.3f} s"
    )

    print(
        f"[TEST 1] XY MSE before = "
        f"{xy_mse_before:.6f} m²"
    )

    print(
        f"[TEST 1] YZ MSE before = "
        f"{yz_mse_before:.6f} m²"
    )

    print(
        f"[TEST 1] XY MSE after = "
        f"{xy_mse_after:.6f} m²"
    )

    print(
        f"[TEST 1] YZ MSE after = "
        f"{yz_mse_after:.6f} m²"
    )

    print()
    print(
        "[TEST 1] Actual displacements:"
    )

    for i in range(
        node.drone_count
    ):

        print(
            f"Drone {i}: "
            f"{actual_displacements[i]:.3f} m"
        )

    print()
    print(
        "[TEST 1] Final positions:"
    )

    print_positions(
        node
    )

    save_test_result(
        test_name=(
            "TEST 1 — 3 M TRANSLATION"
        ),
        duration=movement[
            "duration"
        ],
        node=node,
        start_positions=start_positions,
        final_positions=final_positions,
        xy_mse_before=(
            xy_mse_before
        ),
        yz_mse_before=(
            yz_mse_before
        ),
        xy_mse_after=(
            xy_mse_after
        ),
        yz_mse_after=(
            yz_mse_after
        ),
    )


# TEST 2

def test_2(
    node,
    rng,
):

    print()
    print(
        "=" * 60
    )

    print(
        "TEST 2 — RANDOM POSITIONS -> "
        "ORTHOGONAL STRAIGHT LINE"
    )

    print(
        "=" * 60
    )

    # CURRENT WORLD POSITIONS

    current_world_positions = (
        np.array(
            [
                get_world_position(
                    node,
                    i,
                )
                for i in range(
                    node.drone_count
                )
            ]
        )
    )

    # RANDOM POSITIONS

    random_world_targets = []

    for i in range(
        node.drone_count
    ):

        target = (
            current_world_positions[
                i
            ].copy()
        )

        target[0] += (
            rng.uniform(
                *TEST2_RANDOM_X_RANGE
            )
        )

        target[1] += (
            rng.uniform(
                *TEST2_RANDOM_Y_RANGE
            )
        )

        random_world_targets.append(
            target
        )

    random_world_targets = (
        np.array(
            random_world_targets
        )
    )

    random_local_targets = (
        np.array(
            [
                world_target_to_local(
                    node,
                    i,
                    random_world_targets[
                        i
                    ],
                )
                for i in range(
                    node.drone_count
                )
            ]
        )
    )

    print()
    print(
        "[TEST 2] Randomizing positions "
        "using direct velocity control."
    )

    random_result = (
        move_to_targets(
            node,
            random_local_targets,
            TEST2_SPEED_MPS,
            "TEST 2 — RANDOMIZE POSITIONS",
        )
    )

    random_start_positions = (
        random_result[
            "initial_positions"
        ]
    )

    # SIGNAL

    print()
    print(
        f"[SIGNAL] Waiting "
        f"{TEST2_SIGNAL_DELAY_S:.2f} s."
    )

    hold_drones(
        node,
        TEST2_SIGNAL_DELAY_S,
    )

    # CURRENT WORLD POSITIONS

    random_world_positions = (
        np.array(
            [
                get_world_position(
                    node,
                    i,
                )
                for i in range(
                    node.drone_count
                )
            ]
        )
    )

    mean_x = (
        np.mean(
            random_world_positions[
                :,
                0,
            ]
        )
    )

    mean_y = (
        np.mean(
            random_world_positions[
                :,
                1,
            ]
        )
    )

    mean_z = (
        np.mean(
            random_world_positions[
                :,
                2,
            ]
        )
    )

    # CREATE GAZEBO-X LINE

    line_world_targets = []

    for i in range(
        node.drone_count
    ):

        offset = (
            i
            - (node.drone_count - 1)
            / 2.0
        ) * TEST2_LINE_SPACING_M

        line_world_targets.append(
            np.array(
                [
                    mean_x + offset,
                    mean_y,
                    mean_z,
                ],
                dtype=float,
            )
        )

    line_world_targets = (
        np.array(
            line_world_targets
        )
    )

    line_local_targets = (
        np.array(
            [
                world_target_to_local(
                    node,
                    i,
                    line_world_targets[
                        i
                    ],
                )
                for i in range(
                    node.drone_count
                )
            ]
        )
    )

    print()
    print(
        "[TEST 2] Forming horizontal "
        "Gazebo-X line."
    )

    line_result = (
        move_to_targets(
            node,
            line_local_targets,
            TEST2_SPEED_MPS,
            "TEST 2 — FORMATION LINE",
        )
    )

    final_positions = (
        line_result[
            "final_positions"
        ]
    )

    final_world_positions = (
        np.array(
            [
                get_world_position(
                    node,
                    i,
                )
                for i in range(
                    node.drone_count
                )
            ]
        )
    )

    line_errors = []

    for i in range(
        node.drone_count
    ):

        line_errors.append(
            np.linalg.norm(
                final_world_positions[
                    i
                ]
                - line_world_targets[
                    i
                ]
            )
        )

    line_rmse = math.sqrt(
        np.mean(
            np.array(
                line_errors
            ) ** 2
        )
    )

    print()
    print(
        "[TEST 2] RESULTS"
    )

    print(
        f"[TEST 2] Randomization time = "
        f"{random_result['duration']:.3f} s"
    )

    print(
        f"[TEST 2] Lineup time = "
        f"{line_result['duration']:.3f} s"
    )

    print(
        f"[TEST 2] Line spacing = "
        f"{TEST2_LINE_SPACING_M:.3f} m"
    )

    print(
        f"[TEST 2] Line RMSE = "
        f"{line_rmse:.3f} m"
    )

    print()
    print(
        "[TEST 2] Final positions:"
    )

    print_positions(
        node
    )

    save_test_result(
        test_name=(
            "TEST 2 — RANDOM "
            "POSITIONS -> LINE"
        ),
        duration=(
            random_result[
                "duration"
            ]
            + line_result[
                "duration"
            ]
        ),
        node=node,
        start_positions=(
            random_start_positions
        ),
        final_positions=(
            final_positions
        ),
        line_rmse=line_rmse,
    )


# TEST 3

def test_3(
    node,
    rng,
):

    print()
    print(
        "=" * 60
    )

    print(
        "TEST 3 — FLAG TURN / SPLIT / "
        "1 M TRANSLATION"
    )

    print(
        "=" * 60
    )

    direction = rng.choice(
        [
            "LEFT",
            "RIGHT",
        ]
    )

    majority_count = (
        node.drone_count // 2
        + 1
    )

    minority_count = (
        node.drone_count
        - majority_count
    )

    print(
        f"[FLAG] Signal direction = "
        f"{direction}"
    )

    print(
        f"[GROUP] Majority group = "
        f"{majority_count} drones"
    )

    print(
        f"[GROUP] Minority group = "
        f"{minority_count} drones"
    )

    # CURRENT WORLD POSITIONS

    current_world_positions = (
        np.array(
            [
                get_world_position(
                    node,
                    i,
                )
                for i in range(
                    node.drone_count
                )
            ]
        )
    )

    mean_x = (
        np.mean(
            current_world_positions[
                :,
                0,
            ]
        )
    )

    mean_y = (
        np.mean(
            current_world_positions[
                :,
                1,
            ]
        )
    )

    mean_z = (
        np.mean(
            current_world_positions[
                :,
                2,
            ]
        )
    )

    # INITIAL LINE

    line_world_targets = []

    for i in range(
        node.drone_count
    ):

        offset = (
            i
            - (node.drone_count - 1)
            / 2.0
        ) * TEST3_GROUP_SPACING_M

        line_world_targets.append(
            np.array(
                [
                    mean_x,
                    mean_y + offset,
                    mean_z,
                ],
                dtype=float,
            )
        )

    line_world_targets = (
        np.array(
            line_world_targets
        )
    )

    line_local_targets = (
        np.array(
            [
                world_target_to_local(
                    node,
                    i,
                    line_world_targets[
                        i
                    ],
                )
                for i in range(
                    node.drone_count
                )
            ]
        )
    )

    # INITIAL LINEUP

    print()
    print(
        "[TEST 3] Forming initial line."
    )

    lineup_result = (
        move_to_targets(
            node,
            line_local_targets,
            TEST3_SPEED_MPS,
            "TEST 3 — INITIAL LINEUP",
        )
    )

    # SPLIT DIRECTION

    if direction == "LEFT":

        majority_sign = (
            +1.0
        )

        minority_sign = (
            -1.0
        )

    else:

        majority_sign = (
            -1.0
        )

        minority_sign = (
            +1.0
        )

    # SPLIT

    split_world_targets = (
        line_world_targets.copy()
    )

    for i in range(
        node.drone_count
    ):

        if i < majority_count:

            split_world_targets[
                i,
                1,
            ] += (
                majority_sign
                * TEST3_GROUP_SPACING_M
            )

        else:

            split_world_targets[
                i,
                1,
            ] += (
                minority_sign
                * TEST3_GROUP_SPACING_M
            )

    split_local_targets = (
        np.array(
            [
                world_target_to_local(
                    node,
                    i,
                    split_world_targets[
                        i
                    ],
                )
                for i in range(
                    node.drone_count
                )
            ]
        )
    )

    print()
    print(
        f"[FLAG] FLAG TURNS "
        f"{direction}"
    )

    print(
        "[TEST 3] Splitting groups."
    )

    split_result = (
        move_to_targets(
            node,
            split_local_targets,
            TEST3_SPEED_MPS,
            "TEST 3 — GROUP SPLIT",
        )
    )

    # TRANSLATE BOTH GROUPS EAST

    translation_world_targets = (
        split_world_targets.copy()
    )

    translation_world_targets[
        :,
        0,
    ] += (
        TEST3_TRANSLATE_M
    )

    translation_local_targets = (
        np.array(
            [
                world_target_to_local(
                    node,
                    i,
                    translation_world_targets[
                        i
                    ],
                )
                for i in range(
                    node.drone_count
                )
            ]
        )
    )

    print()
    print(
        "[TEST 3] Translating both "
        "groups +1.00 m East."
    )

    translation_result = (
        move_to_targets(
            node,
            translation_local_targets,
            TEST3_SPEED_MPS,
            "TEST 3 — GROUP TRANSLATION",
        )
    )

    final_positions = (
        translation_result[
            "final_positions"
        ]
    )

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
        f"[TEST 3] Initial lineup time = "
        f"{lineup_result['duration']:.3f} s"
    )

    print(
        f"[TEST 3] Split time = "
        f"{split_result['duration']:.3f} s"
    )

    print(
        f"[TEST 3] Translation time = "
        f"{translation_result['duration']:.3f} s"
    )

    print()
    print(
        "[TEST 3] Final positions:"
    )

    print_positions(
        node
    )

    save_test_result(
        test_name=(
            "TEST 3 — FLAG / SPLIT / "
            "TRANSLATE"
        ),
        duration=(
            lineup_result[
                "duration"
            ]
            + split_result[
                "duration"
            ]
            + translation_result[
                "duration"
            ]
        ),
        node=node,
        start_positions=(
            lineup_result[
                "initial_positions"
            ]
        ),
        final_positions=(
            final_positions
        ),
        flag_direction=direction,
        majority_count=(
            majority_count
        ),
        minority_count=(
            minority_count
        ),
    )


# CSV

def save_test_result(
    test_name,
    duration,
    node,
    start_positions,
    final_positions,
    xy_mse_before=None,
    yz_mse_before=None,
    xy_mse_after=None,
    yz_mse_after=None,
    line_rmse=None,
    flag_direction=None,
    majority_count=None,
    minority_count=None,
):

    fields = [

        "timestamp",

        "test",

        "n_drones",

        "uav_namespace_pattern",

        "spawn_spacing_m",

        "spawn_altitude_m",

        "control_rate_hz",

        "position_topic",

        "vehicle_command_ack_topic",

        "vehicle_status_enabled",

        "px4_sys_id_start",

        "force_arm_enabled",

        "force_arm_value",

        "max_horizontal_speed_mps",

        "max_vertical_speed_mps",

        "position_tolerance_m",

        "speed_tolerance_mps",

        "movement_start_timeout_s",

        "movement_start_speed_mps",

        "movement_start_displacement_m",

        "test_timeout_s",

        "test_duration_s",

        "test1_translate_m",

        "test1_speed_mps",

        "test2_line_spacing_m",

        "test2_speed_mps",

        "test3_group_spacing_m",

        "test3_translate_m",

        "test3_speed_mps",

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

    path = Path(
        DEFAULT_RESULTS_CSV
    )

    file_exists = (
        path.exists()
    )

    row = {
        field: ""
        for field in fields
    }

    row[
        "timestamp"
    ] = (
        datetime.now().isoformat(
            timespec="seconds"
        )
    )

    row[
        "test"
    ] = test_name

    row[
        "n_drones"
    ] = node.drone_count

    row[
        "uav_namespace_pattern"
    ] = (
        "/fmu for UAV 0; "
        "/px4_<index>/fmu for UAV 1+"
    )

    row[
        "spawn_spacing_m"
    ] = SPAWN_SPACING_M

    row[
        "spawn_altitude_m"
    ] = SPAWN_ALTITUDE_M

    row[
        "control_rate_hz"
    ] = CONTROL_RATE_HZ

    row[
        "position_topic"
    ] = LOCAL_POSITION_TOPIC

    row[
        "vehicle_command_ack_topic"
    ] = VEHICLE_COMMAND_ACK_TOPIC

    row[
        "vehicle_status_enabled"
    ] = False

    row[
        "px4_sys_id_start"
    ] = PX4_SYS_ID_START

    row[
        "force_arm_enabled"
    ] = True

    row[
        "force_arm_value"
    ] = FORCE_ARM_VALUE

    row[
        "max_horizontal_speed_mps"
    ] = MAX_HORIZONTAL_SPEED_MPS

    row[
        "max_vertical_speed_mps"
    ] = MAX_VERTICAL_SPEED_MPS

    row[
        "position_tolerance_m"
    ] = POSITION_TOLERANCE_M

    row[
        "speed_tolerance_mps"
    ] = SPEED_TOLERANCE_MPS

    row[
        "movement_start_timeout_s"
    ] = MOVEMENT_START_TIMEOUT_S

    row[
        "movement_start_speed_mps"
    ] = MOVEMENT_START_SPEED_MPS

    row[
        "movement_start_displacement_m"
    ] = MOVEMENT_START_DISPLACEMENT_M

    row[
        "test_timeout_s"
    ] = TEST_TIMEOUT_S

    row[
        "test_duration_s"
    ] = duration

    row[
        "test1_translate_m"
    ] = TEST1_TRANSLATE_M

    row[
        "test1_speed_mps"
    ] = TEST1_SPEED_MPS

    row[
        "test2_line_spacing_m"
    ] = TEST2_LINE_SPACING_M

    row[
        "test2_speed_mps"
    ] = TEST2_SPEED_MPS

    row[
        "test3_group_spacing_m"
    ] = TEST3_GROUP_SPACING_M

    row[
        "test3_translate_m"
    ] = TEST3_TRANSLATE_M

    row[
        "test3_speed_mps"
    ] = TEST3_SPEED_MPS

    row[
        "xy_mse_before_m2"
    ] = xy_mse_before

    row[
        "yz_mse_before_m2"
    ] = yz_mse_before

    row[
        "xy_mse_after_m2"
    ] = xy_mse_after

    row[
        "yz_mse_after_m2"
    ] = yz_mse_after

    row[
        "line_rmse_m"
    ] = line_rmse

    row[
        "flag_direction"
    ] = flag_direction

    row[
        "majority_count"
    ] = majority_count

    row[
        "minority_count"
    ] = minority_count

    # START POSITIONS

    if isinstance(
        start_positions,
        np.ndarray,
    ):

        row[
            "start_positions"
        ] = repr(
            np.round(
                start_positions,
                4,
            ).tolist()
        )

    else:

        row[
            "start_positions"
        ] = repr(
            start_positions
        )

    # FINAL POSITIONS

    if isinstance(
        final_positions,
        np.ndarray,
    ):

        row[
            "final_positions"
        ] = repr(
            np.round(
                final_positions,
                4,
            ).tolist()
        )

    else:

        row[
            "final_positions"
        ] = repr(
            final_positions
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
            not file_exists
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
        f"      {path.resolve()}"
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

def main():

    global DEFAULT_RESULTS_CSV

    parser = (
        argparse.ArgumentParser()
    )

    parser.add_argument(
        "--n",
        type=int,
        default=DEFAULT_DRONE_COUNT,
        help=(
            "Number of drones."
        ),
    )

    parser.add_argument(
        "--seed",
        type=int,
        default=DEFAULT_RANDOM_SEED,
        help=(
            "Random seed."
        ),
    )

    parser.add_argument(
        "--csv",
        type=str,
        default=DEFAULT_RESULTS_CSV,
        help=(
            "CSV output path."
        ),
    )

    args = (
        parser.parse_args()
    )

    if args.n < 2:

        raise ValueError(
            "At least 2 drones "
            "are required."
        )

    # IMPORTANT: Do NOT use: global RANDOM_SEED This fixes the scoping error you
    # encountered earlier. The seed is passed directly into the RNG

    rng = random.Random(
        args.seed
    )


    DEFAULT_RESULTS_CSV = (
        args.csv
    )

    print()
    print(
        "=" * 60
    )

    print(
        "DRONE SWARM CONTROLLER"
    )

    print(
        "=" * 60
    )

    print(
        f"[CONFIG] Number of drones = "
        f"{args.n}"
    )

    print(
        f"[CONFIG] Control rate = "
        f"{CONTROL_RATE_HZ:.1f} Hz"
    )

    print(
        "[CONFIG] Control mode = "
        "OFFBOARD VELOCITY"
    )

    print(
        f"[CONFIG] Maximum horizontal "
        f"velocity = "
        f"{MAX_HORIZONTAL_SPEED_MPS:.2f} m/s"
    )

    print(
        f"[CONFIG] Force arm in SITL = "
        f"True"
    )

    print(
        f"[CONFIG] Force arm value = "
        f"{FORCE_ARM_VALUE:.0f}"
    )

    print(
        "[CONFIG] VehicleStatus "
        "subscription disabled."
    )

    print(
        "[CONFIG] PX4 local position = "
        "NED "
        "(x=North, y=East, z=Down)"
    )

    print(
        "[CONFIG] Actual ROS namespaces:"
    )

    for i in range(
        args.n
    ):

        namespace = (
            DroneSwarmNode.get_uav_namespace(
                i
            )
        )

        if namespace == "":

            namespace_display = "/"

        else:

            namespace_display = (
                namespace
            )

        print(
            f"           UAV {i}: "
            f"{namespace_display}"
        )

    print(
        "[CONFIG] OFFBOARD command = "
        "param1=1, param2=6"
    )

    print(
        "[CONFIG] ARM command = "
        "param1=1, param2=21196"
    )

    print(
        f"[CONFIG] CSV = "
        f"{DEFAULT_RESULTS_CSV}"
    )

    print(
        "=" * 60
    )

    rclpy.init()

    node = (
        DroneSwarmNode(
            args.n
        )
    )

    try:

        initialize_drones(
            node
        )

        while rclpy.ok():

            print_menu()

            selection = (
                input(
                    "Select a test: "
                )
                .strip()
                .lower()
            )

            # TEST 1

            if selection == "1":

                try:

                    test_1(
                        node
                    )

                    hold_drones(
                        node,
                        0.5,
                    )

                except Exception as error:

                    node.stop_drones()

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
                        f"TEST 1: "
                        f"{error}"
                    )

                    print()

            # TEST 2

            elif selection == "2":

                try:

                    test_2(
                        node,
                        rng,
                    )

                    hold_drones(
                        node,
                        0.5,
                    )

                except Exception as error:

                    node.stop_drones()

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
                        f"TEST 2: "
                        f"{error}"
                    )

                    print()

            # TEST 3

            elif selection == "3":

                try:

                    test_3(
                        node,
                        rng,
                    )

                    hold_drones(
                        node,
                        0.5,
                    )

                except Exception as error:

                    node.stop_drones()

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
                        f"TEST 3: "
                        f"{error}"
                    )

                    print()

            # QUIT

            elif selection == "q":

                print()
                print(
                    "[QUIT] Exiting swarm test."
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
            "[INTERRUPT] "
            "Keyboard interrupt received."
        )

    finally:

        # STOP VELOCITY

        node.stop_drones()

        spin_for(
            node,
            0.5,
        )

        # LAND

        print()
        print(
            "[SHUTDOWN] Sending LAND commands..."
        )

        for i in range(
            node.drone_count
        ):

            try:

                node.land_drone(
                    i
                )

            except Exception:

                pass

        spin_for(
            node,
            1.0,
        )

        # DESTROY ROS NODE

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


# RUN

if __name__ == "__main__":

    main()
