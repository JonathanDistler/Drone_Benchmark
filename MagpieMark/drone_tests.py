#!/usr/bin/env python3

# PX4 / GAZEBO OBSTACLE COURSE Velocity-Controlled Test Suite This script connects to PX4
# through MAVSDK, waits until the vehicle is airborne, enters Offboard mode, and then runs ONE
# selected test COURSE COORDINATE SYSTEM The obstacle course is defined in the Gazebo world
# Logical course coordinates: forward = direction through the obstacle course lateral =
# left/right across the course down = vertical downward For this world, the conversion to PX4
# NED is: PX4 North = course lateral PX4 East = course forward PX4 Down = course down The
# vehicle yaw is held at 90 degrees so that the nose points approximately along the
# course-forward direction. TEST 1: python drone_tests.py 1 TEST 2: python drone_tests.py 2
# TEST 3: python drone_tests.py 3 The lateral positioning portions are intentionally NOT
# included in the measured flight time, distance, or velocity metrics

import asyncio
import csv
import math
import random
import os
import sys
import time

from dataclasses import dataclass, field
from typing import List, Optional, Tuple

from mavsdk import System

from mavsdk.offboard import (
    OffboardError,
    VelocityNedYaw,
    PositionNedYaw,
)


# CONNECTION

SYSTEM_ADDRESS = "udpin://127.0.0.1:14540"


# CONTROL

CONTROL_RATE_HZ = 20.0
CONTROL_PERIOD = 1.0 / CONTROL_RATE_HZ


# VELOCITY LIMITS

MAX_HORIZONTAL_SPEED_MPS = 3.0
MAX_VERTICAL_SPEED_MPS = 0.75


# EXPECTED TAKEOFF

MIN_AIRBORNE_ALTITUDE_M = 1.0
AIRBORNE_TIMEOUT_S = 20.0


# YAW

COURSE_YAW_DEG = 90.0



# MOVE TO A POSITION TARGET
async def move_to_ned_position(
    drone: System,
    target_north: float,
    target_east: float,
    target_down: float,
    name: str,
    tolerance_m: float = 0.20,
    speed_tolerance_mps: float = 0.25,
    timeout_s: float = 30.0,
):
    # Move to one absolute PX4 local-NED position using PositionNedYaw only. The same
    # absolute position target is continuously sent until the vehicle reaches the target
    # and is moving slowly. No velocity command is generated
    print()
    print("-" * 60)
    print(f"[POSITION] {name}")
    print(
        f"[TARGET] N={target_north:+.2f}, "
        f"E={target_east:+.2f}, D={target_down:+.2f}"
    )
    print(
        f"[CONTROL] PositionNedYaw only | "
        f"tolerance={tolerance_m:.2f} m | "
        f"speed tolerance={speed_tolerance_mps:.2f} m/s"
    )
    print("-" * 60)

    start = time.monotonic()
    next_print = start

    while True:
        now = time.monotonic()

        north_error = target_north - STATE.north_m
        east_error = target_east - STATE.east_m
        down_error = target_down - STATE.down_m

        position_error = math.sqrt(
            north_error ** 2
            + east_error ** 2
            + down_error ** 2
        )

        horizontal_speed = math.sqrt(
            STATE.north_velocity_mps ** 2
            + STATE.east_velocity_mps ** 2
        )

        await drone.offboard.set_position_ned(
            PositionNedYaw(
                float(target_north),
                float(target_east),
                float(target_down),
                float(COURSE_YAW_DEG),
            )
        )

        if now >= next_print:
            print(
                f"[POSITION] {name} | "
                f"pos N={STATE.north_m:+.2f} "
                f"E={STATE.east_m:+.2f} "
                f"D={STATE.down_m:+.2f} | "
                f"error={position_error:.2f} m | "
                f"speed={horizontal_speed:.2f} m/s"
            )
            next_print = now + 0.50

        if (
            position_error <= tolerance_m
            and horizontal_speed <= speed_tolerance_mps
        ):
            await drone.offboard.set_position_ned(
                PositionNedYaw(
                    float(target_north),
                    float(target_east),
                    float(target_down),
                    float(COURSE_YAW_DEG),
                )
            )

            print()
            print(
                f"[GOAL] {name} REACHED | "
                f"pos N={STATE.north_m:+.2f} "
                f"E={STATE.east_m:+.2f} "
                f"D={STATE.down_m:+.2f} | "
                f"error={position_error:.2f} m | "
                f"speed={horizontal_speed:.2f} m/s"
            )
            return

        if now - start > timeout_s:
            raise RuntimeError(
                f"Position target '{name}' was not reached "
                f"within {timeout_s:.1f} s."
            )

        await asyncio.sleep(CONTROL_PERIOD)

# TEST 1 — CONE WEAVE

TEST1_FORWARD_SPEED = 1.0
TEST1_SIDE_SPEED = 0.75

TEST1_SEGMENTS = [
    (
        "FORWARD",
        TEST1_FORWARD_SPEED,
        0.0,
        0.0,
        4.0,
    ),
    (
        "WEAVE RIGHT",
        TEST1_FORWARD_SPEED,
        TEST1_SIDE_SPEED,
        0.0,
        2.0,
    ),
    (
        "FORWARD",
        TEST1_FORWARD_SPEED,
        0.0,
        0.0,
        3.0,
    ),
    (
        "WEAVE LEFT",
        TEST1_FORWARD_SPEED,
        -TEST1_SIDE_SPEED,
        0.0,
        4.0,
    ),
    (
        "FORWARD",
        TEST1_FORWARD_SPEED,
        0.0,
        0.0,
        3.0,
    ),
    (
        "WEAVE RIGHT",
        TEST1_FORWARD_SPEED,
        TEST1_SIDE_SPEED,
        0.0,
        4.0,
    ),
    (
        "FORWARD",
        TEST1_FORWARD_SPEED,
        0.0,
        0.0,
        3.0,
    ),
    (
        "WEAVE LEFT",
        TEST1_FORWARD_SPEED,
        -TEST1_SIDE_SPEED,
        0.0,
        4.0,
    ),
    (
        "FORWARD",
        TEST1_FORWARD_SPEED,
        0.0,
        0.0,
        4.0,
    ),
]


# TEST 2 — FLAG

TEST2_SIDE_SPEED = 1.50
TEST2_FORWARD_SPEED = 2.00

# SDF flag lane is Y = -8 m. Starting lane is Y = 0 m. Therefore translate RIGHT by 8 m
TEST2_TRANSLATE_DISTANCE_M = 8.0
TEST2_TRANSLATE_TIME_S = (
    TEST2_TRANSLATE_DISTANCE_M / TEST2_SIDE_SPEED
)

TEST2_APPROACH_TIME_S = 7.0
TEST2_PASS_TIME_S = 6.0

# Dynamic flag component. The FLAG itself is assigned a random direction (LEFT or RIGHT) once
# per run. The drone then follows that same direction when passing the flag
TEST2_FLAG_OFFSET_M = 2.5
TEST2_RANDOM_SEED = None


# TEST 3 — BOX

TEST3_FORWARD_SPEED = 2.00

# SDF box center is Y = +7.5 m. Move an additional 1.5 m LEFT from the box center for clearance
TEST3_TRANSLATE_DISTANCE_M = 9.0
TEST3_TRANSLATE_TIME_S = (
    TEST3_TRANSLATE_DISTANCE_M / TEST2_SIDE_SPEED
)

TEST3_ENTRY_TIME_S = 5.0
TEST3_THROUGH_TIME_S = 8.0

# Box height target for Test 3 The bottom of the box opening is 1.3335 m above the ground
# Target the drone at .5 m above the ground so it stays within the opening
TEST3_BOX_ALTITUDE_M = .5

# Positional acceptance for Test 3 positioning
TEST3_POSITION_TOLERANCE_M = 0.20
TEST3_SPEED_TOLERANCE_MPS = 0.25


# OUTPUT

OUTPUT_ROOT = os.path.expanduser(
    "~/drone_obstacle_course/test_results"
)


# VEHICLE STATE

@dataclass
class VehicleState:
    connected: bool = False
    armed: bool = False
    in_air: bool = False

    north_m: float = 0.0
    east_m: float = 0.0
    down_m: float = 0.0

    north_velocity_mps: float = 0.0
    east_velocity_mps: float = 0.0
    down_velocity_mps: float = 0.0

    yaw_deg: float = 0.0
    relative_altitude_m: float = 0.0


STATE = VehicleState()


# FLIGHT SAMPLE

@dataclass
class FlightSample:
    time_s: float

    north_m: float
    east_m: float
    down_m: float

    velocity_north_mps: float
    velocity_east_mps: float
    velocity_down_mps: float

    command_forward_mps: float
    command_lateral_mps: float
    command_down_mps: float

    command_north_mps: float
    command_east_mps: float

    yaw_deg: float
    altitude_m: float
    test_segment: str


# METRICS

@dataclass
class Metrics:
    start_time: float = 0.0

    samples: List[FlightSample] = field(
        default_factory=list
    )

    total_distance_m: float = 0.0
    horizontal_distance_m: float = 0.0

    maximum_speed_mps: float = 0.0
    maximum_horizontal_speed_mps: float = 0.0

    maximum_altitude_m: float = 0.0
    minimum_altitude_m: float = float("inf")

    previous_position: Optional[Tuple[float, float, float]] = None
    previous_time: Optional[float] = None

    # Dynamic flag results (Test 2)
    flag_direction: str = ""
    actual_flag_lateral_change_m: float = 0.0
    flag_direction_correct: Optional[bool] = None


# RESET MEASURED METRICS

def reset_metrics(metrics: Metrics):
    # Start a fresh measured interval. This is deliberately called ONLY when the vehicle
    # is in the correct lane for the selected obstacle. Any positioning flight before this
    # call is excluded from the reported metrics

    metrics.start_time = time.perf_counter()
    metrics.samples.clear()

    metrics.total_distance_m = 0.0
    metrics.horizontal_distance_m = 0.0

    metrics.maximum_speed_mps = 0.0
    metrics.maximum_horizontal_speed_mps = 0.0

    metrics.maximum_altitude_m = 0.0
    metrics.minimum_altitude_m = float("inf")

    metrics.previous_position = None
    metrics.previous_time = None

    metrics.flag_direction = ""
    metrics.actual_flag_lateral_change_m = 0.0
    metrics.flag_direction_correct = None


# PRINT HEADER

def print_header():
    print()
    print("=" * 60)
    print("PX4 / GAZEBO OBSTACLE COURSE")
    print("=" * 60)
    print("CONTROL MODE: VELOCITY")
    print("COURSE FRAME: GAZEBO WORLD")
    print("NED POSITION CONTROL: DISABLED")
    print("METRICS: START ONLY AFTER CORRECT OBSTACLE LANE")
    print("=" * 60)
    print()


# PRINT VEHICLE STATUS

def print_vehicle_status():
    speed = math.sqrt(
        STATE.north_velocity_mps ** 2
        + STATE.east_velocity_mps ** 2
        + STATE.down_velocity_mps ** 2
    )

    print(
        "[DRONE] "
        f"NED=["
        f"{STATE.north_m:+7.3f}, "
        f"{STATE.east_m:+7.3f}, "
        f"{STATE.down_m:+7.3f}] m   "
        f"VEL=["
        f"{STATE.north_velocity_mps:+6.2f}, "
        f"{STATE.east_velocity_mps:+6.2f}, "
        f"{STATE.down_velocity_mps:+6.2f}] m/s   "
        f"ALT={STATE.relative_altitude_m:5.2f} m   "
        f"SPEED={speed:4.2f} m/s   "
        f"YAW={STATE.yaw_deg:+7.2f} deg"
    )


# CONNECTION TELEMETRY

async def monitor_connection(drone: System):
    try:
        async for state in drone.core.connection_state():
            STATE.connected = state.is_connected
    except asyncio.CancelledError:
        pass


# POSITION TELEMETRY

async def monitor_position(drone: System):
    try:
        async for position in drone.telemetry.position():
            STATE.relative_altitude_m = float(
                position.relative_altitude_m
            )
    except asyncio.CancelledError:
        pass
    except Exception as exc:
        print(
            f"[TELEMETRY] Position monitor stopped: {exc}"
        )


# NED TELEMETRY

async def monitor_ned(drone: System):
    try:
        async for pv in drone.telemetry.position_velocity_ned():
            STATE.north_m = float(pv.position.north_m)
            STATE.east_m = float(pv.position.east_m)
            STATE.down_m = float(pv.position.down_m)

            STATE.north_velocity_mps = float(
                pv.velocity.north_m_s
            )
            STATE.east_velocity_mps = float(
                pv.velocity.east_m_s
            )
            STATE.down_velocity_mps = float(
                pv.velocity.down_m_s
            )
    except asyncio.CancelledError:
        pass
    except Exception as exc:
        print(
            f"[TELEMETRY] NED monitor stopped: {exc}"
        )


# HEADING TELEMETRY

async def monitor_heading(drone: System):
    try:
        async for heading in drone.telemetry.heading():
            STATE.yaw_deg = float(heading.heading_deg)
    except asyncio.CancelledError:
        pass
    except Exception as exc:
        print(
            f"[TELEMETRY] Heading monitor stopped: {exc}"
        )


# ARMED TELEMETRY

async def monitor_armed(drone: System):
    try:
        async for armed in drone.telemetry.armed():
            STATE.armed = bool(armed)
    except asyncio.CancelledError:
        pass
    except Exception as exc:
        print(
            f"[TELEMETRY] Armed monitor stopped: {exc}"
        )


# IN-AIR TELEMETRY

async def monitor_in_air(drone: System):
    try:
        async for in_air in drone.telemetry.in_air():
            STATE.in_air = bool(in_air)
    except asyncio.CancelledError:
        pass
    except Exception as exc:
        print(
            f"[TELEMETRY] In-air monitor stopped: {exc}"
        )


# START TELEMETRY

async def start_telemetry(drone: System):
    print("[DRONE] Starting telemetry monitors...")

    tasks = [
        asyncio.create_task(monitor_connection(drone)),
        asyncio.create_task(monitor_position(drone)),
        asyncio.create_task(monitor_ned(drone)),
        asyncio.create_task(monitor_heading(drone)),
        asyncio.create_task(monitor_armed(drone)),
        asyncio.create_task(monitor_in_air(drone)),
    ]

    await asyncio.sleep(2.0)

    print("[DRONE] Checking telemetry...")
    print(
        "[DRONE] Position = "
        f"[{STATE.north_m:.2f}, "
        f"{STATE.east_m:.2f}, "
        f"{STATE.down_m:.2f}]"
    )
    print(
        "[DRONE] Velocity = "
        f"[{STATE.north_velocity_mps:.2f}, "
        f"{STATE.east_velocity_mps:.2f}, "
        f"{STATE.down_velocity_mps:.2f}]"
    )
    print(
        "[DRONE] Relative altitude = "
        f"{STATE.relative_altitude_m:.2f} m"
    )
    print(f"[DRONE] In air = {STATE.in_air}")
    print(f"[DRONE] Armed = {STATE.armed}")
    print(f"[DRONE] Yaw = {STATE.yaw_deg:.2f} deg")

    return tasks


# CONNECT

async def connect_drone() -> System:
    drone = System()

    print(f"[DRONE] Connecting to {SYSTEM_ADDRESS}")
    await drone.connect(system_address=SYSTEM_ADDRESS)

    print("[DRONE] Waiting for PX4 connection...")

    async for state in drone.core.connection_state():
        if state.is_connected:
            STATE.connected = True
            print("[DRONE] Connected.")
            return drone


# COURSE FRAME -> PX4 NED

def course_to_ned(
    forward_mps: float,
    lateral_mps: float,
    down_mps: float,
):
    # Convert logical obstacle-course velocity into PX4 NED. Course forward -> PX4 East
    # Course lateral -> PX4 North Course down -> PX4 Down

    north_mps = lateral_mps
    east_mps = forward_mps

    return north_mps, east_mps, down_mps


# SEND VELOCITY

async def send_course_velocity(
    drone: System,
    forward_mps: float,
    lateral_mps: float,
    down_mps: float,
    yaw_deg: float = COURSE_YAW_DEG,
):
    # Send velocity in the logical obstacle-course frame

    (
        north_mps,
        east_mps,
        down_mps,
    ) = course_to_ned(
        forward_mps,
        lateral_mps,
        down_mps,
    )

    horizontal_speed = math.sqrt(
        north_mps ** 2
        + east_mps ** 2
    )

    if horizontal_speed > MAX_HORIZONTAL_SPEED_MPS:
        scale = (
            MAX_HORIZONTAL_SPEED_MPS
            / horizontal_speed
        )
        north_mps *= scale
        east_mps *= scale

    down_mps = max(
        -MAX_VERTICAL_SPEED_MPS,
        min(MAX_VERTICAL_SPEED_MPS, down_mps),
    )

    await drone.offboard.set_velocity_ned(
        VelocityNedYaw(
            float(north_mps),
            float(east_mps),
            float(down_mps),
            float(yaw_deg),
        )
    )


# UPDATE METRICS

def update_metrics(
    metrics: Metrics,
    forward_mps: float,
    lateral_mps: float,
    down_mps: float,
    segment_name: str,
):
    # Record one measured sample. metrics.start_time MUST already have been initialized at
    # the point where scoring is supposed to begin

    now = time.perf_counter()

    if metrics.start_time <= 0.0:
        metrics.start_time = now

    position = (
        STATE.north_m,
        STATE.east_m,
        STATE.down_m,
    )

    if (
        metrics.previous_position is not None
        and metrics.previous_time is not None
    ):
        dx = (
            position[0]
            - metrics.previous_position[0]
        )
        dy = (
            position[1]
            - metrics.previous_position[1]
        )
        dz = (
            position[2]
            - metrics.previous_position[2]
        )

        step_distance = math.sqrt(
            dx ** 2
            + dy ** 2
            + dz ** 2
        )

        step_horizontal = math.sqrt(
            dx ** 2
            + dy ** 2
        )

        metrics.total_distance_m += step_distance
        metrics.horizontal_distance_m += step_horizontal

    speed = math.sqrt(
        STATE.north_velocity_mps ** 2
        + STATE.east_velocity_mps ** 2
        + STATE.down_velocity_mps ** 2
    )

    horizontal_speed = math.sqrt(
        STATE.north_velocity_mps ** 2
        + STATE.east_velocity_mps ** 2
    )

    metrics.maximum_speed_mps = max(
        metrics.maximum_speed_mps,
        speed,
    )

    metrics.maximum_horizontal_speed_mps = max(
        metrics.maximum_horizontal_speed_mps,
        horizontal_speed,
    )

    metrics.maximum_altitude_m = max(
        metrics.maximum_altitude_m,
        STATE.relative_altitude_m,
    )

    metrics.minimum_altitude_m = min(
        metrics.minimum_altitude_m,
        STATE.relative_altitude_m,
    )

    (
        command_north_mps,
        command_east_mps,
        _,
    ) = course_to_ned(
        forward_mps,
        lateral_mps,
        down_mps,
    )

    sample = FlightSample(
        time_s=now - metrics.start_time,
        north_m=STATE.north_m,
        east_m=STATE.east_m,
        down_m=STATE.down_m,
        velocity_north_mps=STATE.north_velocity_mps,
        velocity_east_mps=STATE.east_velocity_mps,
        velocity_down_mps=STATE.down_velocity_mps,
        command_forward_mps=forward_mps,
        command_lateral_mps=lateral_mps,
        command_down_mps=down_mps,
        command_north_mps=command_north_mps,
        command_east_mps=command_east_mps,
        yaw_deg=STATE.yaw_deg,
        altitude_m=STATE.relative_altitude_m,
        test_segment=segment_name,
    )

    metrics.samples.append(sample)

    metrics.previous_position = position
    metrics.previous_time = now


# WAIT UNTIL AIRBORNE

async def wait_until_airborne(drone: System):
    print()
    print("[DRONE] Waiting for the vehicle to be airborne...")

    start = time.monotonic()

    while True:
        if (
            STATE.in_air
            or STATE.relative_altitude_m >= MIN_AIRBORNE_ALTITUDE_M
        ):
            print("[DRONE] Vehicle is airborne.")
            print(
                f"[DRONE] Altitude = "
                f"{STATE.relative_altitude_m:.2f} m"
            )
            return

        if time.monotonic() - start > AIRBORNE_TIMEOUT_S:
            raise RuntimeError(
                "Vehicle did not become airborne. "
                "Run 'commander arm -f' and "
                "'commander takeoff' in the PX4 terminal "
                "before starting this Python program."
            )

        await asyncio.sleep(0.1)


# START OFFBOARD

async def start_offboard(drone: System):
    print()
    print("[OFFBOARD] Sending initial zero-velocity setpoints...")

    await send_course_velocity(
        drone,
        0.0,
        0.0,
        0.0,
        COURSE_YAW_DEG,
    )

    for _ in range(20):
        await send_course_velocity(
            drone,
            0.0,
            0.0,
            0.0,
            COURSE_YAW_DEG,
        )
        await asyncio.sleep(CONTROL_PERIOD)

    print("[OFFBOARD] Starting...")

    try:
        await drone.offboard.start()
    except OffboardError as exc:
        print()
        print("[ERROR] Offboard start failed.")
        print(f"        {exc}")
        print()
        print("Vehicle status:")
        print(f"    Armed: {STATE.armed}")
        print(f"    In air: {STATE.in_air}")
        print(
            f"    Altitude: "
            f"{STATE.relative_altitude_m:.2f} m"
        )
        raise

    print("[OFFBOARD] Active.")


# HOLD ZERO VELOCITY

async def hold_zero_velocity(
    drone: System,
    metrics: Optional[Metrics],
    duration_s: float,
    name: str,
    record_metrics: bool = True,
):
    # Hold zero velocity for a specified time. record_metrics=False is used for
    # positioning/transition holds that must NOT count toward obstacle-test performance

    print(
        f"[DRONE] Holding for {duration_s:.1f} s..."
    )

    end_time = time.monotonic() + duration_s

    while time.monotonic() < end_time:
        await send_course_velocity(
            drone,
            0.0,
            0.0,
            0.0,
            COURSE_YAW_DEG,
        )

        if record_metrics and metrics is not None:
            update_metrics(
                metrics,
                0.0,
                0.0,
                0.0,
                name,
            )

        await asyncio.sleep(CONTROL_PERIOD)


# RUN VELOCITY SEGMENT

async def run_velocity_segment(
    drone: System,
    metrics: Optional[Metrics],
    name: str,
    forward_mps: float,
    lateral_mps: float,
    down_mps: float,
    duration_s: float,
    record_metrics: bool = True,
):
    # Run a constant course-frame velocity segment. When record_metrics=False, this
    # segment is considered positioning only and contributes nothing to the measured
    # flight metrics

    print()
    print("-" * 60)
    print(f"[TEST] {name}")
    print(
        "[TEST] Course velocity = "
        f"[forward={forward_mps:+.2f}, "
        f"lateral={lateral_mps:+.2f}, "
        f"down={down_mps:+.2f}] m/s"
    )

    (
        north_cmd,
        east_cmd,
        down_cmd,
    ) = course_to_ned(
        forward_mps,
        lateral_mps,
        down_mps,
    )

    print(
        "[TEST] PX4 NED command = "
        f"[N={north_cmd:+.2f}, "
        f"E={east_cmd:+.2f}, "
        f"D={down_cmd:+.2f}] m/s"
    )
    print(f"[TEST] Yaw = {COURSE_YAW_DEG:+.1f} deg")
    print(f"[TEST] Duration = {duration_s:.1f} s")
    print(
        f"[TEST] Metrics recorded = {record_metrics}"
    )
    print("-" * 60)

    start = time.monotonic()
    next_print = start

    while True:
        now = time.monotonic()
        elapsed = now - start

        if elapsed >= duration_s:
            break

        await send_course_velocity(
            drone,
            forward_mps,
            lateral_mps,
            down_mps,
            COURSE_YAW_DEG,
        )

        if record_metrics and metrics is not None:
            update_metrics(
                metrics,
                forward_mps,
                lateral_mps,
                down_mps,
                name,
            )

        if now >= next_print:
            print_vehicle_status()
            next_print = now + 1.0

        await asyncio.sleep(CONTROL_PERIOD)


# TEST 1 — CONE WEAVE

async def run_test_1(
    drone: System,
    metrics: Metrics,
):
    print()
    print("=" * 60)
    print("TEST 1: CONE WEAVE")
    print("=" * 60)
    print()
    print("Course direction: FORWARD")
    print("Pattern: RIGHT -> LEFT -> RIGHT")
    print("Velocity commands: ENABLED")
    print()

    # Test 1 starts at the correct lane, so the measured timer starts here
    reset_metrics(metrics)
    print(
        "[METRICS] Timer STARTED: Test 1 begins in the cone lane."
    )
    print()

    for segment in TEST1_SEGMENTS:
        (
            name,
            forward_mps,
            lateral_mps,
            down_mps,
            duration_s,
        ) = segment

        await run_velocity_segment(
            drone,
            metrics,
            name,
            forward_mps,
            lateral_mps,
            down_mps,
            duration_s,
            record_metrics=True,
        )

    # Hold after completion, but this is not part of the measured test
    await hold_zero_velocity(
        drone,
        None,
        2.0,
        "TEST 1 FINISH HOLD",
        record_metrics=False,
    )

    print()
    print("=" * 60)
    print("TEST 1 COMPLETE")
    print("=" * 60)


# TEST 2 — FLAG

async def run_test_2(
    drone: System,
    metrics: Metrics,
):
    # Test 2 sequence: 1. Translate RIGHT into the flag lane (Y = -8 m). This positioning
    # is NOT measured. 2. Move FORWARD until the drone is approximately 1 m in front of
    # the flag. This is also NOT measured. 3. Randomly orient the flag LEFT or RIGHT. 4
    # Start the measured timer here, once the drone is lined up immediately in front of
    # the flag. 5. Move 0.90 m in the SAME direction as the flag. 6. Stop and print [GOAL]
    # when the lateral goal is reached. 7. Continue FORWARD past the flag. The dynamic
    # component is therefore the flag orientation, not a random choice made independently
    # by the drone

    FLAG_GOAL_TOLERANCE_M = 0.20
    FLAG_GOAL_SPEED_TOLERANCE_MPS = 0.25

    # Test 2 SDF: flag = (X=5, Y=-8) Test 2 start: start = (X=-1, Y=-8) Stop approximately
    # 1 m before the flag: X=4, Y=-8 This requires 5 m of FORWARD travel after the lateral
    # lane translation
    TEST2_FLAG_APPROACH_DISTANCE_M = 5.0

    # Keep the random-direction maneuver inside 1 m
    TEST2_FLAG_LATERAL_DISTANCE_M = 0.90

    print()
    print("=" * 60)
    print("TEST 2: FLAG")
    print("=" * 60)
    print()

    # STEP 1: move into the flag lane. This positioning is NOT measured
    print("STEP 1: TRANSLATE RIGHT TO FLAG LANE")
    print("        Starting lane: Y = 0")
    print("        Flag lane:     Y = -8")
    print(
        f"        Lateral speed = {-TEST2_SIDE_SPEED:+.2f} m/s"
    )
    print(
        f"        Translation distance = "
        f"{TEST2_TRANSLATE_DISTANCE_M:.2f} m"
    )
    print(
        f"        Translation time = "
        f"{TEST2_TRANSLATE_TIME_S:.2f} s"
    )
    print("        Metrics during translation = DISABLED")
    print()

    # RIGHT = negative course lateral = negative PX4 North
    await run_velocity_segment(
        drone,
        None,
        "FLAG: TRANSLATE RIGHT",
        0.0,
        -TEST2_SIDE_SPEED,
        0.0,
        TEST2_TRANSLATE_TIME_S,
        record_metrics=False,
    )

    await hold_zero_velocity(
        drone,
        None,
        0.25,
        "FLAG LANE ARRIVAL",
        record_metrics=False,
    )

    # STEP 2: move FORWARD until approximately 1 m before flag. This is positioning and is
    # NOT measured
    print()
    print("STEP 2: LINE UP WITH THE FLAG")
    print("        Flag center:      X = +5.00, Y = -8.00")
    print("        Stop point:       X = +4.00, Y = -8.00")
    print("        Distance to flag: 1.00 m")
    print(
        f"        Forward speed = "
        f"{TEST2_FORWARD_SPEED:+.2f} m/s"
    )
    print(
        f"        Approach distance = "
        f"{TEST2_FLAG_APPROACH_DISTANCE_M:.2f} m"
    )
    print("        Metrics during approach = DISABLED")
    print()

    await run_velocity_segment(
        drone,
        None,
        "FLAG: FORWARD LINE-UP",
        TEST2_FORWARD_SPEED,
        0.0,
        0.0,
        TEST2_FLAG_APPROACH_DISTANCE_M / TEST2_FORWARD_SPEED,
        record_metrics=False,
    )

    await hold_zero_velocity(
        drone,
        None,
        0.25,
        "FLAG FRONT ARRIVAL",
        record_metrics=False,
    )

    # STEP 3: determine the FLAG orientation. The drone follows the flag's direction
    if TEST2_RANDOM_SEED is not None:
        random.seed(TEST2_RANDOM_SEED)

    flag_direction = random.choice(("LEFT", "RIGHT"))

    if flag_direction == "LEFT":
        flag_lateral_speed = +TEST2_SIDE_SPEED
    else:
        flag_lateral_speed = -TEST2_SIDE_SPEED

    print()
    print("[TEST 2] Vehicle is now lined up with the flag.")
    print("[TEST 2] Flag position: approximately 1.00 m ahead.")
    print("[TEST 2] DYNAMIC FLAG DIRECTION:")
    print(
        f"        FLAG = {flag_direction}"
    )
    print(
        "[TEST 2] DRONE WILL FLY IN THE SAME DIRECTION "
        "AS THE FLAG."
    )
    print(
        f"[TEST 2] Lateral maneuver = "
        f"{TEST2_FLAG_LATERAL_DISTANCE_M:.2f} m"
    )

    # STEP 4: START METRICS NOW. Everything before this point is positioning and excluded
    reset_metrics(metrics)

    flag_test_start_north = STATE.north_m

    print("[METRICS] Timer STARTED NOW.")
    print("[METRICS] Lane translation and forward line-up are NOT measured.")
    print()

    # STEP 5: move laterally in the direction of the FLAG
    print(
        f"[TEST 2] Lateral goal = "
        f"{TEST2_FLAG_LATERAL_DISTANCE_M:.2f} m "
        f"in the {flag_direction} direction."
    )
    print(
        "[TEST 2] The drone will stop at the lateral goal, "
        "print GOAL REACHED, then continue forward."
    )
    print()

    next_print = time.monotonic()
    goal_start = time.monotonic()

    while True:
        current_offset = STATE.north_m - flag_test_start_north

        if flag_direction == "LEFT":
            signed_progress = current_offset
        else:
            signed_progress = -current_offset

        remaining = (
            TEST2_FLAG_LATERAL_DISTANCE_M
            - signed_progress
        )

        horizontal_speed = math.sqrt(
            STATE.north_velocity_mps ** 2
            + STATE.east_velocity_mps ** 2
        )

        # Goal requires position AND low speed
        if (
            signed_progress
            >= TEST2_FLAG_LATERAL_DISTANCE_M
            - FLAG_GOAL_TOLERANCE_M
            and horizontal_speed
            <= FLAG_GOAL_SPEED_TOLERANCE_MPS
        ):
            await send_course_velocity(
                drone,
                0.0,
                0.0,
                0.0,
                COURSE_YAW_DEG,
            )

            update_metrics(
                metrics,
                0.0,
                0.0,
                0.0,
                f"FLAG LATERAL GOAL - {flag_direction}",
            )

            actual_lateral_change = (
                STATE.north_m
                - flag_test_start_north
            )

            if flag_direction == "LEFT":
                direction_correct = actual_lateral_change > 0.0
            else:
                direction_correct = actual_lateral_change < 0.0

            metrics.flag_direction = flag_direction
            metrics.actual_flag_lateral_change_m = (
                actual_lateral_change
            )
            metrics.flag_direction_correct = direction_correct

            print()
            print(
                f"[GOAL] FLAG LATERAL GOAL REACHED | "
                f"direction={flag_direction} | "
                f"progress={signed_progress:+.2f} m | "
                f"target={TEST2_FLAG_LATERAL_DISTANCE_M:.2f} m | "
                f"speed={horizontal_speed:.2f} m/s"
            )
            print(
                "[STOP] Drone stopped at the flag-direction goal."
            )
            print(
                f"[DIRECTION CHECK] Expected={flag_direction} | "
                f"Actual lateral change={actual_lateral_change:+.2f} m | "
                f"CORRECT={direction_correct}"
            )
            print()
            break

        await send_course_velocity(
            drone,
            0.0,
            flag_lateral_speed,
            0.0,
            COURSE_YAW_DEG,
        )

        update_metrics(
            metrics,
            0.0,
            flag_lateral_speed,
            0.0,
            f"FLAG FOLLOW DIRECTION: {flag_direction}",
        )

        now = time.monotonic()
        if now >= next_print:
            print(
                f"[FLAG] direction={flag_direction} | "
                f"progress={signed_progress:+.2f}/"
                f"{TEST2_FLAG_LATERAL_DISTANCE_M:.2f} m | "
                f"remaining={max(remaining, 0.0):.2f} m | "
                f"speed={horizontal_speed:.2f} m/s"
            )
            next_print = now + 1.0

        if now - goal_start > 15.0:
            raise RuntimeError(
                "Test 2 flag-direction goal was not reached "
                "within 15 seconds."
            )

        await asyncio.sleep(CONTROL_PERIOD)

    # STEP 6: continue FORWARD past the flag. This remains measured
    print()
    print("[TEST 2] Lateral goal complete.")
    print("[TEST 2] Now moving FORWARD past the flag.")
    print()

    await run_velocity_segment(
        drone,
        metrics,
        f"FLAG FORWARD APPROACH ({flag_direction})",
        TEST2_FORWARD_SPEED,
        0.0,
        0.0,
        TEST2_APPROACH_TIME_S,
        record_metrics=True,
    )

    await run_velocity_segment(
        drone,
        metrics,
        f"FLAG FORWARD PASS ({flag_direction})",
        TEST2_FORWARD_SPEED,
        0.0,
        0.0,
        TEST2_PASS_TIME_S,
        record_metrics=True,
    )

    await hold_zero_velocity(
        drone,
        None,
        2.0,
        "TEST 2 FINISH HOLD",
        record_metrics=False,
    )

    print()
    print("=" * 60)
    print("TEST 2 COMPLETE")
    print("=" * 60)

# TEST 3 — BOX

async def run_test_3(
    drone: System,
    metrics: Metrics,
):
    print()
    print("=" * 60)
    print("TEST 3: BOX PASS-THROUGH")
    print("=" * 60)
    print()

    # Capture the PX4 local-NED position at the start of Test 3. The box center is at
    # Gazebo Y = +7.5 m, and Gazebo +Y maps to PX4 +North for this simulation
    reference_n = STATE.north_m
    reference_e = STATE.east_m
    # Use a fixed altitude inside the box opening instead of the current local-NED down
    # position, which can be near zero after takeoff
    box_target_down = -TEST3_BOX_ALTITUDE_M

    box_lane_n = reference_n + TEST3_TRANSLATE_DISTANCE_M
    box_lane_e = reference_e

    print("STEP 1: MOVE LEFT UNTIL ALIGNED WITH BOX")
    print("        Looking down the course from the start, LEFT = +North")
    print("        Box center:    Gazebo Y = +7.50 m")
    print(f"        Start North:   {reference_n:+.2f} m")
    print(f"        Target North:  {box_lane_n:+.2f} m")
    print(f"        Target East:   {box_lane_e:+.2f} m")
    print(f"        Target Alt:    {TEST3_BOX_ALTITUDE_M:+.2f} m")
    print(f"        Target Down:   {box_target_down:+.2f} m")
    print("        Control: PositionNedYaw")
    print("        Metrics during positioning = DISABLED")
    print()

    # Use an absolute positional target instead of a timed lateral velocity. This
    # guarantees that the drone actually reaches the box lane before the forward test
    # begins
    await move_to_ned_position(
        drone,
        box_lane_n,
        box_lane_e,
        box_target_down,
        "BOX LANE ALIGNMENT",
        TEST3_POSITION_TOLERANCE_M,
        TEST3_SPEED_TOLERANCE_MPS,
        timeout_s=30.0,
    )

    print()
    print("[GOAL] BOX LANE ALIGNMENT REACHED")
    print(f"       PX4 N={STATE.north_m:+.2f} m")
    print(f"       PX4 E={STATE.east_m:+.2f} m")
    print(f"       Altitude={STATE.relative_altitude_m:.2f} m")
    print("[TEST 3] Drone is now aligned with the center of the box lane.")
    print("[METRICS] Timer STARTED NOW.")
    print("[METRICS] Lateral positioning is NOT measured.")
    print()

    # Reset here, after lateral positioning is complete
    reset_metrics(metrics)

    # The box center is at Gazebo X = +3.0 m. The Test 3 start position is approximately X
    # = -1.0 m, so the forward entry begins about 4 m later
    print("STEP 2: MOVE FORWARD INTO / THROUGH BOX")
    print(f"        Forward speed = {TEST3_FORWARD_SPEED:+.2f} m/s")
    print("        Metrics are ENABLED from this point forward.")
    print()

    await run_velocity_segment(
        drone,
        metrics,
        "BOX FORWARD ENTRY",
        TEST3_FORWARD_SPEED,
        0.0,
        0.0,
        TEST3_ENTRY_TIME_S,
        record_metrics=True,
    )

    await run_velocity_segment(
        drone,
        metrics,
        "BOX FORWARD THROUGH OPENING",
        TEST3_FORWARD_SPEED,
        0.0,
        0.0,
        TEST3_THROUGH_TIME_S,
        record_metrics=True,
    )

    await hold_zero_velocity(
        drone,
        None,
        2.0,
        "TEST 3 FINISH HOLD",
        record_metrics=False,
    )

    print()
    print("=" * 60)
    print("TEST 3 COMPLETE")
    print("=" * 60)


# STOP OFFBOARD

async def stop_offboard(
    drone: System,
):
    print()
    print("[OFFBOARD] Sending zero velocity...")

    try:
        await send_course_velocity(
            drone,
            0.0,
            0.0,
            0.0,
            COURSE_YAW_DEG,
        )
    except Exception as exc:
        print(
            "[WARN] Zero-velocity stop failed: "
            f"{exc}"
        )

    try:
        await drone.offboard.stop()
        print("[OFFBOARD] Offboard stopped.")
    except Exception as exc:
        print("[WARN] Offboard stop failed:")
        print(f"       {exc}")


# LAND

async def land_vehicle(drone: System):
    print()
    print("[DRONE] Landing...")

    try:
        await drone.action.land()
        print("[DRONE] Landing command sent.")
    except Exception as exc:
        print("[WARN] Landing command failed:")
        print(f"       {exc}")
        return

    start = time.monotonic()

    while time.monotonic() - start < 20.0:
        if (
            not STATE.in_air
            and STATE.relative_altitude_m < 0.25
        ):
            break

        await asyncio.sleep(0.25)

    print("[DRONE] Landing sequence complete.")


# DISARM

async def disarm_vehicle(drone: System):
    if not STATE.armed:
        print("[DRONE] Already disarmed.")
        return

    print("[DRONE] Disarming...")

    try:
        await drone.action.disarm()
        print("[DRONE] Disarmed.")
    except Exception as exc:
        print("[WARN] Disarm command failed:")
        print(f"       {exc}")


# SAVE CSV


def save_csv(
    metrics: Metrics,
    test_name: str,
):
    output_directory = os.path.join(
        OUTPUT_ROOT,
        test_name,
    )

    os.makedirs(
        output_directory,
        exist_ok=True,
    )

    path = os.path.join(
        output_directory,
        "flight_data.csv",
    )

    with open(
        path,
        "w",
        newline="",
    ) as file:
        writer = csv.writer(file)

        writer.writerow([
            "time_s",
            "north_m",
            "east_m",
            "down_m",
            "velocity_north_mps",
            "velocity_east_mps",
            "velocity_down_mps",
            "command_forward_mps",
            "command_lateral_mps",
            "command_down_mps",
            "command_north_mps",
            "command_east_mps",
            "yaw_deg",
            "altitude_m",
            "test_segment",
        ])

        for sample in metrics.samples:
            writer.writerow([
                sample.time_s,
                sample.north_m,
                sample.east_m,
                sample.down_m,
                sample.velocity_north_mps,
                sample.velocity_east_mps,
                sample.velocity_down_mps,
                sample.command_forward_mps,
                sample.command_lateral_mps,
                sample.command_down_mps,
                sample.command_north_mps,
                sample.command_east_mps,
                sample.yaw_deg,
                sample.altitude_m,
                sample.test_segment,
            ])

    print()
    print("[OUTPUT] Flight data saved to:")
    print(f"         {path}")


# SAVE SUMMARY


def save_summary(
    metrics: Metrics,
    test_name: str,
):
    output_directory = os.path.join(
        OUTPUT_ROOT,
        test_name,
    )

    os.makedirs(
        output_directory,
        exist_ok=True,
    )

    path = os.path.join(
        output_directory,
        "summary.txt",
    )

    if metrics.samples:
        total_time = metrics.samples[-1].time_s
    else:
        total_time = 0.0

    if total_time > 0.0:
        average_speed = (
            metrics.total_distance_m / total_time
        )
    else:
        average_speed = 0.0

    with open(path, "w") as file:
        file.write(
            "PX4 / GAZEBO OBSTACLE COURSE\n"
        )
        file.write(
            "========================================\n\n"
        )

        file.write(f"Test: {test_name}\n")
        file.write("Control mode: Velocity\n")
        file.write("Position targets: Disabled\n")
        file.write("Laser: Disabled\n")
        file.write("MLE: Disabled\n")
        file.write("Kalman filtering: Disabled\n\n")

        file.write("COURSE / NED MAPPING\n")
        file.write(
            "----------------------------------------\n"
        )
        file.write("Course forward -> PX4 East\n")
        file.write("Course lateral -> PX4 North\n")
        file.write("Course down -> PX4 Down\n\n")

        file.write("METRIC TIMING\n")
        file.write(
            "----------------------------------------\n"
        )

        if test_name == "test1_cone_weave":
            file.write(
                "Metrics started when Test 1 began in its\n"
                "correct obstacle lane.\n\n"
            )
        elif test_name == "test2_flag":
            file.write(
                "Metrics started only after translation to\n"
                "the flag lane at Y = -8 m.\n\n"
            )
        elif test_name == "test3_box":
            file.write(
                "Metrics started only after translation to\n"
                "the box lane at Y = +7.5 m.\n\n"
            )

        file.write("FLIGHT METRICS\n")
        file.write(
            "----------------------------------------\n"
        )
        file.write(
            f"Measured flight time: {total_time:.3f} s\n"
        )
        file.write(
            f"3D distance: {metrics.total_distance_m:.3f} m\n"
        )
        file.write(
            f"Horizontal distance: "
            f"{metrics.horizontal_distance_m:.3f} m\n"
        )
        file.write(
            f"Average speed: {average_speed:.3f} m/s\n"
        )
        file.write(
            f"Maximum speed: "
            f"{metrics.maximum_speed_mps:.3f} m/s\n"
        )
        file.write(
            f"Maximum horizontal speed: "
            f"{metrics.maximum_horizontal_speed_mps:.3f} m/s\n"
        )
        file.write(
            f"Maximum altitude: "
            f"{metrics.maximum_altitude_m:.3f} m\n"
        )

        if metrics.minimum_altitude_m != float("inf"):
            file.write(
                f"Minimum altitude: "
                f"{metrics.minimum_altitude_m:.3f} m\n"
            )

        file.write(
            f"Position samples: {len(metrics.samples)}\n"
        )

        if test_name == "test2_flag":
            file.write("\nDYNAMIC FLAG RESULT\n")
            file.write("----------------------------------------\n")
            file.write(
                f"Flag direction: {metrics.flag_direction}\n"
            )
            file.write(
                f"Actual lateral change: "
                f"{metrics.actual_flag_lateral_change_m:.3f} m\n"
            )
            file.write(
                f"Direction followed correctly: "
                f"{metrics.flag_direction_correct}\n"
            )

    print("[OUTPUT] Summary saved to:")
    print(f"         {path}")


# MAIN TEST RUNNER

async def run_test(test_number: int):
    test_names = {
        1: "test1_cone_weave",
        2: "test2_flag",
        3: "test3_box",
    }

    if test_number not in test_names:
        raise ValueError("Test must be 1, 2, or 3.")

    test_name = test_names[test_number]

    print_header()
    print(f"Selected test: {test_number}")
    print(f"Name: {test_name}")
    print("ONLY THIS TEST WILL RUN.")
    print()

    drone: Optional[System] = None
    telemetry_tasks = []
    metrics = Metrics()
    offboard_active = False

    try:
        # CONNECT
        drone = await connect_drone()

        # TELEMETRY
        telemetry_tasks = await start_telemetry(drone)

        # VEHICLE SHOULD ALREADY BE AIRBORNE
        print()
        print(
            "[DRONE] Python will NOT arm or take off the vehicle."
        )
        print("[DRONE] PX4 should already have:")
        print("         commander arm -f")
        print("         commander takeoff")

        await wait_until_airborne(drone)

        # START OFFBOARD
        await start_offboard(drone)
        offboard_active = True

        # INITIAL ZERO VELOCITY HOLD IS NOT MEASURED
        await hold_zero_velocity(
            drone,
            None,
            2.0,
            "INITIAL HOLD",
            record_metrics=False,
        )

        # RUN SELECTED TEST
        if test_number == 1:
            await run_test_1(drone, metrics)

        elif test_number == 2:
            await run_test_2(drone, metrics)

        elif test_number == 3:
            await run_test_3(drone, metrics)

        # STOP OFFBOARD
        await stop_offboard(drone)
        offboard_active = False

        # LAND
        await land_vehicle(drone)

        # DISARM
        await disarm_vehicle(drone)

        # SAVE RESULTS
        save_csv(metrics, test_name)
        save_summary(metrics, test_name)

        # COMPLETE
        print()
        print("=" * 60)
        print("FLIGHT COMPLETE")
        print("=" * 60)
        print()

    except KeyboardInterrupt:
        print()
        print("=" * 60)
        print("[STOP] Ctrl+C detected.")
        print("=" * 60)

        if drone is not None:
            try:
                if offboard_active:
                    await stop_offboard(drone)
            except Exception:
                pass

            try:
                await land_vehicle(drone)
            except Exception:
                pass

            try:
                await disarm_vehicle(drone)
            except Exception:
                pass

    except Exception as exc:
        print()
        print("=" * 60)
        print("[ERROR] FLIGHT FAILED")
        print("=" * 60)
        print()
        print(f"{type(exc).__name__}: {exc}")
        print()

        if drone is not None:
            try:
                await send_course_velocity(
                    drone,
                    0.0,
                    0.0,
                    0.0,
                    COURSE_YAW_DEG,
                )
            except Exception:
                pass

            if offboard_active:
                try:
                    await drone.offboard.stop()
                except Exception:
                    pass

            try:
                if STATE.armed:
                    await land_vehicle(drone)
            except Exception:
                pass

            try:
                await disarm_vehicle(drone)
            except Exception:
                pass

    finally:
        for task in telemetry_tasks:
            task.cancel()

        if telemetry_tasks:
            await asyncio.gather(
                *telemetry_tasks,
                return_exceptions=True,
            )


# COMMAND LINE

def main():
    if len(sys.argv) != 2:
        print()
        print("Usage:")
        print("    python drone_tests.py 1")
        print("    python drone_tests.py 2")
        print("    python drone_tests.py 3")
        print()
        sys.exit(1)

    try:
        test_number = int(sys.argv[1])
    except ValueError:
        print("[ERROR] Test number must be 1, 2, or 3.")
        sys.exit(1)

    if test_number not in (1, 2, 3):
        print("[ERROR] Test number must be 1, 2, or 3.")
        sys.exit(1)

    try:
        asyncio.run(run_test(test_number))
    except KeyboardInterrupt:
        print()
        print("[STOP] Ctrl+C.")


# ENTRY POINT

if __name__ == "__main__":
    main()
