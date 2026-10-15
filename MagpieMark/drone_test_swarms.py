#!/usr/bin/env python3

import argparse
import asyncio
import math
import random
import time
from dataclasses import dataclass

from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw


# CONFIGURATION

DEFAULT_UDP_BASE_PORT = 14540

CONTROL_RATE_HZ = 20
CONTROL_PERIOD = 1.0 / CONTROL_RATE_HZ

DRONE_ALTITUDE_M = 2.0

POSITION_TOLERANCE_M = 0.25
SPEED_TOLERANCE_MPS = 0.35
TEST_TIMEOUT_S = 60.0

# TEST 1
TEST1_TRANSLATE_M = 3.0

# TEST 2
TEST2_RANDOM_X_RANGE = (-3.0, 3.0)
TEST2_RANDOM_Y_RANGE = (-3.0, 3.0)
TEST2_LINE_SPACING_M = 1.5

# TEST 3
TEST3_GROUP_SPACING_M = 1.5
TEST3_TRANSLATE_M = 1.0

RANDOM_SEED = None


# DATA STRUCTURES

@dataclass
class DroneState:
    north_m: float = 0.0
    east_m: float = 0.0
    down_m: float = 0.0
    north_speed_mps: float = 0.0
    east_speed_mps: float = 0.0
    down_speed_mps: float = 0.0


# MAVSDK HELPERS

async def connect_one_drone(index: int, base_port: int):
    uri = f"udpin://127.0.0.1:{base_port + index}"

    drone = System()

    print(f"[CONNECT] Drone {index}: {uri}")
    await drone.connect(system_address=uri)

    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"[CONNECTED] Drone {index}")
            break

    return drone


async def connect_drones(n: int, base_port: int):
    return await asyncio.gather(
        *(connect_one_drone(i, base_port) for i in range(n))
    )


async def get_state(drone: System):
    position = await drone.telemetry.position()
    velocity = await drone.telemetry.velocity_ned()

    return DroneState(
        north_m=position.north_m,
        east_m=position.east_m,
        down_m=-position.relative_altitude_m,
        north_speed_mps=velocity.north_m_s,
        east_speed_mps=velocity.east_m_s,
        down_speed_mps=velocity.down_m_s,
    )


async def get_states(drones):
    return await asyncio.gather(*(get_state(drone) for drone in drones))


async def wait_for_airborne(drones):
    start = time.monotonic()

    while time.monotonic() - start < 30.0:

        states = await get_states(drones)

        if all(-state.down_m >= 1.0 for state in states):
            print("[AIRBORNE] All drones are airborne.")
            return

        await asyncio.sleep(0.2)

    raise TimeoutError("Not all drones became airborne.")


# ARM / OFFBOARD

async def initialize_one_drone(index: int, drone: System):
    print(f"[ARM] Drone {index}")

    await drone.action.arm()

    state = await get_state(drone)

    await drone.offboard.set_position_ned(
        PositionNedYaw(
            float(state.north_m),
            float(state.east_m),
            float(-DRONE_ALTITUDE_M),
            90.0,
        )
    )

    try:
        await drone.offboard.start()

    except OffboardError as error:
        print(
            f"[ERROR] Drone {index} offboard start failed: "
            f"{error._result.result}"
        )
        raise


async def arm_and_start_offboard(drones):
    await asyncio.gather(
        *(initialize_one_drone(i, drone) for i, drone in enumerate(drones))
    )

    await wait_for_airborne(drones)


async def stop_offboard(drones):
    for drone in drones:
        try:
            await drone.offboard.stop()
        except Exception:
            pass


# GEOMETRY / DIAGNOSTICS

def mean_squared_pairwise_distance(states, plane):
    values = []

    for i in range(len(states)):
        for j in range(i + 1, len(states)):

            if plane == "xy":

                dn = states[i].north_m - states[j].north_m
                de = states[i].east_m - states[j].east_m

                values.append(dn * dn + de * de)

            elif plane == "yz":

                de = states[i].east_m - states[j].east_m
                dd = states[i].down_m - states[j].down_m

                values.append(de * de + dd * dd)

            else:
                raise ValueError("plane must be 'xy' or 'yz'")

    if not values:
        return 0.0

    return sum(values) / len(values)


def calculate_centroid(states):
    north = sum(state.north_m for state in states) / len(states)
    east = sum(state.east_m for state in states) / len(states)
    down = sum(state.down_m for state in states) / len(states)

    return north, east, down


# POSITION CONTROL

async def move_drones_to_targets(
    drones,
    targets,
    label,
    timeout_s=TEST_TIMEOUT_S,
):
    print()
    print("------------------------------------------------------------")
    print(f"[POSITION] {label}")
    print("------------------------------------------------------------")

    targets = [(n, e, -DRONE_ALTITUDE_M) for n, e, _ in targets]
    start = time.monotonic()

    while True:

        states = await get_states(drones)

        all_reached = True

        for drone, state, target in zip(drones, states, targets):

            target_n, target_e, target_d = target

            position_error = math.sqrt(
                (target_n - state.north_m) ** 2
                + (target_e - state.east_m) ** 2
                + (target_d - state.down_m) ** 2
            )

            speed = math.sqrt(
                state.north_speed_mps ** 2
                + state.east_speed_mps ** 2
                + state.down_speed_mps ** 2
            )

            if (
                position_error > POSITION_TOLERANCE_M
                or speed > SPEED_TOLERANCE_MPS
            ):
                all_reached = False

            await drone.offboard.set_position_ned(
                PositionNedYaw(
                    float(target_n),
                    float(target_e),
                    float(target_d),
                    90.0,
                )
            )

        if all_reached:
            return time.monotonic() - start

        if time.monotonic() - start > timeout_s:
            raise TimeoutError(f"{label} timed out.")

        await asyncio.sleep(CONTROL_PERIOD)


async def hold_positions(drones, duration_s):
    end_time = time.monotonic() + duration_s

    while time.monotonic() < end_time:

        states = await get_states(drones)

        for drone, state in zip(drones, states):

            await drone.offboard.set_position_ned(
                PositionNedYaw(
                    float(state.north_m),
                    float(state.east_m),
                    float(-DRONE_ALTITUDE_M),
                    90.0,
                )
            )

        await asyncio.sleep(CONTROL_PERIOD)


# TEST 1 — 3 M SWARM TRANSLATION

async def run_test_1(drones):

    print()
    print("============================================================")
    print("TEST 1 — 3 M SWARM TRANSLATION")
    print("============================================================")

    states_before = await get_states(drones)

    xy_mse_before = mean_squared_pairwise_distance(states_before, "xy")
    yz_mse_before = mean_squared_pairwise_distance(states_before, "yz")

    print()
    print("[TEST 1 BEFORE TRANSLATION]")
    print(
        f"Mean squared pairwise distance in XY plane: "
        f"{xy_mse_before:.6f} m^2"
    )
    print(
        f"Mean squared pairwise distance in YZ plane: "
        f"{yz_mse_before:.6f} m^2"
    )

    # Every drone gets the exact same translation
    targets = [
        (
            state.north_m,
            state.east_m + TEST1_TRANSLATE_M,
            -DRONE_ALTITUDE_M,
        )
        for state in states_before
    ]

    print()
    print(
        f"[COMMAND] All drones translate "
        f"+{TEST1_TRANSLATE_M:.2f} m in East."
    )

    elapsed = await move_drones_to_targets(
        drones,
        targets,
        "TEST 1 — 3 M TRANSLATION",
    )

    states_after = await get_states(drones)

    xy_mse_after = mean_squared_pairwise_distance(states_after, "xy")
    yz_mse_after = mean_squared_pairwise_distance(states_after, "yz")

    print()
    print("[TEST 1 RESULTS]")
    print(f"Translation time: {elapsed:.3f} s")
    print(
        f"Mean squared pairwise distance in XY plane: "
        f"{xy_mse_after:.6f} m^2"
    )
    print(
        f"Mean squared pairwise distance in YZ plane: "
        f"{yz_mse_after:.6f} m^2"
    )

    return (
        elapsed,
        xy_mse_before,
        yz_mse_before,
        xy_mse_after,
        yz_mse_after,
    )


# TEST 2 — RANDOM POSITIONS -> STRAIGHT LINE

async def run_test_2(drones, rng):

    print()
    print("============================================================")
    print("TEST 2 — RANDOM POSITIONS -> STRAIGHT LINE")
    print("============================================================")

    states = await get_states(drones)

    random_targets = []

    for state in states:

        random_north = rng.uniform(*TEST2_RANDOM_X_RANGE)
        random_east = rng.uniform(*TEST2_RANDOM_Y_RANGE)

        random_targets.append(
            (
                state.north_m + random_north,
                state.east_m + random_east,
                -DRONE_ALTITUDE_M,
            )
        )

    print("[SETUP] Moving drones to random starting positions.")

    await move_drones_to_targets(
        drones,
        random_targets,
        "TEST 2 — RANDOMIZE START POSITIONS",
    )

    await hold_positions(drones, 1.0)

    print()
    print("============================================================")
    print("[SIGNAL] TEST 2 READY")
    print("============================================================")

    input(
        "Press ENTER to produce the signal and start Test 2 timing..."
    )

    current_states = await get_states(drones)

    mean_north = sum(
        state.north_m for state in current_states
    ) / len(current_states)

    mean_east = sum(
        state.east_m for state in current_states
    ) / len(current_states)

    # The final line is parallel to East
    start_east = (
        mean_east
        - 0.5 * (len(drones) - 1) * TEST2_LINE_SPACING_M
    )

    targets = []

    for i in range(len(drones)):

        targets.append(
            (
                mean_north,
                start_east + i * TEST2_LINE_SPACING_M,
                -DRONE_ALTITUDE_M,
            )
        )

    start_time = time.monotonic()

    await move_drones_to_targets(
        drones,
        targets,
        "TEST 2 — FORM STRAIGHT LINE",
    )

    elapsed = time.monotonic() - start_time

    final_states = await get_states(drones)

    line_error = 0.0

    for state, target in zip(final_states, targets):

        dn = state.north_m - target[0]
        de = state.east_m - target[1]

        line_error += dn * dn + de * de

    line_rmse = math.sqrt(
        line_error / len(drones)
    )

    print()
    print("[TEST 2 RESULTS]")
    print(f"Formation time: {elapsed:.3f} s")
    print(f"Final XY formation RMSE: {line_rmse:.4f} m")

    return elapsed, line_rmse


# TEST 3 — FLAG TURN / SPLIT / 1 M TRANSLATION

async def run_test_3(drones, rng):

    print()
    print("============================================================")
    print("TEST 3 — FLAG TURN / SPLIT / 1 M TRANSLATION")
    print("============================================================")

    direction = rng.choice(("LEFT", "RIGHT"))

    majority_count = len(drones) // 2 + 1
    minority_count = len(drones) - majority_count

    print(f"[FLAG] Signal direction = {direction}")
    print(f"[GROUP] Majority group = {majority_count} drones")
    print(f"[GROUP] Minority group = {minority_count} drones")

    # First form a straight line

    states = await get_states(drones)

    mean_north, mean_east, _ = calculate_centroid(states)

    start_east = (
        mean_east
        - 0.5 * (len(drones) - 1) * TEST3_GROUP_SPACING_M
    )

    line_targets = []

    for i in range(len(drones)):

        line_targets.append(
            (
                mean_north,
                start_east + i * TEST3_GROUP_SPACING_M,
                -DRONE_ALTITUDE_M,
            )
        )

    await move_drones_to_targets(
        drones,
        line_targets,
        "TEST 3 — INITIAL LINEUP",
    )

    # FLAG TURN

    if direction == "LEFT":

        majority_sign = -1.0
        minority_sign = +1.0

    else:

        majority_sign = +1.0
        minority_sign = -1.0

    middle_index = (len(drones) - 1) / 2.0

    # The first n/2+1 drones form the majority group
    split_targets = []

    for i in range(len(drones)):

        if i < majority_count:
            sign = majority_sign
        else:
            sign = minority_sign

        distance_from_center = (
            abs(i - middle_index)
            * TEST3_GROUP_SPACING_M
        )

        split_targets.append(
            (
                mean_north
                + sign * distance_from_center,
                start_east + i * TEST3_GROUP_SPACING_M,
                -DRONE_ALTITUDE_M,
            )
        )

    print()
    print("------------------------------------------------------------")
    print(f"[FLAG] FLAG TURNS {direction}")
    print("[SIGNAL] Starting Test 3 timer now.")
    print("------------------------------------------------------------")

    start_time = time.monotonic()

    await move_drones_to_targets(
        drones,
        split_targets,
        "TEST 3 — SPLIT INTO TWO GROUPS",
    )

    # BOTH GROUPS TRANSLATE 1 M

    translation_targets = [
        (
            target_n,
            target_e + TEST3_TRANSLATE_M,
            target_d,
        )
        for target_n, target_e, target_d in split_targets
    ]

    await move_drones_to_targets(
        drones,
        translation_targets,
        "TEST 3 — BOTH GROUPS TRANSLATE 1 M",
    )

    elapsed = time.monotonic() - start_time

    print()
    print("[TEST 3 RESULTS]")
    print(f"Flag direction: {direction}")
    print(f"Majority group: {majority_count} drones")
    print(f"Minority group: {minority_count} drones")
    print(f"Split + translation time: {elapsed:.3f} s")

    return elapsed, direction


# MAIN

async def run_all_tests(n, base_port, seed):

    if n < 2:
        raise ValueError("Swarm size N must be at least 2.")

    rng = random.Random(seed)

    print("============================================================")
    print("DRONE SWARM TEST SUITE")
    print("============================================================")
    print(f"Number of drones: N={n}")
    print(f"MAVSDK base port: {base_port}")
    print("============================================================")

    drones = await connect_drones(n, base_port)

    try:

        await arm_and_start_offboard(drones)

        results_1 = await run_test_1(drones)

        await hold_positions(drones, 1.0)

        results_2 = await run_test_2(drones, rng)

        await hold_positions(drones, 1.0)

        results_3 = await run_test_3(drones, rng)

        print()
        print("============================================================")
        print("ALL TESTS COMPLETE")
        print("============================================================")

        print()
        print("TEST 1:")
        print(f"  Translation time = {results_1[0]:.3f} s")
        print(
            "  XY MSE before = "
            f"{results_1[1]:.6f} m^2"
        )
        print(
            "  YZ MSE before = "
            f"{results_1[2]:.6f} m^2"
        )
        print(
            "  XY MSE after = "
            f"{results_1[3]:.6f} m^2"
        )
        print(
            "  YZ MSE after = "
            f"{results_1[4]:.6f} m^2"
        )

        print()
        print("TEST 2:")
        print(f"  Formation time = {results_2[0]:.3f} s")
        print(
            f"  Final XY formation RMSE = "
            f"{results_2[1]:.4f} m"
        )

        print()
        print("TEST 3:")
        print(f"  Flag direction = {results_3[1]}")
        print(
            f"  Split + translation time = "
            f"{results_3[0]:.3f} s"
        )

    finally:

        await stop_offboard(drones)


# ARGUMENTS

def parse_args():

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--n",
        type=int,
        default=8,
        help="Number of drones in the swarm.",
    )

    parser.add_argument(
        "--base-port",
        type=int,
        default=DEFAULT_UDP_BASE_PORT,
        help="First MAVSDK UDP port.",
    )

    parser.add_argument(
        "--seed",
        type=int,
        default=RANDOM_SEED,
        help="Random seed.",
    )

    return parser.parse_args()


# RUN

if __name__ == "__main__":

    args = parse_args()

    asyncio.run(
        run_all_tests(
            n=args.n,
            base_port=args.base_port,
            seed=args.seed,
        )
    )
