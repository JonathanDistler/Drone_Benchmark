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

# Arm / startup timing
CONNECTION_TIMEOUT_S = 30.0
HEALTH_TIMEOUT_S = 30.0
ARM_RETRY_COUNT = 5
ARM_RETRY_DELAY_S = 2.0

# TEST 1
TEST1_TRANSLATE_M = 1.0

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

    start = time.monotonic()

    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"[CONNECTED] Drone {index}")
            return drone

        if time.monotonic() - start > CONNECTION_TIMEOUT_S:
            raise TimeoutError(f"Drone {index} connection timed out.")

    raise TimeoutError(f"Drone {index} connection timed out.")


async def connect_drones(n: int, base_port: int):
    drones = []

    # Connect sequentially to avoid overwhelming MAVSDK's callback queues when several PX4
    # instances start at nearly the same time
    for i in range(n):
        drone = await connect_one_drone(i, base_port)
        drones.append(drone)
        await asyncio.sleep(0.25)

    return drones


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


async def wait_for_health(index: int, drone: System):
    print(f"[HEALTH] Waiting for Drone {index} position estimate...")

    start = time.monotonic()

    async for health in drone.telemetry.health():

        # Local position is what PositionNedYaw needs for these tests. Home position
        # is also a useful indication that PX4 has completed its normal SITL
        # initialization checks
        if health.is_local_position_ok and health.is_home_position_ok:
            print(f"[HEALTH] Drone {index} ready for arming.")
            return

        if time.monotonic() - start > HEALTH_TIMEOUT_S:
            raise TimeoutError(
                f"Drone {index} did not become ready for arming "
                f"within {HEALTH_TIMEOUT_S:.0f} s."
            )

    raise TimeoutError(f"Drone {index} health check timed out.")


# ARM / OFFBOARD

async def initialize_one_drone(index: int, drone: System):

    await wait_for_health(index, drone)

    state = await get_state(drone)

    # Send the initial setpoint before arming / starting Offboard. This gives PX4 a valid
    # position target from the beginning
    await drone.offboard.set_position_ned(
        PositionNedYaw(
            float(state.north_m),
            float(state.east_m),
            float(state.down_m),
            90.0,
        )
    )

    await asyncio.sleep(0.5)

    print(f"[ARM] Drone {index}")

    for attempt in range(1, ARM_RETRY_COUNT + 1):

        try:
            await drone.action.arm()
            print(f"[ARMED] Drone {index}")
            break

        except Exception as error:

            if attempt >= ARM_RETRY_COUNT:
                raise RuntimeError(
                    f"Drone {index} failed to arm after "
                    f"{ARM_RETRY_COUNT} attempts: {error}"
                ) from error

            print(
                f"[ARM WAIT] Drone {index} arm attempt {attempt} denied: "
                f"{error}"
            )
            print(
                f"[ARM WAIT] Retrying Drone {index} in "
                f"{ARM_RETRY_DELAY_S:.1f} s..."
            )
            await asyncio.sleep(ARM_RETRY_DELAY_S)

    print(f"[OFFBOARD] Starting Drone {index}")

    try:
        await drone.offboard.start()
        print(f"[OFFBOARD] Drone {index} active")

    except OffboardError as error:
        print(
            f"[ERROR] Drone {index} offboard start failed: "
            f"{error._result.result}"
        )
        raise


async def arm_and_start_offboard(drones):

    # Complete ground-hold preparation for every member before any climb
    preparation = await asyncio.gather(
        *(initialize_one_drone(i, drone) for i, drone in enumerate(drones)),
        return_exceptions=True,
    )
    failures = [f"Drone {i}: {result}" for i, result in enumerate(preparation)
                if isinstance(result, BaseException)]
    if failures:
        await stop_offboard(drones)
        raise RuntimeError("Swarm preparation failed: " + "; ".join(failures))

    states = await get_states(drones)
    release = await asyncio.gather(
        *(drone.offboard.set_position_ned(PositionNedYaw(
            float(state.north_m), float(state.east_m),
            float(-DRONE_ALTITUDE_M), 90.0,
        )) for drone, state in zip(drones, states)),
        return_exceptions=True,
    )
    failures = [f"Drone {i}: {result}" for i, result in enumerate(release)
                if isinstance(result, BaseException)]
    if failures:
        await stop_offboard(drones)
        raise RuntimeError("Swarm takeoff failed: " + "; ".join(failures))

    # Do not let the menu's current-position hold interrupt a slow climb
    deadline = time.monotonic() + TEST_TIMEOUT_S
    while True:
        states = await asyncio.wait_for(
            get_states(drones), timeout=max(0.01, deadline - time.monotonic()))
        pending = [str(i) for i, state in enumerate(states)
                   if abs(state.down_m + DRONE_ALTITUDE_M) > POSITION_TOLERANCE_M
                   or abs(state.down_speed_mps) > SPEED_TOLERANCE_MPS]
        if not pending:
            return
        if time.monotonic() >= deadline:
            raise TimeoutError("Takeoff not completed for drone(s): " + ", ".join(pending))
        await asyncio.sleep(CONTROL_PERIOD)


async def stop_offboard(drones):
    for i, drone in enumerate(drones):
        try:
            await drone.offboard.stop()
        except Exception:
            pass

        try:
            await drone.action.land()
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


def print_positions(states):
    for i, state in enumerate(states):
        print(
            f"Drone {i}: "
            f"N={state.north_m:+.2f} m, "
            f"E={state.east_m:+.2f} m, "
            f"Alt={-state.down_m:+.2f} m"
        )


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

        max_position_error = 0.0
        max_speed = 0.0

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

            max_position_error = max(max_position_error, position_error)
            max_speed = max(max_speed, speed)

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

        print(
            f"[POSITION] {label} | "
            f"max error={max_position_error:.2f} m | "
            f"max speed={max_speed:.2f} m/s"
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


# TESTS 1 AND 2 — START-REFERENCED POSITION ERROR

def position_targets(states):
    return [(s.north_m, s.east_m, -DRONE_ALTITUDE_M) for s in states]


def plane_mse(states, targets, plane):
    # Mean squared projected position error per drone, in m^2
    axes = {"xy": (0, 1), "yz": (1, 2)}[plane]
    positions = [(s.north_m, s.east_m, s.down_m) for s in states]
    return sum(
        sum((position[axis] - target[axis]) ** 2 for axis in axes)
        for position, target in zip(positions, targets)
    ) / len(states)


async def run_test_1(drones, start_targets=None):
    print("\nTEST 1 — 1 M FORWARD TRANSLATION: XY/YZ MSE")
    if start_targets is not None:
        await move_drones_to_targets(drones, start_targets, "TEST 1 — START LINE SETUP")
    states_before = await get_states(drones)
    starts = position_targets(states_before)
    # The existing controller uses yaw=90 degrees, so forward is East
    targets = [(n, e + TEST1_TRANSLATE_M, d) for n, e, d in starts]
    elapsed = await move_drones_to_targets(drones, targets, "TEST 1 — 1 M FORWARD")
    states_after = await get_states(drones)
    xy_mse = plane_mse(states_after, targets, "xy")
    yz_mse = plane_mse(states_after, targets, "yz")
    print(f"Translation time: {elapsed:.3f} s")
    print(f"XY MSE relative to start + 1 m forward: {xy_mse:.6f} m^2")
    print(f"YZ MSE relative to start + 1 m forward: {yz_mse:.6f} m^2")
    return elapsed, xy_mse, yz_mse


async def run_test_2(drones, rng, start_targets=None):
    print("\nTEST 2 — RANDOM POSITIONS -> ORIGINAL START LINE")
    if start_targets is None:
        start_targets = position_targets(await get_states(drones))
    await move_drones_to_targets(drones, start_targets, "TEST 2 — START LINE SETUP")
    await asyncio.to_thread(input, "Press ENTER to cue random positioning...")
    random_targets = [
        (n + rng.uniform(*TEST2_RANDOM_X_RANGE),
         e + rng.uniform(*TEST2_RANDOM_Y_RANGE), d)
        for n, e, d in start_targets
    ]
    await move_drones_to_targets(drones, random_targets, "TEST 2 — RANDOM POSITION SETUP")
    # Setup must finish before timing the return; no prompt or setup time is measured
    elapsed = await move_drones_to_targets(
        drones, start_targets, "TEST 2 — RETURN TO ORIGINAL START LINE"
    )
    final_states = await get_states(drones)
    xy_mse = plane_mse(final_states, start_targets, "xy")
    yz_mse = plane_mse(final_states, start_targets, "yz")
    print(f"Return time: {elapsed:.3f} s")
    print(f"Final XY return MSE: {xy_mse:.6f} m^2")
    print(f"Final YZ return MSE: {yz_mse:.6f} m^2")
    return elapsed, xy_mse, yz_mse


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

    # The first n/2+1 drones follow the flag direction. The remaining drones move in the
    # opposite direction
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


# TEST MENU

async def test_menu(drones, rng, start_targets):

    while True:

        print()
        print("============================================================")
        print("DRONE SWARM TEST MENU")
        print("============================================================")
        print("1 = Test 1: Forward 1 m + start-referenced XY/YZ MSE")
        print("2 = Test 2: Random positions -> original start line")
        print("3 = Test 3: Flag turn -> split -> translate 1 m")
        print("q = Quit")
        print("============================================================")

        choice = input("Select a test: ").strip().lower()

        try:

            if choice == "1":

                await run_test_1(drones, start_targets)
                await hold_positions(drones, 1.0)

            elif choice == "2":

                await run_test_2(drones, rng, start_targets)
                await hold_positions(drones, 1.0)

            elif choice == "3":

                await run_test_3(drones, rng)
                await hold_positions(drones, 1.0)

            elif choice == "q":

                print("[EXIT] Leaving test menu.")
                return

            else:

                print("[ERROR] Please enter 1, 2, 3, or q.")

        except Exception as error:

            print()
            print("============================================================")
            print("[TEST ERROR]")
            print("============================================================")
            print(error)
            print()
            print("The swarm will remain connected so you can select another test.")


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

        print()
        print("============================================================")
        print("[READY] ALL DRONES ARMED AND IN OFFBOARD")
        print("============================================================")

        states = await get_states(drones)
        print_positions(states)

        await hold_positions(drones, 1.0)

        await test_menu(drones, rng, position_targets(states))

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
