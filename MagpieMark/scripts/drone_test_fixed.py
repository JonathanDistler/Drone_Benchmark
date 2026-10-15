#!/usr/bin/env python3

# Simple PX4 / Gazebo Test-1 waypoint controller. Purpose: Keep the controller as simple as
# possible for the first successful test. Control: PositionNedYaw ONLY. Behavior: 1. Take off
# to approximately 2 m. 2. Record the post-takeoff PX4 local-NED position. 3. Fly to one
# position waypoint. 4. Wait until that waypoint is reached and the horizontal speed is low
# 5. Print a GOAL message. 6. Move to the next waypoint. 7. Repeat through the cone course. 8
# Hold the final position and land. There is intentionally NO: - PID / PD controller -
# velocity controller - moving lookahead - trajectory interpolation - acceleration shaping -
# 3.5 s dwell - Test 2 / Test 3 logic The only goal here is to make the drone reliably visit
# the requested positions one at a time. Optimization can come after that works. Gazebo course
# from the supplied SDF: Start = (-1, 0) Cone 1 = ( 2, 0) Cone 2 = ( 5, 0) Cone 3 = ( 8, 0)
# Empirically established frame for this simulation: Gazebo +X -> PX4 +East Gazebo +Y -> PX4
# +North Gazebo +Z -> PX4 -Down

import asyncio
import math
import time

from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw


# CONNECTION / FLIGHT SETTINGS

SYSTEM_ADDRESS = "udpin://127.0.0.1:14540"

TAKEOFF_ALTITUDE_M = 2.0

CONTROL_RATE_HZ = 10.0
PRINT_RATE_HZ = 2.0

POSITION_TOLERANCE_M = 0.20
HORIZONTAL_SPEED_TOLERANCE_MPS = 0.25

WAYPOINT_TIMEOUT_S = 45.0
FINAL_HOLD_S = 1.0

YAW_DEG = 0.0

# GAZEBO COURSE

CONE_1_X = 2.0
CONE_2_X = 5.0
CONE_3_X = 8.0
CONE_Y = 0.0

GAZEBO_START_X = -1.0
GAZEBO_START_Y = 0.0

# Distance from each cone center for the cone-pass point
CONE_PASS_OFFSET_M = 0.75

# Exact midpoint between each adjacent pair of cones
GAP_12_X = (CONE_1_X + CONE_2_X) / 2.0  # 3.5
GAP_23_X = (CONE_2_X + CONE_3_X) / 2.0  # 6.5

# TEST-1 WAYPOINTS IN GAZEBO X/Y These are the ONLY horizontal points the drone is asked to
# visit. The drone reaches each one before the next target is released

WAYPOINTS_GAZEBO = [
    # Pass Cone 1 on the +Y side
    (CONE_1_X, CONE_Y + CONE_PASS_OFFSET_M),

    # Exact midpoint between Cone 1 and Cone 2
    (GAP_12_X, CONE_Y),

    # Pass Cone 2 on the -Y side
    (CONE_2_X, CONE_Y - CONE_PASS_OFFSET_M),

    # Exact midpoint between Cone 2 and Cone 3
    (GAP_23_X, CONE_Y),

    # Pass Cone 3 on the +Y side
    (CONE_3_X, CONE_Y + CONE_PASS_OFFSET_M),

    # Finish back on the course centerline
    (10.0, CONE_Y),
]

WAYPOINT_NAMES = [
    "CONE 1 PASS",
    "GAP 1-2 MIDPOINT",
    "CONE 2 PASS",
    "GAP 2-3 MIDPOINT",
    "CONE 3 PASS",
    "FINISH",
]


# HELPERS


def distance_3d(n, e, d, tn, te, td):
    return math.sqrt((tn - n) ** 2 + (te - e) ** 2 + (td - d) ** 2)


def distance_xy(n, e, tn, te):
    return math.sqrt((tn - n) ** 2 + (te - e) ** 2)


def speed_xy(vn, ve):
    return math.sqrt(vn * vn + ve * ve)


def gazebo_to_px4(gx, gy, start_n, start_e):
    # Convert the Gazebo world XY coordinates used by the supplied SDF to the PX4
    # local-NED horizontal coordinates observed in this setup. Gazebo +X -> PX4 +East
    # Gazebo +Y -> PX4 +North
    dx = gx - GAZEBO_START_X
    dy = gy - GAZEBO_START_Y

    target_n = start_n + dy
    target_e = start_e + dx

    return target_n, target_e


def print_course_geometry():
    print()
    print("=" * 68)
    print("TEST 1 - SIMPLE POSITION WAYPOINTS")
    print("=" * 68)
    print("CONTROL: PositionNedYaw ONLY")
    print("NO PID / NO VELOCITY CONTROL / NO MOVING LOOKAHEAD")
    print()
    print("GAZEBO COURSE")
    print(f"    Start  = ({GAZEBO_START_X:+.2f}, {GAZEBO_START_Y:+.2f})")
    print(f"    Cone 1 = ({CONE_1_X:+.2f}, {CONE_Y:+.2f})")
    print(f"    Cone 2 = ({CONE_2_X:+.2f}, {CONE_Y:+.2f})")
    print(f"    Cone 3 = ({CONE_3_X:+.2f}, {CONE_Y:+.2f})")
    print()
    print("DESIRED PATH")
    print("    Cone 1 pass  -> midpoint 1-2 -> Cone 2 pass")
    print("    -> midpoint 2-3 -> Cone 3 pass -> finish")
    print()
    print(f"    Cone-pass offset = {CONE_PASS_OFFSET_M:.2f} m")
    print(f"    Position tolerance = {POSITION_TOLERANCE_M:.2f} m")
    print(f"    Speed tolerance = {HORIZONTAL_SPEED_TOLERANCE_MPS:.2f} m/s")
    print("    No intentional dwell between waypoints")
    print("=" * 68)
    print()


async def wait_for_connection(drone):
    print("[CONNECT] Waiting for PX4 connection...")

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("[CONNECT] PX4 connected.")
            return


async def wait_until_armed(drone):
    async for armed in drone.telemetry.armed():
        if armed:
            print("[ARM] Vehicle armed.")
            return


async def wait_until_landed(drone):
    async for in_air in drone.telemetry.in_air():
        if not in_air:
            return


async def read_local_position_velocity(drone):
    pv = await drone.telemetry.position_velocity_ned().__anext__()

    position = pv.position
    velocity = pv.velocity

    return (
        position.north_m,
        position.east_m,
        position.down_m,
        velocity.north_m_s,
        velocity.east_m_s,
        velocity.down_m_s,
    )


async def wait_for_takeoff_altitude(drone):
    print()
    print("[TAKEOFF] Waiting for relative altitude...")

    async for altitude in drone.telemetry.altitude():
        rel_alt = altitude.altitude_relative_m
        print(f"[TAKEOFF] Relative altitude = {rel_alt:+.2f} m")

        if rel_alt >= TAKEOFF_ALTITUDE_M * 0.80:
            print("[TAKEOFF] Takeoff altitude reached.")
            return


async def move_to_position_and_stop(
    drone,
    waypoint_index,
    total_waypoints,
    name,
    target_n,
    target_e,
    target_d,
):
    # Simple point-to-point positional control. The same absolute PositionNedYaw target is
    # sent repeatedly until: position error <= tolerance AND horizontal speed <= speed
    # tolerance Then the waypoint is declared reached and the next waypoint begins

    print()
    print("-" * 68)
    print(
        f"[POSITION] WP {waypoint_index}/{total_waypoints} | {name}"
    )
    print(
        f"[TARGET] PX4 N={target_n:+.2f}, E={target_e:+.2f}, D={target_d:+.2f}"
    )
    print("[CONTROL] Direct PositionNedYaw target")
    print("[CONTROL] Same absolute target repeated until reached")
    print("-" * 68)

    start_time = time.monotonic()
    last_print = 0.0

    while True:
        now = time.monotonic()

        if now - start_time > WAYPOINT_TIMEOUT_S:
            raise RuntimeError(
                f"Waypoint {waypoint_index} timed out after "
                f"{WAYPOINT_TIMEOUT_S:.1f} s"
            )

        # IMPORTANT: direct position target; no velocity command
        await drone.offboard.set_position_ned(
            PositionNedYaw(
                target_n,
                target_e,
                target_d,
                YAW_DEG,
            )
        )

        n, e, d, vn, ve, vd = await read_local_position_velocity(drone)

        err = distance_3d(n, e, d, target_n, target_e, target_d)
        xy_err = distance_xy(n, e, target_n, target_e)
        vxy = speed_xy(vn, ve)

        # Print at a readable rate
        if now - last_print >= 1.0 / PRINT_RATE_HZ:
            print(
                f"[FLIGHT] WP {waypoint_index}/{total_waypoints} | "
                f"pos N={n:+.2f} E={e:+.2f} D={d:+.2f} | "
                f"err={err:.2f} m | xy_err={xy_err:.2f} m | "
                f"speed={vxy:.2f} m/s"
            )
            last_print = now

        # Require BOTH position and low speed before releasing the waypoint
        if err <= POSITION_TOLERANCE_M and vxy <= HORIZONTAL_SPEED_TOLERANCE_MPS:
            print(
                f"[GOAL] WP {waypoint_index}/{total_waypoints} REACHED | "
                f"{name} | "
                f"pos N={n:+.2f} E={e:+.2f} D={d:+.2f} | "
                f"error={err:.2f} m | speed={vxy:.2f} m/s"
            )
            return

        await asyncio.sleep(1.0 / CONTROL_RATE_HZ)


async def run_test_1(drone):
    print_course_geometry()

    # Takeoff
    print("[ACTION] Setting takeoff altitude...")
    await drone.action.set_takeoff_altitude(TAKEOFF_ALTITUDE_M)

    print("[ACTION] Arming...")
    await drone.action.arm()
    await wait_until_armed(drone)

    print("[ACTION] Taking off...")
    await drone.action.takeoff()
    await wait_for_takeoff_altitude(drone)

    # Give EKF/local position a moment to settle after takeoff
    await asyncio.sleep(1.0)

    # Establish the local-NED reference AFTER takeoff. Keep this altitude for every waypoint
    start_n, start_e, start_d, _, _, _ = await read_local_position_velocity(drone)

    print()
    print("[REFERENCE] POST-TAKEOFF PX4 LOCAL-NED")
    print(f"            North = {start_n:+.3f} m")
    print(f"            East  = {start_e:+.3f} m")
    print(f"            Down  = {start_d:+.3f} m")

    print()
    print("[FRAME]")
    print("        Gazebo +X -> PX4 +East")
    print("        Gazebo +Y -> PX4 +North")
    print("        Gazebo +Z -> PX4 -Down")
    print()
    print("[PATH]")

    # Convert the Gazebo path to PX4 local N/E using the measured post-takeoff local
    # origin. Hold the measured D coordinate
    waypoints_px4 = []

    for i, ((gx, gy), name) in enumerate(zip(WAYPOINTS_GAZEBO, WAYPOINT_NAMES), start=1):
        tn, te = gazebo_to_px4(gx, gy, start_n, start_e)
        td = start_d

        waypoints_px4.append((tn, te, td))

        print(
            f"    WP {i}: {name:<22} "
            f"Gazebo ({gx:+.2f}, {gy:+.2f}) -> "
            f"PX4 (N={tn:+.2f}, E={te:+.2f}, D={td:+.2f})"
        )

    print()
    print("[OFFBOARD] Sending initial position setpoint...")

    # PX4 requires a setpoint before starting offboard
    first_n, first_e, first_d = waypoints_px4[0]
    await drone.offboard.set_position_ned(
        PositionNedYaw(first_n, first_e, first_d, YAW_DEG)
    )

    print("[OFFBOARD] Starting...")
    try:
        await drone.offboard.start()
    except OffboardError as exc:
        print(f"[ERROR] Offboard start failed: {exc}")
        raise

    print("[OFFBOARD] Started.")

    # Execute each point and WAIT for it to settle
    for i, (name, (tn, te, td)) in enumerate(
        zip(WAYPOINT_NAMES, waypoints_px4), start=1
    ):
        await move_to_position_and_stop(
            drone,
            i,
            len(waypoints_px4),
            name,
            tn,
            te,
            td,
        )

    print()
    print("=" * 68)
    print("[TEST 1] ALL WAYPOINTS REACHED")
    print("[TEST 1] Holding final position briefly...")
    print("=" * 68)

    final_n, final_e, final_d = waypoints_px4[-1]
    await drone.offboard.set_position_ned(
        PositionNedYaw(final_n, final_e, final_d, YAW_DEG)
    )

    await asyncio.sleep(FINAL_HOLD_S)

    n, e, d, vn, ve, vd = await read_local_position_velocity(drone)
    final_err = distance_3d(n, e, d, final_n, final_e, final_d)
    final_speed = speed_xy(vn, ve)

    print(
        f"[FINAL] pos N={n:+.3f} E={e:+.3f} D={d:+.3f} | "
        f"error={final_err:.3f} m | speed={final_speed:.3f} m/s"
    )

    print("[OFFBOARD] Stopping offboard mode...")
    try:
        await drone.offboard.stop()
    except OffboardError as exc:
        print(f"[WARNING] Offboard stop: {exc}")

    print("[ACTION] Landing...")
    await drone.action.land()
    await wait_until_landed(drone)

    print("[RESULT] Test 1 complete.")


async def main():
    drone = System()

    try:
        await drone.connect(system_address=SYSTEM_ADDRESS)
        await wait_for_connection(drone)
        await run_test_1(drone)

    except KeyboardInterrupt:
        print("\n[USER] Ctrl+C received.")

        try:
            await drone.offboard.stop()
        except Exception:
            pass

        try:
            await drone.action.land()
        except Exception:
            pass

    except Exception as exc:
        print()
        print("=" * 68)
        print(f"[ERROR] {type(exc).__name__}: {exc}")
        print("=" * 68)

        try:
            await drone.offboard.stop()
        except Exception:
            pass

        try:
            await drone.action.land()
        except Exception:
            pass

        raise


if __name__ == "__main__":
    asyncio.run(main())
