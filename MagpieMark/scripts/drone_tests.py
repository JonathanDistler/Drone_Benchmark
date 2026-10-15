#!/usr/bin/env python3

# Simple PX4 / Gazebo Test-1 waypoint controller with run metrics. CONTROL: PositionNedYaw
# ONLY. TEST-1 PATH: Cone 1 pass -> midpoint between Cone 1 and Cone 2 -> Cone 2 inside-corner
# pass -> midpoint between Cone 2 and Cone 3 -> Cone 3 pass -> finish The waypoint controller
# remains intentionally simple: - one absolute PositionNedYaw target at a time - no PID / PD -
# no velocity command - no moving lookahead - no trajectory interpolation - no intentional
# dwell between waypoints Metrics: The timer starts at the instant the takeoff
# relative-altitude threshold is reached. Telemetry is sampled continuously from that point
# until the test finishes and is saved as a CSV file that opens directly in Excel / Google
# Sheets

import asyncio
import csv
import math
import time
from datetime import datetime

from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw


# CONNECTION / FLIGHT SETTINGS

SYSTEM_ADDRESS = "udpin://127.0.0.1:14540"

TAKEOFF_ALTITUDE_M = 2.0

CONTROL_RATE_HZ = 10.0
PRINT_RATE_HZ = 2.0
METRICS_RATE_HZ = 10.0

POSITION_TOLERANCE_M = 0.20
HORIZONTAL_SPEED_TOLERANCE_MPS = 0.25

WAYPOINT_TIMEOUT_S = 45.0
YAW_DEG = 0.0

METRICS_OUTPUT_PREFIX = "drone_test1_metrics"


# GAZEBO COURSE

CONE_1_X = 2.0
CONE_2_X = 5.0
CONE_3_X = 8.0
CONE_Y = 0.0

GAZEBO_START_X = -1.0
GAZEBO_START_Y = 0.0

CONE_PASS_OFFSET_M = 0.75

# Cone 2 is intentionally NOT placed exactly at (5.0, -0.75). It is shifted slightly forward
# in +X and slightly inward toward the center of the weave, creating the requested inside
# corner
CONE_2_X_OFFSET_M = 0.10
CONE_2_PASS_Y_OFFSET_M = -0.75

GAP_12_X = (CONE_1_X + CONE_2_X) / 2.0
GAP_23_X = (CONE_2_X + CONE_3_X) / 2.0


# TEST-1 WAYPOINTS IN GAZEBO X/Y

WAYPOINTS_GAZEBO = [
    (CONE_1_X, CONE_Y + CONE_PASS_OFFSET_M),
    (GAP_12_X, CONE_Y),
    (CONE_2_X + CONE_2_X_OFFSET_M, CONE_Y + CONE_2_PASS_Y_OFFSET_M),
    (GAP_23_X, CONE_Y),
    (CONE_3_X, CONE_Y + CONE_PASS_OFFSET_M),
    (10.0, CONE_Y),
]

WAYPOINT_NAMES = [
    "CONE 1 PASS",
    "GAP 1-2 MIDPOINT",
    "CONE 2 INSIDE CORNER",
    "GAP 2-3 MIDPOINT",
    "CONE 3 PASS",
    "FINISH",
]


# METRICS

METRIC_FIELDS = [
    "elapsed_s",
    "wall_time",
    "waypoint_index",
    "waypoint_name",
    "target_n_m",
    "target_e_m",
    "target_d_m",
    "north_m",
    "east_m",
    "down_m",
    "velocity_n_m_s",
    "velocity_e_m_s",
    "velocity_d_m_s",
    "horizontal_speed_m_s",
    "speed_3d_m_s",
    "horizontal_error_m",
    "position_error_3d_m",
    "gazebo_x_m",
    "gazebo_y_m",
    "incremental_distance_m",
    "cumulative_distance_m",
]


class RunMetrics:
    def __init__(self):
        self.start_time = None
        self.rows = []
        self.previous_position = None
        self.cumulative_distance = 0.0

    def start(self):
        self.start_time = time.monotonic()
        self.rows = []
        self.previous_position = None
        self.cumulative_distance = 0.0

    def record(
        self,
        waypoint_index,
        waypoint_name,
        target_n,
        target_e,
        target_d,
        n,
        e,
        d,
        vn,
        ve,
        vd,
        start_n,
        start_e,
    ):
        if self.start_time is None:
            return

        if self.previous_position is None:
            incremental_distance = 0.0
        else:
            pn, pe, pd = self.previous_position
            incremental_distance = math.sqrt(
                (n - pn) ** 2 +
                (e - pe) ** 2 +
                (d - pd) ** 2
            )

        self.cumulative_distance += incremental_distance
        self.previous_position = (n, e, d)

        elapsed = time.monotonic() - self.start_time
        horizontal_speed = math.sqrt(vn * vn + ve * ve)
        speed_3d = math.sqrt(vn * vn + ve * ve + vd * vd)
        horizontal_error = math.sqrt(
            (target_n - n) ** 2 +
            (target_e - e) ** 2
        )
        position_error = math.sqrt(
            (target_n - n) ** 2 +
            (target_e - e) ** 2 +
            (target_d - d) ** 2
        )

        gazebo_x = GAZEBO_START_X + (e - start_e)
        gazebo_y = GAZEBO_START_Y + (n - start_n)

        self.rows.append({
            "elapsed_s": elapsed,
            "wall_time": datetime.now().isoformat(timespec="milliseconds"),
            "waypoint_index": waypoint_index,
            "waypoint_name": waypoint_name,
            "target_n_m": target_n,
            "target_e_m": target_e,
            "target_d_m": target_d,
            "north_m": n,
            "east_m": e,
            "down_m": d,
            "velocity_n_m_s": vn,
            "velocity_e_m_s": ve,
            "velocity_d_m_s": vd,
            "horizontal_speed_m_s": horizontal_speed,
            "speed_3d_m_s": speed_3d,
            "horizontal_error_m": horizontal_error,
            "position_error_3d_m": position_error,
            "gazebo_x_m": gazebo_x,
            "gazebo_y_m": gazebo_y,
            "incremental_distance_m": incremental_distance,
            "cumulative_distance_m": self.cumulative_distance,
        })

    def save_csv(self, path):
        with open(path, "w", newline="", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=METRIC_FIELDS)
            writer.writeheader()
            writer.writerows(self.rows)

    def summary(self):
        if not self.rows:
            return {}

        return {
            "test_time_s": self.rows[-1]["elapsed_s"],
            "total_distance_m": self.rows[-1]["cumulative_distance_m"],
            "max_horizontal_speed_m_s": max(
                row["horizontal_speed_m_s"] for row in self.rows
            ),
            "max_3d_speed_m_s": max(
                row["speed_3d_m_s"] for row in self.rows
            ),
            "max_vertical_speed_m_s": max(
                abs(row["velocity_d_m_s"]) for row in self.rows
            ),
            "average_horizontal_speed_m_s": sum(
                row["horizontal_speed_m_s"] for row in self.rows
            ) / len(self.rows),
            "samples": len(self.rows),
        }


class MetricsState:
    # Shared state read by the continuous metrics logger

    def __init__(self):
        self.waypoint_index = 0
        self.waypoint_name = "TAKEOFF / INITIALIZATION"
        self.target_n = 0.0
        self.target_e = 0.0
        self.target_d = 0.0
        self.running = True


async def metrics_logger(
    drone,
    metrics,
    state,
    start_n,
    start_e,
):
    # Record flight data continuously from timer start until test end

    period = 1.0 / METRICS_RATE_HZ

    while state.running:
        try:
            n, e, d, vn, ve, vd = await read_local_position_velocity(drone)

            metrics.record(
                state.waypoint_index,
                state.waypoint_name,
                state.target_n,
                state.target_e,
                state.target_d,
                n,
                e,
                d,
                vn,
                ve,
                vd,
                start_n,
                start_e,
            )
        except Exception as exc:
            print(f"[METRICS WARNING] {type(exc).__name__}: {exc}")

        await asyncio.sleep(period)


# HELPERS


def distance_3d(n, e, d, tn, te, td):
    return math.sqrt(
        (tn - n) ** 2 +
        (te - e) ** 2 +
        (td - d) ** 2
    )


def distance_xy(n, e, tn, te):
    return math.sqrt(
        (tn - n) ** 2 +
        (te - e) ** 2
    )


def speed_xy(vn, ve):
    return math.sqrt(vn * vn + ve * ve)


def gazebo_to_px4(gx, gy, start_n, start_e):
    # Convert the supplied Gazebo XY coordinates to PX4 local-NED. Empirically established
    # for this simulation: Gazebo +X -> PX4 +East Gazebo +Y -> PX4 +North
    dx = gx - GAZEBO_START_X
    dy = gy - GAZEBO_START_Y

    target_n = start_n + dy
    target_e = start_e + dx

    return target_n, target_e


def print_course_geometry():
    print()
    print("=" * 72)
    print("TEST 1 - SIMPLE POSITION WAYPOINTS + RUN METRICS")
    print("=" * 72)
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
    print("    Cone 1 pass  -> midpoint 1-2 -> Cone 2 inside corner")
    print("    -> midpoint 2-3 -> Cone 3 pass -> finish")
    print()
    print(f"    Cone-pass offset = {CONE_PASS_OFFSET_M:.2f} m")
    print(
        "    Cone 2 inside-corner point = "
        f"({CONE_2_X + CONE_2_X_OFFSET_M:+.2f}, "
        f"{CONE_Y + CONE_2_PASS_Y_OFFSET_M:+.2f})"
    )
    print(f"    Position tolerance = {POSITION_TOLERANCE_M:.2f} m")
    print(
        "    Speed tolerance = "
        f"{HORIZONTAL_SPEED_TOLERANCE_MPS:.2f} m/s"
    )
    print("    No intentional dwell between waypoints")
    print()
    print("METRICS")
    print("    Timer starts when takeoff relative-altitude threshold is reached")
    print("    CSV output is Excel / Google Sheets compatible")
    print("=" * 72)
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
        # IMPORTANT for the installed MAVSDK version: Altitude uses altitude_relative_m
        rel_alt = altitude.altitude_relative_m
        print(f"[TAKEOFF] Relative altitude = {rel_alt:+.2f} m")

        if rel_alt >= TAKEOFF_ALTITUDE_M * 0.80:
            print("[TAKEOFF] Takeoff altitude reached.")
            return rel_alt


async def move_to_position_and_stop(
    drone,
    waypoint_index,
    total_waypoints,
    name,
    target_n,
    target_e,
    target_d,
    metrics_state,
):
    # Go directly to one absolute position and wait for it to settle

    metrics_state.waypoint_index = waypoint_index
    metrics_state.waypoint_name = name
    metrics_state.target_n = target_n
    metrics_state.target_e = target_e
    metrics_state.target_d = target_d

    print()
    print("-" * 72)
    print(
        f"[POSITION] WP {waypoint_index}/{total_waypoints} | {name}"
    )
    print(
        f"[TARGET] PX4 N={target_n:+.2f}, "
        f"E={target_e:+.2f}, D={target_d:+.2f}"
    )
    print("[CONTROL] Direct PositionNedYaw target")
    print("[CONTROL] Same absolute target repeated until reached")
    print("[CONTROL] No intentional dwell")
    print("-" * 72)

    start_time = time.monotonic()
    last_print = 0.0

    while True:
        now = time.monotonic()

        if now - start_time > WAYPOINT_TIMEOUT_S:
            raise RuntimeError(
                f"Waypoint {waypoint_index} timed out after "
                f"{WAYPOINT_TIMEOUT_S:.1f} s"
            )

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

        if now - last_print >= 1.0 / PRINT_RATE_HZ:
            print(
                f"[FLIGHT] WP {waypoint_index}/{total_waypoints} | "
                f"pos N={n:+.2f} E={e:+.2f} D={d:+.2f} | "
                f"err={err:.2f} m | xy_err={xy_err:.2f} m | "
                f"speed={vxy:.2f} m/s"
            )
            last_print = now

        if (
            err <= POSITION_TOLERANCE_M
            and vxy <= HORIZONTAL_SPEED_TOLERANCE_MPS
        ):
            print(
                f"[GOAL] WP {waypoint_index}/{total_waypoints} REACHED | "
                f"{name} | "
                f"pos N={n:+.2f} E={e:+.2f} D={d:+.2f} | "
                f"error={err:.2f} m | speed={vxy:.2f} m/s"
            )
            return

        await asyncio.sleep(1.0 / CONTROL_RATE_HZ)


# TEST 1


async def run_test_1(drone):
    print_course_geometry()

    print("[ACTION] Setting takeoff altitude...")
    await drone.action.set_takeoff_altitude(TAKEOFF_ALTITUDE_M)

    print("[ACTION] Arming...")
    await drone.action.arm()
    await wait_until_armed(drone)

    print("[ACTION] Taking off...")
    await drone.action.takeoff()

    # The timer begins exactly when the takeoff relative-altitude threshold is reached
    takeoff_relative_altitude = await wait_for_takeoff_altitude(drone)

    metrics = RunMetrics()
    metrics.start()

    print()
    print(
        "[TIMER] Started at relative altitude = "
        f"{takeoff_relative_altitude:+.2f} m"
    )

    # Give local position a moment to settle, but this does NOT create a waypoint dwell
    # The metrics timer continues during this period
    await asyncio.sleep(1.0)

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

    waypoints_px4 = []

    print()
    print("[PATH]")
    for i, ((gx, gy), name) in enumerate(
        zip(WAYPOINTS_GAZEBO, WAYPOINT_NAMES), start=1
    ):
        tn, te = gazebo_to_px4(gx, gy, start_n, start_e)
        td = start_d
        waypoints_px4.append((tn, te, td))

        print(
            f"    WP {i}: {name:<24} "
            f"Gazebo ({gx:+.2f}, {gy:+.2f}) -> "
            f"PX4 (N={tn:+.2f}, E={te:+.2f}, D={td:+.2f})"
        )

    # Shared metrics state starts before offboard and is updated before every waypoint
    # The recorder samples continuously at 10 Hz
    metrics_state = MetricsState()
    metrics_state.target_d = start_d

    metrics_task = asyncio.create_task(
        metrics_logger(
            drone,
            metrics,
            metrics_state,
            start_n,
            start_e,
        )
    )

    try:
        print()
        print("[OFFBOARD] Sending initial position setpoint...")

        first_n, first_e, first_d = waypoints_px4[0]
        metrics_state.waypoint_index = 1
        metrics_state.waypoint_name = WAYPOINT_NAMES[0]
        metrics_state.target_n = first_n
        metrics_state.target_e = first_e
        metrics_state.target_d = first_d

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
                metrics_state,
            )

        # One final sample at the finish target, without adding a dwell
        final_n, final_e, final_d = waypoints_px4[-1]
        await drone.offboard.set_position_ned(
            PositionNedYaw(final_n, final_e, final_d, YAW_DEG)
        )

        n, e, d, vn, ve, vd = await read_local_position_velocity(drone)
        final_err = distance_3d(n, e, d, final_n, final_e, final_d)
        final_speed = speed_xy(vn, ve)

        print()
        print("=" * 72)
        print("[TEST 1] ALL WAYPOINTS REACHED")
        print("[TEST 1] No intentional final dwell.")
        print("=" * 72)
        print(
            f"[FINAL] pos N={n:+.3f} E={e:+.3f} D={d:+.3f} | "
            f"error={final_err:.3f} m | speed={final_speed:.3f} m/s"
        )

    finally:
        # Stop the background metrics logger before leaving the test
        metrics_state.running = False
        try:
            await metrics_task
        except Exception as exc:
            print(f"[METRICS WARNING] Logger shutdown: {exc}")

    # Save the run metrics BEFORE landing, so the spreadsheet corresponds exactly to the
    # run from the relative-altitude timer onward
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    metrics_path = f"{METRICS_OUTPUT_PREFIX}_{stamp}.csv"
    metrics.save_csv(metrics_path)

    summary = metrics.summary()

    print()
    print("[METRICS] Saved spreadsheet-compatible CSV:")
    print(f"          {metrics_path}")
    if summary:
        print(f"          Test time          = {summary['test_time_s']:.2f} s")
        print(f"          Total distance     = {summary['total_distance_m']:.2f} m")
        print(
            "          Max horizontal v   = "
            f"{summary['max_horizontal_speed_m_s']:.2f} m/s"
        )
        print(
            "          Max 3D speed       = "
            f"{summary['max_3d_speed_m_s']:.2f} m/s"
        )
        print(
            "          Max vertical speed = "
            f"{summary['max_vertical_speed_m_s']:.2f} m/s"
        )
        print(
            "          Avg horizontal v   = "
            f"{summary['average_horizontal_speed_m_s']:.2f} m/s"
        )
        print(f"          Samples            = {summary['samples']}")

    print("[OFFBOARD] Stopping offboard mode...")
    try:
        await drone.offboard.stop()
    except OffboardError as exc:
        print(f"[WARNING] Offboard stop: {exc}")

    print("[ACTION] Landing...")
    await drone.action.land()
    await wait_until_landed(drone)

    print("[RESULT] Test 1 complete.")


# MAIN


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
        print("=" * 72)
        print(f"[ERROR] {type(exc).__name__}: {exc}")
        print("=" * 72)

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
