# Run and test a ROS 2 swarm

See the [position and velocity control guide](flight_controls.md) for coordinates,
mode selection, speed limits, stop behavior, console commands, and Python/RL examples.

See [swarm benchmarks and behaviors](swarm_benchmarks.md) for the recorded translation, random-to-line, and partition/translation tests.

Complete [installation](run_and_share.md) first. Commands assume the project
root in each terminal. Start SITL in Terminal 1 (explicitly selecting three drones;
the Python launcher defaults to one):

```bash
/usr/bin/python3 scripts/start_simulation.py --ros system --n 3
```

The launcher builds the packages and starts Gazebo, PX4, DDS, GCS links, drone
interfaces, and CSV recording. It accepts 1–250 vehicle IDs; practical capacity
depends on hardware. Use `--px4-dir` to select a PX4 checkout, `--headless` to
omit the Gazebo GUI, and `--spawn-height` / `--takeoff-height` for separate spawn
and flight heights. Logs are in `results/startup`. See [Conda setup](miniconda_setup.md)
for the isolated Conda launch mode.

One shared Micro XRCE-DDS Agent bridges every PX4 instance to ROS. Each instance
runs its own PX4 commander and flight controllers; the ROS launch instantiates
one `DroneNode` at `/drone_i` for each instance, with system ID `i+1`.
Instance zero uses `/fmu`, and the other instances use `/px4_i/fmu`.
The `swarm_controller` coordinates these independent drone interfaces.
For an individual console, run `ros2 run swarm_mavsdk_ros2 control_console --drone 0`
with the matching overlay sourced.

In the single-drone console, enter `startup`, `arm`, and `takeoff`, waiting for
each command to finish. Arming waits for PX4 to report OFFBOARD mode and passing
preflight checks while position setpoints continue streaming. If readiness is
not confirmed within four seconds, the pending arm request is cancelled; inspect
`results/startup/px4-0.log` for preflight failures before retrying `arm`.

In Terminal 2:

```bash
source /opt/ros/jazzy/setup.bash
source install-system/local_setup.bash
python3 scripts/swarm_validity_test.py --n 3
```

Add `--fly` to test takeoff, six 0.5 m movements, hold, landing, and disarming.
The default expected height is 2 m; `--height` changes the test expectation only.
Failures exit nonzero. Cleanup requests disarm only after fresh landed telemetry.
Use `--list-commands` for the Python command catalog.

## Manual commands

```bash
ros2 run swarm_mavsdk_ros2 swarm_command startup --n 3
ros2 run swarm_mavsdk_ros2 swarm_command arm --n 3
ros2 run swarm_mavsdk_ros2 swarm_command takeoff --n 3
ros2 run swarm_mavsdk_ros2 swarm_command forward 0.5 --n 3
ros2 run swarm_mavsdk_ros2 swarm_command land --n 3
```

Wait for each command to finish. Startup prepares streaming; arming is explicit.
Takeoff completes after every member reaches altitude and slows down in armed
OFFBOARD mode. After touchdown, run `swarm_command disarm --n 3`.
For one vehicle use `drone_command ... --drone 0`. Ctrl-C stops the launcher.

## Single-drone obstacle tests

Choose the single-drone environment explicitly (system ROS, even from an activated Conda shell):

```bash
# Empty world for manual movement practice (also the shortcut's default)
bash scripts/start_single_drone_ros2.sh blank
# Obstacle test world
bash scripts/start_single_drone_ros2.sh test
# Start the test world and carry out all three tests automatically
bash scripts/start_single_drone_ros2.sh test --tests 1 2 3
# Select one test and velocity control
bash scripts/start_single_drone_ros2.sh test --tests 1 --test-mode velocity
```

Automatic tests wait for simulator and ROS readiness, prepare takeoff, and run
in the requested order. Progress logs are `results/startup/test-N.log`; test
CSVs are under `results/tests`. After completion the drone holds; run
`ros2 run swarm_mavsdk_ros2 drone_command land --drone 0` from the terminal
below to land. A failed test exits the launcher with an error and stops its
simulation processes. `--headless` runs without the Gazebo GUI.

For manual movement, prepare the drone in a second terminal first:

```bash
source /opt/ros/jazzy/setup.bash
source install-system/local_setup.bash
ros2 run swarm_mavsdk_ros2 drone_command startup --drone 0
ros2 run swarm_mavsdk_ros2 drone_command arm --drone 0
ros2 run swarm_mavsdk_ros2 drone_command takeoff --drone 0
ros2 run swarm_mavsdk_ros2 drone_command forward 0.5 --drone 0 --mode position
ros2 run swarm_mavsdk_ros2 drone_command right 0.5 --drone 0 --mode velocity --speed 0.4
```

Wait for each command to finish. Directional commands wait for armed/OFFBOARD
status before publishing; they require explicit preparation. Obstacle test
commands perform their own preparation and reject the blank world.

Launch `scripts/start_simulation.py --ros system --n 1 --world obstacles`, then
source `/opt/ros/jazzy/setup.bash` and `install-system/local_setup.bash` in another
terminal. Run these commands in order, waiting for each to complete:

```bash
ros2 run swarm_mavsdk_ros2 drone_command test 1
ros2 run swarm_mavsdk_ros2 drone_command test 2
ros2 run swarm_mavsdk_ros2 drone_command test 3
```

Each test prepares control streaming and arms/takes off when needed. Test 1
streams a continuous sinusoidal weave with lateral peaks at the three cones,
crossing the lane center between them. Test 2 prints a random integer: `0` passes on the near side of
the flag (toward the central cone lane), and `1` passes on the far side (away
from that lane). The flag path curves continuously to a 0.9 m lateral peak
at the flag and returns to the lane center after passing. Tests 1 and 2
slow only at the start and finish; intermediate peaks do not require a stop. Test 3 flies along the center of the box opening and exits
the other end. Commands print waypoint progress and only report completion
after the final waypoint is reached. The drone holds there until the next
command; use `drone_command land` to land.

The interactive `control_console --drone 0` accepts the same `test 1`, `test 2`,
and `test 3` commands. Runs append to `results/tests/all_tests.csv`.

## CSV results

Each shared-launcher session creates a unique directory under `results/ros2/`.
The shared Python launcher sets this path explicitly; `RESULTS_DIR` only applies
to the older shell launcher. For a separate recorder, set its `output_dir` parameter. The recorder prints the exact path and writes:

| File | Contents |
| --- | --- |
| `topics.csv` | Receipt times, topic, ROS type, and complete message as a JSON cell |
| `positions.csv` | Receipt times, topic, drone ID, frame, North/East/Down in metres |
| `<UTC timestamp>.csv` | Latest complete telemetry and individual scalar columns at each physics clock tick, including ordinary manual flight |
| `timeline.jsonl` | Durable timeline spool retained for recovery after interruption |
| `inventory.csv` | Discovered topics, subscription status, and errors |

The timeline starts at recorder initialization (`test=session`, `test_status=idle`),
then records test names and status events in the same file. It needs Gazebo's
native physics clock or the ROS `/clock` bridge; without either, `topics.csv`
and `positions.csv` still record arrivals but the timeline has no samples.
Each topic retains its complete JSON cell and receipt timestamp. Nested scalars
and array elements also get individual columns such as
`drone_0.px4.in.trajectory_setpoint.position.0`. PX4 instance zero `/fmu`
normalizes to `drone_0.px4`; `/px4_i/fmu` normalizes to `drone_i.px4`.
Late fields expand the schema; earlier rows have empty cells. Array elements
removed by a later message become empty rather than retaining old values.
Values are latest received samples, not synchronized measurements; check each
topic's `received_ros_ns` for age. No telemetry is fabricated for missing sensors.

Each drone publishes `/drone_i/control_state` at 20 Hz with architecture,
selected mode, position goal and streamed reference, velocity goal, yaw goal,
position reference speed, takeoff height, startup/landing/takeoff flags, and
OFFBOARD stream count. The recorder expands its JSON into individual columns.
PX4 topics add actual setpoints, control flags, status, attitude, acceleration,
battery, validity flags, and other available telemetry. Test-specific CSVs
include requested mode and speed alongside targets, tracking error, measured
velocity, acceleration, battery/power, and freshness metrics. Wide files are
exported every ten seconds and on shutdown; full physics timelines can be large.

Recording discovers all visible ROS topics every second, including PX4 inputs,
outputs, command topics, logs, and topics added later. Message definitions must
be installed; unavailable types are reported and retried. Files flush every
second and on graceful shutdown. Disk failures stop the recorder with an error.
Services are not topics and are not captured as request/response records.

Subscriptions use best-effort, volatile QoS to accept both reliable and
best-effort publishers. Recording begins after discovery; earlier messages,
retained history, and samples lost under load cannot be recovered. Check
`inventory.csv` for coverage. JSON cells preserve arrays and nested fields;
NaN/Infinity use Python JSON's numeric extensions for PX4 unset values.

Positions are each vehicle's **local NED**, not a shared world frame. Up is
negative Z; missing peer positions remain NaN. Odometry and fleet/peer arrays
produce separate rows, identified by topic. The public odometry orientation is
currently an identity quaternion; use PX4 attitude telemetry for yaw analysis.

To record an existing ROS session separately:

```bash
ros2 run swarm_mavsdk_ros2 topic_recorder --ros-args -p output_dir:=results/ros2
```

Inspect live interfaces with `ros2 topic list -t` and `ros2 service list -t`.
Drone 0 uses `/fmu/...`; others use `/px4_i/fmu/...`. Public interfaces live under
`/drone_i/`, with fleet positions at `/swarm/positions_ned`.

## Plot positions in MATLAB

Run `scripts/plot_drone_positions.m` in MATLAB and select a run's `positions.csv`.
Alternatively, set `csvPath` at the top of the script. Each drone gets a 3D panel
with North/East/Up axes, start/end markers, and a shared elapsed-time color scale
(blue to yellow). Times are relative to the first selected sample in the run.

The plot prefers each drone's state topic, falls back to `/swarm/positions_ned`,
and excludes duplicate peer arrays. Panels use individual local frames; they do
not reconstruct fleet spacing in world coordinates. Set `outputPath` to save a
PNG. The script uses `tiledlayout` and `exportgraphics` (MATLAB R2020a or later).

## Regression tests

```bash
PYTHONPATH="$PWD/ros2/swarm_mavsdk_ros2:$PYTHONPATH" \
  ROS_DOMAIN_ID=91 /usr/bin/python3 -m pytest ros2/swarm_mavsdk_ros2/test -q
```

These tests exercise control sequencing, stale telemetry, timeouts, launch
mapping, and CSV/discovery failures without flying a vehicle. Run the SITL
`--fly` check separately to assess simulated flight behavior.

Swarm benchmarks command a shared 2 m flight height for lineup, random
positioning, split, and translation. Random positioning uses separated
horizontal paths without altitude staging.

## Position and velocity flight validation

With a matching headless or graphical SITL launch running, run:

```bash
python3 scripts/validate_flight_controls.py --n 3 --controls-only
```

This prepares and takes off the fleet, moves drone zero to destinations and forward/backward in both modes, moves the
whole fleet forward and backward in both modes, then lands and disarms after
fresh landed telemetry. Use `--n 1 --controls-only` for a single vehicle. Omit
`--controls-only` with at least three drones to include all three swarm benchmarks
in both modes. CSVs are under `results/control_validation/` and the launcher's
`results/ros2/` run directory.

To validate the edge architecture, restart the launcher with
`--architecture edge:drone_0` and pass the same argument to the validator:

```bash
python3 scripts/validate_flight_controls.py --n 3 --controls-only --architecture edge:drone_0
```

Regression tests cover both modes through real ROS transport to all three
members in central and edge configurations; SITL results separately establish
flight tracking. See [control validation](control_validation.md) for the latest measured results.
