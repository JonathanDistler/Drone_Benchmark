# Swarm benchmarks and behaviors

See the [position and velocity control guide](flight_controls.md) for coordinates,
mode selection, speed limits, stop behavior, console commands, and Python/RL examples.

Start the test world in Terminal 1. Multi-drone launches default to the swarm world;
single-drone launches default to the obstacle world. An explicit `--world` overrides
this choice. The default launcher build picks up these changes:

```bash
cd ~/drone_obstacle_course
/usr/bin/python3 scripts/start_simulation.py --world swarm --n 3
```

In Terminal 2:

```bash
cd ~/drone_obstacle_course
source /opt/ros/jazzy/setup.bash
source install-system/local_setup.bash
ros2 run swarm_mavsdk_ros2 swarm_tests --n 3 --seed 42
```

Enter `startup`, `arm`, and `takeoff`, waiting for each command to finish.
Then enter `test 1`, `test 2`, or `test 3` (bare numbers also work).
Enter `land` when finished, wait for touchdown, then `quit`. In the same
sourced terminal run `ros2 run swarm_mavsdk_ros2 swarm_command disarm --n 3`.
The benchmark menu does not accept `disarm`. Quitting does not land automatically.
Stop the launcher with Ctrl+C after landing.

| Swarm test | Setup, excluded from timing and CSV samples | Recorded behavior |
| --- | --- | --- |
| 1 | Settled line in the test arena | 3 m East translation, with XY/YZ pairwise MSE before and after |
| 2 | Separated random starting positions | Time to reach and settle into an East-parallel line; final target RMSE |
| 3 | Line, then majority/minority partition | Both groups translate 1 m East; XY/YZ pairwise MSE before and after |

Test 3 requires at least three drones. The first `n//2 + 1` drones are the
majority; the rest are the minority. The groups move to opposite North offsets
before measurement starts. `--direction left` selects negative North for the
majority, while `right` selects positive North. Without an explicit direction,
the seed controls the choice. Every member receives a nonzero partition offset,
including the center member of an odd-sized swarm.

Tests require the dedicated `drone_obstacle_course_swarm` world (`--world swarm`),
with a large 100 × 100 m open ground plane, and form up at its center.
Single-drone cone and flag tests default to tracking a sinusoidal position reference at 20 Hz.
Both solo and swarm test CLIs accept `--mode velocity --speed 0.4`; Python
test APIs accept `mode="velocity", speed_mps=.4`. Velocity routes use bounded
feedback and settle at their final destination. Interactive menus retain defaults.
The drone weaves continuously past the cones and arcs around the selected side
of the flag, then returns to the lane. Forward speed eases at the start and end
and its reference peaks at 0.5 m/s by default; intermediate points require no arrival wait or stop.
Setup and final destinations still settle.
Solo setup uses a bounded 1 m/s velocity approach by default, moves directly
from the clear start corridor, and settles only at the final start position.
Returns from inside the course still climb above the obstacles before retreating.
The sinusoidal reference pauses when tracking error exceeds 15 cm, so a slow
vehicle cannot skip the alternating cone offsets. A bounded timeout fails a
stalled run instead of recording it as completed.
Custom Python
routes can use `client.move_to_and_wait(north, east, down, settle=False)` for
intermediate points and the default `settle=True` for their final point.
Fleet movement services queued ROS callbacks throughout each 20 Hz control
period to keep telemetry current as the number of drones increases.
Swarm destinations use a per-drone progress timeout (90 seconds by default),
allowing slow simulations to finish. Each member must settle or reduce its
remaining distance by at least 5 cm within that interval. A stalled member stops
the fleet and reports its target, actual position, error, speed, and movement phase.

Swarm reorganization commands every drone to the same flight altitude. Clear
direct routes move together; crossing routes use temporary horizontal parking
slots and detours around stationary members, moving one drone at a time.
These maneuvers keep the same commanded altitude throughout and can take longer
than a direct formation change. `staged=False` requires clear direct routes.

The solo obstacle course (`--world obstacles`) is reserved for single-drone tests.
Swarm benchmarks require an already armed, airborne fleet in OFFBOARD mode. Every setup destination must settle
before recording begins. Add `--wait-for-signal` to press Enter after setup and
start timing explicitly.

One-shot tests, after startup/arm/takeoff:

```bash
ros2 run swarm_mavsdk_ros2 swarm_command test 1 --n 3
ros2 run swarm_mavsdk_ros2 swarm_command test 2 --n 3 --seed 42
ros2 run swarm_mavsdk_ros2 swarm_command test 3 --n 3 --direction left
ros2 run swarm_mavsdk_ros2 swarm_tests --n 3 --test all --seed 42
```

All tests append a common telemetry schema to `results/swarm_tests/<UTC-session-timestamp>.csv`.
Each row includes run/test/drone IDs, phase, status, elapsed time, common-frame
position, local velocity, targets, error, and cumulative distance. Summaries
append to `results/swarm_tests/summary.csv`, including duration, XY/YZ MSE,
target RMSE, seed, partition direction/counts, and any recorded failure.
`--output-dir` changes the directory. Samples and a failed summary are preserved
if recorded movement fails; setup failures create no test run.

Pairwise MSE means the average squared distance over all distinct drone pairs
in the indicated plane. XY uses North/East, and YZ uses East/Down. It measures
swarm spread; before/after equality checks preservation during translation.
Target RMSE reports destination accuracy in all three dimensions. Positions use
a common NED frame. The shared launcher's x500_vision external-vision estimator
already includes the world spawn displacement in PX4 NED telemetry, so the
controller does not add that displacement again.

## General behaviors

These work in any launcher world. Blank-world example:

```bash
/usr/bin/python3 scripts/start_simulation.py --world blank --n 3
```

Use a new Terminal 2 with the same ROS environment setup:

```bash
ros2 run swarm_mavsdk_ros2 swarm_command startup --n 3
ros2 run swarm_mavsdk_ros2 swarm_command arm --n 3
ros2 run swarm_mavsdk_ros2 swarm_command takeoff --n 3

ros2 run swarm_mavsdk_ros2 swarm_command forward 2 --n 3
ros2 run swarm_mavsdk_ros2 swarm_command backward 2 --n 3
ros2 run swarm_mavsdk_ros2 swarm_command translate --ned 0 3 0 --n 3
ros2 run swarm_mavsdk_ros2 swarm_command formation --shape line --spacing 1.5 --n 3
ros2 run swarm_mavsdk_ros2 swarm_command formation --shape grid --spacing 2 --n 3
ros2 run swarm_mavsdk_ros2 swarm_command formation --shape circle --spacing 2 --n 3
ros2 run swarm_mavsdk_ros2 swarm_command randomize 3 --seed 42 --n 3
ros2 run swarm_mavsdk_ros2 swarm_command split --direction left --n 3
ros2 run swarm_mavsdk_ros2 swarm_command yaw --radians 1.5708 --n 3
ros2 run swarm_mavsdk_ros2 swarm_command yaw_by --radians -1.5708 --n 3
ros2 run swarm_mavsdk_ros2 swarm_command stop --n 3
ros2 run swarm_mavsdk_ros2 swarm_command land --n 3
ros2 run swarm_mavsdk_ros2 swarm_command disarm --n 3
```

Python example; save it in the project and run with `/usr/bin/python3` after
sourcing the ROS environment:

```python
from swarm_mavsdk_ros2 import Swarm

with Swarm(3) as swarm:
    swarm.startup()
    swarm.arm()
    swarm.takeoff()
    swarm.forward(2)
    swarm.backward(2)
    swarm.formation('grid', spacing_m=2)
    swarm.formation('circle', spacing_m=2)
    swarm.randomize(radius_m=3, seed=42)
    swarm.formation('line', spacing_m=1.5)
    swarm.split('left')
    swarm.translate(east=1)
    print(swarm.centroid())
    swarm.hold_position()
    swarm.land()
    swarm.disarm()
```

Run all benchmarks in Python with the **swarm world** loaded:

```python
from swarm_mavsdk_ros2 import Swarm

with Swarm(3) as swarm:
    swarm.startup()
    swarm.arm()
    swarm.takeoff()
    for number in (1, 2, 3):
        print(swarm.run_test(f'test {number}', seed=42))
    swarm.land()
    swarm.disarm()
```

`swarm.go_to([(north, east, down), ...])` accepts one common-frame target per
member. Formation changes and randomization stage drones at separate altitudes,
move horizontally, then descend, preventing crossing formation paths. This uses
1.5 m between altitude layers and can climb high for large fleets. Pass
`staged=False` only when direct paths are clear; intersecting synchronized paths
are rejected. Translation preserves relative offsets and does not use altitude
staging. Motion waits for every member to reach its target and settle, checks
fresh telemetry and armed/OFFBOARD mode, and requests a fleet hold on failure.

General behaviors check drone separation, but do not plan around arbitrary
world obstacles. Live flight validation remains outstanding. The original
`swarm.move_to(...)` remains an absolute target in each drone's **local** frame;
use `go_to` for distinct destinations in the common frame.

The single-drone tests remain available through `Drone.run_test`. Test 2 now
approaches the flag, follows a simulated left/right signal with a 0.9 m lateral
move, then passes forward. Use `drone.run_test('test 2', flag_direction='left')`
or `seed=42` for a repeatable simulated signal; this does not actuate the mesh or
read a camera. The flag policy uses three measured targets (approach, lateral
response, finish), removing the redundant forward target. The cone policy uses
four targets with alternating 0.68 m offsets and crosses the centerline between
cones. Its planned distance is 11.78 m versus 11.94 m previously, with the same
start, finish, and altitude. Segment clearance is checked against a 0.50 m
envelope around the pole footprint; this assumes a 0.25 m vehicle radius and
0.15 m tracking tolerance. These are efficient static-course policies, not a
proof of globally optimal flight. Actual travel distance and tracking clearance
require simulation validation. Single-drone logs remain in
`results/tests/<UTC-session-timestamp>.csv`.

Single-drone CSV rows include NED position (`north`, `east`, `down`, metres),
NED velocity (`vn`, `ve`, `vd`, m/s), total/horizontal speed, local altitude,
NED acceleration, heading, position/velocity validity flags, armed/OFFBOARD
status, telemetry ages, PX4 timestamps, target error, cumulative distance,
requested control mode, and requested speed (m/s).
Battery columns include voltage (V), current (A), remaining fraction,
discharged capacity (mAh), temperature (C), predicted remaining time (s),
connection status, and warning. `power_w` is electrical battery power computed
as voltage times current; it is not mechanical motor power. Battery data older
than five seconds, local telemetry older than two seconds, and invalid or
unavailable sensor readings are blank. Simulation may not supply meaningful
battery current, so power can remain blank. Restart the drone nodes to enable
the new telemetry forwarding. Existing CSV headers are upgraded on the next
test while preserving historical rows with blank new fields. Samples cover
the measured test route, excluding preparation, takeoff, and landing.

The swarm sample CSV also includes each member’s detailed telemetry and battery
fields, control mode, requested speed, common-frame centroid, pairwise XY/YZ MSE,
target RMSE, and minimum separation. Missing sensor data uses empty cells.
The launcher recorder preserves full ROS payloads in `topics.csv` and numeric
positions in `positions.csv` under a unique `results/ros2` run directory.
A timestamped wide CSV starts with the recording session and is sampled by the
Gazebo physics clock. Test events annotate its name and status fields. It includes
all discovered drone and swarm payloads, receipt timestamps, individual nested
scalar/array columns, and each drone's `control_state` (architecture, mode,
goals, streamed reference, speed configuration, and lifecycle flags).
These wide rows contain the latest received values, not synchronized sensor samples.
The recorder exports the timeline every 10 seconds and on clean shutdown.
