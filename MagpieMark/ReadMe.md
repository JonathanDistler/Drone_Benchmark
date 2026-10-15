# Drone obstacle course

A ROS 2 / PX4 / Gazebo benchmark for individual drones and drone swarms,
with static obstacle routes, swarm behaviors, and Python interfaces for RL.
The research goal is to compare a physical test bed with its simulation.

## Start here

Open this folder in VS Code and use **Terminal → New Terminal**. Commands assume
Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, a built PX4 checkout, and a DDS agent.
A ZIP contains the project files; it does not install those dependencies.
For a new machine, complete [installation and sharing](docs/run_and_share.md)
first. For RoboStack, use [Conda setup](docs/miniconda_setup.md) and its matching
overlay instead of the system commands below. Replace `~/drone_obstacle_course`
with your extracted project path when different.

## Single drone

Terminal 1:

```bash
cd ~/drone_obstacle_course
/usr/bin/python3 scripts/start_simulation.py --ros system --n 1 --world blank
```

Wait for `Ready: 1 drones`. The launcher builds the ROS packages and starts
Gazebo, PX4, the DDS agent, MAVProxy, ROS interfaces, and recording. It does not
open a control console. Keep this terminal running.

Terminal 2:

```bash
cd ~/drone_obstacle_course
source /opt/ros/jazzy/setup.bash
source install-system/local_setup.bash
ros2 run swarm_mavsdk_ros2 control_console --drone 0
```

Enter one command at a time and wait for completion:

```text
startup
arm
takeoff
forward 0.5
backward 0.5
land
disarm
quit
```

Wait for touchdown before disarming. Quitting a console does not stop the
simulation or cancel active targets. After landing, press Ctrl-C in Terminal 1.

## Single-drone obstacle tests

Stop the previous session, then launch:

```bash
/usr/bin/python3 scripts/start_simulation.py --ros system --n 1 --world obstacles --tests 1 2 3
```

This prepares flight and runs the three routes automatically. On success the
drone holds its last destination; use the sourced Terminal 2 to land and disarm:

```bash
ros2 run swarm_mavsdk_ros2 drone_command land --drone 0
ros2 run swarm_mavsdk_ros2 drone_command disarm --drone 0
```

For interactive tests, omit `--tests` and enter `test 1`, `test 2`, or `test 3`
in the control console. See [test execution and recording](docs/swarm_validity.md).

## Swarm and drone-led control

Restart Terminal 1 with:

```bash
/usr/bin/python3 scripts/start_simulation.py --ros system --n 3 --world swarm
```

After `Ready: 3 drones`, source the system overlay in Terminal 2 as above:

```bash
ros2 run swarm_mavsdk_ros2 swarm_command startup --n 3
ros2 run swarm_mavsdk_ros2 swarm_command arm --n 3
ros2 run swarm_mavsdk_ros2 swarm_command takeoff --n 3
ros2 run swarm_mavsdk_ros2 swarm_command forward 0.5 --n 3
ros2 run swarm_mavsdk_ros2 swarm_command land --n 3
ros2 run swarm_mavsdk_ros2 swarm_command disarm --n 3
```

For benchmarks, after startup/arm/takeoff run:

```bash
ros2 run swarm_mavsdk_ros2 swarm_tests --n 3 --test all --seed 42
```

Then land and disarm. For a drone-led fleet, add `--architecture edge:drone_0`
to the launcher and every fleet CLI invocation. Counts and architectures must
match the running session. See [benchmarks and Python behaviors](docs/swarm_benchmarks.md).

## Guides

- [Installation, two-terminal workflow, ZIP export, and recipient setup](docs/run_and_share.md)
- [Conda environment and native simulator dependencies](docs/miniconda_setup.md)
- [Coordinates, position/velocity control, Python, and RL](docs/flight_controls.md)
- [Obstacle tests, validation, CSV recording, and MATLAB plots](docs/swarm_validity.md)
- [Swarm benchmarks and behaviors](docs/swarm_benchmarks.md)
- [Dated control validation results](docs/control_validation.md)
- [Recorded environment versions](environment-locks/README.md)

The active package is `ros2/swarm_mavsdk_ros2`; its name does not imply a
runtime MAVSDK dependency. Root-level MAVSDK tests, `run_drone_swarm.bash`,
`scripts/start_swarm_ros2.sh`, and the other ROS packages are older workflows.
Use the shared Python launcher for the process documented here. The single-drone
shell wrapper is supported, but defaults to the blank world and system ROS.
