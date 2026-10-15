# PX4 SITL ROS 2 command interface

This is the active package. `DroneNode` communicates with one PX4 vehicle;
`DroneCommandClient` and the public `Drone` / `Swarm` APIs expose flight commands.
Despite the package name, the active interface uses PX4 ROS messages, not MAVSDK.

Follow the [project quick start](../../ReadMe.md) for complete two-terminal
commands and [installation and sharing](../../docs/run_and_share.md) for prerequisites.
Launch from the project root:

```bash
/usr/bin/python3 scripts/start_simulation.py --ros system --n 3 --world swarm
```

Wait for `Ready: 3 drones`. In a second terminal, from the project root:

```bash
source /opt/ros/jazzy/setup.bash
source install-system/local_setup.bash
ros2 run swarm_mavsdk_ros2 swarm_command startup --n 3
ros2 run swarm_mavsdk_ros2 swarm_command arm --n 3
ros2 run swarm_mavsdk_ros2 swarm_command takeoff --n 3
ros2 run swarm_mavsdk_ros2 swarm_command forward 0.5 --n 3
ros2 run swarm_mavsdk_ros2 swarm_command land --n 3
ros2 run swarm_mavsdk_ros2 swarm_command disarm --n 3
```

Wait for each command and for touchdown before disarming. The launcher owns the
simulation processes; Ctrl-C stops them. It does not open an interactive console.
Use `control_console --drone 0` for one vehicle. For Conda, activate the same
environment in both terminals, launch with its `python --ros conda`, and source
`install-conda/local_setup.bash` in the control terminal. Do not mix ROS overlays.

## Interfaces and mapping

Drone IDs are zero indexed. Drone `i` has PX4 system ID `i+1`, public interfaces
under `/drone_i`, and PX4 namespace `/fmu` for zero or `/px4_i/fmu` otherwise.
`/drone_i/state` exposes odometry; `/drone_i/peer_states_ned` exposes ordered peer
poses (unavailable positions are NaN). Central fleet positions are published at
`/swarm/positions_ned`. Inspect the live session with `ros2 topic list -t` and
`ros2 service list -t`; available PX4 topics depend on firmware/message definitions.
The launcher spawns vehicle `i` at Gazebo pose `0, i*spacing, spawn_height`.
Default spacing is 2 m, spawn height 0.24 m, and takeoff target 2 m.

The central architecture starts a separate fleet controller. With
`--architecture edge:drone_0`, drone zero hosts fleet control; pass the same
architecture to fleet commands and `Swarm(...)`. Requests/results use
`/drone_0/swarm/request` and `/drone_0/swarm/results`; fleet positions use
`/drone_0/swarm/positions_ned`. Automatic leader replacement is not implemented.

The launcher starts one shared DDS agent on UDP 8888 and a MAVProxy heartbeat
connection for each drone: PX4 local UDP `25000+i`, MAVProxy receive UDP
`24000+i`, target system `i+1`. Logs are under `results/startup`.
Its vision SITL parameter profile is simulation-specific; physical integration
requires its own estimator, frame alignment, networking, and flight configuration.

## Control, tests, and recording

- [Position/velocity semantics, Python, and RL adapters](../../docs/flight_controls.md)
- [Obstacle tests, recorder outputs, regression tests, and flight validation](../../docs/swarm_validity.md)
- [Fleet formations, translation, randomization, partitioning, and benchmarks](../../docs/swarm_benchmarks.md)
- [Dated measured results](../../docs/control_validation.md)

`startup` prepares a hold stream; explicitly arm and take off afterward.
Direct position and velocity targets return after dispatch and remain active
until replaced or stopped. `stop` captures a position hold. Directional and
route commands have their own completion checks; wait before issuing the next
command. Position references advance at `--position-speed` (default 0.4 m/s);
measured motion depends on PX4 tracking. `--speed` on movement CLIs selects the
velocity-mode speed. Land, wait for touchdown, disarm, then stop the launcher.

After code changes, restart the launcher without `--skip-build`. It rebuilds
only this package and the bundled `px4_msgs` into the selected mode's overlay.
Older shell launchers and manual `install/setup.bash` instructions are separate
legacy workflows and are not required for the shared launcher.
