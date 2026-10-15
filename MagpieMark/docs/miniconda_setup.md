# Movement, startup, and Miniconda

See the [position and velocity control guide](flight_controls.md) for coordinates,
mode selection, speed limits, stop behavior, console commands, and Python/RL examples.

This guide targets this project's Ubuntu 24.04 Linux installation. The new
`scripts/start_simulation.py` starts one or more drones without GNOME Terminal
or a shell launcher. It owns the agent, PX4, MAVProxy, and ROS launch processes,
writes logs to `results/startup`, and stops its processes on Ctrl-C. It still
reads vendor `.bash` environment hooks internally; changing a script's filename
alone would not replace Bash orchestration.

## How movement is defined

The active implementation is `ros2/swarm_mavsdk_ros2/swarm_mavsdk_ros2/commands.py`
and `drone_node.py`, not the old root-level MAVSDK tests.

| Command | Change in PX4 local NED coordinates |
| --- | --- |
| `forward(d)` | north += abs(d) |
| `backward(d)` | north -= abs(d) |
| `right(d)` | east += abs(d) |
| `left(d)` | east -= abs(d) |
| `up(d)` / `height_rise(d)` | down -= abs(d) |
| `down(d)` / `height_drop(d)` | down += abs(d) |

Distances are metres, velocity targets are metres/second, yaw is radians.
Directions use fixed local axes, so yaw does not change the meaning of forward.
`move_to(north, east, down)` specifies an absolute local destination;
`forward(1)` specifies an offset from the latest measured position.

Relative position movement waits for fresh telemetry and a settled destination
before returning. The node advances position references at `position_speed_mps`
(default 0.4 m/s; launcher flag `--position-speed`). PX4 tracking determines
measured speed. The movement CLI `--speed` selects velocity-mode speed. Velocity mode streams a requested velocity until measured Euclidean
travel reaches the requested distance, then sends zero velocity. Use `stop`
to capture a position hold. This is a distance
condition, not a guarantee of exact final position. Swarm coordination adds
its own completion checks; inspect `swarm_controller.py` for those.

The ROS messages here carry NED values even though many ROS applications use
ENU. Convert explicitly when integrating those applications. Gazebo spawn
poses use the simulator's world coordinates; do not treat its spawn Z as PX4
local down or assume the two frames share an origin.

## Ground height and startup

The launchers default to a spawn height of 0.24 m, separately from the
2 m takeoff target. Override it with `--spawn-height` in the Python launcher
or `SPAWN_HEIGHT` in the shell swarm launchers. The installed x500 base model's
lowest collision box is centered at -0.2195 m with a height of 0.015 m,
putting its lower face at -0.227 m. The 0.24 m default provides 0.013 m
clearance when the spawn pose replaces the model pose. Live Gazebo settling
has not been verified; other models may require a different spawn height.

Both `Drone` and `Swarm` expose `move_to(north, east, down)`,
`set_velocity(north, east, down)`, `set_position_control()`, and
`set_velocity_control()`. Position units are metres and velocity units are
metres per second. A swarm position target applies in each vehicle's own
local NED frame; use `swarm.drone(i).move_to(...)` for distinct targets.
Velocity targets remain active until replaced or stopped. Fleet lifecycle
services are discovered for every member before dispatch, and every request
is sent before waiting for responses. Relative movement and takeoff wait
for the whole fleet to settle before returning.

`startup()` captures the current measured local position and streams a hold
target. It does not arm or climb. Then explicitly call `arm()` and `takeoff()`.
The current takeoff target is `down = -takeoff_height` relative to the PX4 local
origin; it is not a terrain-following altitude or an offset added to spawn Z.

Try the Python launcher with the existing system installation first:

```bash
cd ~/drone_obstacle_course
/usr/bin/python3 scripts/start_simulation.py --ros system --n 1
```

In another terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/drone_obstacle_course/install-system/local_setup.bash
ros2 run swarm_mavsdk_ros2 control_console --drone 0
```

Run `startup`, `arm`, `takeoff`, then movement commands and `land` in the
console. The launcher's Ctrl-C terminates simulation processes; use `land`
before ending a flight if you want a complete landing record.

## Version choices

| Component | Choice / local evidence |
| --- | --- |
| OS | Ubuntu 24.04; this machine reports 24.04.3 |
| ROS 2 | Jazzy, either system packages or RoboStack, in separate processes/builds |
| Python | Conda 3.12; existing system environment reports 3.12.3 |
| Gazebo | Harmonic (gz-sim 8); installed CLI is 8.11.0 |
| PX4 | Keep existing v1.17.0, commit `d6f12ad1c4f70ad3230afd7d86e971421e02fef4` |
| XRCE-DDS agent | PX4 documents v2.4.3; installed agent version was not identified |
| px4_msgs | Local package says 2.0.1; message definitions must match PX4 firmware |
| MAVProxy | 1.8.74, as installed in the existing MAVLink venv |
| pymavlink | 2.4.49, as installed in the existing MAVLink venv |
| Build tools / ROS patch versions | Resolve together using Conda, then export exact package builds |

PX4 documents Jazzy + Harmonic on Ubuntu 24.04 and the agent build in its
[ROS 2 guide](https://docs.px4.io/main/en/ros2/user_guide).
Follow the matching firmware documentation before changing the agent major
version. A package.xml version alone does not establish px4_msgs compatibility;
compare the message definitions against the PX4 checkout, especially those
listed in `src/modules/uxrce_dds_client/dds_topics.yaml`. Preserve the current
source tree until that match is verified.

## 1. Install Miniconda

For a new machine without Conda, download the Linux installer
for your CPU architecture from the
[official Miniconda page](https://www.anaconda.com/docs/getting-started/miniconda/install),
verify the published SHA256, then run it. For Linux x86_64, assuming the
installer is downloaded to Downloads:

```bash
bash ~/Downloads/Miniconda3-latest-Linux-x86_64.sh -b -p "$HOME/miniconda3"
source "$HOME/miniconda3/etc/profile.d/conda.sh"
conda config --set auto_activate_base false
```

The installer release is a bootstrap tool, not the project's Python version;
the environment file selects Python 3.12. Keep the downloaded installer and
its checksum if you need to reproduce the bootstrap exactly.

## 2. Create an isolated ROS environment

Do not source `/opt/ros/jazzy/setup.bash` or the existing `install/setup.bash`
in the Conda terminal. If `.bashrc` sources either automatically, start a
terminal without those lines. For a fresh environment from an existing shell:

```bash
env -u PYTHONPATH -u PYTHONHOME -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH \
    -u COLCON_PREFIX_PATH -u LD_LIBRARY_PATH -u ROS_DISTRO -u ROS_VERSION \
    -u ROS_PYTHON_VERSION -u RMW_IMPLEMENTATION -u VIRTUAL_ENV \
    PATH=/usr/local/bin:/usr/bin:/bin bash --noprofile --norc
source "$HOME/miniconda3/etc/profile.d/conda.sh"
cd ~/drone_obstacle_course
CONDA_CHANNEL_PRIORITY=strict conda env create --file environment.yml
conda activate drone-ros-jazzy
```

This installs RoboStack's ROS binaries built for Conda. It avoids mixing the
system `rclpy` with Conda Python, a combination the
[ROS Python guide](https://docs.ros.org/en/jazzy/How-To-Guides/Using-Python-Packages.html)
warns about. RoboStack also explicitly requires avoiding system ROS setup;
see its [Conda instructions](https://robostack.github.io/conda.html).

`environment.yml` includes the active ROS interface dependencies, message
generators, build tools, tests, and the MAVProxy GCS heartbeat dependency.
It is a starting specification, not an exact lockfile or a verified solve on
every machine. This installation's successful resolved setup is recorded in
[`environment-locks`](../environment-locks/README.md). ROS patch versions and native libraries must be solved together;
do not copy system `pip freeze` wholesale into Conda.

Keep `pytest<9`: this installation's Jazzy `launch_testing` plugin uses a
pytest collection hook removed in pytest 9. The validated Conda version is
pytest 8.4.2.

The active drone interface does not import MAVSDK despite its package name.
If you still use the legacy scripts, add the locally observed version:

```bash
python -m pip install 'mavsdk==3.15.3'
```

Gymnasium, PyTorch, computer vision, and analysis packages are optional;
the active control interface does not require them. Add them only for the
algorithm you actually run, and select PyTorch builds to match the machine's
CPU/GPU setup before locking the environment.

## 3. Check and build

```bash
python -c 'import sys, rclpy; print(sys.executable); print(rclpy.__file__)'
python -m pip check
ros2 pkg prefix rclpy
python scripts/start_simulation.py --ros conda --n 1
```

The Python executable and rclpy prefix should point inside the Conda
environment. The launcher builds into `build-conda`, `install-conda`, and
`log-conda` so existing system build artifacts are preserved. It explicitly
selects the Conda Python for CMake message generation.

PX4, Gazebo Harmonic, and the system XRCE agent stay native on Ubuntu; the
launcher removes Conda paths from their process environment. ROS nodes and
MAVProxy run in Conda. This migrates the ROS/Python workspace to Miniconda,
while retaining the existing simulator and firmware toolchain. Moving those
native components into Conda too would be a separate toolchain migration.

If startup fails, inspect `results/startup/agent.log`, `px4-0.log`,
`mavproxy-0.log`, and `ros.log`. Abandoned PX4 sockets are removed automatically;
live PX4 instances cause an early refusal, so stop the previous session before
retrying. A second agent on UDP
8888 is also a conflict; stop the previous session's agent first.

## 4. Control and validate

Open another clean terminal and activate the same Conda environment:

```bash
source "$HOME/miniconda3/etc/profile.d/conda.sh"
conda activate drone-ros-jazzy
cd ~/drone_obstacle_course
source install-conda/local_setup.bash
ros2 run swarm_mavsdk_ros2 control_console --drone 0
```

First validate one drone: startup, arm, takeoff, a small move, and land.
Observe the ground settling before arming. After touchdown, disarm. Then stop the launcher and start:

```bash
python scripts/start_simulation.py --ros conda --n 3
```

In the control terminal, use the swarm interface:

```bash
ros2 run swarm_mavsdk_ros2 swarm_command startup --n 3
ros2 run swarm_mavsdk_ros2 swarm_command arm --n 3
ros2 run swarm_mavsdk_ros2 swarm_command takeoff --n 3
ros2 run swarm_mavsdk_ros2 swarm_command forward 0.5 --n 3
ros2 run swarm_mavsdk_ros2 swarm_command land --n 3
```

After all vehicles touch down, run `ros2 run swarm_mavsdk_ros2 swarm_command disarm --n 3`.

Run the existing unit suite with the rebuilt overlay sourced:

```bash
ROS_DOMAIN_ID=91 python -m pytest ros2/swarm_mavsdk_ros2/test -q
```

See [swarm validity checks](swarm_validity.md) for measured takeoff and movement
validation. Only use `--skip-build` after a successful build in the same mode.

## 5. Freeze the successful setup

After dependency checks, unit tests, and a real SITL run pass:

```bash
mkdir -p environment-locks
conda list --explicit > environment-locks/conda-linux-64.txt
conda env export > environment-locks/environment-resolved.yml
python -m pip freeze > environment-locks/pip-resolved.txt
dpkg-query -W 'gz-*' 'libgz-*' > environment-locks/gazebo-packages.txt
git -C ~/PX4-Autopilot rev-parse HEAD > environment-locks/px4-commit.txt
sha256sum /usr/local/bin/MicroXRCEAgent > environment-locks/agent-sha256.txt
```

Adjust the agent path to `command -v MicroXRCEAgent`. Record its source tag and
build options if available. This workspace's `.git` and px4_msgs `.git` metadata
were unavailable during inspection; preserve the px4_msgs source alongside
the lockfiles or record its commit from the original repository.

The explicit Conda export records native package builds on the same platform;
it does not include pip packages. The resolved YAML includes pip requirements.
Keep both plus the source snapshots and native simulator versions. Exact
reproduction depends on all of these, not just the Python version.

Direct targets are also available from the command line:

```bash
ros2 run swarm_mavsdk_ros2 drone_command position --drone 0 --ned 1 0 -2
ros2 run swarm_mavsdk_ros2 swarm_command velocity --n 3 --ned 0.5 0 0
ros2 run swarm_mavsdk_ros2 swarm_command stop --n 3
```

Direct targets return after dispatch; use directional commands when the caller
must wait for the entire swarm to reach and settle at its destination.

## World selection and obstacle tests

Start either world with the shared launcher (the single-drone shell wrapper accepts the same flags):

```bash
/usr/bin/python3 scripts/start_simulation.py --world obstacles --n 1
# Or:
/usr/bin/python3 scripts/start_simulation.py --world blank --n 1
```

Open `ros2 run swarm_mavsdk_ros2 control_console --drone 0` in a terminal
with the ROS setup and install-system overlay sourced. Run `startup`, `arm`,
`takeoff`, then `test 1`, `test 2`, or `test 3`. One-shot commands also support
`ros2 run swarm_mavsdk_ros2 drone_command test 2`.

Python supports formatted strings:

```python
from swarm_mavsdk_ros2.behavior import Drone
with Drone(0) as drone:
    drone.startup()
    drone.arm()
    drone.takeoff()
    for number in (1, 2, 3):
        csv_path = drone.run_test(f"test {number}")
    drone.land()
    drone.disarm()
```

Tests use predefined routes for this static course: cone weaving, flag passage,
and the box opening. They do not detect arbitrary obstacles. The runner checks
that the obstacle world and all expected course entities are present in the live
Gazebo scene. It translates through the clear corridor at Gazebo X=-2 to the
selected start, waits for arrival and low speed, then starts the timer and CSV
sampling. All runs append to `results/tests/all_tests.csv` with run/test IDs,
waypoints, targets, position, velocity, error, cumulative distance, and final
completed/failed status. Transit, takeoff, and landing are excluded. Failed
navigation preserves recorded samples and commands a hold.

The launcher spawns at Gazebo X=0; test conversion uses Gazebo +X as local East,
+Y as local North, and +Z as negative Down. Test 3 follows the actual rotated
mesh opening center (Y=8.5584, Z=0.5464), rather than the stale SDF comment.
The dated control checks do not establish obstacle clearance; see
[validation scope](control_validation.md). Relative position
commands wait for arrival, so `forward 2` followed by `backward 2` returns near
the original position within the controller's arrival tolerance (0.10 m per leg).
