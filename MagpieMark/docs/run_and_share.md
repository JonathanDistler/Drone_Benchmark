# Install, run, and share the project

See the [position and velocity control guide](flight_controls.md) for coordinates,
mode selection, speed limits, stop behavior, console commands, and Python/RL examples.


## Choose the environment

Run commands in a VS Code Bash terminal on Ubuntu 24.04 (or a matching Linux
installation). Opening this folder in VS Code does not install ROS or the
simulator. Replace `~/drone_obstacle_course` with your actual extracted path.

For a new machine, follow **Recipient: install native prerequisites**, unpack
the project, create `.venv`, then follow **Recipient: run, control, and test**.
For the existing Conda installation, use the next section. Keep system ROS and
RoboStack in separate terminals and source the matching overlay in Terminal 2:

| Mode | Terminal 1 launcher | Terminal 2 environment / overlay |
| --- | --- | --- |
| System | `/usr/bin/python3 scripts/start_simulation.py --ros system` | `/opt/ros/jazzy/setup.bash`, `install-system/local_setup.bash` |
| Conda | activated environment's `python scripts/start_simulation.py --ros conda` | same Conda environment, `install-conda/local_setup.bash` |

The Python launcher defaults to **one drone in the obstacle world**. With
`--n 3`, the default is the swarm world. Select `--world blank` explicitly for
manual practice. It starts recording and services, but opens no console.
Wait for `Ready: N drones` before sending commands from Terminal 2.
The build runs automatically; use `--skip-build` only after a successful build
and rebuild after source changes. Do not start an extra agent or simulator.

## Run on the current machine with Conda

In Terminal 1:

```bash
cd ~/drone_obstacle_course
source ~/miniconda3/etc/profile.d/conda.sh
conda activate drone-ros-jazzy
python scripts/start_simulation.py --ros conda --n 1 --world blank
```

Wait for `Ready: 1 drones`. The first build can take several minutes.
Use `--headless` to omit the Gazebo GUI. The launcher starts the agent, Gazebo,
PX4, MAVProxy, drone interfaces, fleet controller, and recorder.

In Terminal 2:

```bash
cd ~/drone_obstacle_course
source ~/miniconda3/etc/profile.d/conda.sh
conda activate drone-ros-jazzy
source install-conda/local_setup.bash
ros2 run swarm_mavsdk_ros2 control_console --drone 0
```

Enter these commands one at a time, waiting for each to finish:

```text
startup
arm
takeoff
forward 0.5
backward 0.5
right 0.5
left 0.5
up 0.5
down 0.5
land
```

Wait for touchdown, then enter `disarm` and `quit`. Press Ctrl-C in Terminal 1
to stop the simulation. `startup` prepares a hold stream; arming and takeoff
are explicit. Takeoff defaults to 2 m; spawn height defaults to 0.24 m.
Movement directions are fixed local NED axes, as explained in
[the movement guide](miniconda_setup.md).

To run three drones, stop the previous simulation and change Terminal 1 to:

```bash
python scripts/start_simulation.py --ros conda --n 3
```

In Terminal 2, activate Conda and source `install-conda/local_setup.bash` again.
Run each line separately:

```bash
ros2 run swarm_mavsdk_ros2 swarm_command startup --n 3
ros2 run swarm_mavsdk_ros2 swarm_command arm --n 3
ros2 run swarm_mavsdk_ros2 swarm_command takeoff --n 3
ros2 run swarm_mavsdk_ros2 swarm_command forward 0.5 --n 3
ros2 run swarm_mavsdk_ros2 swarm_command land --n 3
```

After all drones touch down:

```bash
ros2 run swarm_mavsdk_ros2 swarm_command disarm --n 3
```

Use `control_console --drone 1` or `--drone 2` to control an individual member.
Every member has its own PX4 commander and ROS interface; they share one DDS agent.

## Coordinated takeoff

Use `swarm_command takeoff --n 3` to dispatch the entire fleet together.
The controller discovers every takeoff service before sending any request,
then queues all requests without waiting between members. Each drone retains
its continuously warmed OFFBOARD stream instead of restarting its warmup.
Takeoff still waits until every drone reaches and settles at its configured
height before allowing the next command. Independent ROS timers and PX4
controllers can produce small differences in physical liftoff; this is not
an exact shared simulation-tick trigger.

Restart the launcher after a source change and allow its normal build (omit
`--skip-build`) so the installed nodes include the change.

## Make a ZIP for VS Code or another machine

Stop the simulation first. From a terminal, run:

```bash
cd ~/drone_obstacle_course
zip -r /tmp/drone_obstacle_course-source.zip . \
  -x '.git/*' '.venv/*' '.agents/*' '.codex/*' '.aws/*' \
     'build/*' 'build-*/*' 'install/*' 'install-*/*' \
     'log/*' 'log-*/*' 'results/*' 'test_results/*' 'exports/*' \
     '__pycache__/*' '*/__pycache__/*' '*.pyc' \
     '.pytest_cache/*' '*/.pytest_cache/*' '*.egg-info/*' \
     '*.log' '*.tlog' '*.tlog.raw' '*.ulg' '*.zip' '*.tar.gz'
unzip -t /tmp/drone_obstacle_course-source.zip
sha256sum /tmp/drone_obstacle_course-source.zip
```

Use a fresh output filename for each export: `zip` updates an existing archive
and can retain obsolete entries. Copy the finished ZIP out of `/tmp` to permanent
storage before restarting the machine. This archive contains the contents of
the project folder, so the recipient must extract into a new project directory.
Source, CAD, models, worlds, documentation, requirements, and environment records
are included; generated builds, environments, logs, results, and archives are excluded.
Keep any experimental files you need in a separate results archive.

A ZIP is a source snapshot, not a VS Code extension or a dependency installer.
It does not include `~/PX4-Autopilot` or system ROS, Gazebo, and agent installations.

The curated tar exporter remains available as an alternative:

```bash
/usr/bin/python3 scripts/export_project.py
sha256sum exports/drone_obstacle_course-source.tar.gz
```

It exports the active runtime sources and setup documents, and preserves existing
archives. Use `--output exports/another-name.tar.gz` for a subsequent export.
The tar archive contains a top-level `drone_obstacle_course` directory; it does
not include the older experimental packages or CAD files.

## Recipient: install native prerequisites

Target Ubuntu 24.04 Linux with its `/usr/bin/python3` (Python 3.12).
Use a clean terminal with Conda deactivated. Follow the official
[ROS Jazzy binary installation](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
and [Gazebo Harmonic installation](https://gazebosim.org/docs/harmonic/install_ubuntu/)
to configure their apt repositories. Then install:

```bash
sudo apt update
sudo apt install ros-jazzy-ros-base ros-jazzy-geometry-msgs \
  ros-jazzy-nav-msgs ros-jazzy-std-msgs ros-jazzy-std-srvs \
  ros-jazzy-rcl-interfaces ros-jazzy-rosidl-default-generators \
  ros-jazzy-rosidl-default-runtime python3-colcon-common-extensions \
  python3-venv python3-pip python3-numpy build-essential cmake git gz-harmonic
```

Retrieve PX4 at the recorded revision in a **new checkout**:

```bash
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
git checkout d6f12ad1c4f70ad3230afd7d86e971421e02fef4
git submodule update --init --recursive
bash Tools/setup/ubuntu.sh
```

Follow any reboot instructions from the installer, then build:

```bash
cd ~/PX4-Autopilot
make px4_sitl_default
```

PX4's [Ubuntu setup guide](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu)
describes the native toolchain setup. Check `gz sim --versions`: this project
uses Harmonic (gz-sim 8).

Build the agent from the v2.4.3 source recommended in PX4's
[ROS guide](https://docs.px4.io/main/en/ros2/user_guide), using a **new directory**:

```bash
cd ~
git clone --branch v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel 2
sudo cmake --install build
sudo ldconfig
command -v MicroXRCEAgent
```

This installs a native agent on PATH. The source tag matches the tested machine;
this standalone build can use different DDS libraries/build options. Keep the
recorded native versions in `environment-locks` for comparison and verify SITL
on the recipient's machine. The launcher starts the agent automatically.

## Recipient: unpack and create .venv

For the ZIP above, put it in your home directory, then extract into a new,
empty directory (choose another destination if one already exists):

```bash
mkdir ~/drone_obstacle_course
unzip ~/drone_obstacle_course-source.zip -d ~/drone_obstacle_course
```

For the curated tar archive instead:

```bash
cd ~
tar -xzf drone_obstacle_course-source.tar.gz
```

Open the extracted `drone_obstacle_course` folder in VS Code. Create the Python
environment from its terminal:

```bash
cd ~/drone_obstacle_course
/usr/bin/python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install -r requirements-venv.txt
python -m pip check
```

`--system-site-packages` exposes Ubuntu's Python packages to the venv.
System ROS supplies rclpy, message generators, and ROS libraries. The venv
supplies MAVProxy and additional Python/test dependencies. The launcher uses
system Python for ROS nodes and the project `.venv/bin/mavproxy.py` for MAVProxy;
it keeps PX4, Gazebo, and agent libraries outside the venv.

The existing machine's older `.venv` was created without system site packages.
For an existing project, inspect `.venv/pyvenv.cfg` and preserve an old environment
before creating the fresh environment described here.

Check the ROS/Python combination:

```bash
source /opt/ros/jazzy/setup.bash
python -c 'import sys, rclpy; print(sys.executable); print(rclpy.__file__)'
```

Python should be under the project `.venv`; rclpy should be under `/opt/ros/jazzy`.
The launcher also retains the older `~/mavlink_venv` fallback and accepts an
explicit `--mavproxy-bin /absolute/path/to/mavproxy.py` override.

## Recipient: run, control, and test

In Terminal 1:

```bash
cd ~/drone_obstacle_course
source .venv/bin/activate
/usr/bin/python3 scripts/start_simulation.py --ros system --n 1 --world blank
```

Wait for readiness. This builds `build-system`, `install-system`, and `log-system`
on the new machine. In Terminal 2:

```bash
cd ~/drone_obstacle_course
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
source install-system/local_setup.bash
ros2 run swarm_mavsdk_ros2 control_console --drone 0
```

Use the same console sequence above. For three drones, stop the launcher first,
restart it with `--n 3`, and use the swarm commands above with the system overlay.
Once this mode has built successfully, `--skip-build` avoids rebuilding.

With the system overlay sourced, run:

```bash
ROS_DOMAIN_ID=91 python -m pytest ros2/swarm_mavsdk_ros2/test -q
python scripts/swarm_validity_test.py --n 1 --fly
```

The flight test requires the matching one-drone launcher to be running and
commands startup, arming, flight, landing, and disarming. Change both counts to
3 to validate a swarm. Unit tests use a separate DDS domain from the simulation.
Logs are in `results/startup`; recordings are under `results/ros2`.

After successful recipient validation, freeze the venv's local pip packages:

```bash
python -m pip freeze --local > requirements-venv.lock.txt
/usr/bin/python3 scripts/export_project.py --output exports/drone_obstacle_course-venv-locked.tar.gz
```

The exporter includes that lockfile when present. A future recipient can install
from `requirements-venv.lock.txt` instead of `requirements-venv.txt`. Keep the
native installation instructions and native version records alongside it;
pip requirements cannot install ROS, Gazebo, PX4, or the DDS agent. The Conda
`pip-resolved.txt` inventory is not a requirements file for this system-venv setup.

To validate both control modes and all three swarm benchmarks in a fresh
three-drone `--world swarm --headless` simulation, wait for `Ready: 3 drones`,
source the matching ROS overlay, then run:

```bash
/usr/bin/python3 scripts/validate_flight_controls.py --n 3
```

The script commands startup, arming, takeoff, single-drone moves, and all three
swarm tests in position and velocity modes, then lands and disarms the fleet.
Each invocation writes a unique directory under `results/control_validation`.
The launcher’s topic recorder also saves the full fleet telemetry.

Add `--controls-only` for position and velocity destination/directional checks
without benchmarks; this also supports `--n 1`. For the edge fleet, launch with
`--architecture edge:drone_0` and pass the same argument to the validator.
See [recording coverage](swarm_validity.md#csv-results) and
[latest validation results](control_validation.md).

## Troubleshooting and shutdown

Inspect `results/startup/build.log` for builds, `agent.log` for DDS,
`gazebo.log` for simulator startup, `px4-0.log` for vehicle zero,
`mavproxy-0.log` for its heartbeat link, and `readiness.log`, `interfaces.log`,
and `ros.log` for startup and ROS interfaces. Startup log names are reused;
copy a failing session before launching again. Flight CSVs have unique run directories.

Stop the previous launcher before starting another. A live PX4 instance or an
agent already holding UDP 8888 prevents a fresh session. If ROS commands find
no services, check that the launcher printed readiness, the control terminal
sources the correct overlay, and both terminals use the same `ROS_DOMAIN_ID`
(default 0). Run isolated regression tests on a different domain, such as 91.

Land, wait for touchdown, disarm, then press Ctrl-C in the launcher terminal.
Ctrl-C stops its processes; it does not command a landing. Automatic obstacle
test success leaves the drone holding; a failed automatic test shuts down the
launcher. See [flight validation](swarm_validity.md) for checks to run after setup.
