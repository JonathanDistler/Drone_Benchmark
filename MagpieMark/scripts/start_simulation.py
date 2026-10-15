#!/usr/bin/env python3
# Own a PX4/Gazebo swarm lifecycle, using system ROS or activated RoboStack

import argparse
import errno
import math
import os
from pathlib import Path
import shutil
import signal
import socket
import stat
import subprocess
import sys
import time


PROJECT = Path(__file__).resolve().parents[1]

# Existing native Micro XRCE-DDS Agent installation
XRCE_WS = Path.home() / "px4_ros_uxrce_dds_ws"

XRCE_AGENT_BIN = (
    XRCE_WS
    / "install"
    / "microxrcedds_agent"
    / "bin"
    / "MicroXRCEAgent"
)

XRCE_AGENT_LIB = (
    XRCE_WS
    / "install"
    / "microxrcedds_agent"
    / "lib"
)

ROS_JAZZY_LIB = Path("/opt/ros/jazzy/lib")


def source_environment(environment, *paths):
    # Source vendor environment hooks and return the resulting environment
    #
    # Bash is used only to read environment hooks, not to launch services
    result = subprocess.run(
        [
            "/bin/bash",
            "--noprofile",
            "--norc",
            "-c",
            'for file in "$@"; do source "$file" >&2 || exit; done; env -0',
            "environment",
            *map(str, paths),
        ],
        env=environment,
        check=True,
        stdout=subprocess.PIPE,
    )

    return dict(
        item.split("=", 1)
        for item in result.stdout.decode().split("\0")
        if item
    )


def system_environment():
    # Create a clean native Ubuntu environment
    #
    # Conda/ROS overlay variables are removed so PX4, Gazebo, and other native
    # programs do not accidentally load libraries from the Conda RoboStack
    # environment
    environment = os.environ.copy()

    for key in list(environment):
        if key.startswith(
            (
                "CONDA",
                "ROS_",
                "AMENT_",
                "COLCON_",
                "CMAKE_PREFIX",
                "PX4_",
            )
        ) or key in (
            "PYTHONPATH",
            "PYTHONHOME",
            "LD_LIBRARY_PATH",
            "VIRTUAL_ENV",
            "PX4_GZ_STANDALONE",
            "RMW_IMPLEMENTATION",
            "QT_PLUGIN_PATH",
            "QT_QPA_PLATFORM_PLUGIN_PATH",
        ):
            environment.pop(key, None)

    environment["PATH"] = (
        "/usr/local/sbin:"
        "/usr/local/bin:"
        "/usr/sbin:"
        "/usr/bin:"
        "/sbin:"
        "/bin"
    )

    return environment


def agent_environment():
    # Create the native environment used specifically by MicroXRCEAgent
    #
    # The agent was built in:
    #
    # ~/px4_ros_uxrce_dds_ws
    #
    # and requires both its own shared libraries and the Fast DDS/Fast CDR
    # libraries installed with ROS 2 Jazzy
    #
    # These paths are limited only to the MicroXRCEAgent process
    environment = system_environment()

    # Make the existing MicroXRCEAgent executable visible
    environment["PATH"] = (
        str(XRCE_AGENT_BIN.parent)
        + ":"
        + environment["PATH"]
    )

    # These are the exact library paths that allowed the existing agent to run
    environment["LD_LIBRARY_PATH"] = ":".join(
        [
            str(XRCE_AGENT_LIB),
            str(ROS_JAZZY_LIB),
        ]
    )

    return environment


def find_microxrce_agent():
    # Locate the Micro XRCE-DDS Agent
    #
    # Prefer the known workspace installation. Fall back to a native PATH lookup
    if XRCE_AGENT_BIN.is_file():
        return str(XRCE_AGENT_BIN)

    environment = agent_environment()

    found = shutil.which(
        "MicroXRCEAgent",
        path=environment["PATH"],
    )

    if found is not None:
        return found

    raise RuntimeError(
        "MicroXRCEAgent must be installed on the system PATH or at:\n"
        f"  {XRCE_AGENT_BIN}"
    )


def system_mavproxy_path(override=None):
    # Prefer a portable project venv; retain the existing host installation
    if override is not None:
        return str(Path(override).expanduser().resolve())
    local = PROJECT / ".venv/bin/mavproxy.py"
    if local.is_file():
        return str(local)
    return str(Path.home() / "mavlink_venv/bin/mavproxy.py")


class Processes:
    # Own process groups, including helpers spawned by ROS and Gazebo

    def __init__(self, logs):
        self.logs = logs
        self.children = []

    def start(self, name, command, environment, cwd=PROJECT):
        path = self.logs / f"{name}.log"
        with path.open("wb") as log:
            process = subprocess.Popen(command, env=environment, cwd=cwd,
                                       stdout=log, stderr=subprocess.STDOUT,
                                       stdin=subprocess.DEVNULL, start_new_session=True)
        self.children.append((name, process))
        print(f"Started {name}; log: {path}", flush=True)
        return process

    def check(self):
        for name, process in self.children:
            if process.poll() is not None:
                raise RuntimeError(f"{name} exited ({process.returncode}); inspect {self.logs / (name + '.log')}")

    def wait_for(self, predicate, description, timeout=90):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            ready = predicate()
            self.check()
            if ready:
                return
            time.sleep(0.5)
        raise RuntimeError(f"Timed out waiting for {description}; inspect {self.logs}")

    def close(self):
        # Signal groups even when their leader has exited: descendants may remain
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
            for name, process in reversed(self.children):
                try:
                    if name == "ros" and sig == signal.SIGINT:
                        # ROS launch forwards SIGINT to its nodes. Signalling
                        # its group too would interrupt node cleanup twice
                        if process.poll() is None:
                            os.kill(process.pid, sig)
                    else:
                        os.killpg(process.pid, sig)
                except ProcessLookupError:
                    pass
            deadline = time.monotonic() + 3
            for _, process in self.children:
                try:
                    process.wait(timeout=max(0.01, deadline - time.monotonic()))
                except subprocess.TimeoutExpired:
                    pass
            if sig != signal.SIGKILL:
                time.sleep(0.3)


def run(command, environment, cwd=PROJECT, **kwargs):
    return subprocess.run(list(map(str, command)), env=environment, cwd=cwd,
                          check=True, **kwargs)


def clear_stale_px4_socket(instance, directory=Path("/tmp")):
    # Remove an abandoned daemon socket, preserving live or uncertain connections
    path = directory / f"px4-sock-{instance}"
    try:
        original = path.lstat()
    except FileNotFoundError:
        return
    if not stat.S_ISSOCK(original.st_mode):
        raise RuntimeError(f"PX4 socket path {path} is not a socket; inspect it before retrying")
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as probe:
        probe.settimeout(2)
        try:
            probe.connect(str(path))
        except OSError as exc:
            if exc.errno == errno.ENOENT:
                return
            if exc.errno != errno.ECONNREFUSED:
                raise RuntimeError(f"Cannot verify whether PX4 instance {instance} is inactive: {exc}") from exc
        else:
            raise RuntimeError(f"PX4 instance {instance} is already active; stop the previous session before retrying")
    try:
        current = path.lstat()
        if (current.st_dev, current.st_ino) != (original.st_dev, original.st_ino):
            raise RuntimeError(f"PX4 socket {path} changed during startup; retry")
        path.unlink()
    except FileNotFoundError:
        return
    print(f"Removed stale PX4 socket: {path}", flush=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description='Own a PX4/Gazebo swarm lifecycle, using system ROS or activated RoboStack')
    parser.add_argument("--ros", choices=("system", "conda"), default="system")
    parser.add_argument("--world", choices=("obstacles", "swarm", "blank"),
                        help="World (default: obstacles for one drone, swarm for multiple drones)")
    parser.add_argument("--architecture", default="central", help="central or edge:drone_<id>")
    parser.add_argument("--n", type=int, default=1)
    parser.add_argument("--spawn-height", type=float, default=0.24)
    parser.add_argument("--takeoff-height", type=float, default=2.0)
    parser.add_argument("--position-speed", type=float, default=0.4,
                        help="Position reference speed in m/s after takeoff.")
    parser.add_argument("--spacing", type=float, default=2.0)
    parser.add_argument("--px4-dir", type=Path, default=Path.home() / "PX4-Autopilot")
    parser.add_argument("--skip-build", action="store_true")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--tests", nargs="+", type=int, choices=(1, 2, 3),
                        help="Run single-drone obstacle tests after readiness (prepares takeoff automatically).")
    parser.add_argument("--test-mode", choices=("position", "velocity"), default="position")
    parser.add_argument("--mavproxy-bin", type=Path,
                        help="Override MAVProxy executable (system mode prefers .venv/bin/mavproxy.py)")
    args = parser.parse_args(argv)
    if not 1 <= args.n <= 250:
        parser.error("--n must be between 1 and 250")
    sys.path.insert(0, str(PROJECT / "ros2/swarm_mavsdk_ros2"))
    from swarm_mavsdk_ros2.architecture import leader_id
    try:
        leader_id(args.architecture, args.n)
    except ValueError as error:
        parser.error(str(error))
    if args.world is None:
        args.world = "swarm" if args.n > 1 else "obstacles"
    if args.tests and (args.n != 1 or args.world != "obstacles"):
        parser.error("--tests requires --n 1 --world obstacles")
    for name in ("spawn_height", "takeoff_height", "spacing", "position_speed"):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0:
            parser.error(f"--{name.replace('_', '-')} must be finite and positive")

    logs = PROJECT / "results/startup"
    logs.mkdir(parents=True, exist_ok=True)
    processes = Processes(logs)
    native = system_environment()
    # PX4 and ROS must agree on the DDS domain, including nonzero domains
    native["ROS_DOMAIN_ID"] = os.environ.get("ROS_DOMAIN_ID", "0")
    native["GZ_IP"] = os.environ.get("GZ_IP", "127.0.0.1")
    native["ROS_LOG_DIR"] = str(logs / "ros-internal")
    native = source_environment(native, PROJECT / "scripts/px4_vision_sitl_parameters.sh")
    px4_dir = args.px4_dir.resolve()
    build = px4_dir / "build/px4_sitl_default"
    binary = build / "bin/px4"
    world_name = {"obstacles": "drone_obstacle_course",
                  "swarm": "drone_obstacle_course_swarm", "blank": "blank"}[args.world]
    world = PROJECT / "worlds" / f"{world_name}.sdf"
    if not binary.is_file() or not world.is_file():
        raise RuntimeError(f"Missing PX4 executable {binary} or world {world}")
    agent = find_microxrce_agent()
    for i in range(args.n):
        clear_stale_px4_socket(i)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        probe.bind(("0.0.0.0", 8888))

    if args.ros == "system":
        ros = source_environment(native, "/opt/ros/jazzy/setup.bash")
        python = "/usr/bin/python3"
        proxy = system_mavproxy_path(args.mavproxy_bin)
    else:
        prefix = os.environ.get("CONDA_PREFIX")
        if not prefix or Path(sys.executable).resolve().parent != Path(prefix).resolve() / "bin":
            raise RuntimeError("Activate drone-ros-jazzy and run this launcher with its Python for --ros conda")
        ros = native.copy()
        ros["CONDA_PREFIX"] = prefix
        ros["PATH"] = f"{prefix}/bin:" + native["PATH"]
        ros = source_environment(ros, Path(prefix) / "setup.bash")
        python = str(Path(prefix) / "bin/python")
        proxy = shutil.which("mavproxy.py", path=ros["PATH"])
        if args.mavproxy_bin is not None:
            proxy = str(args.mavproxy_bin.expanduser().resolve())
    if not proxy or not Path(proxy).is_file():
        raise RuntimeError("MAVProxy unavailable; install the documented MAVLink dependencies")
    overlay = PROJECT / f"install-{args.ros}"
    if not args.skip_build:
        print(f"Building {args.ros} ROS overlay; log: {logs / 'build.log'}", flush=True)
        with (logs / "build.log").open("wb") as output:
            run(["colcon", "--log-base", f"log-{args.ros}", "build",
                 "--build-base", f"build-{args.ros}", "--install-base", overlay,
                 "--paths", PROJECT / "px4_msgs", PROJECT / "ros2/swarm_mavsdk_ros2",
                 "--symlink-install", "--cmake-args", f"-DPython3_EXECUTABLE={python}",
                 f"-DPYTHON_EXECUTABLE={python}"], ros, stdout=output, stderr=subprocess.STDOUT)
    ros = source_environment(ros, overlay / "local_setup.bash")
    native["GZ_SIM_RESOURCE_PATH"] = f"{px4_dir}/Tools/simulation/gz/models:{PROJECT}/models:{PROJECT}/worlds:{px4_dir}/Tools/simulation/gz/worlds"
    # Starting Gazebo ourselves must reproduce PX4's gz_env.sh sensor plugins
    native["GZ_SIM_SYSTEM_PLUGIN_PATH"] = str(build / "src/modules/simulation/gz_plugins")
    native["GZ_SIM_SERVER_CONFIG_PATH"] = str(px4_dir / "src/modules/simulation/gz_bridge/server.config")
    native["PX4_GZ_STANDALONE"] = "1"
    native["PX4_GZ_WORLD"] = world_name
    native["PX4_SYS_AUTOSTART"] = "4005"
    native["PX4_SIM_MODEL"] = "gz_x500_vision"

    def interrupt(signum, frame):
        raise KeyboardInterrupt

    previous = signal.signal(signal.SIGTERM, interrupt)
    owned_sockets = {}
    try:
        processes.start("agent", [agent, "udp4", "-p", "8888"], agent_environment())
        processes.start("gazebo", ["gz", "sim", "-r", *(["-s"] if args.headless else []), str(world)], native)
        for i in range(args.n):
            environment = native.copy()
            environment["PX4_GZ_MODEL_POSE"] = f"0,{i * args.spacing},{args.spawn_height},0,0,0"
            work = logs / f"px4-instance-{i}"
            work.mkdir(exist_ok=True)
            processes.start(f"px4-{i}", [str(binary), str(build / "etc"), "-s", str(build / "etc/init.d-posix/rcS"), "-d", "-i", str(i), "-w", str(work)], environment)
            log = logs / f"px4-{i}.log"
            def booted():
                socket_path = Path(f"/tmp/px4-sock-{i}")
                if socket_path.exists():
                    owned_sockets[socket_path] = socket_path.stat().st_ino
                return "Startup script returned successfully" in log.read_text(errors="replace")
            processes.wait_for(booted, f"PX4 {i} startup")
            with (logs / f"parameters-{i}.log").open("wb") as output:
                for key, value in native.items():
                    if key.startswith("PX4_PARAM_"):
                        for operation in ("set", "compare"):
                            run([build / "bin/px4-param", "--instance", i, operation, key[10:], value], native, timeout=5, stdout=output, stderr=subprocess.STDOUT)
                run([build / "bin/px4-mavlink", "--instance", i, "start", "-x", "-u", 25000+i, "-o", 24000+i, "-t", "127.0.0.1", "-m", "onboard", "-r", "100000"], native, timeout=5, stdout=output, stderr=subprocess.STDOUT)
            proxy_dir = logs / f"mavproxy-{i}"
            proxy_dir.mkdir(exist_ok=True)
            processes.start(f"mavproxy-{i}", [proxy, "--non-interactive", f"--master=udpin:127.0.0.1:{24000+i}", f"--target-system={i+1}", f"--state-basedir={proxy_dir}", f"--logfile={proxy_dir}/mav.tlog"], ros)
        readiness = processes.start("readiness", [python, str(PROJECT / "scripts/wait_for_sitl_ready.py"), str(args.n)], ros)
        print("Waiting for DDS telemetry, valid estimators, and GCS heartbeats.", flush=True)
        while readiness.poll() is None:
            for name, process in processes.children:
                if process is not readiness and process.poll() is not None:
                    raise RuntimeError(f"{name} exited; inspect {logs}")
            time.sleep(0.5)
        processes.children.remove(("readiness", readiness))
        if readiness.returncode:
            raise RuntimeError(f"DDS/estimator readiness failed; inspect {logs / 'readiness.log'}")
        processes.start("ros", ["ros2", "launch", "swarm_mavsdk_ros2", "swarm.launch.py", f"num_drones:={args.n}", f"architecture:={args.architecture}", f"takeoff_altitude:={args.takeoff_height}", f"position_speed_mps:={args.position_speed}", f"world_name:={world_name}", f"spawn_spacing:={args.spacing}", f"output_dir:={PROJECT}/results/ros2"], ros)
        interfaces = processes.start("interfaces", [python, str(PROJECT / "scripts/wait_for_sitl_ready.py"), str(args.n), "--interfaces"], ros)
        while interfaces.poll() is None:
            for name, process in processes.children:
                if process is not interfaces and process.poll() is not None:
                    raise RuntimeError(f"{name} exited; inspect {logs / (name + '.log')}")
            time.sleep(0.5)
        processes.children.remove(("interfaces", interfaces))
        if interfaces.returncode:
            raise RuntimeError(f"Drone interface readiness failed; inspect {logs / 'interfaces.log'} and {logs / 'ros.log'}")
        print(f"Ready: {args.n} drones in {args.world} world. Call startup, arm, takeoff explicitly. Ctrl-C stops this session.", flush=True)
        for test in args.tests or []:
            name = f"test-{test}"
            print(f"Running obstacle test {test} ({args.test_mode}); progress: {logs / (name + '.log')}", flush=True)
            process = processes.start(name, ["ros2", "run", "swarm_mavsdk_ros2", "drone_command",
                "test", str(test), "--drone", "0", "--mode", args.test_mode,
                "--speed", str(args.position_speed)], ros)
            while process.poll() is None:
                for child_name, child in processes.children:
                    if child is not process and child.poll() is not None:
                        raise RuntimeError(f"{child_name} exited; inspect {logs / (child_name + '.log')}")
                time.sleep(.5)
            processes.children.remove((name, process))
            if process.returncode:
                raise RuntimeError(f"Test {test} failed; inspect {logs / (name + '.log')}")
            print(f"Test {test} passed. CSVs: {PROJECT / 'results/tests'}", flush=True)
        if args.tests:
            print("Tests complete; drone holds its destination. Use drone_command land to land. Ctrl-C stops this session.", flush=True)
        while True:
            processes.check()
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("Stopping simulation processes.", flush=True)
    finally:
        processes.close()
        # PX4 can leave its UNIX socket behind on startup failure. Only remove
        # sockets observed in this session, after their process has stopped
        for socket_path, inode in owned_sockets.items():
            if socket_path.exists() and socket_path.stat().st_ino == inode:
                socket_path.unlink()
        signal.signal(signal.SIGTERM, previous)


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, OSError, subprocess.SubprocessError) as exc:
        print(f"Startup failed: {exc}", file=sys.stderr)
        sys.exit(1)
