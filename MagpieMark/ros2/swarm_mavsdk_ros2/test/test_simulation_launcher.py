# Launcher regression checks without starting the native simulator
import importlib.util
import errno
import socket
from pathlib import Path
from unittest.mock import Mock, patch

import pytest


path = Path(__file__).resolve().parents[3] / "scripts/start_simulation.py"
spec = importlib.util.spec_from_file_location("simulation_launcher", path)
launcher = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launcher)


def test_stale_socket_is_removed(tmp_path):
    path = tmp_path / "px4-sock-0"
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as server:
        server.bind(str(path))
    launcher.clear_stale_px4_socket(0, tmp_path)
    assert not path.exists()


def test_live_socket_is_preserved(tmp_path):
    path = tmp_path / "px4-sock-0"
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as server:
        server.bind(str(path))
        server.listen()
        with pytest.raises(RuntimeError, match="already active"):
            launcher.clear_stale_px4_socket(0, tmp_path)
    assert path.exists()


def test_missing_socket_is_harmless(tmp_path):
    launcher.clear_stale_px4_socket(0, tmp_path)


def test_regular_file_is_preserved(tmp_path):
    path = tmp_path / "px4-sock-0"
    path.write_text("keep")
    with pytest.raises(RuntimeError, match="not a socket"):
        launcher.clear_stale_px4_socket(0, tmp_path)
    assert path.read_text() == "keep"


def test_uncertain_socket_is_preserved(tmp_path):
    path = tmp_path / "px4-sock-0"
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as server:
        server.bind(str(path))
    with patch.object(launcher.socket, "socket") as probe:
        probe.return_value.__enter__.return_value.connect.side_effect = PermissionError(errno.EACCES, "denied")
        with pytest.raises(RuntimeError, match="Cannot verify"):
            launcher.clear_stale_px4_socket(0, tmp_path)
    assert path.exists()


def test_native_environment_excludes_conda_and_ros_libraries():
    with patch.dict(launcher.os.environ, {
        "CONDA_PREFIX": "/conda", "LD_LIBRARY_PATH": "/conda/lib",
        "PYTHONPATH": "/ros/python", "AMENT_PREFIX_PATH": "/overlay",
        "PX4_GZ_STANDALONE": "1", "DISPLAY": ":0",
    }, clear=True):
        environment = launcher.system_environment()
    assert environment["DISPLAY"] == ":0"
    assert all(key not in environment for key in (
        "CONDA_PREFIX", "LD_LIBRARY_PATH", "PYTHONPATH",
        "AMENT_PREFIX_PATH", "PX4_GZ_STANDALONE"))
    assert "/conda" not in environment["PATH"]


def test_agent_receives_its_own_native_library_paths():
    environment = launcher.agent_environment()
    assert str(launcher.XRCE_AGENT_LIB) in environment["LD_LIBRARY_PATH"]
    assert str(launcher.ROS_JAZZY_LIB) in environment["LD_LIBRARY_PATH"]


def test_system_mode_finds_mavproxy_in_relocated_project(tmp_path):
    executable = tmp_path / ".venv/bin/mavproxy.py"
    executable.parent.mkdir(parents=True)
    executable.touch()
    with patch.object(launcher, "PROJECT", tmp_path):
        assert launcher.system_mavproxy_path() == str(executable)


def test_explicit_mavproxy_path_overrides_project_venv(tmp_path):
    executable = tmp_path / "custom/bin/mavproxy.py"
    assert launcher.system_mavproxy_path(executable) == str(executable)


def test_process_failure_identifies_log(tmp_path):
    processes = launcher.Processes(tmp_path)
    process = Mock(returncode=7)
    process.poll.return_value = 7
    processes.children.append(("agent", process))
    with pytest.raises(RuntimeError, match="agent.log"):
        processes.check()


def test_cleanup_signals_groups_even_after_parent_exits(tmp_path):
    processes = launcher.Processes(tmp_path)
    process = Mock(pid=123)
    process.poll.return_value = 0
    processes.children.append(("agent", process))
    with patch.object(launcher.os, "killpg") as kill, patch.object(launcher.time, "sleep"):
        processes.close()
    assert [call.args for call in kill.call_args_list] == [
        (123, launcher.signal.SIGINT), (123, launcher.signal.SIGTERM),
        (123, launcher.signal.SIGKILL)]


def test_ros_launch_receives_one_interrupt_to_forward_to_nodes(tmp_path):
    processes = launcher.Processes(tmp_path)
    process = Mock(pid=123)
    process.poll.return_value = None
    processes.children.append(("ros", process))
    with patch.object(launcher.os, "kill") as parent, patch.object(launcher.os, "killpg") as group, patch.object(launcher.time, "sleep"):
        processes.close()
    parent.assert_called_once_with(123, launcher.signal.SIGINT)
    assert [call.args for call in group.call_args_list] == [
        (123, launcher.signal.SIGTERM), (123, launcher.signal.SIGKILL)]


@pytest.mark.parametrize("arguments", [["--n", "0"], ["--n", "251"],
                                        ["--spawn-height", "nan"],
                                        ["--takeoff-height", "-1"],
                                        ["--world", "blank", "--tests", "1"],
                                        ["--n", "3", "--tests", "1"]])
def test_invalid_configuration_fails_before_starting_processes(arguments):
    with pytest.raises(SystemExit) as error:
        launcher.main(arguments)
    assert error.value.code == 2
