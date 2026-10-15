# Exercise CSV integrity and live ROS discovery without commanding a vehicle

import csv
import json
import math
import time
from unittest.mock import patch

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from swarm_mavsdk_ros2.topic_recorder import CsvRun, TopicRecorder


def rows(path):
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def test_full_payload_burst_and_unique_runs(tmp_path):
    run = CsvRun(tmp_path)
    second = CsvRun(tmp_path)
    assert run.path != second.path
    payload = {"text": 'commas, "quotes"\nUnicode: é', "nested": {"array": [1, 2, 3]}, "unknown": float("nan")}
    for i in range(10000):
        run.record("/test", "custom/msg/Test", payload, i)
    run.close()
    run.close()
    second.close()
    messages = rows(run.path / "topics.csv")
    assert len(messages) == 10000
    restored = json.loads(messages[-1]["data_json"])
    assert restored["text"] == payload["text"]
    assert restored["nested"] == payload["nested"]
    assert math.isnan(restored["unknown"])
    assert messages[-1]["received_ros_ns"] == "9999"


def test_positions_and_missing_peer_values(tmp_path):
    run = CsvRun(tmp_path)
    pose = {"position": {"x": 1., "y": 2., "z": -3.}}
    run.record("/drone_12/state", "nav_msgs/msg/Odometry", {"header": {"frame_id": "drone_12_ned"}, "pose": {"pose": pose}}, 123)
    missing = {"position": dict.fromkeys(("x", "y", "z"), float("nan"))}
    run.record("/swarm/positions_ned", "geometry_msgs/msg/PoseArray", {"poses": [pose, missing]}, 124)
    run.close()
    positions = rows(run.path / "positions.csv")
    assert [r["drone_id"] for r in positions] == ["12", "0", "1"]
    assert positions[0]["down_m"] == "-3.0"
    assert positions[0]["frame_id"] == "drone_12_ned"
    assert positions[2]["north_m"] == "nan"


def test_disk_errors_surface(tmp_path):
    run = CsvRun(tmp_path)
    with patch.object(run, "topics") as writer:
        writer.writerow.side_effect = OSError("disk full")
        with pytest.raises(OSError, match="disk full"):
            run.record("/test", "std_msgs/msg/String", {"data": "test"}, 0)
    run.close()
    blocked = tmp_path / "file"
    blocked.write_text("not a directory")
    with pytest.raises(FileExistsError):
        CsvRun(blocked)


def test_late_topics_best_effort_and_shutdown_flush(tmp_path):
    rclpy.init(args=["--ros-args", "-p", f"output_dir:={tmp_path}"])
    recorder = TopicRecorder()
    publisher = Node("recorder_test_publisher")
    executor = SingleThreadedExecutor()
    executor.add_node(recorder)
    executor.add_node(publisher)
    path = recorder.run.path
    try:
        recorder.discover()
        state_pub = publisher.create_publisher(Odometry, "/drone_7/state", qos_profile_sensor_data)
        text_pub = publisher.create_publisher(String, "/late_test_topic", 10)
        state = Odometry()
        state.pose.pose.position.z = -2.5
        text = String(data='hello, "world"\nnext line')
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            state_pub.publish(state)
            text_pub.publish(text)
            executor.spin_once(timeout_sec=0.02)
            recorder.run.flush()
            messages = rows(path / "topics.csv")
            if any(r["topic"] == "/late_test_topic" for r in messages) and rows(path / "positions.csv"):
                break
        else:
            pytest.fail("Late topics were not recorded within 10 seconds")
        count = len(recorder.recorded)
        recorder.discover()
        assert len(recorder.recorded) == count
        recorder.run.record("/final", "std_msgs/msg/String", {"data": "last"}, 1)
    finally:
        executor.shutdown()
        publisher.destroy_node()
        recorder.destroy_node()
        rclpy.shutdown()
    assert any(json.loads(r["data_json"]) == {"data": text.data} for r in rows(path / "topics.csv"))
    assert rows(path / "topics.csv")[-1]["topic"] == "/final"
    assert rows(path / "positions.csv")[0]["down_m"] == "-2.5"


def test_unknown_message_type_retries_without_warning_flood(tmp_path):
    rclpy.init(args=["--ros-args", "-p", f"output_dir:={tmp_path}"])
    recorder = TopicRecorder()
    try:
        with patch.object(recorder, "get_topic_names_and_types", return_value=[("/unknown", ["missing/msg/Unknown"]) ]):
            recorder.discover()
            recorder.discover()
            assert len(rows(recorder.run.path / "inventory.csv")) == 1
            with patch("swarm_mavsdk_ros2.topic_recorder.get_message", return_value=String):
                recorder.discover()
            assert ("/unknown", "missing/msg/Unknown") in recorder.recorded
            assert rows(recorder.run.path / "inventory.csv")[-1]["status"] == "subscribed"
    finally:
        recorder.destroy_node()
        rclpy.shutdown()


def test_physics_timeline_combines_tests_and_late_columns(tmp_path):
    run = CsvRun(tmp_path)
    run.record('/drone_0/velocity_ned', 'geometry_msgs/msg/Vector3', {'x': .4}, 10)
    run.physics_step(100)  # No test has started yet.
    run.begin_test('test 1')
    run.physics_step(101)
    run.physics_step(101)  # Duplicate clock is not a second physics step.
    run.record('/px4_1/fmu/out/vehicle_attitude', 'px4_msgs/msg/VehicleAttitude', {'q': [1,0,0,0]}, 11)
    run.begin_test('test 1', 'completed')
    run.physics_step(102)
    path = run.timeline_path
    run.begin_test('test 2')
    run.physics_step(103)
    run.close()
    assert run.timeline_path == path
    samples = rows(path)
    assert len(samples) == 3
    assert [r['test'] for r in samples] == ['test 1', 'test 1', 'test 2']
    assert samples[0]['drone_1.px4.out.vehicle_attitude'] == ''
    assert json.loads(samples[1]['drone_1.px4.out.vehicle_attitude'])['q'] == [1,0,0,0]
    assert json.loads(samples[2]['drone_0.velocity_ned'])['x'] == .4


def test_export_snapshot_allows_physics_and_new_columns_to_continue(tmp_path):
    import threading
    from unittest.mock import Mock
    run = CsvRun(tmp_path)
    run.begin_test('test 1')
    run.physics_step(1)
    entered, release, appended = threading.Event(), threading.Event(), threading.Event()
    writer = Mock()
    def slow_row(row):
        entered.set()
        assert release.wait(5)
    writer.writerow.side_effect = slow_row
    def append():
        run.record('/late', 'std_msgs/msg/String', {'data': 'new'}, 2)
        run.physics_step(2)
        appended.set()
    with patch('swarm_mavsdk_ros2.topic_recorder.csv.DictWriter', return_value=writer):
        export = threading.Thread(target=run.export_timeline)
        export.start()
        assert entered.wait(2)
        physics = threading.Thread(target=append)
        physics.start()
        try:
            assert appended.wait(2), 'CSV export blocked physics recording'
        finally:
            release.set()
            export.join(5)
            physics.join(5)
    run.close()
    samples = rows(run.timeline_path)
    assert len(samples) == 2
    assert samples[0]['late'] == ''
    assert json.loads(samples[1]['late']) == {'data': 'new'}
    assert run.spool_path.exists()  # The durable spool survives an interrupted process.


def test_flat_columns_cover_targets_flags_battery_and_shrinking_arrays(tmp_path):
    run = CsvRun(tmp_path)
    run.begin_test('session', 'idle')
    run.record('/drone_0/control_state', 'std_msgs/msg/String', {'data': json.dumps({
        'mode': 'velocity', 'architecture': 'edge:drone_0',
        'velocity_target_ned_mps': [.4, 0, 0], 'started': True})}, 10)
    run.record('/fmu/out/battery_status_v1', 'px4_msgs/msg/BatteryStatus',
               {'voltage_v': 16., 'connected': True, 'voltage_cell_v': [4., 4.]}, 11)
    run.physics_step(1)
    run.record('/fmu/out/battery_status_v1', 'px4_msgs/msg/BatteryStatus',
               {'voltage_v': 15., 'connected': True, 'voltage_cell_v': [3.9]}, 12)
    run.physics_step(2)
    run.close()
    samples = rows(run.timeline_path)
    assert samples[0]['drone_0.control_state.mode'] == 'velocity'
    assert samples[0]['drone_0.control_state.architecture'] == 'edge:drone_0'
    assert samples[0]['drone_0.control_state.velocity_target_ned_mps.0'] == '0.4'
    assert samples[0]['drone_0.px4.out.battery_status_v1.voltage_v'] == '16.0'
    assert samples[0]['drone_0.px4.out.battery_status_v1.connected'] == 'True'
    assert samples[1]['drone_0.px4.out.battery_status_v1.voltage_cell_v.1'] == ''
