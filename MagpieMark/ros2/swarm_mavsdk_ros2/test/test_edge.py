import json
import pytest
import queue
import threading
from types import SimpleNamespace
from unittest.mock import Mock, patch

from swarm_mavsdk_ros2.edge import EdgeLeader, EdgeCommandClient
from swarm_mavsdk_ros2.behavior import Swarm


def test_edge_swarm_keeps_member_commands_and_topic_api():
    swarm = Swarm(3, architecture=f'edge:drone_{0}')
    swarm.controller = Mock()
    swarm.forward(1)
    swarm.drone(0).land()
    swarm.publish_topic('command/yaw_rad', 'std_msgs/msg/Float64', {'data': 0.5})
    swarm.controller.command_all.assert_called_once_with('move_forward', 1)
    swarm.controller.command.assert_called_once_with(0, 'land')
    swarm.controller.publish_topic.assert_called_once_with(
        'command/yaw_rad', 'std_msgs/msg/Float64', {'data': 0.5}, drone_ids=None,
    )


def test_leader_executes_request_and_returns_correlated_result():
    leader = object.__new__(EdgeLeader)
    leader.drone = SimpleNamespace(drone_id=0, swarm_size=3)
    leader.requests = queue.Queue()
    leader.requests.put({'id': 'request-1', 'method': 'command_all', 'args': ['land']})
    leader.stopping = threading.Event()
    leader.results = Mock()
    leader.results.publish.side_effect = lambda message: leader.stopping.set()
    controller = Mock()
    controller.command_all.return_value = {0: 'landing', 1: 'landing', 2: 'landing'}
    with patch('swarm_mavsdk_ros2.edge.SwarmController', return_value=controller) as factory:
        with patch('swarm_mavsdk_ros2.edge.rclpy.ok', return_value=True), \
                patch('swarm_mavsdk_ros2.edge.rclpy.executors.SingleThreadedExecutor'):
            leader.run()
    factory.assert_called_once_with(3, namespace='/drone_0', use_global_arguments=False)
    controller.command_all.assert_called_once_with('land')
    result = json.loads(leader.results.publish.call_args.args[0].data)
    assert result['id'] == 'request-1'
    assert set(result['result']) == {'0', '1', '2'}
    controller.destroy_node.assert_called_once()


def test_recorder_subscription_does_not_release_command_before_leader():
    client = object.__new__(EdgeCommandClient)
    client.publisher = Mock(topic_name='/drone_0/swarm/request')
    client.publisher.get_subscription_count.return_value = 1
    leader = SimpleNamespace(node_namespace='/drone_0', node_name='drone')
    recorder = SimpleNamespace(node_namespace='/', node_name='topic_recorder')
    client.get_publishers_info_by_topic = Mock(return_value=[leader])
    client.get_subscriptions_info_by_topic = Mock(return_value=[recorder])
    assert not client.leader_is_ready()
    client.get_subscriptions_info_by_topic.return_value = [recorder, leader]
    assert client.leader_is_ready()


def test_leader_handles_ros_shutdown_while_idle():
    leader = object.__new__(EdgeLeader)
    leader.drone = SimpleNamespace(drone_id=0, swarm_size=3)
    leader.requests = queue.Queue()
    leader.stopping = threading.Event()
    controller = Mock()
    import rclpy
    with patch('swarm_mavsdk_ros2.edge.SwarmController', return_value=controller), \
            patch('swarm_mavsdk_ros2.edge.rclpy.ok', return_value=True), \
            patch('swarm_mavsdk_ros2.edge.rclpy.executors.SingleThreadedExecutor'), \
            patch('swarm_mavsdk_ros2.edge.rclpy.spin_once',
                  side_effect=rclpy.executors.ExternalShutdownException):
        leader.run()
    controller.destroy_node.assert_called_once()


@pytest.mark.parametrize('architecture', ['central', 'edge:drone_0'])
def test_edge_ros_commands_and_topics_reach_leader_and_peers(architecture):
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from std_msgs.msg import Float64
    from std_srvs.srv import Trigger
    from swarm_mavsdk_ros2.edge import EdgeCommandClient

    rclpy.init()
    members = [Node('member', namespace=f'/drone_{i}') for i in range(3)]
    members[0].drone_id = 0
    members[0].swarm_size = 3
    commands = []
    topics = []
    targets = []
    from geometry_msgs.msg import PoseStamped, TwistStamped
    executor = SingleThreadedExecutor()
    def landed(drone_id, request, response):
        commands.append(drone_id)
        response.success = True
        response.message = 'landing'
        return response
    for i, member in enumerate(members):
        member.create_service(Trigger, 'stop', lambda req, res: res)
        member.create_service(Trigger, 'land', lambda req, res, i=i: landed(i, req, res))
        member.create_subscription(Float64, 'custom_topic',
                                   lambda msg, i=i: topics.append((i, msg.data)), 10)
        member.create_subscription(PoseStamped, 'command/position_ned',
            lambda msg, i=i: targets.append((i, 'position', msg.pose.position.x,
                                             msg.pose.position.y, msg.pose.position.z)), 10)
        member.create_subscription(TwistStamped, 'command/velocity_ned',
            lambda msg, i=i: targets.append((i, 'velocity', msg.twist.linear.x,
                                             msg.twist.linear.y, msg.twist.linear.z)), 10)
        executor.add_node(member)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    # Reproduce recorder-first discovery: the result publisher is visible
    # before the actual request subscription, while the recorder is matched.
    from std_msgs.msg import String
    recorder = Node('topic_recorder')
    recorder.create_subscription(String, '/drone_0/swarm/request', lambda msg: None, 10)
    result_publisher = members[0].create_publisher(String, '/drone_0/swarm/results', 10)
    client = EdgeCommandClient(0)
    import time
    deadline = time.monotonic() + 3
    while (client.publisher.get_subscription_count() == 0 or
           client.count_publishers('/drone_0/swarm/results') == 0):
        assert time.monotonic() < deadline
        rclpy.spin_once(client, timeout_sec=0.05)
    assert not client.leader_is_ready()
    leader = EdgeLeader(members[0])
    if architecture == 'central':
        client.destroy_node()
        from swarm_mavsdk_ros2.swarm_controller import SwarmController
        client = SwarmController(3)
    try:
        result = client.command_all('land')
        assert {int(i) for i in result} == {0, 1, 2}
        assert sorted(commands) == [0, 1, 2]
        assert client.publish_topic('custom_topic', 'std_msgs/msg/Float64', {'data': 0.5}) == [0, 1, 2]
        import time
        deadline = time.monotonic() + 3
        while len(topics) < 3 and time.monotonic() < deadline:
            rclpy.spin_once(client, timeout_sec=0.05)
        assert sorted(topics) == [(0, 0.5), (1, 0.5), (2, 0.5)]
        client.command_all('move_to', 1., 2., -3.)
        client.command_all('set_velocity', .4, -.2, 0.)
        deadline = time.monotonic() + 3
        while len(targets) < 6 and time.monotonic() < deadline:
            time.sleep(.01)
        assert sorted(targets) == sorted([
            (i, mode, *values) for i in range(3)
            for mode, values in [('position', (1., 2., -3.)), ('velocity', (.4, -.2, 0.))]
        ])
    finally:
        leader.close()
        client.destroy_node()
        executor.shutdown()
        thread.join()
        for member in members:
            member.destroy_node()
        recorder.destroy_node()
        rclpy.shutdown()
