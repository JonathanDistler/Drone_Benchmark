"""Timestamped test sessions and recorder events through the public ROS graph."""
from datetime import datetime, timezone
from functools import wraps
import json
from pathlib import Path

from std_msgs.msg import String
from rclpy.node import Node
from .spinning import spin_for


def session_path(owner, output_dir):
    root = Path(output_dir).resolve()
    paths = getattr(owner, '_test_session_paths', None)
    if paths is None:
        paths = owner._test_session_paths = {}
    if root not in paths:
        paths[root] = root / (datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S_%f') + '.csv')
    return paths[root]


def recorded_test(function):
    @wraps(function)
    def run(owner, test, *args, **kwargs):
        node = getattr(owner, 'node', owner)
        if not isinstance(node, Node):
            return function(owner, test, *args, **kwargs)
        publisher = node.create_publisher(String, '/testing/test_event', 10)
        try:
            spin_for(node, .2)
            publisher.publish(String(data=json.dumps({'test': str(test), 'status': 'running',
                                                       'requests': kwargs}, default=str)))
            spin_for(node, .05)
            try:
                result = function(owner, test, *args, **kwargs)
            except BaseException as exc:
                publisher.publish(String(data=json.dumps({'test': str(test), 'status': 'failed', 'results': str(exc)})))
                spin_for(node, .1)
                raise
            publisher.publish(String(data=json.dumps({'test': str(test), 'status': 'completed', 'results': result})))
            spin_for(node, .1)
            return result
        finally:
            node.destroy_publisher(publisher)
    return run
