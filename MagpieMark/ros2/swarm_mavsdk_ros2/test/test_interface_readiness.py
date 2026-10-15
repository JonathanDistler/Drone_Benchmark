# The launcher must not report readiness with drone zero absent
import importlib.util
from pathlib import Path
from unittest.mock import Mock, patch

import pytest

path = Path(__file__).resolve().parents[3] / 'scripts/wait_for_sitl_ready.py'
spec = importlib.util.spec_from_file_location('sitl_readiness', path)
readiness = importlib.util.module_from_spec(spec)
spec.loader.exec_module(readiness)


@pytest.mark.parametrize('missing_state,missing_service', [(0, None), (None, 0), (None, None)])
def test_requires_state_and_services_from_drone_zero(missing_state, missing_service):
    node = Mock()
    callbacks = {}
    node.create_subscription.side_effect = lambda kind, topic, callback, qos: callbacks.update({topic: callback})
    node.create_client.side_effect = lambda kind, topic: Mock(
        service_is_ready=Mock(return_value=topic != f'/drone_{missing_service}/arm'))

    def spin(node, timeout_sec):
        for topic, callback in callbacks.items():
            if topic != f'/drone_{missing_state}/state':
                callback(Mock())

    clock = iter([0.0, 0.1] + [0.1] * 20 + [2.0] * 50)
    with patch.object(readiness.rclpy, 'spin_once', side_effect=spin), patch.object(
            readiness.time, 'monotonic', side_effect=lambda: next(clock)):
        if missing_state is None and missing_service is None:
            readiness.interfaces_ready(node, 3, 1.0)
        else:
            with pytest.raises(SystemExit, match=r'drones \[0\]'):
                readiness.interfaces_ready(node, 3, 1.0)
