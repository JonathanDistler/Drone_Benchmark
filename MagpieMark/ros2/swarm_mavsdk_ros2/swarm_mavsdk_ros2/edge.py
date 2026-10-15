# Run swarm commands inside the leader drone's process
import json
import queue
import threading
import time
import uuid

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .swarm_controller import SwarmController


METHODS = {"command", "command_all", "formation", "randomize", "split",
           "translate", "go_to", "run_test", "centroid", "publish_topic"}


class EdgeLeader:
    def __init__(self, drone):
        self.drone = drone
        self.requests = queue.Queue(maxsize=32)
        prefix = f"/drone_{drone.drone_id}/swarm"
        self.results = drone.create_publisher(String, f"{prefix}/results", 10)
        drone.create_subscription(String, f"{prefix}/request", self.receive, 10)
        self.stopping = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def receive(self, message):
        try:
            request = json.loads(message.data)
            if not isinstance(request, dict) or request.get("method") not in METHODS:
                raise ValueError("Unknown swarm method")
            self.requests.put_nowait(request)
        except (ValueError, TypeError, queue.Full) as error:
            self.drone.get_logger().error(str(error))

    def run(self):
        controller = SwarmController(
            self.drone.swarm_size, namespace=f"/drone_{self.drone.drone_id}",
            # Launch remaps this process to /drone_N/drone. The embedded
            # controller must retain its own name and parameter services.
            use_global_arguments=False,
        )
        executor = rclpy.executors.SingleThreadedExecutor()
        controller._command_executor = executor
        try:
            while rclpy.ok() and not self.stopping.is_set():
                try:
                    request = self.requests.get_nowait()
                except queue.Empty:
                    rclpy.spin_once(controller, executor=executor, timeout_sec=0.05)
                    continue
                response = {"id": request.get("id")}
                try:
                    response["result"] = getattr(controller, request["method"])(
                        *request.get("args", []), **request.get("kwargs", {})
                    )
                    encoded = json.dumps(response)
                except Exception as error:
                    encoded = json.dumps({"id": request.get("id"), "error": str(error)})
                self.results.publish(String(data=encoded))
        except rclpy.executors.ExternalShutdownException:
            pass
        finally:
            executor.shutdown()
            controller.destroy_node()

    def close(self):
        self.stopping.set()
        self.thread.join()


class EdgeCommandClient(Node):
    def __init__(self, drone_id):
        super().__init__("edge_command_client")
        prefix = f"/drone_{drone_id}/swarm"
        self.publisher = self.create_publisher(String, f"{prefix}/request", 10)
        self.responses = {}
        self._command_executor = rclpy.executors.SingleThreadedExecutor()
        self.create_subscription(String, f"{prefix}/results", self.receive, 10)

    def receive(self, message):
        response = json.loads(message.data)
        self.responses[response["id"]] = response

    def leader_is_ready(self):
        # The recorder also subscribes to requests. Its discovery must not
        # release a volatile command before the leader's subscription exists.
        request_topic = self.publisher.topic_name
        result_topic = request_topic.rsplit("/", 1)[0] + "/results"
        publishers = self.get_publishers_info_by_topic(result_topic)
        subscriptions = self.get_subscriptions_info_by_topic(request_topic)
        leaders = {(item.node_namespace, item.node_name) for item in publishers}
        return (self.publisher.get_subscription_count() > 0 and any(
            (item.node_namespace, item.node_name) in leaders
            for item in subscriptions
        ))

    def __getattr__(self, method):
        if method not in METHODS:
            raise AttributeError(method)
        def send(*args, **kwargs):
            deadline = time.monotonic() + 10
            while not self.leader_is_ready():
                if time.monotonic() >= deadline:
                    raise TimeoutError("Leader drone is unavailable")
                rclpy.spin_once(self, executor=self._command_executor, timeout_sec=0.05)
            request_id = uuid.uuid4().hex
            self.publisher.publish(String(data=json.dumps({
                "id": request_id, "method": method, "args": args, "kwargs": kwargs
            })))
            deadline = time.monotonic() + max(300, kwargs.get("timeout_s", 0) + 10)
            while request_id not in self.responses:
                if time.monotonic() >= deadline:
                    raise TimeoutError("Leader command timed out; it may still be running")
                rclpy.spin_once(self, executor=self._command_executor, timeout_sec=0.05)
            response = self.responses.pop(request_id)
            if "error" in response:
                raise RuntimeError(response["error"])
            return response.get("result")
        return send

    def destroy_node(self):
        self._command_executor.shutdown()
        return super().destroy_node()
