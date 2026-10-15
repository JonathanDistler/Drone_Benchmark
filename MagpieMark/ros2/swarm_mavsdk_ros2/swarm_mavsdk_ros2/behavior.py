# Context-managed Python commands for single drones and swarms

from __future__ import annotations

import rclpy
from rclpy.node import Node

from .commands import DroneCommandClient
from .swarm_rl import SwarmRlInterface
from .swarm_controller import SwarmController


class _RosContext:
    def __enter__(self):
        self._owns_rclpy = not rclpy.ok()
        if self._owns_rclpy:
            rclpy.init()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()


class Drone(_RosContext):
    # Python behavior interface for one drone, using IDs that start at zero

    def __init__(self, drone_id: int = 0, *, node: Node | None = None):
        if drone_id < 0:
            raise ValueError("drone_id must be non-negative.")
        self.drone_id = drone_id
        self._external_node = node
        self.node = node
        self.client = None
        self._owns_rclpy = False

    def __enter__(self):
        super().__enter__()
        if self.node is None:
            self.node = rclpy.create_node(f"drone_behavior_{self.drone_id}")
        self.client = DroneCommandClient(self.node, f"/drone_{self.drone_id}")
        return self

    def close(self):
        if self.node is not None and self._external_node is None:
            self.node.destroy_node()
        self.node = None
        self.client = None
        if self._owns_rclpy and rclpy.ok():
            rclpy.shutdown()
        self._owns_rclpy = False

    def _call(self, method: str, *args, **kwargs):
        if self.client is None:
            raise RuntimeError("Use Drone inside a 'with Drone(...)' block.")
        return getattr(self.client, method)(*args, **kwargs)

    def run_test(self, command, **kwargs): return self._call("run_test", command, **kwargs)

    def startup(self): return self._call("startup")
    def arm(self): return self._call("arm")
    def disarm(self): return self._call("disarm")
    def takeoff(self): return self._call("takeoff")
    def land(self): return self._call("land")
    def stop(self): return self._call("stop")
    def hold_position(self): return self._call("hold_position")
    def set_position_control(self): return self._call("set_position_control")
    def set_velocity_control(self): return self._call("set_velocity_control")
    def move_to(self, north: float, east: float, down: float):
        return self._call("move_to", north, east, down)
    def set_velocity(self, north: float, east: float, down: float):
        return self._call("set_velocity", north, east, down)

    def forward(self, distance_m=1.0, **kwargs): return self._call("move_forward", distance_m, **kwargs)
    def backward(self, distance_m=1.0, **kwargs): return self._call("move_backward", distance_m, **kwargs)
    def left(self, distance_m=1.0, **kwargs): return self._call("move_left", distance_m, **kwargs)
    def right(self, distance_m=1.0, **kwargs): return self._call("move_right", distance_m, **kwargs)
    def up(self, distance_m=1.0, **kwargs): return self._call("move_up", distance_m, **kwargs)
    def down(self, distance_m=1.0, **kwargs): return self._call("move_down", distance_m, **kwargs)
    height_rise = up
    height_drop = down
    def set_yaw(self, yaw_rad: float): return self._call("set_yaw", yaw_rad)
    def yaw_by(self, delta_rad: float): return self._call("yaw_by", delta_rad)
    def yaw(self, yaw_rad: float): return self.set_yaw(yaw_rad)


class Swarm(_RosContext):
    # Python behavior interface applying commands to every drone

    def __init__(self, drone_count: int, *, rl_max_speed_mps: float = 1.0,
                 architecture: str = "central"):
        if drone_count < 1:
            raise ValueError("drone_count must be at least one.")
        from .architecture import leader_id
        self.leader_id = leader_id(architecture, drone_count)
        self.architecture = architecture
        self.drone_count = drone_count
        if rl_max_speed_mps <= 0.0:
            raise ValueError("rl_max_speed_mps must be positive.")
        self.rl_max_speed_mps = rl_max_speed_mps
        self.controller = None
        self.rl = None
        self._owns_rclpy = False

    def __enter__(self):
        super().__enter__()
        if self.leader_id is None:
            self.controller = SwarmController(self.drone_count)
            self.rl = SwarmRlInterface(self.controller, self.rl_max_speed_mps)
        else:
            from .edge import EdgeCommandClient
            self.controller = EdgeCommandClient(self.leader_id)
        return self

    def close(self):
        if self.controller is not None:
            self.controller.destroy_node()
            self.controller = None
            self.rl = None
        if self._owns_rclpy and rclpy.ok():
            rclpy.shutdown()
        self._owns_rclpy = False

    def _call(self, method: str, *args, **kwargs):
        if self.controller is None:
            raise RuntimeError("Use Swarm inside a 'with Swarm(...)' block.")
        return self.controller.command_all(method, *args, **kwargs)

    def startup(self): return self._call("startup")
    def arm(self): return self._call("arm")
    def disarm(self): return self._call("disarm")
    def takeoff(self): return self._call("takeoff")
    def land(self): return self._call("land")
    def stop(self): return self._call("stop")
    def hold_position(self): return self._call("hold_position")
    def set_position_control(self): return self._call("set_position_control")
    def set_velocity_control(self): return self._call("set_velocity_control")
    def move_to(self, north: float, east: float, down: float):
        # Set the same target in each member's local NED frame
        return self._call("move_to", north, east, down)
    def set_velocity(self, north: float, east: float, down: float):
        return self._call("set_velocity", north, east, down)
    def forward(self, distance_m=1.0, **kwargs): return self._call("move_forward", distance_m, **kwargs)
    def backward(self, distance_m=1.0, **kwargs): return self._call("move_backward", distance_m, **kwargs)
    def left(self, distance_m=1.0, **kwargs): return self._call("move_left", distance_m, **kwargs)
    def right(self, distance_m=1.0, **kwargs): return self._call("move_right", distance_m, **kwargs)
    def up(self, distance_m=1.0, **kwargs): return self._call("move_up", distance_m, **kwargs)
    def down(self, distance_m=1.0, **kwargs): return self._call("move_down", distance_m, **kwargs)
    height_rise = up
    height_drop = down
    def set_yaw(self, yaw_rad: float): return self._call("set_yaw", yaw_rad)
    def yaw_by(self, delta_rad: float): return self._call("yaw_by", delta_rad)
    def yaw(self, yaw_rad: float): return self.set_yaw(yaw_rad)

    def run_test(self, command, **kwargs):
        return self._behavior_call("run_test", command, **kwargs)

    def _behavior_call(self, method, *args, **kwargs):
        if self.controller is None:
            raise RuntimeError("Use Swarm inside a 'with Swarm(...)' block.")
        return getattr(self.controller, method)(*args, **kwargs)

    def formation(self, shape="line", spacing_m=1.5, **kwargs):
        return self._behavior_call("formation", shape, spacing_m, **kwargs)

    def go_to(self, targets, **kwargs):
        return self._behavior_call("go_to", targets, **kwargs)

    def translate(self, north=0., east=0., down=0., **kwargs):
        return self._behavior_call("translate", north, east, down, **kwargs)

    def randomize(self, radius_m=3., **kwargs):
        return self._behavior_call("randomize", radius_m, **kwargs)

    def split(self, direction="left", spacing_m=1.5, **kwargs):
        return self._behavior_call("split", direction, spacing_m, **kwargs)

    def publish_topic(self, topic, message_type, fields, drone_ids=None, **kwargs):
        return self._behavior_call("publish_topic", topic, message_type, fields,
                                   drone_ids=drone_ids, **kwargs)

    def centroid(self): return self._behavior_call("centroid")

    def drone(self, drone_id: int) -> DroneCommandClient:
        # Access one member's lower-level client for custom behaviors
        if self.controller is None:
            raise RuntimeError("Use Swarm inside a 'with Swarm(...)' block.")
        if drone_id < 0 or drone_id >= self.drone_count:
            raise IndexError(f"Unknown drone ID {drone_id}.")
        if self.leader_id is not None:
            from functools import partial
            class Member:
                def __getattr__(member, method):
                    return partial(self.controller.command, drone_id, method)
            return Member()
        return self.controller.drones[drone_id]


__all__ = ["Drone", "Swarm"]
