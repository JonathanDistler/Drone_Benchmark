# Small, dependency-free adapter for reinforcement-learning controllers. This module
# deliberately does not depend on Gymnasium, PyTorch, or a specific trainer. A training
# project can wrap :class:`DroneRlInterface` in its own environment while keeping all PX4 and
# ROS 2 details in one place

from dataclasses import dataclass
from typing import Protocol, Sequence
import time
import math

import rclpy

from .commands import DroneCommandClient


@dataclass(frozen=True)
class DroneObservation:
    # Estimator state provided to a policy, expressed in local NED

    north_m: float
    east_m: float
    down_m: float
    north_velocity_mps: float
    east_velocity_mps: float
    down_velocity_mps: float


class Policy(Protocol):
    # Minimal contract implemented by an RL policy

    def act(self, observation: DroneObservation) -> Sequence[float]:
        # Return a North, East, Down velocity action in m/s
        pass


class DroneRlInterface:
    # Translate bounded velocity actions into the normal DroneNode API

    def __init__(self, client: DroneCommandClient, max_speed_mps: float = 1.0):
        if not math.isfinite(max_speed_mps) or max_speed_mps <= 0.0:
            raise ValueError("max_speed_mps must be positive.")
        self.client = client
        self.max_speed_mps = max_speed_mps

    def observation(self) -> DroneObservation:
        self.client.wait_for_state()
        state = self.client.state
        position = state.pose.pose.position
        velocity = state.twist.twist.linear
        return DroneObservation(
            position.x, position.y, position.z,
            velocity.x, velocity.y, velocity.z,
        )

    def apply_velocity_action(self, action: Sequence[float]) -> None:
        # Apply a normalized three-axis action in the inclusive range [-1, 1]
        if len(action) != 3:
            raise ValueError("An action must contain [north, east, down].")
        if not all(math.isfinite(float(value)) for value in action):
            raise ValueError("Action values must be finite.")
        bounded = [max(-1.0, min(1.0, float(value))) for value in action]
        self.client.set_velocity(*(value * self.max_speed_mps for value in bounded))

    def apply_position_action(self, action: Sequence[float]) -> None:
        # Absolute local NED coordinates in metres, without normalization.
        if len(action) != 3:
            raise ValueError("An action must contain [north, east, down].")
        target = [float(value) for value in action]
        self.client._validate_vector(*target)
        self.client.move_to(*target)

    def step(self, action: Sequence[float], duration_s: float = 0.1, *, mode="velocity") -> DroneObservation:
        # Apply an action for one control interval and return the next state
        if not math.isfinite(duration_s) or duration_s <= 0.0:
            raise ValueError("duration_s must be positive.")
        if mode not in ("position", "velocity"):
            raise ValueError("mode must be position or velocity.")
        if mode == "position":
            self.apply_position_action(action)
        else:
            self.apply_velocity_action(action)
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self.client.node, timeout_sec=0.01)
        if mode == "velocity":
            self.client.set_velocity(0.0, 0.0, 0.0)
        return self.observation()
