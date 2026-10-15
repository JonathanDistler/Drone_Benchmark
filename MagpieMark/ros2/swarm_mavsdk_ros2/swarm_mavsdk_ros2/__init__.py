# ROS 2 interfaces and behavior helpers for PX4 drones

__all__ = ["Drone", "Swarm"]


def __getattr__(name):
    if name in __all__:
        from .behavior import Drone, Swarm
        return {"Drone": Drone, "Swarm": Swarm}[name]
    raise AttributeError(name)
