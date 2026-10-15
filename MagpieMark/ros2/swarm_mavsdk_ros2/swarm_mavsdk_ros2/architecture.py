# Select central control or a drone that leads the swarm
import re


def leader_id(architecture: str, drone_count: int) -> int | None:
    if architecture == "central":
        return None
    match = re.fullmatch(r"edge:drone_(0|[1-9][0-9]*)", architecture)
    if match is None:
        raise ValueError("Use central or edge:drone_<id> for architecture")
    selected = int(match.group(1))
    if not 0 <= selected < drone_count:
        raise ValueError("Leader drone ID must be within the swarm")
    return selected
