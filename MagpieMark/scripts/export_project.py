#!/usr/bin/env python3
# Export the active simulator workspace without host environments or logs
import argparse
from pathlib import Path
import tarfile


PROJECT = Path(__file__).resolve().parents[1]
SOURCES = [
    "ReadMe.md", "scripts/start_simulation.py", "scripts/export_project.py",
    "scripts/start_single_drone_ros2.sh", "scripts/validate_flight_controls.py",
    "scripts/plot_drone_positions.m",
    "scripts/px4_sitl_parameters.sh", "scripts/px4_vision_sitl_parameters.sh",
    "scripts/wait_for_sitl_ready.py", "scripts/swarm_validity_test.py",
    "ros2/swarm_mavsdk_ros2", "px4_msgs", "worlds", "models", "docs",
    "environment.yml", "requirements-venv.txt", "environment-locks",
]
EXCLUDED_DIRECTORIES = {".git", ".venv", "__pycache__", ".pytest_cache", "results"}


def include(member):
    path = Path(member.name)
    if any(part in EXCLUDED_DIRECTORIES or part.endswith(".egg-info") for part in path.parts):
        return None
    if path.name.startswith("mav.") or path.name.endswith((".pyc", ".backup", ".log", ".csv", ".ulg")):
        return None
    # Source symlinks outside this workspace would not be portable
    if member.issym() or member.islnk():
        raise ValueError(f"Export requires regular source files: {member.name}")
    return member


def main():
    parser = argparse.ArgumentParser(description='Export the active simulator workspace without host environments or logs')
    parser.add_argument("--output", type=Path,
                        default=PROJECT / "exports/drone_obstacle_course-source.tar.gz")
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    # Exclusive creation prevents accidentally replacing a previous export
    sources = SOURCES.copy()
    if (PROJECT / "requirements-venv.lock.txt").is_file():
        sources.append("requirements-venv.lock.txt")
    with tarfile.open(args.output, "x:gz") as archive:
        for relative in sources:
            archive.add(PROJECT / relative, arcname=f"drone_obstacle_course/{relative}", filter=include)
    print(f"Exported {args.output.resolve()} ({args.output.stat().st_size:,} bytes)")
    print("Recipients: follow drone_obstacle_course/docs/run_and_share.md")


if __name__ == "__main__":
    main()
