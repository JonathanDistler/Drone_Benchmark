# Source this file from ~/.bashrc to make the drone_* functions available in every interactive
# Bash terminal

_drone_project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
_drone_ros_setup_done=0

_drone_source_setup() {
    local setup_file="$1"
    if [[ ! -f "$setup_file" ]]; then
        printf 'Drone setup file not found: %s\n' "$setup_file" >&2
        return 1
    fi
    local had_nounset=0
    [[ $- == *u* ]] && had_nounset=1
    set +u
    # ROS setup files may reference optional, unset environment variables

    # shellcheck disable=SC1090
    source "$setup_file"
    if (( had_nounset )); then
        set -u
    fi
}

drone_ros_setup() {
    if (( _drone_ros_setup_done )); then
        return 0
    fi
    _drone_source_setup "${ROS_SETUP:-/opt/ros/jazzy/setup.bash}" || return
    _drone_source_setup "${DRONE_WORKSPACE_SETUP:-$_drone_project_dir/install/setup.bash}" || return
    _drone_ros_setup_done=1
}

drone_command() {
    if (( $# < 2 )); then
        echo "Usage: drone_command <drone-id> <command> [arguments...]" >&2
        return 2
    fi
    local drone_id="$1"
    shift
    drone_ros_setup || return
    ros2 run swarm_mavsdk_ros2 drone_command "$@" --drone "$drone_id"
}

drone_arm() { drone_command "${1:-0}" arm; }
drone_takeoff() { drone_command "${1:-0}" takeoff; }
drone_land() { drone_command "${1:-0}" land; }

# Start control streaming for drones 0, 1, and 2 by default. Pass IDs to select another swarm,
# for example: drone_startup 0 2
drone_startup() {
    local drone_ids
    if (( $# )); then
        drone_ids=("$@")
    else
        drone_ids=(0 1 2)
    fi
    drone_ros_setup || return
    local drone_id
    for drone_id in "${drone_ids[@]}"; do
        if ! [[ "$drone_id" =~ ^[0-9]+$ ]]; then
            printf 'Invalid drone ID: %s\n' "$drone_id" >&2
            return 2
        fi
        ros2 run swarm_mavsdk_ros2 drone_command startup --drone "$drone_id" || return
    done
}

# Loading this helper from ~/.bashrc also prepares direct `ros2 run` commands once per
# interactive shell, without requiring another source command
drone_ros_setup
