#!/usr/bin/env bash

# Start a SITL swarm; Ctrl-C stops the processes launched here

set -euo pipefail

# Suspend nounset while sourcing ROS setup scripts
source_environment() {
    local setup_file="$1"
    set +u
    # shellcheck disable=SC1090
    source "$setup_file"
    set -u
}

usage() {
    echo "Usage: $0 [N | --n N] (default: DRONE_COUNT or 3; range: 1-250)"
}

case "$#" in
    0) DRONE_COUNT="${DRONE_COUNT:-3}" ;;
    1)
        if [[ "$1" == "--help" || "$1" == "-h" ]]; then
            usage
            exit 0
        fi
        DRONE_COUNT="$1"
        ;;
    2)
        if [[ "$1" != "--n" ]]; then usage >&2; exit 1; fi
        DRONE_COUNT="$2"
        ;;
    *) usage >&2; exit 1 ;;
esac
# Bound the string before arithmetic to reject overflow as well as invalid IDs
# This PX4 build documents MAV_SYS_ID as 1-250; instance i uses ID i+1
if ! [[ "$DRONE_COUNT" =~ ^[1-9][0-9]{0,2}$ ]] || (( DRONE_COUNT > 250 )); then
    usage >&2
    exit 1
fi

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
PX4_BIN="$PX4_DIR/build/px4_sitl_default/bin/px4"
PX4_SHUTDOWN="$PX4_DIR/build/px4_sitl_default/bin/px4-shutdown"
PX4_LISTENER="$PX4_DIR/build/px4_sitl_default/bin/px4-listener"
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
WORLD_NAME="drone_obstacle_course_swarm"
WORLD_SOURCE="$PROJECT_DIR/worlds/$WORLD_NAME.sdf"
WORLD_DESTINATION="$PX4_DIR/Tools/simulation/gz/worlds/$WORLD_NAME.sdf"
AGENT_LOG="${TMPDIR:-/tmp}/drone_swarm_xrce_agent.log"
NODE_LOG="${TMPDIR:-/tmp}/drone_swarm_ros_nodes.log"
SETUP_LOG="${TMPDIR:-/tmp}/drone_swarm_setup.log"

# Use the same Gazebo Transport address for every PX4 instance
export GZ_IP="${GZ_IP:-127.0.0.1}"

if [[ ! -x "$PX4_BIN" || ! -x "$PX4_LISTENER" || ! -f "$ROS_SETUP" ]]; then
    echo "Set PX4_DIR and ROS_SETUP to your PX4 SITL and ROS 2 installation."
    exit 1
fi
if [[ ! -f "$WORLD_SOURCE" ]] || ! command -v MicroXRCEAgent >/dev/null 2>&1; then
    echo "The swarm world or MicroXRCEAgent is unavailable."
    exit 1
fi

prepare_environment() {
    source_environment "$ROS_SETUP"
    source "$PROJECT_DIR/scripts/px4_vision_sitl_parameters.sh"
    cd "$PROJECT_DIR"
    export GZ_SIM_RESOURCE_PATH="$PX4_DIR/Tools/simulation/gz/models:$PX4_DIR/Tools/simulation/gz/worlds:${GZ_SIM_RESOURCE_PATH:-}"
    cp "$WORLD_SOURCE" "$WORLD_DESTINATION"
    colcon build --paths "$PROJECT_DIR/px4_msgs" "$PROJECT_DIR/ros2/swarm_mavsdk_ros2" \
        --symlink-install --allow-overriding px4_msgs
    source_environment "$PROJECT_DIR/install/setup.bash"
}

agent_pid=""
controller_pid=""
px4_pids=()
mavproxy_pids=()
cleanup() {
    for pid in "${mavproxy_pids[@]}"; do
        kill -TERM -- "-$pid" 2>/dev/null || true
    done
    if [[ -n "$controller_pid" ]]; then
        kill -TERM -- "-$controller_pid" 2>/dev/null || true
    fi

    # Shut down PX4 daemons before terminating their process groups
    if [[ -x "$PX4_SHUTDOWN" ]]; then
        for drone_id in "${!px4_pids[@]}"; do
            timeout 3 "$PX4_SHUTDOWN" --instance "$drone_id" >/dev/null 2>&1 || true
        done
    fi
    for pid in "${px4_pids[@]}"; do
        # Terminate both the daemon and its process group
        kill -TERM "$pid" 2>/dev/null || true
        kill -TERM -- "-$pid" 2>/dev/null || true
    done
    if [[ -n "$agent_pid" ]]; then
        kill -TERM -- "-$agent_pid" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

start_dds_agent() {
    agent_pattern="(^|/)MicroXRCEAgent[[:space:]]+udp4[[:space:]]+-p[[:space:]]+${PX4_UXRCE_DDS_PORT}([[:space:]]|$)"
    if ! pgrep -f "$agent_pattern" >/dev/null 2>&1; then
        setsid MicroXRCEAgent udp4 -p "$PX4_UXRCE_DDS_PORT" >"$AGENT_LOG" 2>&1 &
        agent_pid=$!
    
        # Check that the DDS agent survived startup
        for _ in $(seq 1 20); do
            if kill -0 "$agent_pid" 2>/dev/null; then
                break
            fi
            sleep 0.1
        done
        if ! kill -0 "$agent_pid" 2>/dev/null; then
            echo "MicroXRCEAgent exited before PX4 was started. Check $AGENT_LOG"
            return 1
        fi
    fi
}

wait_for_gazebo_model() {
    local model_name="$1"
    local px4_pid="$2"
    local log_file="$3"
    local timeout_seconds="${4:-90}"
    local started_at=$SECONDS

    while (( SECONDS - started_at < timeout_seconds )); do
        # Reject exited PX4 processes even if stale Gazebo models remain
        if ! kill -0 "$px4_pid" 2>/dev/null; then
            echo "PX4 exited while starting ${model_name}. Check ${log_file}"
            tail -n 40 "$log_file" 2>/dev/null || true
            return 1
        fi
        if timeout 3 gz model --list 2>/dev/null | grep -Fq "${model_name}"; then
            return 0
        fi
        sleep 1
    done

    echo "Timed out waiting for Gazebo model ${model_name}."
    echo "Check ${TMPDIR:-/tmp}/drone_swarm_px4_0.log and run: gz model --list"
    return 1
}

clear_stale_px4_socket() {
    local instance="$1"
    local socket="/tmp/px4-sock-${instance}"
    [[ -S "$socket" ]] || return 0

    # Remove stale sockets only after checking for a live instance
    if timeout 2 "$PX4_LISTENER" --instance "$instance" vehicle_status -n 1 \
        >/dev/null 2>&1; then
        echo "PX4 instance $instance is already active; stop it before launching a swarm."
        return 1
    fi
    rm -f "$socket"
}

start_simulation() {
    # Instance zero starts Gazebo; remaining instances join its world
    pushd "$PX4_DIR" >/dev/null
    for ((drone_id = 0; drone_id < DRONE_COUNT; drone_id++)); do
        clear_stale_px4_socket "$drone_id"
    done
    for ((drone_id = 0; drone_id < DRONE_COUNT; drone_id++)); do
        # Match the ROS spawn_spacing parameter for common-frame movement
        y_position=$((drone_id * 2))
        # The x500 landing gear extends below its model origin. Spawning at
        # z=0 embeds it in the ground and can leave a vehicle stuck even with
        # full motor output. Let each model settle onto the ground instead
        model_pose="0,${y_position},${SPAWN_HEIGHT:-0.24},0,0,0"
        log_file="${TMPDIR:-/tmp}/drone_swarm_px4_${drone_id}.log"
        if [[ "$drone_id" -eq 0 ]]; then
            # Instance zero must start Gazebo, even in an inherited environment
            setsid env -u PX4_GZ_STANDALONE \
                GZ_IP="$GZ_IP" \
                PX4_GZ_WORLD="$WORLD_NAME" \
                PX4_SYS_AUTOSTART=4005 \
                PX4_SIM_MODEL=gz_x500_vision \
                PX4_GZ_MODEL_POSE="$model_pose" \
                PX4_UXRCE_DDS_PORT="$PX4_UXRCE_DDS_PORT" \
                "$PX4_BIN" -d -i 0 >"$log_file" 2>&1 &
        else
            # Joining instances need the explicit world name
            setsid env \
                GZ_IP="$GZ_IP" \
                PX4_GZ_STANDALONE=1 \
                PX4_GZ_WORLD="$WORLD_NAME" \
                PX4_SYS_AUTOSTART=4005 \
                PX4_SIM_MODEL=gz_x500_vision \
                PX4_GZ_MODEL_POSE="$model_pose" \
                PX4_UXRCE_DDS_PORT="$PX4_UXRCE_DDS_PORT" \
                "$PX4_BIN" -d -i "$drone_id" >"$log_file" 2>&1 &
        fi
        px4_pids+=("$!")
        # Wait for each model before launching the next instance
        wait_for_gazebo_model "x500_vision_${drone_id}" "$!" "$log_file"
    done
    popd >/dev/null
    
    for ((drone_id = 0; drone_id < DRONE_COUNT; drone_id++)); do
        started_at=$SECONDS
        while ! grep -q 'Startup script returned successfully' "${TMPDIR:-/tmp}/drone_swarm_px4_${drone_id}.log"; do
            if ! kill -0 "${px4_pids[$drone_id]}" 2>/dev/null || (( SECONDS - started_at > 60 )); then
                echo "Drone $drone_id startup failed; inspect its PX4 log."
                return 1
            fi
            sleep 1
        done
    done
}

connect_gcs_links() {
    # Give each vehicle its own ground station heartbeat connection
    MAVPROXY_BIN="${MAVPROXY_BIN:-$HOME/mavlink_venv/bin/mavproxy.py}"
    if [[ ! -x "$MAVPROXY_BIN" ]]; then
        echo "Set MAVPROXY_BIN to an installed mavproxy.py executable."
        return 1
    fi
    for ((drone_id = 0; drone_id < DRONE_COUNT; drone_id++)); do
        apply_sitl_parameters "$drone_id" "$PX4_DIR/build/px4_sitl_default/bin/px4-param"
        # Separate ranges for all supported instances, clear of PX4 default ports
        master_port=$((24000 + drone_id))
        local_port=$((25000 + drone_id))
        timeout 5 "$PX4_DIR/build/px4_sitl_default/bin/px4-mavlink" --instance "$drone_id" \
            start -x -u "$local_port" -o "$master_port" -t 127.0.0.1 -m onboard -r 100000
        proxy_dir="${TMPDIR:-/tmp}/drone_swarm_mavproxy_${drone_id}"
        mkdir -p "$proxy_dir"
        setsid "$MAVPROXY_BIN" --non-interactive \
            --master="udpin:127.0.0.1:$master_port" --target-system="$((drone_id + 1))" \
            --state-basedir="$proxy_dir" --logfile="$proxy_dir/mav.tlog" \
            >"$proxy_dir/console.log" 2>&1 &
        mavproxy_pids+=("$!")
    done
    sleep 2
    for pid in "${mavproxy_pids[@]}"; do
        if ! kill -0 "$pid" 2>/dev/null; then
            echo "MAVProxy exited. Inspect /tmp/drone_swarm_mavproxy_*/console.log"
            return 1
        fi
    done
}

# Keep setup chatter in a log, but show errors if a stage fails
report_setup_failure() {
    echo "Swarm setup failed. Check $SETUP_LOG" >&3
    tail -n 40 "$SETUP_LOG" >&3 || true
}

connect_drone_interfaces() {
    python3 "$PROJECT_DIR/scripts/wait_for_sitl_ready.py" "$DRONE_COUNT"
    setsid ros2 launch swarm_mavsdk_ros2 swarm.launch.py \
        world_name:="$WORLD_NAME" spawn_spacing:=2.0 \
        num_drones:="$DRONE_COUNT" output_dir:="${RESULTS_DIR:-$PROJECT_DIR/results/ros2}" \
        > >(tee "$NODE_LOG" >&3) 2>&1 &
    controller_pid=$!
    python3 "$PROJECT_DIR/scripts/wait_for_sitl_ready.py" "$DRONE_COUNT" --interfaces
}

main() {
    echo "Starting simulation with $DRONE_COUNT drones..."
    # ERR is inherited by functions so strict-mode failures retain diagnostics
    exec 3>&2
    set -E
    trap report_setup_failure ERR
    prepare_environment >"$SETUP_LOG" 2>&1
    start_dds_agent >>"$SETUP_LOG" 2>&1
    start_simulation >>"$SETUP_LOG" 2>&1
    connect_gcs_links >>"$SETUP_LOG" 2>&1
    connect_drone_interfaces >>"$SETUP_LOG" 2>&1
    trap - ERR
    echo "Drone interfaces launched. Waiting for explicit startup(), arm(), and takeoff()."
    echo "Logs: $SETUP_LOG and $NODE_LOG"
    wait "$controller_pid"
}

main
