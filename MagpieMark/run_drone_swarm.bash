#!/usr/bin/env bash

# DRONE SWARM LAUNCH SCRIPT

set -e

# DIRECTORIES

PX4_DIR="$HOME/PX4-Autopilot"
PROJECT_DIR="$HOME/drone_obstacle_course"

# Apply reproducible, simulation-only PX4 settings to every process launched below. Physical
# aircraft must retain their normal arming and failsafe setup
source "$PROJECT_DIR/scripts/px4_sitl_parameters.sh"

BUILD_DIR="$PX4_DIR/build/px4_sitl_default"
PX4_BIN="$BUILD_DIR/bin/px4"
DDS_AGENT_LOG="${TMPDIR:-/tmp}/drone_swarm_xrce_agent.log"

# WORLD / MODEL

WORLD_NAME="drone_obstacle_course_swarm"
MODEL_NAME="gz_x500_vision"

WORLD_FILE="$PROJECT_DIR/worlds/$WORLD_NAME.sdf"

# SWARM CONFIGURATION

# Usage: ./run_drone_swarm.bash 3 ./run_drone_swarm.bash 8 Default = 3 drones, with PX4
# instances 0, 1, and 2

N="${1:-3}"

# Distance between drones in Gazebo Y
SPACING=2.0

# Model-origin spawn height, separate from commanded takeoff altitude
# Leave clearance for the x500 landing gear and let it settle onto the ground
ALTITUDE="${SPAWN_HEIGHT:-0.24}"

# First MAVSDK port
BASE_PORT=14540

# X500 Vision airframe
PX4_SYS_AUTOSTART=4005

# CHECK INPUT

if ! [[ "$N" =~ ^[0-9]+$ ]] || [ "$N" -lt 2 ] || [ "$N" -gt 8 ]; then

    echo
    echo "[ERROR] Number of drones must be between 2 and 8."
    echo
    echo "Usage:"
    echo "    ./run_drone_swarm.bash 2"
    echo "    ./run_drone_swarm.bash 8"
    echo

    exit 1

fi

# CHECK FILES

if [ ! -f "$WORLD_FILE" ]; then

    echo
    echo "[ERROR] World file not found:"
    echo "        $WORLD_FILE"
    echo

    exit 1

fi

if [ ! -x "$PX4_BIN" ]; then

    echo
    echo "[ERROR] PX4 executable not found:"
    echo "        $PX4_BIN"
    echo
    echo "Build PX4 first with:"
    echo
    echo "    cd ~/PX4-Autopilot"
    echo "    make px4_sitl_default"
    echo

    exit 1

fi

if ! command -v MicroXRCEAgent >/dev/null 2>&1; then

    echo
    echo "[ERROR] MicroXRCEAgent is not on PATH."
    echo "        Install/source the PX4 ROS 2 XRCE-DDS Agent workspace first."
    echo
    exit 1

fi

# CHECK TERMINAL

if ! command -v gnome-terminal >/dev/null 2>&1; then

    echo
    echo "[ERROR] gnome-terminal is not installed."
    echo
    echo "Install it with:"
    echo "    sudo apt install gnome-terminal"
    echo

    exit 1

fi

# COPY WORLD INTO PX4

cp "$WORLD_FILE" \
   "$PX4_DIR/Tools/simulation/gz/worlds/$WORLD_NAME.sdf"

# PX4 / GAZEBO ENVIRONMENT

cd "$PX4_DIR"

export PX4_GZ_WORLD="$WORLD_NAME"

# Every PX4 instance must use the same Gazebo Transport address.  `make` normally adds this for
# the first vehicle, but this launcher invokes PX4 directly so it must be explicit
export GZ_IP="${GZ_IP:-127.0.0.1}"

export GZ_SIM_RESOURCE_PATH="$PX4_DIR/Tools/simulation/gz/models:$PX4_DIR/Tools/simulation/gz/worlds:${GZ_SIM_RESOURCE_PATH:-}"

# All PX4 instances share one Micro XRCE-DDS Agent.  PX4 assigns a distinct client key and
# namespace for every `-i` instance, so starting one agent per vehicle (or assigning a separate
# UDP 8888 port) is incorrect.  This legacy MAVSDK launcher used to omit the agent entirely,
# leaving otherwise healthy vehicles invisible to ROS 2
agent_pattern="(^|/)MicroXRCEAgent[[:space:]]+udp4[[:space:]]+-p[[:space:]]+${PX4_UXRCE_DDS_PORT}([[:space:]]|$)"
if ! pgrep -f "$agent_pattern" >/dev/null 2>&1; then
    echo "[START] Micro XRCE-DDS Agent on UDP ${PX4_UXRCE_DDS_PORT}"
    setsid MicroXRCEAgent udp4 -p "$PX4_UXRCE_DDS_PORT" >"$DDS_AGENT_LOG" 2>&1 &
    DDS_AGENT_PID=$!
    sleep 1
    if ! kill -0 "$DDS_AGENT_PID" 2>/dev/null; then
        echo "[ERROR] MicroXRCEAgent exited; check $DDS_AGENT_LOG"
        exit 1
    fi
else
    echo "[READY] Reusing Micro XRCE-DDS Agent on UDP ${PX4_UXRCE_DDS_PORT}"
fi

# PRINT CONFIGURATION

echo
echo "============================================================"
echo "DRONE SWARM LAUNCH"
echo "============================================================"
echo "[CONFIG] Number of drones: $N"
echo "[CONFIG] World:            $WORLD_NAME"
echo "[CONFIG] Model:            $MODEL_NAME"
echo "[CONFIG] Spacing:          $SPACING m"
echo "[CONFIG] Altitude:         $ALTITUDE m"
echo "[CONFIG] MAVSDK ports:     $BASE_PORT - $((BASE_PORT + N - 1))"
echo "[CONFIG] XRCE-DDS agent:   UDP $PX4_UXRCE_DDS_PORT (one shared agent)"
echo "============================================================"
echo

# START DRONE 0

# IMPORTANT: Drone 0 starts Gazebo.  It is launched directly, using the same explicit
# environment as every other PX4 instance.  PX4_GZ_STANDALONE is explicitly removed because a
# stale value inherited from a previous terminal would make instance 0 wait for a Gazebo server
# instead of creating it

echo "------------------------------------------------------------"
echo "[START] Drone 0"
echo "[START] PX4 instance: 0"
echo "[START] Pose: 0,0,$ALTITUDE"
echo "------------------------------------------------------------"
echo

gnome-terminal --title="PX4 Drone 0" -- bash -lc "
    cd '$PX4_DIR'

    echo
    echo '============================================================'
    echo 'PX4 DRONE 0'
    echo '============================================================'
    echo 'World:    $WORLD_NAME'
    echo 'Model:    $MODEL_NAME'
    echo 'Instance: 0'
    echo 'Pose:     0,0,$ALTITUDE'
    echo '============================================================'
    echo

    env -u PX4_GZ_STANDALONE \\
        GZ_IP='$GZ_IP' \\
        PX4_GZ_WORLD='$WORLD_NAME' \\
        PX4_SYS_AUTOSTART='$PX4_SYS_AUTOSTART' \\
        PX4_SIM_MODEL='$MODEL_NAME' \\
        PX4_GZ_MODEL_POSE='0,0,$ALTITUDE,0,0,0' \\
        PX4_UXRCE_DDS_PORT='$PX4_UXRCE_DDS_PORT' \\
        GZ_SIM_RESOURCE_PATH='$PX4_DIR/Tools/simulation/gz/models:$PX4_DIR/Tools/simulation/gz/worlds:\${GZ_SIM_RESOURCE_PATH:-}' \\
        '$PX4_BIN' -i 0

    echo
    echo '[PX4] Drone 0 has stopped.'
    echo
    exec bash
" >/dev/null 2>&1 &

# WAIT FOR GAZEBO / DRONE 0

echo
 echo "[WAIT] Waiting for Drone 0 and Gazebo..."

echo

GAZEBO_READY=0

for ((attempt=1; attempt<=30; attempt++)); do

    sleep 5

    if timeout 3 gz model --list >/tmp/drone_swarm_gz_check.txt 2>&1; then

        if grep -q "x500_vision_0" /tmp/drone_swarm_gz_check.txt; then
            GAZEBO_READY=1
            break
        fi

    fi

    echo "[WAIT] Gazebo / Drone 0 not ready yet... ($attempt/30)"

done

if [ "$GAZEBO_READY" -ne 1 ]; then

    echo
    echo "============================================================"
    echo "[ERROR] Drone 0 did not appear in Gazebo."
    echo "============================================================"
    echo
    echo "Run this manually to inspect Gazebo:"
    echo
    echo "    gz model --list"
    echo
    echo "The PX4 Drone 0 terminal should also show the startup error."
    echo

    exit 1

fi

echo
 echo "[READY] Drone 0 is in Gazebo."
echo
echo "[WAIT] Giving Gazebo extra time to finish initializing Drone 0..."
sleep 10
echo

# START REMAINING DRONES

for ((i=1; i<N; i++)); do

    # The first drone is at Y=0. Each additional drone is spaced by 2 m. N=2: Drone 0 -> Y=0
    # Drone 1 -> Y=2 N=8: Drone 0 -> Y=0 Drone 1 -> Y=2 Drone 2 -> Y=4 Drone 3 -> Y=6 Drone 4 ->
    # Y=8 Drone 5 -> Y=10 Drone 6 -> Y=12 Drone 7 -> Y=14

    y=$(awk -v i="$i" -v s="$SPACING" \
        'BEGIN { printf "%.3f", i * s }')

    echo
    echo "------------------------------------------------------------"
    echo "[START] Drone $i"
    echo "[START] PX4 instance: $i"
    echo "[START] Pose: 0,$y,$ALTITUDE"
    echo "[START] MAVSDK port: $((BASE_PORT + i))"
    echo "------------------------------------------------------------"

    gnome-terminal --title="PX4 Drone $i" -- bash -lc "
        cd '$PX4_DIR'

        export GZ_IP='$GZ_IP'
        export PX4_GZ_STANDALONE=1
        export PX4_GZ_WORLD='$WORLD_NAME'
        export PX4_SYS_AUTOSTART='$PX4_SYS_AUTOSTART'
        export PX4_SIM_MODEL='$MODEL_NAME'
        export PX4_GZ_MODEL_POSE='0,$y,$ALTITUDE,0,0,0'
        export PX4_UXRCE_DDS_PORT='$PX4_UXRCE_DDS_PORT'

        export GZ_SIM_RESOURCE_PATH='$PX4_DIR/Tools/simulation/gz/models:$PX4_DIR/Tools/simulation/gz/worlds:\${GZ_SIM_RESOURCE_PATH:-}'

        echo
        echo '============================================================'
        echo 'PX4 DRONE $i'
        echo '============================================================'
        echo 'Model:    $MODEL_NAME'
        echo 'Instance: $i'
        echo 'Pose:     0,$y,$ALTITUDE'
        echo '============================================================'
        echo

        '$PX4_BIN' -i '$i'

        echo
        echo '[PX4] Drone $i has stopped.'
        echo
        exec bash
    " >/dev/null 2>&1 &

    # Wait until this drone actually appears in Gazebo before starting the next drone. This
    # prevents the PX4 instances from being launched faster than Gazebo can initialize them

    DRONE_READY=0

    for ((attempt=1; attempt<=30; attempt++)); do

        sleep 5

        if timeout 3 gz model --list >/tmp/drone_swarm_gz_check.txt 2>&1; then

            if grep -q "x500_vision_$i" /tmp/drone_swarm_gz_check.txt; then
                DRONE_READY=1
                break
            fi

        fi

        echo "[WAIT] Drone $i not in Gazebo yet... ($attempt/30)"

    done

    if [ "$DRONE_READY" -ne 1 ]; then

        echo
        echo "============================================================"
        echo "[ERROR] Drone $i did not appear in Gazebo."
        echo "============================================================"
        echo
        echo "Current Gazebo models:"
        timeout 5 gz model --list || true
        echo
        echo "Check the PX4 Drone $i terminal for errors."
        echo
        exit 1

    fi

    echo "[READY] Drone $i is in Gazebo."
    echo "[WAIT] Giving Gazebo time to settle before starting the next drone..."
    sleep 8

done

# WAIT FOR ALL DRONES

echo
echo "============================================================"
echo "[WAIT] Waiting for all drones to appear in Gazebo..."
echo "============================================================"
echo

ALL_READY=0

for ((attempt=1; attempt<=30; attempt++)); do

    sleep 2

    if timeout 3 gz model --list >/tmp/drone_swarm_gz_check.txt 2>&1; then

        FOUND=0

        for ((i=0; i<N; i++)); do

            if grep -q "x500_vision_$i" /tmp/drone_swarm_gz_check.txt; then
                FOUND=$((FOUND + 1))
            fi

        done

        if [ "$FOUND" -eq "$N" ]; then
            ALL_READY=1
            break
        fi

        echo "[WAIT] Gazebo has $FOUND/$N drones... ($attempt/30)"

    else

        echo "[WAIT] Gazebo query timed out... ($attempt/30)"

    fi

done

# PRINT FINAL MODEL LIST

echo
 echo "============================================================"
echo "[GAZEBO] Current models"
echo "============================================================"

timeout 5 gz model --list || true

# FINAL STATUS

if [ "$ALL_READY" -ne 1 ]; then

    echo
    echo "============================================================"
    echo "[ERROR] Not all drones appeared in Gazebo."
    echo "============================================================"
    echo
    echo "Check the PX4 terminals for the failed instance(s)."
    echo
    exit 1

fi

echo
echo "============================================================"
echo "[READY] $N-DRONE SWARM IS RUNNING"
echo "============================================================"
echo

for ((i=0; i<N; i++)); do

    echo "Drone $i:"
    echo "    PX4 instance: $i"
    echo "    Gazebo model: x500_vision_$i"
    echo "    MAVSDK port:  $((BASE_PORT + i))"
    echo

 done

echo "============================================================"
echo
echo "Run the swarm controller from another terminal:"
echo
echo "    cd $PROJECT_DIR"
echo "    python3 drone_test_swarms.py --n $N"
echo
echo "============================================================"
echo
