#!/usr/bin/env bash

# ============================================================
# PX4 / GAZEBO BLANK-WORLD 3-DRONE SWARM
# ============================================================
#
# Launches:
#
#     Drone 0 -> PX4 instance 0 + Gazebo
#     Drone 1 -> PX4 instance 1
#     Drone 2 -> PX4 instance 2
#
# All three PX4 instances receive the same SITL parameters
#
# MicroXRCEAgent is NOT started by this script
#
# Start separately with:
#
#     MicroXRCEAgent udp4 -p 8888
#
# ============================================================

set -e

# ============================================================
# DIRECTORIES
# ============================================================

PX4_DIR="$HOME/PX4-Autopilot"

BUILD_DIR="$PX4_DIR/build/px4_sitl_default"

PX4_BIN="$BUILD_DIR/bin/px4"

# ============================================================
# GAZEBO CONFIGURATION
# ============================================================

WORLD_NAME="default"

MODEL_NAME="gz_x500"

# ============================================================
# SWARM CONFIGURATION
# ============================================================

N=3

SPACING=2.0

ALTITUDE=0.0

# ============================================================
# DDS CONFIGURATION
# ============================================================

DDS_PORT=8888

# ============================================================
# PX4 CONFIGURATION
# ============================================================

PX4_SYS_AUTOSTART=4001

# ============================================================
# SITL / AUTONOMOUS PARAMETERS
# ============================================================
#
# These are deliberately permissive for simulation
#
# The objective is to remove dependencies on:
#
#     RC
#     GCS
#     battery minimums
#     mission validity
#     external authorization
#     preflight auto-disarm
#
# while keeping the estimator and flight controllers active
#
# PX4 supports PX4_PARAM_<PARAMETER>=<value> overrides
# for simulation sessions
#
# ============================================================

export PX4_PARAM_COM_RC_IN_MODE=4

export PX4_PARAM_COM_DLL_EXCEPT=4

export PX4_PARAM_COM_RCL_EXCEPT=4

export PX4_PARAM_COM_DISARM_PRFLT=-1

export PX4_PARAM_COM_DISARM_LAND=-1

export PX4_PARAM_NAV_RCL_ACT=0

export PX4_PARAM_NAV_DLL_ACT=0

export PX4_PARAM_COM_ARM_BAT_MIN=0

export PX4_PARAM_COM_ARM_MIS_REQ=0

export PX4_PARAM_COM_ARM_AUTH_REQ=0

export PX4_PARAM_COM_ARM_WO_GPS=2

export PX4_PARAM_COM_ARMABLE=1

# ============================================================
# ENVIRONMENT
# ============================================================

cd "$PX4_DIR"

export PX4_SIM_MODEL="$MODEL_NAME"

export PX4_GZ_WORLD="$WORLD_NAME"

export PX4_UXRCE_DDS_PORT="$DDS_PORT"

# ============================================================
# RESOURCE PATH
# ============================================================

export GZ_SIM_RESOURCE_PATH="$PX4_DIR/Tools/simulation/gz/models:$PX4_DIR/Tools/simulation/gz/worlds:${GZ_SIM_RESOURCE_PATH:-}"

# ============================================================
# CHECK PX4
# ============================================================

if [ ! -x "$PX4_BIN" ]; then

    echo
    echo "[ERROR] PX4 executable not found:"
    echo "        $PX4_BIN"
    echo

    exit 1

fi

# ============================================================
# HELPER: WAIT FOR GAZEBO MODEL
# ============================================================

wait_for_model()
{
    local MODEL="$1"

    local TIMEOUT="$2"

    local START

    local NOW

    local ELAPSED

    START=$(date +%s)

    echo

    echo "[WAIT] Waiting for Gazebo model: $MODEL"

    while true; do

        if gz model --list 2>/dev/null | grep -q "^[[:space:]]*-[[:space:]]*$MODEL$"; then

            echo "[READY] Gazebo model detected: $MODEL"

            return 0

        fi

        NOW=$(date +%s)

        ELAPSED=$((NOW - START))

        if [ "$ELAPSED" -ge "$TIMEOUT" ]; then

            echo

            echo "[ERROR] Timed out waiting for $MODEL"

            echo

            gz model --list 2>/dev/null || true

            return 1

        fi

        echo "[WAIT] $MODEL not ready... ${ELAPSED}/${TIMEOUT} s"

        sleep 1

    done
}

# ============================================================
# HELPER: WAIT FOR GAZEBO ITSELF
# ============================================================

wait_for_gazebo()
{
    local TIMEOUT="$1"

    local START

    local NOW

    local ELAPSED

    START=$(date +%s)

    echo

    echo "[WAIT] Waiting for Gazebo..."

    while true; do

        if gz model --list >/dev/null 2>&1; then

            echo "[READY] Gazebo is responding."

            return 0

        fi

        NOW=$(date +%s)

        ELAPSED=$((NOW - START))

        if [ "$ELAPSED" -ge "$TIMEOUT" ]; then

            echo "[ERROR] Gazebo did not respond."

            return 1

        fi

        sleep 1

    done
}

# ============================================================
# CLEANUP
# ============================================================

PIDS=()

cleanup()
{
    echo

    echo "============================================================"

    echo "[SHUTDOWN] Stopping PX4 swarm"

    echo "============================================================"

    for PID in "${PIDS[@]}"; do

        kill "$PID" 2>/dev/null || true

    done

    wait 2>/dev/null || true
}

trap cleanup EXIT INT TERM

# ============================================================
# CONFIGURATION DISPLAY
# ============================================================

echo

echo "============================================================"

echo "PX4 / GAZEBO BLANK-WORLD 3-DRONE SWARM"

echo "============================================================"

echo "[CONFIG] Drones:          $N"

echo "[CONFIG] World:           $WORLD_NAME"

echo "[CONFIG] Model:           $MODEL_NAME"

echo "[CONFIG] Spacing:         $SPACING m"

echo "[CONFIG] Altitude:        $ALTITUDE m"

echo "[CONFIG] DDS port:        $DDS_PORT"

echo

echo "[PX4 PARAM] COM_RC_IN_MODE     = $PX4_PARAM_COM_RC_IN_MODE"

echo "[PX4 PARAM] COM_RCL_EXCEPT     = $PX4_PARAM_COM_RCL_EXCEPT"

echo "[PX4 PARAM] COM_DISARM_PRFLT   = $PX4_PARAM_COM_DISARM_PRFLT"

echo "[PX4 PARAM] COM_DISARM_LAND    = $PX4_PARAM_COM_DISARM_LAND"

echo "[PX4 PARAM] NAV_RCL_ACT        = $PX4_PARAM_NAV_RCL_ACT"

echo "[PX4 PARAM] NAV_DLL_ACT        = $PX4_PARAM_NAV_DLL_ACT"

echo "[PX4 PARAM] COM_ARM_BAT_MIN    = $PX4_PARAM_COM_ARM_BAT_MIN"

echo "[PX4 PARAM] COM_ARM_MIS_REQ    = $PX4_PARAM_COM_ARM_MIS_REQ"

echo "[PX4 PARAM] COM_ARM_AUTH_REQ   = $PX4_PARAM_COM_ARM_AUTH_REQ"

echo "[PX4 PARAM] COM_ARM_WO_GPS     = $PX4_PARAM_COM_ARM_WO_GPS"

echo "[PX4 PARAM] COM_ARMABLE        = $PX4_PARAM_COM_ARMABLE"

echo

echo "============================================================"

# ============================================================
# DRONE 0
# ============================================================

Y0=$(awk -v i=0 -v n="$N" -v s="$SPACING" \
    'BEGIN { printf "%.3f", (i - (n - 1) / 2.0) * s }')

POSE0="0,${Y0},${ALTITUDE},0,0,0"

echo

echo "------------------------------------------------------------"

echo "[START] Drone 0"

echo "[START] PX4 instance: 0"

echo "[START] MAV_SYS_ID:   1"

echo "[START] Pose:          $POSE0"

echo "------------------------------------------------------------"

echo

PX4_SYS_AUTOSTART="$PX4_SYS_AUTOSTART" \
PX4_SIM_MODEL="$MODEL_NAME" \
PX4_GZ_WORLD="$WORLD_NAME" \
PX4_GZ_MODEL_POSE="$POSE0" \
PX4_UXRCE_DDS_PORT="$DDS_PORT" \
"$PX4_BIN" -i 0 > /tmp/px4_swarm_0.log 2>&1 &

PIDS+=("$!")

wait_for_gazebo 60

wait_for_model "x500_0" 60

echo

echo "[WAIT] Letting Drone 0 / EKF settle for 10 seconds..."

sleep 10

# ============================================================
# DRONES 1 AND 2
# ============================================================

for ((i=1; i<N; i++)); do

    Y=$(awk -v i="$i" -v n="$N" -v s="$SPACING" \
        'BEGIN { printf "%.3f", (i - (n - 1) / 2.0) * s }')

    POSE="0,${Y},${ALTITUDE},0,0,0"

    INSTANCE_DIR="$BUILD_DIR/instance_$i"

    mkdir -p "$INSTANCE_DIR"

    echo

    echo "------------------------------------------------------------"

    echo "[START] Drone $i"

    echo "[START] PX4 instance: $i"

    echo "[START] MAV_SYS_ID:   $((i + 1))"

    echo "[START] Pose:          $POSE"

    echo "------------------------------------------------------------"

    echo

    (

        cd "$INSTANCE_DIR"

        export PX4_PARAM_COM_RC_IN_MODE=4

        export PX4_PARAM_COM_RCL_EXCEPT=4

        export PX4_PARAM_COM_DISARM_PRFLT=-1

        export PX4_PARAM_COM_DISARM_LAND=-1

        export PX4_PARAM_NAV_RCL_ACT=0

        export PX4_PARAM_NAV_DLL_ACT=0

        export PX4_PARAM_COM_ARM_BAT_MIN=0

        export PX4_PARAM_COM_ARM_MIS_REQ=0

        export PX4_PARAM_COM_ARM_AUTH_REQ=0

        export PX4_PARAM_COM_ARM_WO_GPS=2

        export PX4_PARAM_COM_ARMABLE=1

        export PX4_SYS_AUTOSTART="$PX4_SYS_AUTOSTART"

        export PX4_SIM_MODEL="$MODEL_NAME"

        export PX4_GZ_MODEL_POSE="$POSE"

        export PX4_UXRCE_DDS_PORT="$DDS_PORT"

        export GZ_SIM_RESOURCE_PATH="$PX4_DIR/Tools/simulation/gz/models:$PX4_DIR/Tools/simulation/gz/worlds:${GZ_SIM_RESOURCE_PATH:-}"

        export PX4_GZ_STANDALONE=1

        exec "$PX4_BIN" -i "$i" -d "$PX4_DIR/build/px4_sitl_default/etc"

    ) > "/tmp/px4_swarm_${i}.log" 2>&1 &

    PIDS+=("$!")

    wait_for_model "x500_$i" 60

    echo

    echo "[WAIT] Letting Drone $i / EKF settle for 5 seconds..."

    sleep 5

done

# ============================================================
# FINAL VERIFICATION
# ============================================================

echo

echo "============================================================"

echo "[VERIFY] Gazebo models"

echo "============================================================"

gz model --list || true

echo

echo "============================================================"

echo "[VERIFY] PX4 processes"

echo "============================================================"

pgrep -af "$PX4_BIN" || true

echo

echo "============================================================"

echo "[VERIFY] ROS 2 local position streams"

echo "============================================================"

if command -v ros2 >/dev/null 2>&1; then

    source /opt/ros/jazzy/setup.bash

    ros2 topic list 2>/dev/null \
        | grep vehicle_local_position_v1 \
        || true

fi

echo

echo "============================================================"

echo "[READY] 3-DRONE SIMULATION IS RUNNING"

echo "============================================================"

echo

echo "PX4 logs:"

echo "    /tmp/px4_swarm_0.log"

echo "    /tmp/px4_swarm_1.log"

echo "    /tmp/px4_swarm_2.log"

echo

echo "ROS 2 controller:"

echo "    source /opt/ros/jazzy/setup.bash"

echo "    source ~/px4_ros2_ws/install/setup.bash"

echo "    ros2 run swarm_mavsdk_ros2 swarm_tests --n 3"

echo

echo "Press Ctrl+C in this terminal to stop the swarm."

echo

echo "============================================================"

# ============================================================
# KEEP SCRIPT ALIVE
# ============================================================

while true; do

    sleep 1

done
