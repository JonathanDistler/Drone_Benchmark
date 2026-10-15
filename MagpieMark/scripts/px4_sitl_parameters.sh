#!/usr/bin/env bash

# PX4 parameters used only for disposable simulation sessions. Source this file before PX4
# starts; PX4 reads PX4_PARAM_* values at startup. Do not source it for a physical aircraft: lab
# hardware needs its normal RC, battery, and failsafe configuration

export PX4_UXRCE_DDS_PORT=8888
export PX4_PARAM_COM_RC_IN_MODE=4
export PX4_PARAM_COM_ARM_WO_GPS=2
export PX4_PARAM_COM_DLL_EXCEPT=4
export PX4_PARAM_COM_RCL_EXCEPT=4
export PX4_PARAM_COM_DISARM_PRFLT=-1
export PX4_PARAM_COM_DISARM_LAND=-1
export PX4_PARAM_NAV_DLL_ACT=0
export PX4_PARAM_NAV_RCL_ACT=0
export PX4_PARAM_COM_ARM_BAT_MIN=0
export PX4_PARAM_COM_ARM_MIS_REQ=0
export PX4_PARAM_COM_ARM_AUTH_REQ=0
export PX4_PARAM_COM_ARMABLE=1

# The custom Gazebo world can report a magnetic field inconsistent with PX4's geographic model
# Disable only this SITL arming check, not estimator validity
export PX4_PARAM_COM_ARM_MAG_STR=0

# Apply after rcS/airframe startup and verify the live instance. Startup scripts and persisted
# parameters can otherwise leave values different from exports
apply_sitl_parameters() {
    local instance="$1" param_bin="$2" variable name value
    for variable in ${!PX4_PARAM_@}; do
        name="${variable#PX4_PARAM_}"
        value="${!variable}"
        timeout 5 "$param_bin" --instance "$instance" set "$name" "$value" || return 1
        timeout 5 "$param_bin" --instance "$instance" compare "$name" "$value" || {
            echo "SITL parameter verification failed: drone $instance $name=$value"
            return 1
        }
    done
}
