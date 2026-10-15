#!/usr/bin/env bash
# Only for Gazebo x500_vision: use its simulated odometry as the EKF source
source "$(dirname "${BASH_SOURCE[0]}")/px4_sitl_parameters.sh"
export PX4_PARAM_EKF2_EV_CTRL=15
export PX4_PARAM_EKF2_HGT_REF=3
export PX4_PARAM_EKF2_GPS_CTRL=0
export PX4_PARAM_EKF2_MAG_TYPE=5
