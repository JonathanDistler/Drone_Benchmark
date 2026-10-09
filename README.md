# Drone_Benchmark
MagpieMark: A Novel Sim-to-Real Benchmarking System for Single and Swarm Drone Test-Beds

# The Goal of This Project

The goal of this project is to create a viable drone benchmark tests that incorporates both static and dynamic elements, as well as viability for singular drones and drone swarms. 

Although there are drone benchmark tests, as per my literature review, they don't incorporate any reinforcement-learning (RL) based testing, dynamic elements, or compatibility with drone swarms - a growing research feature. 

# Basic Assumptions

The following are the main assumptions for the work:

1. The drones in use are (in theory) capable of the desired movement patterns. 

2. All in-vitro drone tests will be compared to a 1:1 Gazebo simulation, with the same mechanisms in play. 

# Basic Research Question

The basic research question is as follows:

> **"Can I build a physical drone benchmark test that is backed with a simulation environment that can test drone computation?"**

# Methodology

The general methodology is to create a representation of the drone test-bed in Gazebo, then transfer the exact setup into a physical environmnet. Then, using basic circuitry and computer vision, the physical tests will be overlapped with the simulation tests to guide drone development. 

# Setup Single Drone 

First, source the environment in the miniconda virtual environment in terminal 1: 

```bash
cd ~/drone_obstacle_course
source ~/miniconda3/etc/profile.d/conda.sh
conda activate drone-ros-jazzy
source install-conda/local_setup.bash
/usr/bin/python3 scripts/start_simulation.py --world blank --n 1
```
Then, in terminal 2:

```bash
cd ~/drone_obstacle_course
source /opt/ros/jazzy/setup.bash
source install-system/local_setup.bash
ros2 run swarm_mavsdk_ros2 control_console --drone 0
```

Finally, enter the commands in the terminal from the list below, with the first three (startup, arm, takeoff) required to instantiate the drone, and all commands followed by an integer or float being primitives with the magnitude definining the distance in meters in which the drone will move.  

```bash
startup
arm
takeoff
forward 2
backward 2
right 1
left 1
up 0.5
down 0.5
land
quit
```

Or, in python: 

```python
python scripts/start_simulation.py --ros conda --n 1 --architecture edge:drone_0

python -m swarm_mavsdk_ros2.command_cli startup --drone 0
python -m swarm_mavsdk_ros2.command_cli arm --drone 0
python -m swarm_mavsdk_ros2.command_cli takeoff --drone 0
python -m swarm_mavsdk_ros2.command_cli forward 0.5 --drone 0
python -m swarm_mavsdk_ros2.command_cli land --drone 0
python -m swarm_mavsdk_ros2.command_cli disarm --drone 0
```

# Setup Single Drone Test 
First, source the environment in the miniconda virtual environment in terminal 1, which requires the specific .STEP files I had generated : 

```bash
cd ~/drone_obstacle_course
source ~/miniconda3/etc/profile.d/conda.sh
conda activate drone-ros-jazzy
source install-conda/local_setup.bash
/usr/bin/python3 scripts/start_simulation.py --world obstacles --n 1
```

Then, in terminal 2: 

```bash
cd ~/drone_obstacle_course
source /opt/ros/jazzy/setup.bash
source install-system/local_setup.bash
ros2 run swarm_mavsdk_ros2 control_console --drone 0
```
Finally, enter the commands in the terminal from the list below. One can string together multiple tests at once (e.g. test 1, test 2, . . . ), with the drone reorienting itself along the "start line" of each new test, which does take some time. This was an artifact of having limited space in the real test-setup, and wanting to preserve the constraints in the simulation. Test 1 represents a cone-weaving task; test 2 repressnts a dynamic flag passage, in which the flag "orients" itself either left or right and the drone has to react to it in order to simulate dynamic on-board compute; test 3 represents a passage through a confined space. Results are saved in "results/tests/all_tests.csv".

```bash
startup
arm
takeoff
test 1
test 2
test 3
land
quit
```

# Swarm Setup - Central Command Node

In terminal 1, 

```bash
cd ~/drone_obstacle_course
source ~/miniconda3/etc/profile.d/conda.sh
conda activate drone-ros-jazzy
source install-conda/local_setup.bash
/usr/bin/python3 scripts/start_simulation.py --world blank --n 3
```

In terminal 2, 

```bash
cd ~/drone_obstacle_course
source /opt/ros/jazzy/setup.bash
source install-system/local_setup.bash
```

Then, following a similar startup sequence to the previous single-drone test: 

```bash
ros2 run swarm_mavsdk_ros2 swarm_command startup --n 3
ros2 run swarm_mavsdk_ros2 swarm_command arm --n 3
ros2 run swarm_mavsdk_ros2 swarm_command takeoff --n 3


ros2 run swarm_mavsdk_ros2 swarm_command forward 2 --n 3
ros2 run swarm_mavsdk_ros2 swarm_command backward 2 --n 3

ros2 run swarm_mavsdk_ros2 swarm_command stop --n 3
ros2 run swarm_mavsdk_ros2 swarm_command land --n 3
```

There are also different functionalities that string together primitives for more complex shapes: 

```bash
ros2 run swarm_mavsdk_ros2 swarm_command formation --shape line --spacing 1.5 --n 3

ros2 run swarm_mavsdk_ros2 swarm_command formation --shape grid --spacing 2 --n 3

ros2 run swarm_mavsdk_ros2 swarm_command formation --shape circle --spacing 2 --n 3
```

Then, for commanding specific drones from the swarm in terminal, with n being defined as the drone id (e.g. 0, 1, 2)

```bash
ros2 run swarm_mavsdk_ros2 drone_command forward 2 --drone n
ros2 run swarm_mavsdk_ros2 drone_command backward 2 --drone n
```
Or, in python: 

```bash
python scripts/start_simulation.py --ros conda --n 3 --architecture edge:drone_0
```

# Swarm Setup - Drone Benchmarks

In terminal 1:
```bash
cd ~/drone_obstacle_course
source ~/miniconda3/etc/profile.d/conda.sh
conda activate drone-ros-jazzy
source install-conda/local_setup.bash
/usr/bin/python3 scripts/start_simulation.py --world swarm --n 3
```

In terminal 2: 
```bash
cd ~/drone_obstacle_course
source /opt/ros/jazzy/setup.bash
source install-system/local_setup.bash
ros2 run swarm_mavsdk_ros2 swarm_tests --n 3 --seed 42
```

Finally, enter the commands in the terminal from the list below. One can string together multiple tests at once (e.g. test 1, test 2, . . . ), with the drone reorienting itself along the "start line" of each new test. Test 1 represents a change in orientation (from along the x to along the y); test 2 represents a change from a random setup to a linear setup; test 3 represents a partition in drones given a signal such that the majority 

dynamic flag passage, in which the flag "orients" itself either left or right and the drone has to react to it in order to simulate dynamic on-board compute; test 3 represents a passage through a confined space. Results are saved in "results/tests/all_tests.csv".

```bash
startup
arm
takeoff
test 1
test 2
test 3
land
quit
```

# Setup Swarm - Drone Leader Command

In terminal 1, with drone_0 being the "command node" of the swarm:
```bash
cd ~/drone_obstacle_course
source ~/miniconda3/etc/profile.d/conda.sh
conda activate drone-ros-jazzy
source install-conda/local_setup.bash
/usr/bin/python3 scripts/start_simulation.py \
  --ros system --n 3 --architecture edge:drone_0
```

In terminal 2:
```bash
cd ~/drone_obstacle_course
source ~/miniconda3/etc/profile.d/conda.sh
conda activate drone-ros-jazzy
source install-conda/local_setup.bash
```

Then, with the drone_0 being the same as used previously:
```bash
ros2 run swarm_mavsdk_ros2 swarm_command startup --n 3 --architecture edge:drone_0

ros2 run swarm_mavsdk_ros2 swarm_command arm --n 3 --architecture edge:drone_0

ros2 run swarm_mavsdk_ros2 swarm_command takeoff --n 3 --architecture edge:drone_0


ros2 run swarm_mavsdk_ros2 swarm_command forward 1 --n 3 --architecture edge:drone_0

ros2 run swarm_mavsdk_ros2 swarm_command right 1 --n 3 --architecture edge:drone_0

ros2 run swarm_mavsdk_ros2 swarm_command formation --shape line --spacing 1.5 --n 3 --architecture edge:drone_0

ros2 run swarm_mavsdk_ros2 swarm_command land --n 3 --architecture edge:drone_0
```

Or, in python: 

```python
swarm() {
    python -m swarm_mavsdk_ros2.swarm_command \
        --n 3 --architecture edge:drone_0 "$@"
}

swarm startup
swarm arm
swarm takeoff


swarm forward 0.5
swarm backward 0.5
swarm left 0.5
swarm right 0.5
swarm up 0.5
swarm down 0.5
```

# Miscellaneous

As an aside, sometimes there are artificats of other simulation which cause the drone swarm to not instantiate in a Gazebo world, simply enter the following commands into terminal:

```bash
pkill -f px4
pkill -f "gz sim"
```

Although this might be redundant, these are all of the ros2 topics: 

```
/drone_0/armed
/drone_0/battery
/drone_0/command/position_ned
/drone_0/command/velocity_ned
/drone_0/command/yaw_rad
/drone_0/local_telemetry
/drone_0/offboard
/drone_0/peer_states_ned
/drone_0/started
/drone_0/state
/drone_0/swarm/positions_ned
/drone_0/swarm/request
/drone_0/swarm/results
/drone_1/armed
/drone_1/battery
/drone_1/command/position_ned
/drone_1/command/velocity_ned
/drone_1/command/yaw_rad
/drone_1/local_telemetry
/drone_1/offboard
/drone_1/peer_states_ned
/drone_1/started
/drone_1/state
/drone_2/armed
/drone_2/battery
/drone_2/command/position_ned
/drone_2/command/velocity_ned
/drone_2/command/yaw_rad
/drone_2/local_telemetry
/drone_2/offboard
/drone_2/peer_states_ned
/drone_2/started
/drone_2/state
/fmu/in/actuator_motors
/fmu/in/actuator_servos
/fmu/in/arming_check_reply_v1
/fmu/in/aux_global_position
/fmu/in/config_control_setpoints
/fmu/in/config_overrides_request
/fmu/in/distance_sensor
/fmu/in/fixed_wing_lateral_setpoint
/fmu/in/fixed_wing_longitudinal_setpoint
/fmu/in/goto_setpoint
/fmu/in/landing_gear
/fmu/in/lateral_control_configuration
/fmu/in/longitudinal_control_configuration
/fmu/in/manual_control_input
/fmu/in/message_format_request
/fmu/in/mode_completed
/fmu/in/obstacle_distance
/fmu/in/offboard_control_mode
/fmu/in/onboard_computer_status
/fmu/in/register_ext_component_request
/fmu/in/rover_attitude_setpoint
/fmu/in/rover_position_setpoint
/fmu/in/rover_rate_setpoint
/fmu/in/rover_speed_setpoint
/fmu/in/rover_steering_setpoint
/fmu/in/rover_throttle_setpoint
/fmu/in/sensor_optical_flow
/fmu/in/telemetry_status
/fmu/in/trajectory_setpoint
/fmu/in/unregister_ext_component
/fmu/in/vehicle_attitude_setpoint_v1
/fmu/in/vehicle_command
/fmu/in/vehicle_command_mode_executor
/fmu/in/vehicle_mocap_odometry
/fmu/in/vehicle_rates_setpoint
/fmu/in/vehicle_thrust_setpoint
/fmu/in/vehicle_torque_setpoint
/fmu/in/vehicle_visual_odometry
/fmu/out/airspeed_validated_v1
/fmu/out/arming_check_request_v1
/fmu/out/battery_status_v1
/fmu/out/collision_constraints
/fmu/out/estimator_status_flags
/fmu/out/failsafe_flags
/fmu/out/gimbal_device_attitude_status
/fmu/out/home_position_v1
/fmu/out/manual_control_setpoint
/fmu/out/message_format_response
/fmu/out/mode_completed
/fmu/out/position_setpoint_triplet
/fmu/out/register_ext_component_reply
/fmu/out/sensor_combined
/fmu/out/timesync_status
/fmu/out/transponder_report
/fmu/out/vehicle_attitude
/fmu/out/vehicle_command_ack
/fmu/out/vehicle_control_mode
/fmu/out/vehicle_global_position
/fmu/out/vehicle_gps_position
/fmu/out/vehicle_land_detected
/fmu/out/vehicle_local_position_v1
/fmu/out/vehicle_odometry
/fmu/out/vehicle_status_v1
/fmu/out/vtol_vehicle_status
/fmu/out/wind
/parameter_events
/px4_1/fmu/in/actuator_motors
/px4_1/fmu/in/actuator_servos
/px4_1/fmu/in/arming_check_reply_v1
/px4_1/fmu/in/aux_global_position
/px4_1/fmu/in/config_control_setpoints
/px4_1/fmu/in/config_overrides_request
/px4_1/fmu/in/distance_sensor
/px4_1/fmu/in/fixed_wing_lateral_setpoint
/px4_1/fmu/in/fixed_wing_longitudinal_setpoint
/px4_1/fmu/in/goto_setpoint
/px4_1/fmu/in/landing_gear
/px4_1/fmu/in/lateral_control_configuration
/px4_1/fmu/in/longitudinal_control_configuration
/px4_1/fmu/in/manual_control_input
/px4_1/fmu/in/message_format_request
/px4_1/fmu/in/mode_completed
/px4_1/fmu/in/obstacle_distance
/px4_1/fmu/in/offboard_control_mode
/px4_1/fmu/in/onboard_computer_status
/px4_1/fmu/in/register_ext_component_request
/px4_1/fmu/in/rover_attitude_setpoint
/px4_1/fmu/in/rover_position_setpoint
/px4_1/fmu/in/rover_rate_setpoint
/px4_1/fmu/in/rover_speed_setpoint
/px4_1/fmu/in/rover_steering_setpoint
/px4_1/fmu/in/rover_throttle_setpoint
/px4_1/fmu/in/sensor_optical_flow
/px4_1/fmu/in/telemetry_status
/px4_1/fmu/in/trajectory_setpoint
/px4_1/fmu/in/unregister_ext_component
/px4_1/fmu/in/vehicle_attitude_setpoint_v1
/px4_1/fmu/in/vehicle_command
/px4_1/fmu/in/vehicle_command_mode_executor
/px4_1/fmu/in/vehicle_mocap_odometry
/px4_1/fmu/in/vehicle_rates_setpoint
/px4_1/fmu/in/vehicle_thrust_setpoint
/px4_1/fmu/in/vehicle_torque_setpoint
/px4_1/fmu/in/vehicle_visual_odometry
/px4_1/fmu/out/airspeed_validated_v1
/px4_1/fmu/out/arming_check_request_v1
/px4_1/fmu/out/battery_status_v1
/px4_1/fmu/out/collision_constraints
/px4_1/fmu/out/estimator_status_flags
/px4_1/fmu/out/failsafe_flags
/px4_1/fmu/out/gimbal_device_attitude_status
/px4_1/fmu/out/home_position_v1
/px4_1/fmu/out/manual_control_setpoint
/px4_1/fmu/out/message_format_response
/px4_1/fmu/out/mode_completed
/px4_1/fmu/out/position_setpoint_triplet
/px4_1/fmu/out/register_ext_component_reply
/px4_1/fmu/out/sensor_combined
/px4_1/fmu/out/timesync_status
/px4_1/fmu/out/transponder_report
/px4_1/fmu/out/vehicle_attitude
/px4_1/fmu/out/vehicle_command_ack
/px4_1/fmu/out/vehicle_control_mode
/px4_1/fmu/out/vehicle_global_position
/px4_1/fmu/out/vehicle_gps_position
/px4_1/fmu/out/vehicle_land_detected
/px4_1/fmu/out/vehicle_local_position_v1
/px4_1/fmu/out/vehicle_odometry
/px4_1/fmu/out/vehicle_status_v1
/px4_1/fmu/out/vtol_vehicle_status
/px4_1/fmu/out/wind
/px4_2/fmu/in/actuator_motors
/px4_2/fmu/in/actuator_servos
/px4_2/fmu/in/arming_check_reply_v1
/px4_2/fmu/in/aux_global_position
/px4_2/fmu/in/config_control_setpoints
/px4_2/fmu/in/config_overrides_request
/px4_2/fmu/in/distance_sensor
/px4_2/fmu/in/fixed_wing_lateral_setpoint
/px4_2/fmu/in/fixed_wing_longitudinal_setpoint
/px4_2/fmu/in/goto_setpoint
/px4_2/fmu/in/landing_gear
/px4_2/fmu/in/lateral_control_configuration
/px4_2/fmu/in/longitudinal_control_configuration
/px4_2/fmu/in/manual_control_input
/px4_2/fmu/in/message_format_request
/px4_2/fmu/in/mode_completed
/px4_2/fmu/in/obstacle_distance
/px4_2/fmu/in/offboard_control_mode
/px4_2/fmu/in/onboard_computer_status
/px4_2/fmu/in/register_ext_component_request
/px4_2/fmu/in/rover_attitude_setpoint
/px4_2/fmu/in/rover_position_setpoint
/px4_2/fmu/in/rover_rate_setpoint
/px4_2/fmu/in/rover_speed_setpoint
/px4_2/fmu/in/rover_steering_setpoint
/px4_2/fmu/in/rover_throttle_setpoint
/px4_2/fmu/in/sensor_optical_flow
/px4_2/fmu/in/telemetry_status
/px4_2/fmu/in/trajectory_setpoint
/px4_2/fmu/in/unregister_ext_component
/px4_2/fmu/in/vehicle_attitude_setpoint_v1
/px4_2/fmu/in/vehicle_command
/px4_2/fmu/in/vehicle_command_mode_executor
/px4_2/fmu/in/vehicle_mocap_odometry
/px4_2/fmu/in/vehicle_rates_setpoint
/px4_2/fmu/in/vehicle_thrust_setpoint
/px4_2/fmu/in/vehicle_torque_setpoint
/px4_2/fmu/in/vehicle_visual_odometry
/px4_2/fmu/out/airspeed_validated_v1
/px4_2/fmu/out/arming_check_request_v1
/px4_2/fmu/out/battery_status_v1
/px4_2/fmu/out/collision_constraints
/px4_2/fmu/out/estimator_status_flags
/px4_2/fmu/out/failsafe_flags
/px4_2/fmu/out/gimbal_device_attitude_status
/px4_2/fmu/out/home_position_v1
/px4_2/fmu/out/manual_control_setpoint
/px4_2/fmu/out/message_format_response
/px4_2/fmu/out/mode_completed
/px4_2/fmu/out/position_setpoint_triplet
/px4_2/fmu/out/register_ext_component_reply
/px4_2/fmu/out/sensor_combined
/px4_2/fmu/out/timesync_status
/px4_2/fmu/out/transponder_report
/px4_2/fmu/out/vehicle_attitude
/px4_2/fmu/out/vehicle_command_ack
/px4_2/fmu/out/vehicle_control_mode
/px4_2/fmu/out/vehicle_global_position
/px4_2/fmu/out/vehicle_gps_position
/px4_2/fmu/out/vehicle_land_detected
/px4_2/fmu/out/vehicle_local_position_v1
/px4_2/fmu/out/vehicle_odometry
/px4_2/fmu/out/vehicle_status_v1
/px4_2/fmu/out/vtol_vehicle_status
/px4_2/fmu/out/wind
/rosout
```