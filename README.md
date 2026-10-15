# Drone_Benchmark
MagpieMark: A Novel Sim-to-Real Benchmarking System for Single and Mutli-UAV Test-Beds

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

bash scripts/start_single_drone_ros2.sh blank
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

Or, for velocity controlled movement, with the first float representing the distance and the second float representing the speed in m/s: 

```bash
forward 0.5 velocity 0.4
right 0.5 velocity 0.4
```

# Setup Single Drone Test 
First, source the environment in the miniconda virtual environment in terminal 1, which requires the specific .STEP files I had generated : 

```bash
cd ~/drone_obstacle_course

source ~/miniconda3/etc/profile.d/conda.sh

conda activate drone-ros-jazzy

source install-conda/local_setup.bash

bash scripts/start_single_drone_ros2.sh test
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

python scripts/start_simulation.py --ros conda --world swarm --n 3
```

In terminal 2, 

```bash
cd ~/drone_obstacle_course

source /opt/ros/jazzy/setup.bash

conda activate drone-ros-jazzy

source install-system/local_setup.bash
```

Then, following a similar startup sequence to the previous single-drone test, with the integers following the primitve commands (e.g. forward) represents the distance:

```bash
ros2 run swarm_mavsdk_ros2 swarm_command startup --n 3

ros2 run swarm_mavsdk_ros2 swarm_command arm --n 3

ros2 run swarm_mavsdk_ros2 swarm_command takeoff --n 3

ros2 run swarm_mavsdk_ros2 swarm_command forward 1 --n 3 --mode position

ros2 run swarm_mavsdk_ros2 swarm_command backward 1 --n 3 --mode position

ros2 run swarm_mavsdk_ros2 drone_command right 1 --drone 0 --mode position

ros2 run swarm_mavsdk_ros2 drone_command left 1 --drone 0 --mode position

ros2 run swarm_mavsdk_ros2 drone_command up 1 --drone 0 --mode position

ros2 run swarm_mavsdk_ros2 drone_command down 1 --drone 0 --mode position

ros2 run swarm_mavsdk_ros2 swarm_command stop --n 3

ros2 run swarm_mavsdk_ros2 swarm_command land --n 3
```

There are also different functionalities that string together primitives for more complex shapes, with the spacing representing the distance between each drone:

```bash
ros2 run swarm_mavsdk_ros2 swarm_command formation --n 3 --shape line --spacing 1.5 --mode velocity --speed 0.3

ros2 run swarm_mavsdk_ros2 swarm_command formation --n 3 --shape grid --spacing 3 --mode position

ros2 run swarm_mavsdk_ros2 swarm_command formation --n 3 --shape circle --spacing 3 --mode position

ros2 run swarm_mavsdk_ros2 swarm_command translate --n 3 --ned 0 1 0 --mode velocity --speed 0.3
```

Then, for commanding specific drones from the swarm in terminal, with n being defined as the drone id (e.g. 0, 1, 2)

```bash
ros2 run swarm_mavsdk_ros2 swarm_command velocity --n 3 --ned 0.2 0 0

ros2 run swarm_mavsdk_ros2 swarm_command stop --n 3
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
     --world swarm --n 3 --architecture edge:drone_0
```

In terminal 2:
```bash
cd ~/drone_obstacle_course

source /opt/ros/jazzy/setup.bash

source install-system/local_setup.bash
```

Then, with the drone_0 being the same as used previously:
```bash

swarm() {
    ros2 run swarm_mavsdk_ros2 swarm_command \
        --n 3 --architecture edge:drone_0 "$@"
}

swarm startup

swarm arm

swarm takeoff


swarm forward 1

swarm backward 1

swarm left 1

swarm right 1

swarm up 1

swarm down 1

swarm stop
```

# Miscellaneous

As an aside, sometimes there are artificats of other simulation which cause the drone swarm to not instantiate in a Gazebo world, simply enter the following commands into terminal:

```bash
pkill -f px4
pkill -f "gz sim"
```
