# Position and velocity controls

The active control interface is `ros2/swarm_mavsdk_ros2`. Position control
specifies a destination; velocity control specifies motion in metres per second.
Commands below run in Terminal 2 from the project root after launcher readiness.
For system ROS, source `/opt/ros/jazzy/setup.bash` and
`install-system/local_setup.bash`; for Conda use the active RoboStack environment
and `install-conda/local_setup.bash`.
See [run and share](run_and_share.md) for launch and environment setup.

## Coordinates and units

| Component | Positive direction | Negative direction | Position units | Velocity units |
| --- | --- | --- | --- | --- |
| North / X | Forward | Backward | m | m/s |
| East / Y | Right | Left | m | m/s |
| Down / Z | Descend | Climb | m | m/s |

These are fixed local NED axes: turning the drone does not change forward.
Absolute position targets use the PX4 local origin, not the current position,
Gazebo world coordinates, or terrain height. A down target of -2 means 2 m above
that origin. ROS consumers expecting ENU must explicitly convert these values.

## Prepare the drone

Run these in order and wait for each command to complete:

```bash
ros2 run swarm_mavsdk_ros2 drone_command startup --drone 0
ros2 run swarm_mavsdk_ros2 drone_command arm --drone 0
ros2 run swarm_mavsdk_ros2 drone_command takeoff --drone 0
```

Startup streams a hold target. Arming is explicit. Takeoff waits for the
configured altitude and armed OFFBOARD telemetry. Lifecycle and yaw commands
retain their dedicated behavior; they do not accept movement mode/speed options.

## Position: dictate a destination

```bash
ros2 run swarm_mavsdk_ros2 drone_command position --drone 0 --ned 2 0 -2
```

Go to North 2 m, East 0 m, Down -2 m, then hold. Repeating this command gives
the same destination. This direct command returns after dispatch, before arrival.
The drone node continues streaming the target after the CLI process exits.

For a destination relative to the latest measured position:

```bash
ros2 run swarm_mavsdk_ros2 drone_command forward 2 --drone 0 --mode position
```

Move another 2 m North and wait for a settled arrival. Position is the default
mode for directional moves. The drone node advances its position reference at
`position_speed_mps` (0.4 m/s by default), keeping it close to the measured
position. This limits reference motion; measured speed still depends on PX4
tracking. `--speed` selects the velocity-mode limit; configure the node's
`position_speed_mps` parameter for position mode.

## Velocity: dictate motion

```bash
ros2 run swarm_mavsdk_ros2 drone_command velocity --drone 0 --ned 0.5 0 0
```

Request 0.5 m/s North with zero East and Down velocity. Each value is a signed
axis component, so `--ned 0.3 0.4 0` requests a horizontal speed of 0.5 m/s.
The direct command returns after dispatch. Its target remains active until
replaced or stopped; exiting the CLI does not stop the drone.

```bash
ros2 run swarm_mavsdk_ros2 drone_command stop --drone 0
```

`stop` captures and holds the current position. A direct `velocity --ned 0 0 0`
requests zero velocity; it does not capture a position hold. Actual motion and
braking depend on the flight controller and vehicle dynamics.

For a distance-limited move:

```bash
ros2 run swarm_mavsdk_ros2 drone_command forward 2 --drone 0 --mode velocity --speed 0.4
```

The single-drone helper tracks the destination with position-error feedback,
caps the velocity vector at the selected speed, slows near arrival, checks fresh
armed OFFBOARD telemetry, and captures a position hold after settling. Sideways
drift does not count as completing a forward move. Timeouts and interruptions
request a hold. The fleet directional helper checks progress along the requested
axis, switches each completed member to its destination position target, and
waits for the entire fleet to settle. These helpers differ from feedback-controlled
formation and test routes described below.

## Interactive console

```bash
ros2 run swarm_mavsdk_ros2 control_console --drone 0
```

Enter each line separately:

```text
startup
arm
takeoff
position 2 0 -2
velocity 0.2 0 0
stop
forward 1 position
right 1 velocity 0.3
stop
land
quit
```

Direct position/velocity commands set targets immediately; allow time to observe
the result before entering the next line. Console directional syntax is
`direction distance [position|velocity] [speed_mps]`. Console `test 1`, `test 2`,
and `test 3` use defaults; select test mode and speed through the CLI or Python.
Quitting the console does not land or cancel an active velocity target.

## Fleet commands and formation routes

Use the count that was launched. Prepare with `swarm_command startup --n 3`,
then `arm --n 3`, then `takeoff --n 3` before these examples:

```bash
ros2 run swarm_mavsdk_ros2 swarm_command velocity --n 3 --ned 0.2 0 0
ros2 run swarm_mavsdk_ros2 swarm_command stop --n 3
ros2 run swarm_mavsdk_ros2 swarm_command forward 2 --n 3 --mode velocity --speed 0.4
ros2 run swarm_mavsdk_ros2 swarm_command formation --n 3 --shape line --mode velocity --speed 0.4
ros2 run swarm_mavsdk_ros2 swarm_command translate --n 3 --ned 0 1 0 --mode velocity --speed 0.4
```

Swarm velocity moves default to 1.0 m/s, including formations, translations,
and swarm tests. Override this with `--speed` or Python `speed_mps`.


A broadcast `position --ned ...` applies the same numbers in each drone's own
local frame; it does not specify one common-world destination. The shared vision simulation reports positions in common world NED. Formation,
`go_to`, and translation behaviors use that frame directly.
Translation's `--ned` is a common-frame offset in metres.

Formation, translation, randomization, and splitting accept `--mode` and
`--speed`. In velocity mode these routes use position-error feedback, cap the
velocity vector's magnitude, slow near the destination, and hold on completion.
Speed is a maximum, not a promise of constant measured speed. Existing path
separation checks and staged routing still apply. Position mode uses the node’s slow position reference (0.4 m/s by default). Add `--architecture edge:drone_0` to fleet commands when using that
architecture; the same movement options are forwarded to the leader.

Wait for each distance move or route to finish before sending another. Direct
velocity targets intentionally remain active and can be replaced with `stop`.

## Obstacle tests and swarm benchmarks

```bash
# Single drone: launch the obstacle world first.
ros2 run swarm_mavsdk_ros2 drone_command test 1 --drone 0 --mode velocity --speed 0.4
# Fleet: launch the swarm world and prepare the airborne fleet first.
ros2 run swarm_mavsdk_ros2 swarm_command test 1 --n 3 --mode velocity --speed 0.4
```

Both test APIs accept `mode="position"` or `mode="velocity"` and `speed_mps`.
Position remains the default. Velocity routes track position references with
bounded feedback and settle at the final destination. Single-drone tests 1 and
2 use a moving sinusoidal reference; their reference timing also uses the
selected speed. Position tracking uses the slow reference and remains subject to PX4 tracking limits.
See [benchmark details](swarm_benchmarks.md) for setup and CSV interpretation.
Interactive test menus retain their default mode; use the commands above or
Python to select mode and speed.

## Python and RL

With an initialized client, use:

```python
client.move_to(2., 0., -2.)             # absolute position; dispatch only
client.set_velocity(.2, 0., 0.)        # continuous NED velocity in m/s
client.stop()                         # capture a position hold
client.move_to_and_wait(2., 0., -2., mode="velocity", speed_mps=.4)
client.run_test("test 1", mode="velocity", speed_mps=.4)
swarm.formation("line", mode="velocity", speed_mps=.4)
```

`Drone` and `Swarm` wrappers expose `move_to`, `set_velocity`, `stop`, and
relative direction helpers. For distinct local targets, use `swarm.drone(i)`.
`DroneCommandClient.track_position(..., mode="velocity", speed_mps=.4)` sends
one feedback update; call repeatedly to track a moving reference.
`move_to_and_wait` handles repeated updates, defaults to settled arrival, and
holds on a completed velocity-mode destination. `settle=False` leaves motion
active for intermediate route points.

RL defaults remain normalized velocity actions: each finite component is clipped
to [-1, 1] and multiplied by `max_speed_mps`. That bound is per axis, so diagonal
speed can exceed `max_speed_mps`. Position actions are absolute local NED metres,
without normalization or clipping:

```python
drone_rl.apply_position_action([2., 0., -2.])
swarm_rl.apply_position_actions({0: [2., 0., -2.], 1: [2., 1., -2.]})
drone_rl.step([2., 0., -2.], duration_s=.1, mode="position")
swarm_rl.step({0: [2., 0., -2.], 1: [2., 1., -2.]}, mode="position")
```

Use exactly one action per fleet member. Position targets remain active after a
position-mode step. Velocity-mode steps send zero velocity after the interval.
Fleet actions are validated before any member receives a target. Raw RL position
actions bypass formation path planning and separation checks.

Both architectures use the same per-drone position and velocity interfaces.
Central commands run in the external controller; `edge:drone_<id>` forwards the
same arguments to the embedded controller in that leader. One-shot single-drone
position and velocity commands wait for the drone service and target subscriber,
then service DDS before exiting.

For CSV coverage and validation commands, see
[recording and validation](swarm_validity.md#csv-results).

Takeoff streams the full altitude goal until the climb reaches that altitude;
subsequent position movement resumes the slow reference. This permits PX4’s
takeoff ramp to complete.

Set the position reference speed for every member at launch, for example:

```bash
/usr/bin/python3 scripts/start_simulation.py --world swarm --n 3 --position-speed 0.3
```

For a direct ROS launch, use `position_speed_mps:=0.3`.
