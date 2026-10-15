# Control validation — 2026-10-14

This is a historical report, not a fresh validation of an extracted archive.
Results directories below are local evidence and are excluded from source ZIPs.
Use the [run guide](run_and_share.md) and rerun validation on a new installation.

System ROS 2 Jazzy, PX4 SITL, Gazebo swarm world, three x500 vision vehicles,
headless execution; an additional isolated one-vehicle launch. Position reference speed: 0.4 m/s. Requested velocity speed:
0.4 m/s. Run the commands in [the validation guide](swarm_validity.md#position-and-velocity-flight-validation)
with the matching architecture and count.

## Automated regression

168 tests passed. Coverage includes position/velocity setpoint generation,
mode switches, fresh telemetry and timeout handling, fleet dispatch barriers,
central and edge ROS transport to all three members, RL action validation,
launch configuration, complete CSV messages, numeric telemetry flattening,
late columns, shrinking arrays, and exports during recording.

## Measured destination moves

Each move commanded drone zero 0.5 m North from its measured start and required
fresh armed OFFBOARD state, error at most 0.10 m, and speed at most 0.15 m/s.
Velocity completion captures a position hold. These measured speeds are outcomes,
not guarantees of the configured reference or velocity limit.

| Architecture | Mode | Samples | Final error (m) | Peak measured speed (m/s) |
| --- | --- | ---: | ---: | ---: |
| central | position | 415 | 0.0997 | 0.1260 |
| central | velocity | 144 | 0.0564 | 0.3610 |
| edge:drone_0 | position | 241 | 0.0997 | 0.1269 |
| edge:drone_0 | velocity | 157 | 0.0578 | 0.3951 |
| standalone (n=1) | position | 128 | 0.0938 | 0.1236 |
| standalone (n=1) | velocity | 66 | 0.0709 | 0.3947 |

Central sample CSVs:
`results/control_validation/20261014_195427_217463/`.
Edge sample CSVs:
`results/control_validation/20261014_200028_770072/`.
Standalone sample CSVs:
`results/control_validation/20261014_200400_409392/`.
The edge and standalone validators additionally exercised the single-drone forward/backward
helpers with 0.2 m moves in each mode.

## Fleet and recording checks

Both central and edge fleets completed 0.5 m forward and backward moves in
both modes, then landed all three vehicles and confirmed disarming. Fleet completion waits
for every member, including drone zero. The isolated one-vehicle run also
completed both destination and directional modes, then landed and disarmed.

The central launch recording is
`results/ros2/20261014_155308_ekwxdheo/20261014_195308_445140.csv`.
Its 2,464 columns include complete topic payloads and scalar/array fields.
Verified column coverage includes mode, architecture, position and velocity
goals, streamed PX4 setpoints, arming status, acceleration, and battery voltage.
The edge run's 2,479 columns are in
`results/ros2/20261014_160013_53idnc6q/20261014_200013_425240.csv`;
all three members' architecture, mode, targets, and reference-speed columns were
verified. The standalone run's 823 columns are in
`results/ros2/20261014_160342_yhrxbu8c/20261014_200342_102250.csv`.
Raw topic messages, numeric position samples, inventory, and durable timeline
spool are kept alongside the wide CSV.

Missing battery current/power remains blank in the route CSV; SITL did not
supply usable current. Receipt timestamps allow assessing freshness in the wide
CSV. Recording captures available ROS publications and cannot recover messages
sent before subscription discovery or dropped under load.

These checks establish destination and directional control in simulation. They
do not establish hardware flight performance or obstacle clearance. The full
three-benchmark matrix was not rerun as part of this control check.
