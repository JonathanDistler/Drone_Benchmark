# Validated environment snapshot

Recorded on this installation on 2026-09-30. This is a historical snapshot;
see [installation and sharing](../docs/run_and_share.md) for current run commands
and [the October control report](../docs/control_validation.md) for later checks.
Referenced flight logs are excluded from source archives.

- System ROS overlay: 54 unit tests passed; one-drone SITL flight passed.
- Conda ROS overlay: 54 unit tests passed; three-drone SITL flight passed.
- Both flight runs checked explicit startup and arming, takeoff to 2 m,
  six 0.5 m position moves, hold, measured touchdown, and disarming.
- Conda `pip check` passed. A subsequent unarmed simulation checked clean
  ROS shutdown and removal of the launcher's PX4 socket.
- All 59 message types listed in the PX4 DDS topic configuration matched
  the local message definitions, ignoring comments and whitespace.

Flight evidence is in `../results/startup/validation-one.log` and
`../results/startup/validation-three-conda.log`. Unit results are in
`../results/startup/tests-system.log` and `../results/startup/tests-conda.log`.
Startup service logs are overwritten on the next launch.

`conda-linux-64.txt` records exact Conda package URLs for Linux x86_64.
`environment-resolved.yml` records resolved dependencies and pip requirements;
its channel order follows `../environment.yml` and excludes defaults.
`pip-resolved.txt` is an inventory, including Conda-managed Python packages
whose metadata can contain build-machine file URLs. Do not install that entire
inventory with pip; use the Conda exports and the YAML's pip section.
Exports were generated without system ROS or workspace overlays on PYTHONPATH.

The remaining files record Gazebo packages, PX4 commit, agent binary checksum,
agent source tag/commit, and agent CMake build options. The agent source is
tagged v2.4.3. `px4-msgs-source.tar.gz` preserves the message sources because
the workspace has no usable px4_msgs Git metadata. Preserve the project and
PX4 source trees too; these exports do not replace those sources or the native
PX4/Gazebo/agent installations.

Jazzy's `launch_testing` plugin requires pytest below 9 on this installation.
The environment specification now pins that compatibility constraint; the
validated Conda environment uses pytest 8.4.2.
