# Agent Notes

## Workspace

- This is a ROS 2 Humble `colcon` workspace; the maintained packages are `src/fast_lio`, `src/livox_ros_driver2`, and `src/mobileess_rtk`.
- Read `CONTEXT.md` before changing navigation, localization, sensor, CAN, or safety behavior; read any relevant ADR if `docs/adr/` is added later.
- Issues and specs live under `.scratch/<feature>/`; follow `docs/agents/issue-tracker.md` and use the canonical labels in `docs/agents/triage-labels.md`.
- `build/`, `install/`, and `log/` are generated workspace artifacts and are ignored or locally generated; do not treat them as source of truth.

## Build And Test

- From the repository root, source ROS and build with `source /opt/ros/humble/setup.bash && colcon build --symlink-install`, then source `install/setup.bash` before running packages.
- Build one package with `colcon build --packages-select fast_lio`, `colcon build --packages-select livox_ros_driver2`, or `colcon build --packages-select mobileess_rtk`.
- Run the Python tests without requiring a ROS launch with `python3 -m pytest src/mobileess_rtk/test src/fast_lio/test`.
- The focused FAST-LIO contract tests are `python3 -m pytest src/fast_lio/test/test_real_runtime_contract.py src/fast_lio/test/test_mapping_safety.py src/fast_lio/test/test_avia_preprocess_differential.py`.
- `src/fast_lio/test/run_isolated_replay.sh` and `run_live_fastlio_observation.sh` are diagnostic-only runtime checks; they require the deployed `/opt/ros/humble` and `/opt/mower_ws` environments and external runtime files.

## Runtime Boundaries

- `fast_lio/launch/mapping.launch.py` is the simulation/default path and uses checked-in configs; `fast_lio_real.launch.py` is the real-sensor path and requires `FAST_LIO_REAL_CONFIG` or `/etc/mower/fast_lio/mid360_real.yaml` with field-calibrated extrinsics.
- Real FAST-LIO consumes Livox `CustomMsg` plus IMU input; do not substitute the simulation topics/config when validating real hardware.
- `mobileess_rtk` launches `rtk_node` with `um982.example.yaml` by default; use a local parameter file for hardware/NTRIP settings and never commit credentials or `*.local.yaml` secrets.
- `src/livox_ros_driver2/build.sh` is destructive: it removes workspace `build/`, `devel/`, and `install/`, swaps package/launch files, and then builds. Use it only when intentionally rebuilding the Livox driver for the selected ROS version.
- Do not start real-sensor or vehicle runtime checks with command, TF, EKF, Nav2, CAN, unlock, or blade publishers already active; the live observation script explicitly rejects those conditions.

## Safety Contracts

- Preserve the hardware/software safety split in `CONTEXT.md`: STM32 owns low-level motor execution and latched safety logic; RK3588/ROS publishes high-level behavior and must not bypass the STM32 CAN boundary.
- Preserve the established TF semantics `earth -> map -> odom -> base_link`, the fixed parking-lot boundary, and the rule that runtime does not modify permanent maps.
