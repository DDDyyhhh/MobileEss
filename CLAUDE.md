# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Issue tracker and domain context

- Issues and specs live as GitHub issues. Follow `docs/agents/issue-tracker.md` and use the labels in `docs/agents/triage-labels.md`.
- Before changing navigation, localization, LiDAR, CAN, or safety behavior, read root `CONTEXT.md` and the relevant map/hardware documentation under `docs/`. If `docs/adr/` exists for the work being done, read the relevant ADRs too.
- `CONTEXT.md` is the authoritative system contract. It defines the fixed parking-lot scope, the TF chain, CAN boundary, map lifecycle, and STM32/RK3588 safety split.

## Development environment

- The local machine is an **editing-only** checkout. A Mutagen session (its `mutagen.yml` is not checked into this repo) synchronizes local `src/` to the RK3588 board workspace `~/MobileEss_ws/src` (board user `cat`, LAN address `192.168.10.81` per `src/mower_console/README.md`). Generated `build/`, `install/`, `log/`, `.vscode`, and Python cache files are not synchronized. Check `mutagen sync list` before relying on a newly edited file being on the board, and do not overwrite unreviewed board-side diffs.
- Run every build, ROS node, bag replay, and hardware check on the RK3588 through SSH. Host-side runs are limited to offline logic tests and the console frontend build (see below); never treat them as vehicle evidence.
- Commands below use the SSH host alias `rk3588`. Each starts a login shell, sources ROS 2 Humble, and uses the board overlay.

```bash
# Build the full board workspace
ssh rk3588 'bash -lc "
  cd ~/MobileEss_ws &&
  source /opt/ros/humble/setup.bash &&
  colcon build --symlink-install
"'

# Rebuild one changed package; source install/setup.bash before ros2 commands
ssh rk3588 'bash -lc "
  cd ~/MobileEss_ws &&
  source /opt/ros/humble/setup.bash &&
  colcon build --packages-select <package> --symlink-install &&
  source install/setup.bash
"'
```

Valid package names include `fast_lio`, `livox_ros_driver2`, `mcu_can_bridge`, `mobileess_rtk`, `mower_console`, `mower_nav2`, and `timed_dynamic_obstacle_layer`. `mower_console` installs its prebuilt `web/dist`, so rebuild the frontend before `colcon build` when the UI changed.

## Tests and lint

The repository does not define a single standalone lint launcher. Run the relevant package tests on the RK3588; packages that opt into `ament_lint_auto` are exercised through `colcon test`.

```bash
# Python contract/unit suites (all on RK3588)
ssh rk3588 'bash -lc "
  cd ~/MobileEss_ws &&
  source /opt/ros/humble/setup.bash &&
  source install/setup.bash &&
  python3 -m pytest \
    src/fast_lio/test \
    src/mcu_can_bridge/test \
    src/mobileess_rtk/test \
    src/mower_console/test \
    src/mower_nav2/test \
    src/timed_dynamic_obstacle_layer/test
"'

# Run one test file or one test case
ssh rk3588 'bash -lc "
  cd ~/MobileEss_ws &&
  source /opt/ros/humble/setup.bash &&
  source install/setup.bash &&
  python3 -m pytest \
    src/mcu_can_bridge/test/test_protocol.py::test_parse_50a_big_endian_signed_wheel_speeds
"'

# Build and run the C++ Nav2 plugin load test; inspect its result
ssh rk3588 'bash -lc "
  cd ~/MobileEss_ws &&
  source /opt/ros/humble/setup.bash &&
  colcon build --packages-select timed_dynamic_obstacle_layer --symlink-install &&
  source install/setup.bash &&
  colcon test --packages-select timed_dynamic_obstacle_layer &&
  colcon test-result --verbose
"'
```

Host-side offline checks (logic only, no ROS hardware):

```bash
PYTHONPATH=src/mobileess_rtk python3 -m pytest src/mobileess_rtk/test
PYTHONPATH=src/mcu_can_bridge python3 -m pytest src/mcu_can_bridge/test
PYTHONPATH=src/mower_console:src/mcu_can_bridge python3 -m pytest src/mower_console/test

# Console frontend (React + three.js, Vite/Vitest)
cd src/mower_console/web && npm ci && npm run build && npm test
```

`src/fast_lio/test/run_isolated_replay.sh` and `run_live_fastlio_observation.sh` are deployment diagnostics, not ordinary unit tests: they require their documented board environments, maps, and bags. Do not use `src/livox_ros_driver2/build.sh` for routine development: it deletes workspace build products and replaces package/launch files while selecting a ROS version.

## Runtime architecture

The vehicle is a ROS 2 Humble system on RK3588; STM32 retains motor execution and latched safety responsibility.

```text
Mid-360 CustomMsg + IMU
  -> livox_ros_driver2
  -> FAST-LIO (/fast_lio/odometry, /cloud_registered)
  -> fixed_map_localizer (sole map -> odom publisher)

STM32 CAN 0x50A four-wheel feedback
  -> mcu_can_bridge (/wheel/odometry/twist)
  -> local robot_localization EKF (sole odom -> base_link publisher)

Nav2 controller -> /cmd_vel_nav -> velocity_smoother -> /cmd_vel
  -> collision_gate -> /cmd_vel_gated -> cmd_vel_can_bridge -> STM32 0x005
```

- `livox_ros_driver2` publishes Mid-360 `CustomMsg` and IMU. Start it before real-sensor FAST-LIO:

  ```bash
  ssh rk3588 'bash -lc "
    cd ~/MobileEss_ws && source /opt/ros/humble/setup.bash && source install/setup.bash &&
    ros2 launch livox_ros_driver2 msg_MID360_launch.py
  "'
  ```

- `fast_lio` implements LIO mapping and fixed-PCD ICP localization. `fast_lio_real.launch.py` requires field-calibrated configuration via `FAST_LIO_REAL_CONFIG` or `/etc/mower/fast_lio/mid360_real.yaml`; its default `publish_tf:=false` preserves EKF ownership of `odom -> base_link`.
- `mcu_can_bridge` only consumes STM32 `0x50A` feedback and produces wheel odometry/diagnostics. `localization.launch.py` starts the CAN bridge and local EKF; global RTK nodes are disabled unless both `enable_global_localization:=true` and `rtk_extrinsics_configured:=true` are supplied.
- `mobileess_rtk` bridges UM982 NMEA/NTRIP to `/rtk/fix`, `/rtk/heading`, and diagnostics. Use a board-local RTK parameter file for serial and NTRIP values; do not commit `*.local.yaml` credentials.
- `mower_nav2` assembles fixed-map localization, Nav2, Livox obstacle adaptation, `TimedDynamicObstacleLayer`, and the final collision gate. Its global costmap is the reviewed static planning map; dynamic Mid-360 evidence belongs only in the local costmap.
- `timed_dynamic_obstacle_layer` is the C++ Nav2 plugin that stores short-lived local obstacle cells.
- `mower_console` is the browser console served from the RK3588 (`aiohttp` server in `server.py`, ROS bridge in `ros_adapter.py`, React frontend in `web/`). `console.launch.py` checks the fixed-map assets (`map.yaml/.pgm/.pcd`, `livox.json`, `mid360_real.yaml`) and starts two Smac lattice planner processes (forward and reverse, which cannot share one process because Humble caches the lattice table per file). One console task owns motion at a time, and each start permit is single-use. The systemd service always launches with `observation_only:=true`. Motion needs a separate board-local config with `motion_enabled`/`accepted_capabilities` plus an explicit `observation_only:=false`. Run records go to `~/MobileEss_data/console-runs/` on the board. Console field reports live in `docs/console/`.

## Runtime and safety contracts

- Preserve exactly one TF publisher per edge: `earth -> map -> odom -> base_link`. In the supported localization chain, the local EKF owns `odom -> base_link`, and `fixed_map_localizer` is the sole `map -> odom` owner. FAST-LIO must use `publish_tf:=false` in that chain; the RTK global EKF is observational and must not take over `map -> odom`.
- RK3588 must never send wheel control directly to OID nodes `1`–`4`. All high-level motion goes through STM32's existing `0x005` CAN boundary; STM32 owns motor control, collision/e-stop/tip safety, and control-frame timeout latching.
- Permanent PCD and approved 2D maps are immutable during normal operation. Create a new map version and review its PGM, boundary, keep-out, and speed-zone assets before deploying it. See `docs/maps/` for capture and fixed-map validation procedures.
- Keep real-vehicle bringup observation-only by default. The Nav2 launch defaults to `enable_motion:=false` and `safety_map_approved:=false`; set **both** true only after the safety map and STM32 `0x005` handoff have been validated at low speed.
- Before a live localization or map-capture check, stop existing command, CAN relay, Nav2, TF/EKF, unlock, and blade publishers as required by the relevant procedure. On a failed field check, command zero speed/stop safely and retain the rosbag, diagnostics, and CAN evidence for investigation.
