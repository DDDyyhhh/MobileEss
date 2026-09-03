# Mower Nav2

This package provides a fixed-site Nav2 integration for the parking-lot map
asset `parking_lot_2026-08-10_v2`.

## Map Generation

Generate the candidate two-dimensional map from the approved PCD:

```bash
python3 src/mower_nav2/scripts/pcd_to_occupancy.py \
  /var/lib/mower/maps/parking_lot_2026-08-10_v2/map.pcd \
  --output-dir src/mower_nav2/maps \
  --z-min 0.08 \
  --z-max 0.80
```

The checked-in `maps/map.yaml` and `maps/map.pgm` use `0.10 m` resolution.
For an existing candidate PGM, inspect isolated static cells before approving
it for global inflation. Generate a non-destructive cleaned candidate with:

```bash
python3 src/mower_nav2/scripts/clean_occupancy_components.py \
  src/mower_nav2/maps/indoor_map/map.pgm \
  /tmp/parking_map_clean_candidate.pgm \
  --min-component-size 5 --connectivity 8
```

Review the candidate in RViz against the original PGM and the physical site;
this command must not overwrite the approved map automatically.

The conversion uses deterministic RANSAC to fit the ground plane, then identifies
a static obstacle only when at least five returns in a cell are `0.15 m` to
`0.80 m` above that plane. This rejects ground noise and overhead structure
without spreading obstacle support into adjacent cells. It is a candidate planning map, not a replacement for the
fixed PCD localization map. Inspect it in RViz and add the parking-lot boundary,
keep-out zones, and speed zones before commanding vehicle motion.

The converter samples RANSAC ground candidates only from the lowest `25%` of
map-frame Z values, then refines the horizontal ground plane from inliers within
`0.08 m`. It counts an obstacle return only when its signed height above that
plane is within `0.15 m` to `0.80 m` by default. This excludes ground noise and
overhead roofs or frames without depending on the map-frame Z origin. Use
`--z-min`, `--z-max`, `--plane-distance-threshold`, `--plane-iterations`, and
`--ground-seed-percentile` to override it after reviewing the PCD.

The fixed-map localizer publishes `map -> odom` at `5 Hz`. RPP, Nav2 costmaps,
behaviors, and the BT navigator use a `0.5 s` TF lookup timeout so they can wait
for the next heartbeat and interpolate it; this is not permission to use a
stale localization transform. The localizer still stops TF refresh and reports
`heartbeat_stale` after its independent `2.5 s` safety limit.

## Controlled Bringup

The launch starts the local EKF (`odom -> base_link`), real FAST-LIO with TF
disabled, the fixed-map localizer as the sole `map -> odom` owner, a Livox
costmap-evidence adapter, and Nav2. The adapter leaves FAST-LIO's
`/livox/lidar` `CustomMsg` unchanged and publishes one local Nav2 stream:

- `/nav2/mid360_obstacles`: ground-relative, clustered obstacle marks.

The local costmap consumes that stream through the local-only
`TimedDynamicObstacleLayer`. It records marks in 5 cm `odom` cells and expires
a cell `0.80 s` after its last observation, independently of sensor raytrace
geometry. Re-observation refreshes the cell immediately. The layer owns only
its transient local grid: it neither writes nor publishes `/map`, the PGM,
static layer, boundaries, keep-outs, or the global costmap. This replaces the
former fixed-origin synthetic clearing rays, which field testing proved cannot
cover every sparse, static-vehicle human-outline cell. Nav2 evidence messages
use converter receipt time by default: the baseline showed that raw Livox
headers arrived `0.236-0.549 s` behind, which causes a rolling local costmap to
query an obsolete sensor pose. This does not alter the raw Livox message
supplied to FAST-LIO. The global costmap deliberately uses only the reviewed
static planning map and inflation; it never consumes live Mid-360 data. The
current active PGM has many singleton static cells, so a
cleaned candidate PGM must be generated and reviewed before deployment. The
Livox driver
itself must already be running and
publishing `/livox/lidar` and `/livox/imu`. The default launch does not open a
CAN command publisher.

```bash
ros2 launch mower_nav2 mower_navigation.launch.py \
  fast_lio_config_file:=/etc/mower/fast_lio/mid360_real.yaml
```

Only after the `0x005` API handoff and low-speed command tests pass, use the
explicit opt-in to start the STM32 command relay:

```bash
ros2 launch mower_nav2 mower_navigation.launch.py \
  enable_motion:=true safety_map_approved:=true
```

The local MPPI controller is responsible for dynamic-obstacle avoidance;
`/plan` remains a static global path and may geometrically cross a transient
obstacle. In ROS 2 Humble MPPI publishes its selected local result only through
the `Optimal Trajectory` namespace in `/trajectories` when `visualize` is
enabled; `/local_plan` is not a Humble MPPI output. MPPI's obstacle critics
consume the inflated local costmap. Its `ObstaclesCritic` potential-field
parameters match the local `inflation_radius: 0.65` and
`cost_scaling_factor: 5.0`.

RK3588 uses a 5 Hz MPPI controller budget. Humble requires controller period
`<= model_dt`, so `controller_frequency: 5.0` and `model_dt: 0.20` match. The
13 model steps preserve a `2.6 s` prediction horizon while reducing per-cycle
trajectory work; the 20 Hz velocity smoother and final Collision Gate remain
independent.

For a supervised MPPI diagnosis, keep `enable_motion:=false` and first record a
no-marker performance baseline:

```bash
src/mower_nav2/scripts/capture_mppi_observation.sh
```

The script refuses to run while `/cmd_vel_can_bridge` exists, records a
20-second rosbag and `controller_top.txt` under `~/mppi_observations/`, and
records the actual MPPI parameter values plus the raw controller topic
`/cmd_vel_nav`. Confirm the controller baseline before candidate tuning:
`/cmd_vel_nav` should approach 5 Hz and controller deadline-miss warnings must
be absent or rare. The final `/cmd_vel` remains the smoother's 20 Hz output and
is not a controller-rate measurement.

Only after that baseline, make a separate active-goal candidate capture. Because
`enable_motion:=false` deliberately prevents progress and the configured
progress checker aborts after 15 seconds, the script requires a shorter marker
capture:

```bash
src/mower_nav2/scripts/capture_mppi_observation.sh \
  ~/mppi_observations 10 visualize
```

It starts rosbag and verifies that `FollowPath.visualize` became true, then
waits for the first non-hidden MPPI `/cmd_vel_nav` output; only then does its
10-second capture interval start. Send the navigation goal when prompted.
add `/trajectories` as a `MarkerArray` and compare its `Candidate Trajectories`
and `Optimal Trajectory` namespaces with `/transformed_global_plan`,
`/local_costmap/costmap_raw`, `/cmd_vel_nav`, `/cmd_vel_gated`, and
`/collision_gate/diagnostics` at the obstacle timestamp. Humble MPPI does not
publish `/local_plan`. Do not tune critics
until that capture identifies whether bypass candidates are absent, rejected by
costmap geometry, or merely out-scored by path-following critics.

A fail-closed Collision Gate follows Nav2's velocity smoother:
`controller_server -> /cmd_vel_nav -> velocity_smoother -> /cmd_vel ->
collision_gate -> /cmd_vel_gated -> cmd_vel_can_bridge`. It predicts the
current circular vehicle footprint for two seconds against the raw local
costmap, and outputs zero velocity for a lethal/inscribed collision, a
footprint leaving the local window, or stale command/costmap/odometry input.
The relay consumes only `/cmd_vel_gated`. It requires a current STM32 `0x507` confirmation of API state `1` before
requesting state `2`, requires a current state `2` confirmation before relaying
nonzero commands, sends a complete `0x005` command frame every `50 ms`, and
clamps output to `0.7 m/s` and `0.5 rad/s`. A stale Nav2 command or stale STM32
status produces zero speed. It also requires a fixed-map localizer `accepted`
diagnostic within `2.5 s`. It never uses the `magic_key` safety-unlock field.
After API run is confirmed, a brief loss of `0x507` telemetry still produces
zero speed, but the relay continues requesting `ctrl_mode=2`; it never restarts
the handshake by transmitting `ctrl_mode=1`.
If STM32 reports a non-`2` API state after it has confirmed API run, the relay
interprets it as remote takeover, stops transmitting `0x005`, and remains
latched until the relay is restarted. This preserves the STM32's handoff to the
operator instead of attempting another API handshake.

`nav2_bringup` and the Nav2 plugins declared in `package.xml` must be installed
on the RK3588 before launch. They are not installed in this development image.
