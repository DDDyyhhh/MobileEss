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
CustomMsg-to-PointCloud2 adapter for Nav2's local obstacle layer, and Nav2.
The Livox driver itself must already be running and publishing `/livox/lidar`
and `/livox/imu`. The default launch does not open a CAN command publisher.

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

The relay consumes Nav2's final smoothed `/cmd_vel` output. It requires a current STM32 `0x507` confirmation of API state `1` before
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
