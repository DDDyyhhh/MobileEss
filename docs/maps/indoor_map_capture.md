# Indoor Map Capture

## Save Rule

`map_capture.launch.py` does not write a PCD on `Ctrl+C`. It enables the
`/map_save` service and sets its destination. Keep FAST-LIO running, call the
service after the manual route is complete, verify its success response and the
file, then stop the launch.

## Controlled Capture

This is an observation-only mapping procedure. Stop Nav2, every vehicle command
publisher, and the fixed-map localizer before capture. Do not enable motion from
the Nav2 launch during this procedure.

In terminal 1, prepare the destination and start the real Mid-360 mapping node:

```bash
source /opt/ros/humble/setup.bash
source ~/MobileEss_ws/install/setup.bash

sudo install -d -o "$USER" -g "$USER" /var/lib/mower/maps/indoor_map

ros2 launch fast_lio map_capture.launch.py \
  config_file:=/home/cat/MobileEss_ws/src/fast_lio/config/mid360.yaml \
  map_file_path:=/var/lib/mower/maps/indoor_map/map.pcd
```

`map_capture.launch.py` fixes the capture odometry frame at `map_capture_odom`
and uses `/livox/lidar`; it does not accept `odom_frame` or `source_topic`
launch arguments. Wait for both `IMU Initial Done` and `Initialize the map
kdtree` before starting the manual route.

Drive manually and slowly through the intended indoor test area. Cover walls,
doorways, pillars, corners, and the intended start and goal areas. Avoid abrupt
point turns, collisions, people, and moving doors. Return near the initial pose
before saving when feasible.

In terminal 2, while terminal 1 is still running, confirm that the service is
available and save exactly once:

```bash
source /opt/ros/humble/setup.bash
source ~/MobileEss_ws/install/setup.bash

ros2 service type /map_save
ros2 service call /map_save std_srvs/srv/Trigger '{}'
ls -lh /var/lib/mower/maps/indoor_map/map.pcd
```

Expected response:

```text
success: true
message: Map saved.
```

Only after the service reports success and `ls` finds a non-empty PCD should
terminal 1 be stopped with `Ctrl+C`.

## Review And Convert

Check the PCD before using it for Nav2. The generated 2D map is a candidate
asset only and does not establish the indoor operating boundary, keep-out zones,
or speed zones.

```bash
python3 ~/MobileEss_ws/src/mower_nav2/scripts/pcd_to_occupancy.py \
  /var/lib/mower/maps/indoor_map/map.pcd \
  --output-dir ~/MobileEss_ws/src/mower_nav2/maps/indoor_map \
  --resolution 0.10 \
  --padding 1.0 \
  --obstacle-height 0.15 \
  --z-min 0.15 \
  --z-max 0.80 \
  --min-obstacle-points 5 \
  --plane-distance-threshold 0.08 \
  --plane-iterations 1000 \
  --ground-seed-percentile 25
```

Inspect `maps/indoor_map/map.pgm` in RViz. Do not overwrite the outdoor
`mower_nav2/maps/map.yaml` and `map.pgm` until the indoor map has been reviewed
and the matching map-capture initial pose is known.

## Observation-Only Nav2 Test

Use the new candidate map only with the fixed-map localizer pointed at its
matching PCD and the capture coordinate system:

```bash
ros2 launch mower_nav2 mower_navigation.launch.py \
  map:=/home/cat/MobileEss_ws/src/mower_nav2/maps/indoor_map/map.yaml \
  map_pcd:=/var/lib/mower/maps/indoor_map/map.pcd \
  initial_x_m:=0.0 initial_y_m:=0.0 initial_yaw_rad:=0.0 \
  fast_lio_config_file:=/home/cat/MobileEss_ws/src/fast_lio/config/mid360.yaml \
  enable_motion:=false
```

Use the same physical start pose and heading as the capture. First observe
`/fixed_map_localization/diagnostics`; only proceed with RViz planning after it
reports accepted alignment and never when it reports `heartbeat_stale`.

## Verification Record

The indoor-map capture and observation-only planning chain was verified after
this procedure:

- `/map_save` successfully produced a non-empty
  `/var/lib/mower/maps/indoor_map/map.pcd` containing wall and pillar features.
- The conversion produced
  `src/mower_nav2/maps/indoor_map/map.yaml` and `map.pgm` at 0.10 m resolution.
- `mower_navigation.launch.py` loaded the matching PCD and 2D map, and
  `fixed_map_localizer` reported `accepted` alignment indoors.
- An RViz `2D Goal Pose` produced a green global plan using the indoor map.

This verification approves observation-only localization and planning of this
map pair. It does not approve `enable_motion:=true`: the indoor operating
boundary, safety buffer, keep-out and speed-zone assets, plus the STM32 `0x005`
command-chain validation, remain required before vehicle-motion testing.
