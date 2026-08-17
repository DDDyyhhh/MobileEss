# mcu_can_bridge

ROS 2 SocketCAN bridge for the STM32 `0x50A` four-wheel actual speed feedback.

## Interface

- Input: Linux SocketCAN `can0`, standard CAN ID `0x50A`, DLC 8
- Encoding: four big-endian signed `int16`, unit `mm/s`, ordered `FL FR RL RR`
- Track width: `0.690 m`
- Output: `/wheel/odometry/twist`
- Output type: `geometry_msgs/msg/TwistWithCovarianceStamped`
- Output frame: `base_link`
- Diagnostics: `/wheel/diagnostics`

The bridge does not transmit CAN frames or take control of the chassis. It only
consumes STM32 feedback from the existing CAN boundary.

The wheel observation is disabled after `150 ms` without a valid `0x50A` frame.
No zero or stale velocity is published during a timeout. After recovery, three
valid frames are required before publication resumes.

## Build

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select mcu_can_bridge --symlink-install
source install/setup.bash
```

The localization launch also requires the ROS 2 Humble
`robot_localization` package.

## Wheel-only Test

```bash
ros2 run mcu_can_bridge mcu_can_bridge --ros-args \
  -p interface:=can0 -p track_width_m:=0.690
ros2 topic echo /wheel/odometry/twist
ros2 topic hz /wheel/odometry/twist
ros2 topic echo /wheel/diagnostics
```

## Local EKF

After installing `ros-humble-robot-localization`:

```bash
ros2 launch mcu_can_bridge localization.launch.py can_interface:=can0
```

This publishes `base_link -> livox_frame` and starts the EKF owner of
`odom -> base_link`. FAST-LIO must keep `publish_tf:=false`.

The verified test covered the `0x50A` stream, diagnostics, wheel twist output,
and the `odom -> base_link -> livox_frame` TF topology. Static suspended-vehicle
testing is not a substitute for dynamic low-speed driving validation.

## RTK Global Transform

The local EKF has passed dynamic driving validation. The optional global chain
adds `navsat_transform_node` and a second EKF which is the sole publisher of
`map -> odom`; it never changes local EKF ownership of `odom -> base_link`.

The UM982 dual-antenna `UNIHEADINGA` output is published as `/rtk/heading` and
supplies the earth-referenced yaw. A single GNSS antenna would not be enough.
The heading must be `SOL_COMPUTED`, have a valid baseline, and remain below the
configured standard-deviation threshold.

The RTK global EKF publishes `/global/odometry/filtered` as an observation
only. It does not publish `map -> odom`: the static and dynamic dual-antenna
validation exposed coordinate-frame feedback through `/odometry/gps`. A later
validated map-matching owner must publish `map -> odom`. The local TF chain
remains:

```text
odom -> base_link -> livox_frame
                 -> rtk_link
```

`base_link -> rtk_link` is the Ant1 position TF, so its axes must align with
`base_link`: use `rtk_yaw_rad:=0.0`. Do not put the Ant1-to-Ant2 baseline angle
in this TF. With Ant1 on the right and Ant2 on the left, `UNIHEADINGA` reports
the Ant1-to-Ant2 bearing relative to true north; the RTK node converts it to
base-link yaw as `-heading` in ROS ENU convention. Verify this convention
before enabling TF: point the vehicle toward true north and confirm
`/rtk/heading` yaw is approximately `+pi/2`; use
`heading_to_base_yaw_offset_rad` only for a measured residual mounting offset.

Keep this RTK-only filter's TF publication disabled even when
`/rtk/heading` is `SOL_COMPUTED`. It remains useful for recording and global
consistency checks, but it is not a `map -> odom` owner.

Before enabling it, measure and record all of the following for the fixed map:

- `base_link -> rtk_link` antenna translation and orientation;
- an RTK-fixed map datum and the map x-axis heading relative to east;
- RTK-fixed horizontal and vertical accuracy for `um982.local.yaml`.

`navsat_transform_node` starts with `wait_for_datum=true`, so it deliberately
does not accept a first-fix origin. Set the surveyed datum using its
`/set_datum` service after the map-to-earth alignment is loaded and verified.
Datum fixes the ENU origin; the dual-antenna heading fixes the earth-referenced
yaw. Do not enable this chain with zero or unverified antenna extrinsics.

Create a local RTK parameter file from `um982.example.yaml`; it is already
configured to publish only GGA quality `4` (RTK fixed) and to attach explicit
covariance. Adjust the accuracy values to receiver/field evidence.

```bash
ros2 launch mcu_can_bridge localization.launch.py \
  can_interface:=can0 \
  enable_global_localization:=true \
  rtk_extrinsics_configured:=true \
  rtk_params_file:=/etc/mower/rtk/um982.local.yaml \
  rtk_x_m:=<measured-x> rtk_y_m:=<measured-y> rtk_z_m:=<measured-z> \
  rtk_roll_rad:=<measured-roll> rtk_pitch_rad:=<measured-pitch> \
  rtk_yaw_rad:=<measured-yaw>
```

The global nodes remain off by default. `rtk_extrinsics_configured:=true` is a
deliberate safety gate: setting it starts the antenna TF,
`navsat_transform_node`, and the global EKF. It starts the RTK node only when
`start_rtk_node:=true`.

To inspect the RTK before enabling global localization, start
`mobileess_rtk` separately. Then set `start_rtk_node:=false` in the localization
launch so the already running node retains exclusive access to the serial port.
