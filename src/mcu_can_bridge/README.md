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
