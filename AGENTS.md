# Repository Guidelines

## Project Overview
MobileEss is an autonomous industrial lawn mower robotics stack built on **ROS 2 Humble**. The project integrates real-time LiDAR-inertial odometry (**FAST-LIO** with Livox Mid-360), RTK-GNSS dual-antenna heading estimation (**UM982** via `mobileess_rtk`), costmap evidence generation and Nav2 navigation (`mower_nav2`), custom dynamic obstacle management (`timed_dynamic_obstacle_layer`), and safety-critical motor/actuator control via CAN bus to an onboard **STM32** hardware controller (`mcu_can_bridge`).

### Dual-Environment Execution Model
- **Development Environment (Rescue Host / 拯救者)**: Local repository (`~/MobileEss`), code editing, Git version control, static analysis, and offline logic/unit testing. Code is synchronized to the onboard computer via Mutagen.
- **Target Execution Environment (RK3588)**: Board path (`~/MobileEss_ws/src` or remote `rk3588`). **All ROS 2 package builds (`colcon`), node executions, sensor drivers, hardware launches, and integration tests MUST be executed on RK3588 via SSH.** Local machine does not run hardware nodes or live ROS sensor pipelines.

---

## Architecture & Data Flow

### System Topology & Compute Split
```
[Sensors]
  ├─ Livox Mid-360 (CustomMsg pointcloud, IMU @ 200Hz) ──► FAST-LIO
  ├─ UM982 GNSS (NMEA GGA + UNIHEADING @ 10-20Hz)     ──► mobileess_rtk
  └─ STM32 Feedback (CAN 0x507 @ 40ms, 0x50A @ 50ms)   ──► mcu_can_bridge

[RK3588 Computing Stack]
  ├─ fast_lio: LiDAR-inertial odometry & Fixed-Map Relocalization (Map-matching)
  ├─ mobileess_rtk: GPS/RTK lat/lon conversion to local map frame & heading
  ├─ mower_nav2:
  │    ├─ livox_costmap_evidence: Pointcloud to 2D occupancy grid & dynamic filtering
  │    ├─ collision_gate: Virtual safety bumper gating cmd_vel
  │    └─ Nav2 BT Navigator / Controller Server (Smooth Path Following)
  └─ mcu_can_bridge: Translates /cmd_vel into 0x005 CAN control frames (50ms)

[Hardware Control Layer]
  └─ STM32 Microcontroller: Owns low-level PID, motor encoders, blade relay, and latched hardware E-stop
```
### Coordinate Frames & TF Tree
The coordinate hierarchy enforces strict spatial conventions:
`earth` ──► `map` ──► `odom` ──► `base_link` ──► `sensor frames` (`livox_frame`, `gps_link`, etc.)
- `earth -> map`: Global geographic frame defined by fixed RTK reference anchor coordinates.
- `map -> odom`: Maintained by `fixed_map_localizer` or global localization fusion; maps are static/read-only in runtime navigation.
- `odom -> base_link`: High-frequency smooth transform provided by FAST-LIO odometry.
- **Safety Boundary**: STM32 owns low-level motor shutdown and CAN watchdog timeout (500ms heartbeat drop). The ROS stack on RK3588 never bypasses STM32 CAN boundaries.

---

## Key Directories

- `src/fast_lio/`: C++ FAST-LIO implementation customized for Livox Mid-360 pointclouds (`CustomMsg`), IMU integration, and fixed PCD map relocalization (`fixed_map_localizer`).
- `src/livox_ros_driver2/`: Official Livox driver for ROS 2 Humble; interfaces directly with Mid-360 LiDAR via Ethernet.
- `src/mobileess_rtk/`: Python driver for UM982 RTK-GNSS dual-antenna module; parses NMEA GGA and UNIHEADING sentences, integrates NTRIP client, and broadcasts RTK status.
- `src/mcu_can_bridge/`: CAN interface package communicating via `can0` with the STM32 board. Publishes vehicle telemetry and translates ROS velocity commands to CAN frames.
- `src/mower_nav2/`: Navigation stack integrations including costmap generators, custom Nav2 behavior trees, PCD-to-occupancy converters, and the `collision_gate` node.
- `src/timed_dynamic_obstacle_layer/`: C++ Nav2 costmap plugin for fading moving obstacles over time.
- `docs/`: System documentation, including `hardware/STM32/` (CAN protocols), `maps/` (surveyed parking lots and metadata), and `agents/` (tracker rules and domain conventions).
- `.scratch/`: Active feature specifications, scratch work, and issue trackers.

---

## Development Commands

### 1. Code Synchronization (Local to RK3588)
Mutagen manages two-way synchronization between the development host and RK3588:
```bash
# Check Mutagen synchronization status
mutagen sync list

# Ensure mutagen session is active (configured in mutagen.yml)
mutagen sync monitor slam_project
```

### 2. Remote Build & Compilation (on RK3588 via SSH)
```bash
# Connect to target board
ssh rk3588

# Workspace root on RK3588
cd ~/MobileEss_ws

# Source ROS 2 environment and build packages
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# Build individual package
colcon build --packages-select fast_lio
colcon build --packages-select mower_nav2
colcon build --packages-select mobileess_rtk
colcon build --packages-select mcu_can_bridge
```

### 3. Remote Node Execution & Launch
```bash
# Source workspace overlays on RK3588
source /opt/ros/humble/setup.bash
source ~/MobileEss_ws/install/setup.bash

# Run hardware CAN bridge
ros2 launch mcu_can_bridge can_bridge.launch.py

# Run RTK node (using real hardware config)
ros2 launch mobileess_rtk rtk.launch.py param_file:=/etc/mower/rtk/um982_real.yaml

# Run FAST-LIO real sensor mapping/odometry
ros2 launch fast_lio fast_lio_real.launch.py

# Run Navigation stack
ros2 launch mower_nav2 mower_navigation.launch.py
```

### 4. Running Tests
- **Host Unit/Logic Tests** (Python packages with standalone logic):
  ```bash
  # Standalone Python tests that don't depend on live ROS middleware
  PYTHONPATH=src/mobileess_rtk python3 -m pytest src/mobileess_rtk/test
  PYTHONPATH=src/mcu_can_bridge python3 -m pytest src/mcu_can_bridge/test
  PYTHONPATH=src/mower_nav2:src/timed_dynamic_obstacle_layer python3 -m pytest src/mower_nav2/test/test_nav2_contract.py src/mower_nav2/test/test_pcd_to_occupancy.py src/mower_nav2/test/test_clean_occupancy_components.py src/timed_dynamic_obstacle_layer/test
  ```
- **Target Remote Integration Tests** (Full ROS 2 environment on RK3588):
  ```bash
  ssh rk3588 "source /opt/ros/humble/setup.bash && source ~/MobileEss_ws/install/setup.bash && python3 -m pytest ~/MobileEss_ws/src/fast_lio/test ~/MobileEss_ws/src/mower_nav2/test"
  ```

---

## Code Conventions & Common Patterns

### ROS 2 & C++ Patterns
- **Standard**: C++17, ISO naming conventions (`UpperCamelCase` for classes, `snake_case` for functions/variables, trailing `_` for private members).
- **Point Cloud Format**: FAST-LIO consumes custom Livox format `livox_ros_driver2::msg::CustomMsg` on `/livox/lidar` and standard `sensor_msgs::msg::Imu` on `/livox/imu`.
- **Memory & Allocation**: Avoid runtime allocations inside LiDAR/pointcloud callback loops. Preallocate buffers and use pointers/references for high-frequency cloud transforms.

### Python & Node Architecture
- **Structure**: Python ROS 2 nodes inherit from `rclpy.node.Node` with clear lifecycle declarations in constructor.
- **Parameters**: Declare and validate all ROS parameters in `__init__`. Provide parameter descriptors and default values.
- **Fail-Safe Defaults**: Nodes processing telemetry or commands must implement timeouts (e.g. 500ms watchdog in `cmd_vel_can_bridge.py` and `collision_gate.py`). If no input arrives, publish 0.0 velocity.

### CAN Bus Communication Protocols (V1.1 Specification)
Detailed in `docs/hardware/STM32/RK3588_CAN协议手册.md`:
- **Bus Parameters**: Classic CAN, 11-bit standard identifiers, 500 kbps, DLC = 8 on `can0`.
- **Control Framing (RK3588 -> STM32)**:
  - **`0x005`** (50 ms cycle, DLC=8): Chassis, blade, lifter, and grass bag command.
    - Byte 0 (`ctrl_mode`): Two-step API startup (`0` = Exit API / Stop & Lock, `1` = API Standby, `2` = API Run).
    - Byte 1 (`vx_raw`, int8): Forward linear velocity `vx = vx_raw * 0.1` m/s (typical range: -10 to 10).
    - Byte 2 (`vy_raw`, int8): Lateral velocity (fixed `0` for differential drive).
    - Byte 3 (`omega_raw`, int8): Yaw rate `omega = omega_raw * 0.1` rad/s (ROS convention: `> 0` left turn).
    - Byte 4 (`blade_level`, uint8): Cutter level (`0` = Off, `1` = 45%, `2` = 50%, `3` = 55% PWM).
    - Byte 5 (`lifter_level`, uint8): Deck height gear (`1` to `5`, corresponding to 2cm–11cm).
    - Byte 6 (`bag_target`, uint8): Grass bag target state (`0` = Close, non-zero = Open).
    - Byte 7 (`magic_key`, uint8): `0xAA` explicitly clears collision lock/system latches; must be `0x00` during normal runtime.
- **Status & Feedback Framing (STM32 -> RK3588)**:
  - **`0x507`** (~40 ms cycle, DLC=8): Command echo & system status.
    - Bytes 0-1 (`vx_feedback`, int16 big-endian): Velocity command echo in m/s (val / 1000).
    - Bytes 2-3 (`omega_feedback`, int16 big-endian): Angular command echo in rad/s (val / 1000).
    - Byte 4 (`ctrl_mode`): Current API state (`0`, `1`, `2`).
    - Byte 5 (`battery_soc`): BMS RSOC (0–100%).
    - Byte 6 (`bms_temperature`): BMS temperature in ℃ (`0xFF` = abnormal/unknown).
    - Byte 7 (`status_flags`): Bitmask (Bit 0: bag open/closed, Bit 1: bag connected, Bit 2: bag full, Bits 4-5: blade level 0–3).
  - **`0x50A`** (~50 ms cycle, DLC=8): 4-wheel actual linear velocity feedback.
    - Four signed `int16` big-endian values in mm/s: FL (Bytes 0-1), FR (Bytes 2-3), RL (Bytes 4-5), RR (Bytes 6-7). Direction compensated by STM32 into unified vehicle frame.
    - `mcu_can_bridge` parses `0x50A` to compute real wheel odometry twist.
- **Watchdog & Safety Timing**: If control frame `0x005` is interrupted for >500 ms, STM32 immediately stops the vehicle and latches safety lock. RK3588 must continuously stream frames at 50 ms.
---

## Important Files

| File Path | Description |
|---|---|
| `mutagen.yml` | Mutagen sync profile between local workstation and remote RK3588 (`~/MobileEss/src` <-> `rk3588:~/Mobile_Ess/src`). |
| `CONTEXT.md` | Authoritative engineering context, safety boundaries, STM32-RK3588 split, and project constraints. |
| `src/fast_lio/launch/fast_lio_real.launch.py` | Production launch file for FAST-LIO on vehicle with field-calibrated sensor extrinsics. |
| `src/fast_lio/src/fixed_map_localizer.cpp` | Scan-to-map relocalization module matching live LiDAR scans against pre-built global PCD maps. |
| `src/mobileess_rtk/mobileess_rtk/rtk_node.py` | UM982 RTK-GNSS serial driver, NTRIP client, and RTK diagnostics publisher. |
| `src/mcu_can_bridge/mcu_can_bridge/cmd_vel_can_bridge.py` | Converts `/cmd_vel` into STM32 CAN 0x005 control messages with safety watchdog and API state management. |

| `src/mower_nav2/mower_nav2/collision_gate.py` | Low-latency safety supervisor monitoring costmap proximity to halt forward velocity. |
| `docs/hardware/STM32/RK3588_CAN协议手册.md` | Formal specification of the CAN communication protocol between RK3588 and STM32. |

---

## Runtime & Tooling Preferences

- **OS Distribution**: Ubuntu 22.04 LTS (x86_64 for workstation, aarch64 on RK3588).
- **ROS Version**: ROS 2 Humble Hawksbill (`/opt/ros/humble`).
- **Build Tool**: `colcon` with CMake / `ament_cmake` (C++) and `setuptools` (Python).
- **Python Runtime**: Python 3.10; packages use standard `setup.cfg` / `setup.py` configurations.
- **Sensors**: Livox Mid-360 LiDAR, Unicore UM982 Dual-Antenna GNSS, onboard STM32 via SocketCAN (`can0`).
- **Configuration Secrets**: Never commit real NTRIP passwords or local calibration files (`*.local.yaml`).

---

## Testing & QA Expectations

1. **Local Pre-Flight Checks**:
   - Run Python contract tests locally before synchronizing or committing:
     ```bash
     PYTHONPATH=src/mobileess_rtk python3 -m pytest src/mobileess_rtk/test
     PYTHONPATH=src/mcu_can_bridge python3 -m pytest src/mcu_can_bridge/test
     ```
   - Run fast-lio contract tests when ROS 2 desktop is sourced:
     ```bash
     source /opt/ros/humble/setup.bash && python3 -m pytest src/fast_lio/test
     ```
2. **Onboard Integration & Hardware Verification**:
   - Deploy code via Mutagen to RK3588.
   - Run full build and package test execution via SSH on RK3588.
   - For real-sensor observation, ensure the safety checks pass: never run tests with active blade or drive motors enabled without physical perimeter safety and vehicle elevation.
