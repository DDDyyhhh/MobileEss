# 确认 RK3588、ROS 2、SocketCAN、SSH 和传感器接口。

Type: task
Status: needs-triage
Blocked by: 无

## 目的
确认 RK3588、ROS 2、SocketCAN、SSH 和传感器接口。

## 前置条件
- 依赖任务 无 必须已有实测通过证据。
- 记录日期、车辆/固件、RK3588 软件、地图和参数版本。

## 硬件与软件
- RK3588、STM32 四驱测试车及本票涉及的传感器/工具。
- Ubuntu 22.04、ROS 2 Humble、SSH、RViz2、ROS 2 CLI；按任务需要增加 CAN/rosbag/Nav2 工具。

## 手动执行步骤
SSH 登录；记录 uname/Ubuntu/ROS 版本、工作空间；查看 CAN 接口；启动现有传感器并列出 topic、frame 和频率。

## 观测点
- CAN 帧与 `candump`；
- ROS 2 topic、频率、消息字段和 TF；
- RViz2 地图/轨迹/代价地图；
- 实车速度、方向、停车和人工接管；
- 日志、rosbag、照片或视频。

## 预期效果
能登录 RK3588；CAN、Mid-360/LIO、RTK、IMU 接口可确认或明确缺失。

## 失败安全退出
停止驱动，不发送运动帧。

## Evidence

### 2026-08-03 RK3588 系统与 ROS 2 基线

执行位置：RK3588 `cat@lubancat:~/MobileEss_ws`。

执行命令：

```bash
date -Is
hostname
uname -a
cat /etc/os-release | grep -E 'PRETTY_NAME|VERSION='
source /opt/ros/humble/setup.bash
printf 'ROS_DISTRO=%s\n' "$ROS_DISTRO"
which ros2
ros2 --version
pwd
```

实测输出：

```text
2026-08-03T15:00:24+08:00
lubancat
Linux lubancat 6.1.84 #1 SMP Fri Jul  3 10:19:32 CST 2026 aarch64 aarch64 aarch64 GNU/Linux
PRETTY_NAME="Ubuntu 22.04.5 LTS"
VERSION="22.04.5 LTS (Jammy Jellyfish)"
ROS_DISTRO=humble
/opt/ros/humble/bin/ros2
usage: ros2 [-h] [--use-python-default-buffering] Call `ros2 <command> -h` for more detailed usage. ...
ros2: error: unrecognized arguments: --version
/home/cat/MobileEss_ws
```

当前结论：

- SSH 登录到 RK3588 成功，主机名为 `lubancat`。
- RK3588 系统为 Ubuntu 22.04.5 LTS，内核为 `6.1.84`，架构为 `aarch64`。
- ROS 2 环境变量为 `ROS_DISTRO=humble`，`ros2` 命令路径为 `/opt/ros/humble/bin/ros2`。
- `ros2 --version` 在当前 ROS 2 CLI 中不支持，返回 `unrecognized arguments: --version`；这不是 ROS 2 缺失，因 `ROS_DISTRO` 和 `which ros2` 已确认 Humble 环境可用。
- 当前工作区路径为 `/home/cat/MobileEss_ws`。

### 2026-08-03 RK3588 SocketCAN 基线

执行位置：RK3588 `cat@lubancat:~/MobileEss_ws`。

安全状态：
- RK3588 未运行底盘控制 ROS 2 节点；未执行 `cansend`。
- 测试期间 RK3588 SocketCAN 仅监听，接口统计 `TX packets 0`。
- 发现接入底盘 CAN 后曾有单个驱动轮偶发抽动；为降低风险，已断开电机与驱动器之间的 UVW，只保留 RK3588 CAN 与底盘控制器连接。
- 完成监听后执行 `sudo ip link set can0 down`。

执行命令：

```bash
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 500000
candump -L -ta -n 100 can0,001:7FF,002:7FF,003:7FF,004:7FF,507:7FF,508:7FF,509:7FF,50A:7FF
ip -details -statistics link show can0
sudo ip link set can0 down
```

CAN 接口状态实测：

```text
can0: <NOARP,UP,LOWER_UP,ECHO>
can state ERROR-ACTIVE (berr-counter tx 0 rx 0)
bitrate 500000 sample-point 0.866
re-started 0, bus-errors 0, arbit-lost 0, error-warn 0, error-pass 0, bus-off 0
RX: bytes 163080, packets 36029, errors 0, dropped 0, missed 0
TX: bytes 0, packets 0, errors 0, dropped 0, carrier 0, collsns 0
```

关键 `candump` 记录节选：

```text
(1785739413.201625) can0 507#0000000000634A06
(1785739413.221577) can0 50A#0000000000000000
(1785739413.241572) can0 507#0000000000634A06
(1785739413.261579) can0 508#0000000000000000
(1785739413.261823) can0 509#0000000000000000
(1785739413.281815) can0 50A#0000000000000000
(1785739413.341643) can0 50A#0000000000000000
(1785739413.401821) can0 50A#0000000000000000
(1785739413.470083) can0 50A#0000000000000000
```

观测到的 CAN ID：
- `0x001`、`0x002`、`0x003`、`0x004`：OID 驱动器相关内部报文/响应。
- `0x507`：底盘状态/速度指令回显帧。
- `0x508`、`0x509`：当前观测为全零，不作为有效里程计依据。
- `0x50A`：已收到四轮实际线速度反馈帧。

`0x50A` 当前解析：

```text
50A#0000000000000000
Byte0-1 前左：0 mm/s = 0.000 m/s
Byte2-3 前右：0 mm/s = 0.000 m/s
Byte4-5 后左：0 mm/s = 0.000 m/s
Byte6-7 后右：0 mm/s = 0.000 m/s
```

当前结论：
- RK3588 `can0` 可配置为经典 CAN 500 kbit/s 并进入 `ERROR-ACTIVE`。
- RK3588 能被动接收底盘控制器 CAN 报文，接收方向和物理链路可用。
- `0x50A` 已由底盘控制器发送并被 RK3588 接收；静止/UVW 断开状态下四轮反馈为零。
- 本票仅确认环境和接口基线；非零轮速、四轮顺序、方向符号和换算精度留给 `04-mcu-50a-wheel-speed.md` 的抬轮/低速验证。

### 2026-08-03 当前工作区 Mid-360/LIO 可见性检查

执行位置：RK3588 `cat@lubancat:~/MobileEss_ws`。

执行命令：

```bash
source /opt/ros/humble/setup.bash
source ~/MobileEss_ws/install/setup.bash 2>/dev/null || true
ros2 topic list -t | grep -Ei 'livox|mid|lidar|point|cloud|lio|odom|imu|tf'
ros2 topic list -t
find ~/ -path '*/install/*' -prune -o -type f \( -name '*.launch.py' -o -name '*.launch.xml' \) -print 2>/dev/null | grep -Ei 'livox|mid|lio|fast|laser|point|cloud'
```

实测输出：

```text
/gps/fix [sensor_msgs/msg/NavSatFix]

/client_count [std_msgs/msg/Int32]
/connected_clients [rosbridge_msgs/msg/ConnectedClients]
/gps/fix [sensor_msgs/msg/NavSatFix]
/parameter_events [rcl_interfaces/msg/ParameterEvent]
/rosout [rcl_interfaces/msg/Log]
/rtk/nmea [std_msgs/msg/String]
/rtk/status [std_msgs/msg/String]

/home/cat/mower_ws/src/fast_lio/launch/fast_lio_real.launch.py
/home/cat/mower_ws/src/fast_lio/launch/mapping.launch.py
```

当前结论：

- 当前工作区 `~/MobileEss_ws` source 后，ROS 图中未发现 Mid-360、Livox、Fast-LIO、点云、LIO odom、IMU、`/tf` 或 `/tf_static` 相关 topic。
- 当前可见 ROS topic 仅包括 rosbridge 和 RTK 相关 topic：`/gps/fix`、`/rtk/nmea`、`/rtk/status` 等。
- 系统中另一个工作空间 `~/mower_ws` 存在 Fast-LIO launch 文件，但用户确认该路径属于另一个项目，不作为当前 `MobileEss_ws` 的 Mid-360/LIO 基线。
- 因此，本票在当前工作区下记录 Mid-360/LIO/内置 IMU 接口为“当前未集成或未启动，topic/frame/频率缺失”。

待补充证据：

- RTK topic、定位状态、频率输出。
- 如后续将 Mid-360/LIO 集成进当前工作区，再补充点云、内置 IMU、LIO odom、TF 的 topic、frame 和频率。
- RViz2/rosbag/照片或视频证据，如后续需要。

## 通过阈值
版本、接口、topic、frame、频率均有记录。

## 解锁下游
02、03、06

## Comments
