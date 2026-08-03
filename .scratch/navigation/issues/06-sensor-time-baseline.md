# 确认 Mid-360/LIO/IMU/RTK 的 topic、频率、frame 和时间戳。

Type: task
Status: needs-triage
Blocked by: 01、02

## 目的
确认 Mid-360/LIO/IMU/RTK 的 topic、频率、frame 和时间戳。

## 前置条件
- 依赖任务 01、02 必须已有实测通过证据。
- 记录日期、车辆/固件、RK3588 软件、地图和参数版本。

## 硬件与软件
- RK3588、STM32 四驱测试车及本票涉及的传感器/工具。
- Ubuntu 22.04、ROS 2 Humble、SSH、RViz2、ROS 2 CLI；按任务需要增加 CAN/rosbag/Nav2 工具。

## 手动执行步骤
静止启动驱动；执行 ros2 topic hz/info 和短 rosbag；检查 header.stamp/frame_id；低速重复观察。

## 观测点
- CAN 帧与 `candump`；
- ROS 2 topic、频率、消息字段和 TF；
- RViz2 地图/轨迹/代价地图；
- 实车速度、方向、停车和人工接管；
- 日志、rosbag、照片或视频。

## 预期效果
时间单调、频率可用、frame 明确、无持续断流。

## 失败安全退出
停止运动和驱动，异常数据不进入建图。

## Evidence
- 执行命令和终端输出；
- 关键 topic/TF/CAN 记录；
- 必要时保存 rosbag、RViz2 截图、照片或视频；
- 记录实际数值与阈值对照。

## 通过阈值
关键话题、频率、frame、异常均记录。

## 解锁下游
07、08、11

## Comments
