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
- 执行命令和终端输出；
- 关键 topic/TF/CAN 记录；
- 必要时保存 rosbag、RViz2 截图、照片或视频；
- 记录实际数值与阈值对照。

## 通过阈值
版本、接口、topic、frame、频率均有记录。

## 解锁下游
02、03、06

## Comments
