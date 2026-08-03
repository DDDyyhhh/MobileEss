# 用 LIO、IMU、轮速 twist 输出 odom→base_link。

Type: task
Status: needs-triage
Blocked by: 05、06

## 目的
用 LIO、IMU、轮速 twist 输出 odom→base_link。

## 前置条件
- 依赖任务 05、06 必须已有实测通过证据。
- 记录日期、车辆/固件、RK3588 软件、地图和参数版本。

## 硬件与软件
- RK3588、STM32 四驱测试车及本票涉及的传感器/工具。
- Ubuntu 22.04、ROS 2 Humble、SSH、RViz2、ROS 2 CLI；按任务需要增加 CAN/rosbag/Nav2 工具。

## 手动执行步骤
配置 robot_localization EKF；静止/直行/转向录 bag；检查 filtered odometry 和 TF；暂停轮速观察。

## 观测点
- CAN 帧与 `candump`；
- ROS 2 topic、频率、消息字段和 TF；
- RViz2 地图/轨迹/代价地图；
- 实车速度、方向、停车和人工接管；
- 日志、rosbag、照片或视频。

## 预期效果
TF 连续唯一；速度方向合理；轮速 pose 不参与；断流不伪造零速。

## 失败安全退出
TF 跳变或车辆异常立即停驶并保存 bag。

## Evidence
- 执行命令和终端输出；
- 关键 topic/TF/CAN 记录；
- 必要时保存 rosbag、RViz2 截图、照片或视频；
- 记录实际数值与阈值对照。

## 通过阈值
EKF、TF、三类动作和轮速失效行为可回放。

## 解锁下游
12、13、16

## Comments
