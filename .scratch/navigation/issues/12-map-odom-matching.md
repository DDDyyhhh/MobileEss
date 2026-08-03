# 验证激光地图匹配校正 map→odom，RTK 只作辅助。

Type: task
Status: needs-triage
Blocked by: 06、07、10、11

## 目的
验证激光地图匹配校正 map→odom，RTK 只作辅助。

## 前置条件
- 依赖任务 06、07、10、11 必须已有实测通过证据。
- 记录日期、车辆/固件、RK3588 软件、地图和参数版本。

## 硬件与软件
- RK3588、STM32 四驱测试车及本票涉及的传感器/工具。
- Ubuntu 22.04、ROS 2 Humble、SSH、RViz2、ROS 2 CLI；按任务需要增加 CAN/rosbag/Nav2 工具。

## 手动执行步骤
加载地图/EKF；走起始区和独立路线；RViz2 对照 TF/RTK；观察 RTK 异常。

## 观测点
- CAN 帧与 `candump`；
- ROS 2 topic、频率、消息字段和 TF；
- RViz2 地图/轨迹/代价地图；
- 实车速度、方向、停车和人工接管；
- 日志、rosbag、照片或视频。

## 预期效果
匹配连续校正；RTK 单帧不造成位姿跳变；TF 语义正确。

## 失败安全退出
匹配失败或跳变立即人工停车并保存日志。

## Evidence
- 执行命令和终端输出；
- 关键 topic/TF/CAN 记录；
- 必要时保存 rosbag、RViz2 截图、照片或视频；
- 记录实际数值与阈值对照。

## 通过阈值
独立路线连续，异常帧不瞬移。

## 解锁下游
13、14

## Comments
