# 在 RK3588 解析 0x50A，发布轮速 Odometry twist。

Type: task
Status: needs-triage
Blocked by: 03、04

## 目的
在 RK3588 解析 0x50A，发布轮速 Odometry twist。

## 前置条件
- 依赖任务 03、04 必须已有实测通过证据。
- 记录日期、车辆/固件、RK3588 软件、地图和参数版本。

## 硬件与软件
- RK3588、STM32 四驱测试车及本票涉及的传感器/工具。
- Ubuntu 22.04、ROS 2 Humble、SSH、RViz2、ROS 2 CLI；按任务需要增加 CAN/rosbag/Nav2 工具。

## 手动执行步骤
运行 bridge；echo/hz 轮速话题；低速直行/转向/停车；停止 0x50A 观察 150 ms stale；恢复帧观察恢复。

## 观测点
- CAN 帧与 `candump`；
- ROS 2 topic、频率、消息字段和 TF；
- RViz2 地图/轨迹/代价地图；
- 实车速度、方向、停车和人工接管；
- 日志、rosbag、照片或视频。

## 预期效果
发布 nav_msgs/Odometry；只使用 twist linear.x/angular.z；不以旧帧或零值伪造。

## 失败安全退出
关闭 bridge 或停止运动发送，断流时不继续注入旧数据。

## Evidence
- 执行命令和终端输出；
- 关键 topic/TF/CAN 记录；
- 必要时保存 rosbag、RViz2 截图、照片或视频；
- 记录实际数值与阈值对照。

## 通过阈值
解析、左右平均、stale 停用、恢复均可观察。

## 解锁下游
11、16

## Comments
