# 验证 STM32 的 0x005 API 接管、50 ms 控制和现有超时停车。

Type: task
Status: needs-triage
Blocked by: 01、02

## 目的
验证 STM32 的 0x005 API 接管、50 ms 控制和现有超时停车。

## 前置条件
- 依赖任务 01、02 必须已有实测通过证据。
- 记录日期、车辆/固件、RK3588 软件、地图和参数版本。

## 硬件与软件
- RK3588、STM32 四驱测试车及本票涉及的传感器/工具。
- Ubuntu 22.04、ROS 2 Humble、SSH、RViz2、ROS 2 CLI；按任务需要增加 CAN/rosbag/Nav2 工具。

## 手动执行步骤
配置 CAN 500 kbit/s；candump；发送 ctrl_mode=1 待机、2 运行；抬轮或极低速发小 vx/omega；停止发送观察约 500 ms。

## 观测点
- CAN 帧与 `candump`；
- ROS 2 topic、频率、消息字段和 TF；
- RViz2 地图/轨迹/代价地图；
- 实车速度、方向、停车和人工接管；
- 日志、rosbag、照片或视频。

## 预期效果
模式按 1→2 变化；方向正确；零速有效；控制帧停止后 MCU 应用层超时停车并锁存。

## 失败安全退出
只用抬轮或最低速度；方向异常立即人工急停。

## Evidence
- 执行命令和终端输出；
- 关键 topic/TF/CAN 记录；
- 必要时保存 rosbag、RViz2 截图、照片或视频；
- 记录实际数值与阈值对照。

## 通过阈值
500 kbit/s、持续控制帧、接管、零速、超时均有记录。

## 解锁下游
04

## Comments
