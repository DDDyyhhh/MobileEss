# 验证 linear.x/angular.z 经 Nav2、bridge、0x005 到四轮。

Type: task
Status: needs-triage
Blocked by: 03、05、11、15

## 目的
验证 linear.x/angular.z 经 Nav2、bridge、0x005 到四轮。

## 前置条件
- 依赖任务 03、05、11、15 必须已有实测通过证据。
- 记录日期、车辆/固件、RK3588 软件、地图和参数版本。

## 硬件与软件
- RK3588、STM32 四驱测试车及本票涉及的传感器/工具。
- Ubuntu 22.04、ROS 2 Humble、SSH、RViz2、ROS 2 CLI；按任务需要增加 CAN/rosbag/Nav2 工具。

## 手动执行步骤
配置差速控制器、linear.y=0；零速接管；低速直行/左右转/停车；逐步提高速度。

## 观测点
- CAN 帧与 `candump`；
- ROS 2 topic、频率、消息字段和 TF；
- RViz2 地图/轨迹/代价地图；
- 实车速度、方向、停车和人工接管；
- 日志、rosbag、照片或视频。

## 预期效果
cmd_vel、0x005 和四轮方向一致；停车有效；无侧向速度。

## 失败安全退出
方向/速度异常立即急停，停止 Nav2 并恢复人工。

## Evidence
- 执行命令和终端输出；
- 关键 topic/TF/CAN 记录；
- 必要时保存 rosbag、RViz2 截图、照片或视频；
- 记录实际数值与阈值对照。

## 通过阈值
直行/左右转/零速均通过，人工可随时接管。

## 解锁下游
17、18、20

## Comments
