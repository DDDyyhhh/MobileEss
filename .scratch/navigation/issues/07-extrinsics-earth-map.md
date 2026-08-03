# 确认传感器外参与固定 earth→map 标定。

Type: task
Status: needs-triage
Blocked by: 06

## 目的
确认传感器外参与固定 earth→map 标定。

## 前置条件
- 依赖任务 06 必须已有实测通过证据。
- 记录日期、车辆/固件、RK3588 软件、地图和参数版本。

## 硬件与软件
- RK3588、STM32 四驱测试车及本票涉及的传感器/工具。
- Ubuntu 22.04、ROS 2 Humble、SSH、RViz2、ROS 2 CLI；按任务需要增加 CAN/rosbag/Nav2 工具。

## 手动执行步骤
测量传感器相对 base_link 安装位姿；采集 RTK 固定点；保存 earth→map 配置；重启检查 TF/映射。

## 观测点
- CAN 帧与 `candump`；
- ROS 2 topic、频率、消息字段和 TF；
- RViz2 地图/轨迹/代价地图；
- 实车速度、方向、停车和人工接管；
- 日志、rosbag、照片或视频。

## 预期效果
外参可复用；标定加载一致；重启不自动改变固定标定。

## 失败安全退出
不覆盖原标定，保存候选并停止导航。

## Evidence
- 执行命令和终端输出；
- 关键 topic/TF/CAN 记录；
- 必要时保存 rosbag、RViz2 截图、照片或视频；
- 记录实际数值与阈值对照。

## 通过阈值
文件有版本/日期/坐标说明，重复加载一致。

## 解锁下游
08、10、12

## Comments
