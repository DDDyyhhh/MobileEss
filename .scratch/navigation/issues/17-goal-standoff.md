# 验证目标校验、目标朝向和 Standoff Point 几何判定。

Type: task
Status: needs-triage
Blocked by: 13、15、16

## 目的
验证目标校验、目标朝向和 Standoff Point 几何判定。

## 前置条件
- 依赖任务 13、15、16 必须已有实测通过证据。
- 记录日期、车辆/固件、RK3588 软件、地图和参数版本。

## 硬件与软件
- RK3588、STM32 四驱测试车及本票涉及的传感器/工具。
- Ubuntu 22.04、ROS 2 Humble、SSH、RViz2、ROS 2 CLI；按任务需要增加 CAN/rosbag/Nav2 工具。

## 手动执行步骤
配置合法/非法目标和 Standoff Point；运行校验；调用 NavigateToPose；观察任务结果。

## 观测点
- CAN 帧与 `candump`；
- ROS 2 topic、频率、消息字段和 TF；
- RViz2 地图/轨迹/代价地图；
- 实车速度、方向、停车和人工接管；
- 日志、rosbag、照片或视频。

## 预期效果
非法目标拒绝；合法目标转为几何目标；Standoff 不宣称充电成功。

## 失败安全退出
不可达时停车上报，不缩小安全边界、不无限重规划。

## Evidence
- 执行命令和终端输出；
- 关键 topic/TF/CAN 记录；
- 必要时保存 rosbag、RViz2 截图、照片或视频；
- 记录实际数值与阈值对照。

## 通过阈值
合法与非法目标结果可复现。

## 解锁下游
18

## Comments
