# Mid-360 首轮现场流程

代码在 `/home/dyh/MobileEss` 开发；以下 ROS 命令全部在 RK3588 执行。
当前固件 `20261009-vehicle-estop-disabled` 停用了 PD1 软件急停，不能用该按钮的软件路径作为停车验收。割刀动力保持隔离，运动前复测遥控停止和独立断电。

## 环境和观测

```bash
ssh cat@192.168.10.81
source /opt/ros/humble/setup.bash
source /home/cat/MobileEss_ws/install/setup.bash
ros2 run mower_nav2 field_workflow sensors
```

另一个 SSH 终端加载同样环境，检查 20 秒点云、IMU、里程计：

```bash
python3 /home/cat/MobileEss_ws/src/mower_nav2/scripts/probe_mid360.py --seconds 20
```

观测和建图入口不启动任何 CAN 控制节点。不并行启动第二个 Livox 驱动、FAST-LIO、固定地图定位或其他速度发布者。
静止检查初始化成功；人工平移和转向时核对里程计方向。启动成功和数据频率不代表地图或自主行驶验收。

## 遥控采集和保存

停止 sensors 后，在封闭场地人工低速遥控采集。每次使用全新的绝对目录名：

```bash
ros2 run mower_nav2 field_workflow capture --session /home/cat/MobileEss_data/maps/field_20261009_01
```

入口同时录制原始点云、IMU、采集里程计和 TF，保存真实配置副本、外参、元数据和待填验收表。
等 FAST-LIO 初始化后再移动，覆盖起点、目标点、墙角、通道及障碍物，回到起点附近检查地图重合。
保持采集进程运行，在第二个终端保存：

```bash
ros2 run mower_nav2 field_workflow save --session /home/cat/MobileEss_data/maps/field_20261009_01
```

只有服务成功且 PCD 非空才能 Ctrl+C 停止采集；退出会同步停止并收尾录包。

```bash
ros2 run mower_nav2 field_workflow convert --session /home/cat/MobileEss_data/maps/field_20261009_01
```

该入口保留 PCD 端点投影原图，不能仅凭地面端点之间的空洞判断采集失败。此次录包另生成地面可见性候选图：

```bash
python3 /home/cat/MobileEss_ws/src/mower_nav2/scripts/bag_to_occupancy.py \
  /home/cat/MobileEss_data/maps/field_20261009_01 \
  --output-dir /home/cat/MobileEss_data/maps/field_20261009_01/visibility_self_filtered \
  --self-filter-config /home/cat/MobileEss_ws/src/mower_nav2/config/livox_obstacle_cloud.yaml
```

输出目录必须全新。只沿实际扫描中有地面回波的方向清除自由空间，近处障碍截断射线；逐帧排除车体，保留未知区域与累计障碍。输出仍是未审查候选图。此命令已执行，再次运行应换新目录，不能覆盖。

检查生成的 map.pgm/map.yaml：不能把未观测区域涂成自由空间。保留候选地图原始副本，人工将允许作业区域之外、缓冲区和禁行区标为占用（PGM 黑色）。首轮整区限速 0.2 m/s，不接入 RTK 或复杂分区速度。
按用户 CAD，车体长 1.08606 m、宽 0.768 m；全局/局部代价地图使用矩形轮廓并加 0.02 m 边界，膨胀半径 0.90 m，collision_gate 保守圆半径 0.70 m。base_link 暂按车体中心，支架等附加件外伸和真实中心仍需量测。检查通道宽度与地图覆盖。无法形成连续可通行区域时重新采集；仅有绿色路径不能批准运动。
现场审查后的静态地图同时进入全局和局部代价地图，限制局部绕行跨越边界。元数据/acceptance.md 由人工记录实际审查结果，不自动批准。

最新逐帧候选图见 `temporal_static_20261010`；其没有整图安全批准。通过真实雷达仅做静止定位及 ComputePathToPose，路径 179 点、约 9.014 m。旧候选仍保留，禁止直接使用 motion 批准参数。

## 无运动定位和规划

```bash
ros2 run mower_nav2 field_workflow observe --session /home/cat/MobileEss_data/maps/field_20261009_01/keepout_review_20261010
```

允许在建图起点附近约 3 m 内、任意朝向启动（2026-10-10 用户将原先 2 m 范围扩大为 3 m）。无需正好停回原点，也不会为了校准自动转动车辆。启动时保持静止，雷达扫描在固定 PCD 中搜索位置和朝向，再连续确认三次；多个不同位置评分接近、特征不足或匹配不稳定时保持未定位。该范围以建图坐标原点为中心，不支持全场任意位置启动。
检查 `/fixed_map_localization/diagnostics` 持续 accepted，`map→odom` 和 `odom→base_link` 各只有一个发布者。失败时先确认车确在 3 m 范围内、环境结构未大变且雷达无遮挡；人工移至有墙角/立柱等明显特征的位置再启动，不放宽质量或歧义门控强行通过。录包变换测试不代替不同停车位置的实车启动验收。
observe 不启动 CAN 控制；RViz 只承担可视化。路径检查用 ComputePathToPose，不发送 NavigateToPose 行驶目标。

重新采集时低速平稳行驶，交叉覆盖墙角、立柱和设备侧面，避免只在空旷中央走圈；桌腿与狭窄通道需从不同侧面观察。照片中的陡坡/台架在首轮作为禁行区，不凭平面地图判断可爬坡。窗口附近的点云缺失或异常应先检查实际回波，照片不能直接证明反射问题。

2026-10-10 用户现场确认当前位置周围空旷并补充照片后，另存局部候选图 `local_review_20261010`。仅修改当前地图位置 (1.9552,0.7678) m 半径 0.65 m 内、完整位于审查区域的格子：104 格经过审查，16 格有修改；区域外不变。依据是人工现场确认，不是没有雷达回波就自动清除。该候选图下静止定位和规划已通过，整图 `map_reviewed` 与 `motion_approved` 仍为 false。新环境或新停车位置不得复用这些坐标自动清图。

## CAN 和运动验收

首先车轮架空，确认割刀隔离、遥控停止与独立断电有效。由现场人员启用 CAN（需要 sudo 时由本人输入密码）：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
ros2 run mower_nav2 field_workflow can-listen --seconds 10
```

只收不发，预期收到 0x507 与 0x50A，且没有 0x005。若 0x50A 缺失，禁止用 0x507 的命令回显代替实际轮速；先核查固件协议。
保存 CAN 日志：`candump -L can0 > /home/cat/MobileEss_data/can-test.log`。
停止 observe，确认地图审查和物理准备完成后启动运动入口：

```bash
ros2 run mower_nav2 field_workflow motion \
  --session /home/cat/MobileEss_data/maps/field_20261009_01/local_review_20261010 \
  --physical-ready --safety-map-approved
```

入口只启动导航，不自动发送目标，也不发送 0xAA 清锁。架空验证 1→2 接管、前进及左转轮向；遥控接管后 bridge 应停止控制，需要人工处理后重新启动。验证切断控制帧后 MCU 约 500 ms 超时停车和锁存。
截至 2026-10-10，局部人工修订候选图已通过当前位置的静止定位及规划，起点格到非自由格的离散距离约 0.608 m；规划路径沿途格中心最小距离约 0.50 m。这些离散数字不包含实际地图误差，也不代替完整车体包络和路径净空检查，整图安全审查尚未完成。不要直接照抄批准参数启动运动。先核对陡坡台架禁行区、实际车体尺寸、作业边界及反馈 CAN，再完成架空和地面验收。
架空验收通过后再开展有人监护的地面目标测试；最大线速度 0.2 m/s、角速度 0.3 rad/s，首轮禁止倒车。
分别记录到点、障碍停车/有限绕行、通道堵塞、雷达断流、定位退化、速度命令中断。障碍点云超过 0.30 s 未更新时 collision_gate 发零；控制命令超过 0.25 s 时 bridge 发零；定位 accepted 超过 2.5 s 未更新时 bridge 发零。停车指令不代表实际制动完成，须现场核对。

## 恢复

旧 Mutagen 会话保持暂停；恢复旧版本前先暂停新的 field 会话，使用板端 `/home/cat/MobileEss_backups/src-before-mid360-20261009.tgz` 恢复选定源码并在 RK3588 重建。保留采集数据和配置，不自动重刷 STM32。

## 移动人体残影候选转换

用户确认红框回波含遥控时本人走动。使用逐帧时序候选转换，不按红框整片删除：

```bash
python3 /home/cat/MobileEss_ws/src/mower_nav2/scripts/bag_to_occupancy.py \
  /home/cat/MobileEss_data/maps/field_20261009_01 \
  --output-dir /home/cat/MobileEss_data/maps/field_20261009_01/temporal_static_20261010 \
  --self-filter-config /home/cat/MobileEss_ws/src/mower_nav2/config/livox_obstacle_cloud.yaml \
  --temporal-static
```

至少三帧障碍命中才形成静态占用；同帧障碍优先。最后一次命中后至少五帧地面支持的可见射线才允许清除，单次或不足观测保留未知，规划器禁止跨未知。此规则不是人体识别，仍需现场核对细杆、桌腿、墙体及作业边界。原 PCD 和旧图保留。
当前障碍高度带为离地 0.15～0.80 m，不能据此保证低矮台阶或高处悬伸物检测；首轮只在审查后的平整区域，坡道台架和台阶设为禁行。CAD 74 mm 离地间隙不代表这些障碍可通行。

## 用户标注禁行区版本

最新候选 `keepout_review_20261010` 保存八个黄色矩形禁行区和截图映射记录。原左侧路线经用户确认可通行；新图默认重规划改为右侧绕行，仍须现场复核或约束为已审查路线，不能直接批准运动。原图不覆盖。

CAN 检查先只收不发。若总线无帧，先检查底层板供电、端口和 CAN H/L 极性，不通过发送控制帧探测。用户本次确认曾 H/L 接反，接线修正后的接收记录独立保存。

H/L 修正后 can0 已收到 0x507/0x50A，静止轮速为零且未发送 0x005；本次 0x50A 周期约 60 ms，仍需在架空运动时检查反馈时效及实际单位。BMS 原始状态 1/255 无法提供有效剩余电量，现场核对供电。下一步仅在四轮稳定架空、割刀隔离、有人可立即停止后开展控制验收。
