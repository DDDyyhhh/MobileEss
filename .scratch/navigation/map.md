# 导航任务总览

## 目标

将 [spec.md](spec.md) 拆成可手动执行、可在四驱实车复现、带明确预期效果和安全退出的最小任务，最终完成：

```text
固定停车场建图 → 自动初始化定位 → Nav2 自主导航 → 到达目标 → 动态障碍处理 → 故障安全停车
```

当前仅创建任务文档，不修改代码、固件或实车配置，不执行上车操作。

## 状态说明

- `needs-triage`：任务已定义，尚未现场验证；
- `claimed`：已开始执行；
- `resolved`：实测通过并已归档证据；
- `Blocked by` 表示必须先有实测通过证据，不是只要求文件存在。

## 任务总表

| 编号 | 任务 | 类型 | 状态 | 依赖 |
|---|---|---|---|---|
| 01 | [环境基线](issues/01-environment-baseline.md) | task | needs-triage | — |
| 02 | [安全测试区](issues/02-safety-test-area.md) | task | needs-triage | 01 |
| 03 | [CAN 0x005 API 接管](issues/03-can-005-api-handoff.md) | task | needs-triage | 01, 02 |
| 04 | [MCU 0x50A 轮速反馈](issues/04-mcu-50a-wheel-speed.md) | task | needs-triage | 03 |
| 05 | [RK CAN bridge](issues/05-rk-can-bridge.md) | task | needs-triage | 03, 04 |
| 06 | [传感器时间基线](issues/06-sensor-time-baseline.md) | task | needs-triage | 01, 02 |
| 07 | [外参与 earth-map 标定](issues/07-extrinsics-earth-map.md) | task | needs-triage | 06 |
| 08 | [建图数据采集](issues/08-mapping-data-capture.md) | task | needs-triage | 02, 06, 07 |
| 09 | [双地图生成清理](issues/09-build-clean-maps.md) | task | needs-triage | 08 |
| 10 | [地图资产固化](issues/10-freeze-map-assets.md) | task | needs-triage | 07, 09 |
| 11 | [odom-base_link EKF](issues/11-odom-ekf.md) | task | resolved | 05, 06 |
| 12 | [map-odom 匹配](issues/12-map-odom-matching.md) | task | needs-triage | 06, 07, 10, 11 |
| 13 | [自动初始化定位门禁](issues/13-localization-startup-gate.md) | task | needs-triage | 10, 12 |
| 14 | [定位退化行为](issues/14-localization-degradation.md) | task | needs-triage | 12, 13 |
| 15 | [Nav2 边界与代价地图](issues/15-nav2-costmaps-boundary.md) | task | needs-triage | 02, 09, 10 |
| 16 | [差速控制器链路](issues/16-differential-drive-control.md) | task | needs-triage | 03, 05, 11, 15 |
| 17 | [目标与 Standoff Point](issues/17-goal-standoff.md) | task | needs-triage | 13, 15, 16 |
| 18 | [无障碍实车导航](issues/18-static-route-navigation.md) | task | needs-triage | 13, 15, 16, 17 |
| 19 | [动态障碍行为](issues/19-dynamic-obstacle-behavior.md) | task | needs-triage | 18 |
| 20 | [故障停车与复位](issues/20-fault-stop-reset.md) | task | needs-triage | 03, 13, 16, 18 |
| 21 | [闭环验收](issues/21-closed-loop-acceptance.md) | task | needs-triage | 14, 18, 19, 20 |

## 依赖图

```text
01 → 02 → 03 → 04 → 05
01 → 06 → 07 → 08 → 09 → 10
05 + 06 → 11
06 + 07 + 10 + 11 → 12 → 13 → 14
02 + 09 + 10 → 15
03 + 05 + 11 + 15 → 16
13 + 15 + 16 → 17 → 18 → 19
03 + 13 + 16 + 18 → 20
14 + 18 + 19 + 20 → 21
```

## 当前 frontier

第一批可按现场条件执行：01。01 通过后执行 02、06；其他任务必须等待依赖票实测通过。

## 执行原则

1. 先在空旷、低速、有人可立即接管的区域执行；
2. 未通过 02，不进行实车运动测试；
3. 未通过 03，不发送非零 `0x005` 运动指令；
4. 未通过 04/05，不把 MCU 轮速接入 EKF；
5. 未通过 09/10，不执行正式导航；
6. 未通过 13，不执行自主运动；
7. 任何失败先停车、保留日志/rosbag/CAN 记录，再记录现象；
8. 不通过增加理论防御逻辑掩盖现场失败。

## Notes

- 第一阶段核心是实际建图、定位和自主导航效果；
- MCU 只做必要的 OID 实时电转速读取和 `0x50A` 四轮线速度输出；
- 不增加 `0x50B`、序号、CRC、协议版本或硬件看门狗；
- `0x507` 仍是速度指令回显，不作为实测轮速；
- `Standoff Point` 只表示几何上可安全驻停，不表示真实充电成功。

## Decisions-so-far

- [spec.md](spec.md) 是总体目标和验收基线；
- 每张票对应一个可执行动作、可判定输出和安全退出点；
- 任务完成必须有实测证据，文件创建不等于任务完成。

## Fog

- RK3588 当前实际系统镜像、ROS 2 工作空间和包目录尚未在任务执行中确认；
- Mid-360、RTK、IMU 的实际驱动、话题名、frame_id、频率和时间基准待 01/06 确认；
- OID 实时电转速查询在当前固件中的具体命令/响应细节待 04 依据手册和抓包落地；
- 地图匹配组件和 Nav2 控制器插件的最终选型待实机环境确认；
- 所有阈值需以现场实测为准，不先假设不存在的能力。

## Evidence index

各票的证据写在对应 issue 的 `## Evidence` 小节；完成后在这里补充日志、rosbag、截图、视频或地图资产的相对路径。

## Comments
