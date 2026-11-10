# CAN停稳时序修复及A4复测

日期：2026-10-21（America/Los_Angeles）。已部署SocketCAN内核到达时间戳修复。A4独立无提示全图搜索38.084秒恢复定位，随后稳定10秒；用户确认“A4一致，我给车子关机了，休息完再测”。现场测试暂停，未启动B1，也没有车辆运动验收。

## 问题及修复

此前接收过滤减少了无关报文负载，A3复测通过，但后续45秒只读采样仍出现339次状态中的7次后端停稳拒绝。同期独立CAN的740帧实际轮速全部为0，7次均满足原停稳条件，最大反馈间隔94.063ms。该记录说明仅做过滤尚未解决全部接收时序问题。

原接收循环把Python处理报文的时间记为到达时间。线程处理积压时，真实连续的反馈会被压缩为同一时刻的一批数据，停稳窗口可能缺少连续历史；旧的排队帧也可能错误更新新鲜度。针对真实RosAdapter接收循环的两个回归用例在修改前均失败，分别覆盖这两种情况。

本次保留内核CAN过滤，启用SO_TIMESTAMPNS，通过recvmsg读取每帧内核到达时间，再以采样的时钟偏移换算到单调时钟。停稳窗口、最新反馈年龄及审计记录使用到达时间。队列处理延迟单独作为诊断返回，不能使旧反馈重新变新。时间戳缺失、截断、非法、明显超前，或采样时钟对耗时超过10ms时，不更新有效反馈，并累计错误。

API新增stationary_evidence，返回判定原因、通道、反馈年龄、窗口帧数、最大间隔、最大轮速及判定时刻；同时返回各通道接收处理延迟和时间戳错误数。独立采样按同一个判定时刻比较，避免用HTTP响应后的另一时刻造成窗口边界差异。

停稳条件保持原值：连续1秒，四轮绝对速度均不超过10mm/s；0x507最新年龄和最大间隔0.25秒，0x50A为0.15秒，并保留原最少帧数及API状态检查。原A4首次尝试记录的右后轮−13mm/s确实越过阈值，该次拒绝仍有效，不改写为软件误判。

## 验证结果

| 检查 | 结果与边界 |
|---|---|
| 修改前回归 | 两个实际接收循环用例均失败：连续到达数据的积压处理；旧排队反馈不能刷新年龄 |
| 修改后逻辑回归 | 控制台48项通过；其中12项覆盖积压、旧帧、时间戳缺失/非法、真实间隔、10/11/−13mm/s及衔接停车API语义 |
| 板端构建 | 仅在RK3588执行，mower_console构建通过 |
| 90秒只读对照 | 958次HTTP采样；预热后942次后端/独立CAN比较，不一致0次，后端停稳拒绝0次 |
| 多观察连接 | 四个WebSocket观察连接分别收到340、340、339、339帧状态；无控制权申请或运动任务 |
| 时序诊断 | 独立及后端时间戳错误0；本轮末尾记录的进程启动后最大处理延迟为0x507约96.7ms、0x50A约83.0ms |
| 只读边界 | 独立CAN未观察到0x005；API始终0、实际四轮最大5mm/s；motion_enabled=false、无任务 |
| 正式A4复测 | 38.084秒重新匹配并稳定10秒；498帧真实点云；四轮最大2mm/s，0帧0x005；现场位置/朝向定性核对通过 |
| 安装核对 | 61文件SHA256全部一致；安装模块解析到目标源码；服务enabled/active/running、NRestarts=0、observation_only:=true |

90秒对照及正式A4属于不同记录。服务启动时的自动定位不另计入正式样本。此次没有修改STM32、CAN协议、地图、定位算法或运动能力开关，也没有验证运动中的停车。现场20组定位目前只完成A点四个实际朝向，见[现场验收记录](field-acceptance-20261020.md)。

## 版本及证据

- [部署清单](kernel_time_fix_20261021/deployment.json)：61文件，manifest按键排序后JSON的SHA256为`24e83648ffbcead50bdf894362980db4ba38705ea3a5c45924ab8447ce94965a`。
- RosAdapter SHA256：`49456b77cc8175db4785ac9800007f82a6fe766d49a039b304e1547d70eccff8`；can_feedback.py：`dd34a2f2bc388dd67dce79995b396af1ec6823ed5fabd17ab992e76ce6dd24cc`。
- 变更前板端备份：`/home/cat/MobileEss_data/console-deploy-20261021T083832368422Z`。
- [过滤后仍复现的原始数据](kernel_time_fix_20261021/before-wheel-observation.json)、[7次不一致对照](kernel_time_fix_20261021/before-wheel-comparison.json)、[时间戳修复后90秒对照](kernel_time_fix_20261021/after-observer.json)。
- [构建日志](kernel_time_fix_20261021/board-build.log)、[安装与服务核对](kernel_time_fix_20261021/runtime.json)、[证据SHA256](kernel_time_fix_20261021/SHA256SUMS)。运行核对是在用户关机前完成，不能代表关机后服务仍在线。
- 独立采样程序：[check_kernel_feedback.py](../../.scratch/console_field/check_kernel_feedback.py)；接收循环回归：[test_can_feedback_timing.py](../../src/mower_console/test/test_can_feedback_timing.py)。ROS相关程序只在板端执行，本机回归使用独立逻辑环境。

![修复后A4正式观察页面](kernel_time_fix_20261021/fixed.jpg)
