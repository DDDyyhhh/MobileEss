# 浏览器导航控制台交付记录

记录日期：2026-10-20（America/Los_Angeles）。包版本 `mower_console 0.1.0`，已部署到 RK3588 `/home/cat/MobileEss_ws/src`，四包构建完成。

后续修复记录：[路线预览导致odom过期修复](odom-stale-fix-20261020.md)、[刷新后等待匹配修复](refresh-state-fix-20261020.md)。本报告以下哈希和测试数量保留首次交付时的版本。

后续接收过滤修复：[CAN接收负载与停稳判定修复](feedback-stop-fix-20261021.md)。最新部署：[内核到达时间戳修复及A4复测](can-kernel-time-fix-20261021.md)，61文件已核对。

用户已确认当前界面验收通过；[后续真实定位现场验收记录](field-acceptance-20261020.md)已开始，尚未触发新一轮运动。

**软件实现和分层验证已交付，系统服务已安装并运行；重启上电、真实多位置定位、新运动能力及网页到点闭环尚未验收。** 本次没有执行车辆运动，也没有修改或刷写 STM32 固件。默认配置和默认启动参数均禁止发车。

## 已实现

| 范围 | 交付内容 |
|---|---|
| 浏览器 | 中文 React/TypeScript 界面；Canvas 二维点选、拖动和缩放；Three.js 三维点云；车辆朝向、路线分段、通行包络、实际轨迹、障碍和诊断 |
| 后端 | aiohttp/rclpy；配对码、同源检查、单 WebSocket 控制席；状态/地图/规划/任务/停止/重定位 API；250ms 心跳、1秒失联停止 |
| 定位 | 全地图自由区域候选、位置/朝向独立候选、竞争解拒绝、后台搜索及120秒上限；代次取消、新鲜帧确认、移动和旧数据拒绝；手工提示须重新匹配 |
| 规划 | 0.10m Smac Lattice 原语；R4 曲线、短直线、原地旋转；优先前进，再尝试倒车；自动枚举终点朝向；40m/连续倒车1m/完整包络检查 |
| 执行 | 前进 RotationShim+MPPI、独立倒车 MPPI、Spin；实际轮速确认衔接停车；600秒上限、偏离路线和反馈失效停止；单次预览许可、重复请求幂等 |
| 停止链 | CAN 桥250ms执行许可，撤销后归零退出API；超时锁存、迟到许可不能恢复；取消action、停止确认；0x507回显与0x50A实际轮速分开显示 |
| 观测启动 | 单雷达/LIO、固定图定位、Nav2、10Hz局部地图、序列化雷达输入、只读CAN；systemd安装脚本和只读CAN接口预检查 |

首版默认限速前进0.3m/s、倒车0.2m/s、角速0.2rad/s。到点检查定位读数误差≤0.25m，并要求新鲜四轮反馈连续1秒各≤10mm/s。未知区域、边界和障碍均拒绝通行。旋转包络额外含0.25m滑移余量；后向观测不足时拒绝倒车。

前进与倒车规划器使用独立进程。Humble 的 `NodeLattice` 对运动表有进程内缓存，不宜在同一进程以相同文件混用不同倒车设置。依据：[Humble NodeLattice 源码](https://api.nav2.org/nav2-humble/html/node__lattice_8cpp_source.html)。路线控制不依赖 Nav2 自动恢复或自动续行。

## 实际验证结果

| 检查 | 结果 | 证据边界 |
|---|---|---|
| 控制台 Python | 27项通过 | 离线任务状态、失联、许可、重复请求、停止确认、审计失败、HTTP控制席和符号链接静态资源 |
| 前端 | 2项坐标测试、TypeScript检查和生产构建通过；二维/三维浏览器检查完成 | 截图使用停车场地图和明确标记的示例位姿，不是车辆现场画面 |
| RK3588构建 | fast_lio、mcu_can_bridge、mower_nav2、mower_console 四包通过 | 最终构建日志见附件；没有在宿主机构建或运行ROS节点 |
| RK3588回归 | 332项通过 | 三个原有包的测试，含新增CAN编码许可与碰撞区域检查；CAN测试使用内存socket |
| 全图定位坐标回放 | 5组坐标×4朝向，20/20通过；约30.19–32.44秒；最大位置误差0.02424m、角度误差0.04183° | 将同一批历史真实录包点云变换到不同坐标。**不能替代5个真实地点、各4种朝向的观测验收**，也未证明U形终点定位 |
| 定位负例 | 静止信号缺失、源帧过期、搜索中移动、搜索超时、纯地面缺结构、双解歧义，6项均拒绝 | 超时负例将预算缩为1秒以验证截止路径；默认120秒正例均在上限内结束，未完成120秒最坏负载实测 |
| 人工辅助 | 旧搜索被代次2取代并成功重新匹配，约14.21秒 | 隔离坐标回放；并非现场手工定位验收 |
| Smac实际插件 | 前进1.5m、旋转90°后前进1.1m、短倒车0.4m，规划与包络检查通过；不安全目标被拒绝 | 独立ROS域，仅ComputePathToPose；没有调用运动action或CAN |
| 安装前只读整栈 | 网页状态API可读，前进/倒车两个规划进程及控制器插件加载；无运动桥；停止后无遗留进程 | 当时CAN为DOWN，没有轮速反馈，定位等待stationary；该阶段未取得导航就绪 |
| 安装后系统服务 | 用户完成sudo安装；enabled/active/running；NRestarts=0；电脑首页及JS/CSS均HTTP200，正式页面正常渲染 | 已运行且启用开机启动，未重启板端验证 |
| 真实静止观测 | 用户确认在图内、静止且无遮挡；实际点云定位accepted，状态READY/healthy；CAN已UP/500kbps，API0，取得新鲜四轮停稳反馈 | 本次单处真实观测通过软件门控；读数约(1.83m, 0.56m, -3°)，尚待现场核对位置，不能代替多地点验收 |
| 真实规划预览 | 正式HTTP接口返回200，实际ComputePathToPose生成1.50m前进路线，任务未启动 | 仅预览，没有调用FollowPath/Spin或任务开始 |
| 独立被动CAN | 3秒收到73帧0x507和50帧0x50A；未收到0x005；末帧API0、四轮0 | 该3秒窗口未观察到控制帧，不能扩展为所有运行时段证明 |
| 服务文件 | systemd单元检查完成 | 系统已有其他单元的权限/未知字段警告；本服务已安装，实测重启上电仍待完成 |

只读整栈关闭时，在定位TF尚未就绪、规划器激活等待中的条件下，Nav2规划器有退出码-6，Livox驱动有退出码-11。所有进程最终退出，没有遗留网页或运动进程，但不能把这个结果写成所有节点正常退出。保留原始日志，后续需在有效传感器/CAN条件下复查启动与关闭；未修改第三方驱动来掩盖错误。

部署时另修复了两个实际问题：colcon的符号链接安装使原目录静态资源路由返回404，已改为安装文件名白名单并增加回归测试；默认DDS通道能发现话题却收不到样本，同一被动订阅脚本默认配置5秒收到0帧，UDP配置收到86帧定位诊断和13帧里程计。全部控制台子进程现统一加载 `config/dds_console_udp.xml`，真实观测与定位随之恢复。该对照指向默认共享内存数据通路异常，未清理或改动其他任务的共享内存。配置依据：[Fast DDS 2.6 UDP传输文档](https://fast-dds.docs.eprosima.com/en/2.6.x/fastdds/transport/udp/udp.html)。

板端ROS命令行诊断也应加载同一配置；旧ros2 CLI daemon若保留旧配置，应先关闭该daemon再诊断，或使用不依赖daemon的被动订阅程序：

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/cat/MobileEss_ws/install/mower_console/share/mower_console/config/dds_console_udp.xml
```

浏览器检查截图（二维和三维为离线示例；正式板端页面已可访问）：

![二维目标与路线预览](validation_20261020/console-2d.jpg)

![三维地图观察](validation_20261020/console-3d.jpg)

## 访问和安装

正式访问地址为 `http://192.168.10.81:8080`。**系统服务已安装并运行，地址可访问，默认运动入口仍禁用。** 浏览器首次需要配对；已打开的正式页面停留在配对入口。

本次用户已完成以下安装；重新安装时由板端管理员执行，按提示输入板端sudo密码：

```bash
ssh -t cat@192.168.10.81 'bash /home/cat/MobileEss_ws/install/mower_console/share/mower_console/scripts/install_service.sh'
```

该脚本安装并立即启动观测服务。服务的特权预检查仅将DOWN的can0配置为500kbps并启用，不发送CAN控制帧；已UP且波特率不符时拒绝改动。服务固定传入 `observation_only:=true`，即便配置误启用运动也不能发车。

查看服务、取配对码：

```bash
ssh cat@192.168.10.81
systemctl status mower-console.service --no-pager
journalctl -u mower-console.service -n 100 --no-pager
cat ~/.config/mower_console/pairing-code
```

配对码只在板端查看，不写入仓库或此报告。浏览器配对后取得控制席。等待定位与实际停稳反馈，再在二维图选点并预览。三维视图用于观察。当前所有能力开关为false，开始按钮保持禁用。

服务启停和恢复原入口：

```bash
sudo systemctl stop mower-console.service
sudo systemctl start mower-console.service
sudo systemctl restart mower-console.service
# 停用开机控制台，回到原有人工场测启动流程：
sudo systemctl disable --now mower-console.service
```

停用后不要同时启动两个雷达/LIO或导航实例。旧导航配置和近提示定位入口保留；本次新运动参数独立存放于mower_console。运行中不要同步或修改配置。源码每次部署有逐文件备份；如需源码回退，应先停服务，审查每次备份清单并逐文件恢复，再在板端构建，不能整仓覆盖现有业务修改。

重新构建命令（仅板端）：

```bash
cd /home/cat/MobileEss_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select fast_lio mcu_can_bridge mower_nav2 mower_console
```

## 部署标识和原始证据

- 地图：`/home/cat/MobileEss_data/maps/field_20261009_01/keepout_AB_short_candidate_20261016`，189×204格，0.10m/格。
- 地图与校准资产版本：`9cf8e79885f4242922ebc47877669c70efb6262b8352fa44357db5d11443b46d`。
- 最终55文件清单摘要：`c400b12cad444623225888b3593f00e709f58343deae2868a79160e93a53fa17`。逐文件SHA256见 [deployment.json](validation_20261020/deployment.json)。
- 板端fixed_map_localizer二进制SHA256：`2f4709a5543cb70bd57464370ebceb9ceb21f38263139a3f9e3dd3dd8321f14d`。
- 最后一次部署备份：`/home/cat/MobileEss_data/console-deploy-20261021T035313695956Z`；该目录仅备份该次有变化的旧文件，更早变更的备份保留在此前部署目录。
- 原始回放、规划、整栈状态和日志：[validation_20261020](validation_20261020/)。板端同名原始文件位于 `/home/cat/MobileEss_data/`，逐例定位日志为 `console-global-replay-final-case-XX.log`。
- 后续真实任务日志：板端 `~/MobileEss_data/console-runs/<任务ID>.jsonl`，记录预览、确认、配置、CAN和最终停车状态。上电和重启不重放这些任务。

## 剩余验收表

| 顺序 | 必须完成的现场事项 | 当前状态 |
|---|---|---|
| 1 | sudo安装系统服务；重启板端验证网页、雷达、LIO、只读CAN自启；验证退出日志和进程清理 | 安装和服务启动已完成，重启上电及正常定位后退出复查待执行 |
| 2 | 恢复CAN，核对新鲜0x507/0x50A、API0、四轮停稳；确认实际位置与地图贴合 | 已完成当前地点检查；A1重新搜索通过，用户于2026-10-21确认车位/朝向一致，静止且无遮挡 |
| 3 | 地图内≥5真实位置×4朝向，包含U形终点，歧义、120秒超时及人工辅助；保存原始数据 | A点4个真实朝向均完成软件与现场定性核对；A3/A4首次失败保留，分别复测通过。用户关机休息，现场测试暂停，恢复后从B1继续；坐标回放不计入真实20组 |
| 4 | 后方实际覆盖/车体遮挡，负轮速符号，架空停止链与许可失效 | 未完成，倒车保持禁用 |
| 5 | 双向旋转滑移；短距倒车逐步到0.2m/s；换向停稳及旋转衔接 | 未完成 |
| 6 | 网页独立选点、预览、开始、≤0.25m到点并四轮停稳；遇障、断线、服务退出、定位失效、旧任务拒绝重放 | 未完成 |

每轮地面试验都必须分别审图、确认整段净空与CH5/独立停止准备，并取得新的单次授权；故障后重新规划和确认，不自动重试。能力验收完成前不修改 `accepted_capabilities` 或 `motion_enabled`。验收后启用也需独立本地配置并显式 `observation_only:=false`，默认开机服务持续保持只读观测。
