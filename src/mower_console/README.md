# MobileEss 浏览器导航控制台

界面运行于 RK3588，电脑通过同一局域网访问 `http://192.168.10.81:8080`。二维地图或三维地面选单目标（可选到达朝向），三维查看静态PCD与实时点云。默认上电只观测，**新运动链尚未完成实车验收，默认不能发车**。历史U形测试不代替本控制台、全局定位、倒车或旋转衔接验收。

界面按仓库根目录的 `DESIGN-apple.md` 重写：地图为主工作区，浅灰背景、系统字体、蓝色操作按钮、胶囊控件和半透明导航。桌面右侧集中显示目的地、路线分段和运动能力；平板与手机使用可展开任务面板，开始/停止操作固定在底部。二维与三维共用图层颜色、目标和路线，支持缩放、全图、回到车辆，以及地图上的定位提示。

会话与数据流在 `web/src/useConsoleSession.ts`，动作条件在 `consoleModel.ts`，页面组件分别位于 `App.tsx`、`MapWorkspace.tsx`、`TaskPanel.tsx`、`Dialogs.tsx`；视觉变量在 `style.css` 和 `mapTheme.ts`。三维按需加载，渲染失败时可回到二维。配对、开始确认与车辆详情支持键盘操作和焦点约束；状态过期时隐藏实时反馈，断连后撤销控制权。预览失效以服务端状态为准，修改目标、定位代次变化或服务重启后须重新预览和确认。

## 当前分支目标与计划（2026-11-16）

分支：`feat/console-obstacle-detour`。本轮目标是交付符合 `DESIGN-apple.md` 的全新导航控制台，并修复首次三维加载时的短暂断连，完成 RK3588 上的观测验证。**本轮目标已完成，实车运动与遇障绕行仍待验收。**

| 工作项 | 状态 | 证据与后续安排 |
|---|---|---|
| 重写界面、组件、样式与交互 | 已完成 | 地图工作区、任务面板、配对/确认/车辆详情；桌面、平板、手机检查通过 |
| 修复首次三维加载断连 | 已完成 | 状态/心跳与按需点云分流；正式服务 5 次冷加载无断连，地图下载期间状态帧最大间隔 0.406 秒 |
| 软件验证、板端部署与观测 | 原发布已完成，审查补丁待部署 | 前端 41 项、当前控制台离线 82 项通过；板端 115 项为原发布结果，车辆关闭后未重跑 |
| 遇障等待、续行与绕行软件 | 已实现，实车待验收 | 继承本分支原有实现；本次界面及网络修复没有执行运动任务 |
| 多位置、多朝向定位验收 | 待现场安排 | 按下文验收表核对至少 5 处 × 4 朝向，保留录包和定位证据 |
| 架空停止链与能力检查 | 待另行现场授权 | 定位验证和现场隔离满足要求后，检查实际轮向、许可失效、退出 API 和停止反馈 |
| 地面目标到达与遇障绕行 | 待前置验收及另行现场授权 | 检查前进/旋转，再逐项验收倒车与绕行；不得以本轮软件或观测结果勾选能力 |

当前执行边界保持 `observation_only:=true`、`motion_enabled=false` 和全部 `accepted_capabilities=false`。上述待验收项是后续计划，不是本轮运动授权；继续开发与验证时仍默认观测模式。

## 构建与部署

宿主机只构建前端和运行离线逻辑测试。本次前端验证使用 Node.js 26.13.0 / npm 11.20.0；较旧 Node 可能不满足界面测试依赖的运行要求。

```bash
cd /Users/dyh/phld/MobileEss/src/mower_console/web
npm ci
npm run build
npm test
cd /Users/dyh/phld/MobileEss
PYTHONPATH=src/mower_console:src/mcu_can_bridge python3 -m pytest src/mower_console/test
```

仅更新界面时，先运行下面的分阶段部署工具。它只上传 `web/`，不会同步 ROS 源码、地图、标定或运动配置，适合板端已有独立改动的情况：

```bash
python3 src/mower_console/scripts/deploy_console_ui.py --host cat@192.168.10.81
```

工具会打印待激活目录和一条 SSH 命令。现场管理员在终端执行该命令并完成 sudo 验证后，脚本校验上传文件与板端基线、停止观测服务、备份旧界面到 `previous-web.tar.gz`、安装新文件、在 RK3588 上构建 `mower_console`，然后重启服务。构建或启动失败时恢复旧界面。必须重启服务，静态资源白名单才会包含新构建的文件名。部署时保留输出的发布目录，供回退使用。

板端安装依赖并构建，先确认 Mutagen beta 是 `/home/cat/MobileEss_ws/src`；不要覆盖板端未审查的差异：

```bash
ssh cat@192.168.10.81
sudo apt-get install python3-aiohttp python3-yaml python3-numpy ros-humble-nav2-smac-planner
cd /home/cat/MobileEss_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select fast_lio mcu_can_bridge mower_nav2 mower_console
source install/setup.bash
ros2 launch mower_console console.launch.py
```

前端 dist 已随包安装。`/api/events` 只传 JSON 状态、控制权与心跳；实时点云和障碍点使用独立的 `/api/points?cloud=0|1` WebSocket（`u8 类型 · 3字节填充 · u32 点数 · float32 xyz`；1=配准点云≤2Hz，2=障碍点）。浏览器发送文本 `next` 请求一批最新数据，收到 `{"type":"ready"}` 后至少等 200ms 再请求，避免持续排队；二维不订阅配准点云。点云连接失效只清空显示点并重连，不延长状态或控制心跳的 1 秒超时。此协议变更需要同时部署后端和前端，不能只运行前端更新工具。配对码在板端 `~/.config/mower_console/pairing-code`，只在本机查看，不提交：

```bash
cat ~/.config/mower_console/pairing-code
```

上电服务安装（会启动观测，不发送行走CAN）：

```bash
bash /home/cat/MobileEss_ws/install/mower_console/share/mower_console/scripts/install_service.sh
systemctl status mower-console.service
journalctl -u mower-console.service -n 80 --no-pager
```

服务安装需要现场管理员执行 sudo。停止及回退：

系统服务的特权预检查仅在 `can0` 为 DOWN 时按协议配置500kbps并启用接口，未发送任何CAN帧。若已UP但波特率不同则拒绝改动。当前用户服务没有linger，因此不能把临时用户服务当作已完成开机启动；使用上面的系统服务安装。

```bash
sudo systemctl disable --now mower-console.service
```

关闭控制台后按原场测入口启动；原导航配置及近提示定位模式保留。启动脚本发现配置或地图资产缺失会退出。重新部署前先停止服务，避免运行中源文件变化。

## 操作

1. 上电后等待CAN反馈、雷达、LIO和全图定位。首次唯一稳定匹配前不显示有效车辆位姿。全图搜索最多120秒；失败后可重试，或通过“车辆详情”打开“定位与车辆”，标出粗略车位、填写朝向后重新匹配。
2. 浏览器配对，点击“取得控制权”。其他页面可观察和停止任务，不能接管正在执行的任务。
3. 在二维地图或三维地面点选目标；需要指定到达朝向时，在二维地图从目标标记拖出箭头，或取消“自动选择到达朝向”后输入角度（路线末尾增加原地转向段）。生成路线预览，核对定位、整车通行范围及原地转向位置。目标或朝向改变须重新预览。预览120秒失效。任务面板显示前进/倒车/转向的验收状态，路线含未验收运动时不能开始。
4. 完成新链路验收后，现场负责人在独立的本地配置中启用相应能力及 `motion_enabled`，停止/重启服务加载。每次开始仍须确认整段净空、CH5与独立停止、作业机构隔离。
5. 执行中 Mid-360 障碍触发安全门控（`predicted footprint cost …`）时，任务不再直接阻塞，而是：
   - 取消当前分段，150 毫秒内确认旧 Nav2 目标结束后，保持许可续租（零速），停稳等待；取消超时立即撤销许可并阻塞，不能以迟到应答继续当前任务。取消被拒绝或无法确认终态时，保留未结束目标，阻止再次接管，需重启观测处理；
   - 剩余路线在门控包络（`obstacle_clearance_m`，默认1.1m = 门控盘0.70m + 局部代价地图内切半径0.404m）内连续1秒无实时障碍点 → 沿原预览路线剩余部分继续；
   - 等待 `obstacle_wait_s`（默认10秒）仍阻塞 → 从当前位姿向同一目标重新规划绕行。绕行路线须通过与预览相同的分段、静态栅格0.98m包络、实时障碍包络、能力验收和总行程42m检查，通过才执行，并在审计文件写入 `replan` 记录；网页显示新路线，原预览以虚线保留；
   - 绕行次数用完（`max_replans`，默认3）、单任务累计等待超过 `obstacle_total_wait_s`（默认60秒）或同一任务反复遇障超过20次 → 停车并阻塞。
   断线、定位、CAN、后方观测或其他数据失效仍立即停车并消耗本次许可；恢复后重新预览和开始；普通网页重连、服务重启不恢复旧任务。

前进0.3m/s，角速0.2rad/s；本阶段 `planners: [ForwardGrid]` 只规划前进和原地转向，不回落 ReverseGrid（倒车验收后才在本地配置加入；届时倒车0.2m/s、连续最多1m）；曲线参考半径至少4m。最多40m/600秒；定位读数目标误差≤0.25m并实际停稳才完成。0x507是命令回显，0x50A才是实际四轮反馈。

## 验收记录

### 2026-11-16 新界面与板端观测

- 分支：`feat/console-obstacle-detour`。前端 39 项测试、TypeScript 检查与 Vite 生产构建通过；宿主机控制台 Python 测试 61 项通过。RTK/CAN 离线合同测试 42 项通过、2 项依赖 ROS 的测试模块跳过；另一个直接导入 ROS 消息的模块在宿主机排除，须在板端执行。
- 板端 source ROS 与 workspace 后，控制台、CAN 桥、RTK 测试合计 111 项通过，无跳过。测试使用内存适配器/套接字，不发实车控制帧。缺少的 pytest/asyncio 测试依赖安装在 `/tmp/mobileess-console-ui-qa-deps`，未改动系统运行依赖；执行时保留 ROS 的原有 `PYTHONPATH`。
- 已在 RK3588 构建并激活 `mower_console`，26 个部署文件的 SHA-256 与发布清单一致。发布和旧版备份目录为 `/home/cat/MobileEss_data/console-ui-apple-20261117T022912Z-6e6b154c`；系统服务恢复 `active/running`，HTTP 返回新构建资源。
- 实时定位 accepted，位置约 `(3.66, 0.84)` m，实际轮速为 0、API 模式为 0。二维选点 `(6.65, 0.69)` m 后生成约 3.05 m、4 分段的真实路线预览；未启动任务。4 秒只读流检查收到 20 个状态帧、7 个点云帧、19 个障碍帧，二进制长度与点数一致。
- 桌面 1440/1024、平板 768、手机 390 宽度均已检查；手机无横向溢出，车辆详情与 Escape 关闭可用。首次进入板端三维时出现一次短暂断连，自动重连并撤销控制权；之后连续切换二维/三维 3 次均保持连接和控制权。当时尚未确定原因，后续已按下一节完成复现、修复和正式服务验证；保留原有 1 秒状态超时规则。
- 服务保持 `observation_only:=true`，`motion_enabled=false`，运动能力未验收，未运行行走 CAN 桥。上述证据仅覆盖界面和板端观测，不构成实车定位精度、行驶、旋转、倒车或遇障绕行验收。

发布回退须在现场终端执行，`release_dir` 使用需恢复的发布目录：

```bash
release_dir=/home/cat/MobileEss_data/console-ui-apple-20261117T022912Z-6e6b154c
sudo systemctl stop mower-console.service
python3 "$release_dir/apply.py" restore
cd /home/cat/MobileEss_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select mower_console --symlink-install
sudo systemctl start mower-console.service
```

### 2026-11-16 首次三维加载断连修复

根因是状态与实时点云共用同一条 WebSocket 和串行发送循环。首次三维加载还会下载约 847 KB 静态地图，在当前局域网传输受限时，点云积压拖住后续状态，触发前端 1 秒状态超时。测得 WebGL 创建约 5–9 ms、首次绘制约 7–14 ms；板端本机状态间隔不超过 0.27 秒，Mac 下载地图时旧实现达到 2.65 秒，问题不是浏览器渲染阻塞。

修复把显示点数据移到独立连接，按浏览器消费进度请求最新批次，二维只取障碍点。状态流、控制心跳、观测配置及运动许可规则保持原有要求。修复版在 RK3588 独立只读中继中使用真实状态/点云复测，同样的地图下载期间状态间隔最高 0.45 秒；5 次刷新后首次切入三维均保持连接，地图加载完成后仍保有控制权。该中继不运行 ROS 节点、不打开 CAN，也不转发运动请求。

验证：`npm test` 41 项通过，`npm run build` 通过；控制台 Python 测试 65 项通过；RK3588 上控制台、CAN 桥与 RTK 测试合计 115 项通过。新增测试覆盖点云发送受阻时状态持续更新、显示连接超时不撤销控制权、按需传输和配对/来源限制。能直接重现原问题的回归命令是：

```bash
PYTHONPATH=src/mower_console:src/mcu_can_bridge python3 -m pytest src/mower_console/test/test_point_stream.py -q
```

正式服务已通过发布包 `/home/cat/MobileEss_data/console-stream-fix-20261117T025911Z-0316735b` 激活，32 个部署文件校验一致，旧前后端保存在该目录的 `previous-package.tar.gz`。激活后在正式 8080 页面再次冷加载三维 5 次，均未断连；实际点云传输期间下载静态地图，状态帧最大间隔为 0.406 秒，包含下载前基线的整轮最高为 0.526 秒。正式安装目录的 115 项测试通过，`motion_enabled=false`、API 0、无任务、未启动行走 CAN 桥。若需回退，先停止服务，执行此发布目录的 `apply.py restore`，再在板端构建 `mower_console` 并重启服务；该备份包含服务端与前端。

慢网络仍会延长静态地图加载时间；真实状态流若超过 1 秒未更新，仍撤销控制权，不将其伪装成正常连接。以上测试未启动车辆，不替代实车运动验收。

### 本次提交与验收归档

本次提交包含设计依据、控制台源码与生产构建、状态/点云分流修复、回归测试、前端部署工具，以及目标、计划和验收记录。提交前只读复核：正式服务 `active/running`，发布清单中的 32 个文件匹配，观测启动参数和全部运动禁用配置保持不变。本轮提交不包含地图、标定、配对码或本地临时诊断文件。

以下截图来自本轮已经完成的实机验证，随代码归档：

| 验收画面 | 对应阶段 |
|---|---|
| [二维地图与真实路线预览](../../docs/validation/console-2026-11-16/board-2d.jpg) | 界面重写后的观测联调，未开始任务 |
| [三维分流修复后的正式服务](../../docs/validation/console-2026-11-16/board-3d-fixed.jpg) | 正式 8080 服务，连接和控制心跳正常，观测模式 |
| [390px 手机布局](../../docs/validation/console-2026-11-16/board-mobile-390.jpg) | 手机界面无横向溢出，任务操作区保持可见 |

### 2026-11-16 PR #3 合并前审查

- 需求审查发现遇障取消应答较慢时许可可能失效：旧实现模拟 0.4 秒取消清理，续租间隔达到约 0.452 秒，超过 0.25 秒许可，但任务仍显示等待障碍。修复不延长取消过程中的许可；150 毫秒内未确认取消即撤销并阻塞，继续等待原取消清理，避免再次取消打断确认。正常取消后重新检查定位、健康和控制权，再进入停稳等待。
- 取消应答不等于目标结束。按 [ROS 2 Humble CancelGoal 定义](https://github.com/ros2/rcl_interfaces/blob/humble/action_msgs/srv/CancelGoal.srv)，核对接受/已结束应答并等待终态；拒绝、超时、未知终态和迟到目标都不能被当作成功。未结束目标或待接受请求保留在适配器中，禁止新目标与再次接管。停止过程即使取消失败也关闭运动进程，取消失败不会被误判为可恢复障碍。
- 规范审查移除了 `RosAdapter.plan` 对必备内部 `config` 属性的兜底，补全对应测试对象；合法的可选规划器默认值保留。
- 本次宿主机复跑：前端 `npm test` **41 项通过**，`npm run build` 通过且产物与已提交版本一致；控制台 Python **82 项通过**，包含慢取消、人工停止、失联、健康故障、拒绝取消、迟到目标、终态及停止清理回归。RTK 离线 **12 项通过**；CAN 离线 **30 项通过、2 个 ROS 依赖模块跳过**，直接依赖 `geometry_msgs` 的 `test_motion_permit_can.py` 留待板端执行。新增代码 `git diff --check` 通过。
- 车辆已由操作员关闭。本次未连接、启动或部署 RK3588；此前 **115 项板端测试、5 次三维冷加载及截图仍对应 `4630a90` 发布**，不作为本审查补丁的板端验收。补丁须在下次上电后先以观测模式部署与复验，运动能力继续全部禁用。

### 后续实车验收边界

| 层级 | 必须核对 | 当前证据应如何表述 |
|---|---|---|
| 宿主机 | UI构建、地图坐标、任务重试、预览变化、断线、许可过期、HTTP控制席 | 离线逻辑/界面检查，不是车辆结果 |
| 板端 | 四包构建、ROS接口、Smac原语、全图搜索超时/歧义/旧结果拒绝 | 构建和无CAN集成结果 |
| 定位实测 | ≥5处×4朝向，包含U形终点；错误地点不accepted；120秒；人工辅助 | 需真实静止或实测录包，合成点云不能代替 |
| 架空检查 | 负速度符号、许可失效、退出API、停止链 | 架空测试，不代替地面结果 |
| 地面能力 | 双向旋转滑移、后方遮挡/障碍、短距倒车逐步到0.2m/s、换向停稳 | 每轮现场审图、隔离监护、新授权；无自动重试 |
| 网页闭环 | 选点到达、遇障、关闭网页、后端退出、定位失效、服务重启拒绝重放 | 逐轮原始轨迹/CAN及操作员观察 |
| 遇障等待/绕行 | 架空：遮挡前方→停稳、许可不撤销、移开后续行；地面0.2m/s：行人穿越后续原路线；路线中段静态障碍→10秒后绕行到达，核对绕行间距；通道堵死→绕行用尽后阻塞并确认停车 | 每轮保留rosbag、门控诊断、CAN与审计 `replan` 记录 |

后向近场覆盖不合格时保持倒车禁用；不得仅凭软件支持负速度或360度雷达规格勾选验收。软件旋转初始包络增加0.25m，仍需现场检查滑移。新任务原始CAN和预览写入板端 `~/MobileEss_data/console-runs/<任务ID>.jsonl`，包含最终停车确认。

现场验收完成后使用独立本地配置，设置相应 `accepted_capabilities` 和 `motion_enabled`，并显式 `observation_only:=false` 启动；默认系统服务始终带 `observation_only:=true`，即使误改配置也不能发车。每轮仍须网页现场确认。

全局搜索复用ICP，以全图自由区候选、独立模式比较和新鲜后续帧确认降低误匹配；环境不可辨认时必须拒绝定位。该实现仍需要上述多位置真实数据验收，不能承诺任意环境一定在120秒内成功。

## 离线界面预览

可在宿主机用包含二维和三维地图的配置执行：

```bash
PYTHONPATH=src/mower_console python3 -m mower_console.server --demo --config /tmp/mower-console-demo.yaml --static /Users/dyh/phld/MobileEss/src/mower_console/web/dist --host 127.0.0.1 --port 8080
```

`--demo` 不导入ROS、不打开CAN；示例位姿有显著离线标记，始终禁止运动。
