# MobileEss 浏览器导航控制台

界面运行于 RK3588，电脑通过同一局域网访问 `http://192.168.10.81:8080`。二维地图选单目标，三维查看点云。默认上电只观测，**新运动链尚未完成实车验收，默认不能发车**。历史U形测试不代替本控制台、全局定位、倒车或旋转衔接验收。

## 构建与部署

宿主机只构建前端和运行离线逻辑测试：

```bash
cd /home/dyh/MobileEss/src/mower_console/web
npm ci
npm run build
npm test
cd /home/dyh/MobileEss
PYTHONPATH=src/mower_console:src/mcu_can_bridge python3 -m pytest src/mower_console/test
```

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

前端 dist 已随包安装。配对码在板端 `~/.config/mower_console/pairing-code`，只在本机查看，不提交：

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

1. 上电后等待CAN反馈、雷达、LIO和全图定位。首次唯一稳定匹配前不显示有效车辆位姿。全图搜索最多120秒；失败后可重试，或在“定位与车辆详情”中标出粗略车位、填写朝向后重新匹配。
2. 浏览器配对，点击“取得控制权”。其他页面可观察和停止任务，不能接管正在执行的任务。
3. 二维地图点选目标，生成路线预览，核对定位、整车通行范围、倒车及原地转向位置。目标改变须重新预览。预览120秒失效。
4. 完成新链路验收后，现场负责人在独立的本地配置中启用相应能力及 `motion_enabled`，停止/重启服务加载。每次开始仍须确认整段净空、CH5与独立停止、作业机构隔离。
5. 遇障、断线、定位或数据失效会停车并消耗本次许可。恢复后重新预览和开始；普通网页重连、服务重启不恢复旧任务。

前进0.3m/s，角速0.2rad/s；倒车0.2m/s、连续最多1m；曲线参考半径至少4m。最多40m/600秒；定位读数目标误差≤0.25m并实际停稳才完成。0x507是命令回显，0x50A才是实际四轮反馈。

## 验收记录

| 层级 | 必须核对 | 当前证据应如何表述 |
|---|---|---|
| 宿主机 | UI构建、地图坐标、任务重试、预览变化、断线、许可过期、HTTP控制席 | 离线逻辑/界面检查，不是车辆结果 |
| 板端 | 四包构建、ROS接口、Smac原语、全图搜索超时/歧义/旧结果拒绝 | 构建和无CAN集成结果 |
| 定位实测 | ≥5处×4朝向，包含U形终点；错误地点不accepted；120秒；人工辅助 | 需真实静止或实测录包，合成点云不能代替 |
| 架空检查 | 负速度符号、许可失效、退出API、停止链 | 架空测试，不代替地面结果 |
| 地面能力 | 双向旋转滑移、后方遮挡/障碍、短距倒车逐步到0.2m/s、换向停稳 | 每轮现场审图、隔离监护、新授权；无自动重试 |
| 网页闭环 | 选点到达、遇障、关闭网页、后端退出、定位失效、服务重启拒绝重放 | 逐轮原始轨迹/CAN及操作员观察 |

后向近场覆盖不合格时保持倒车禁用；不得仅凭软件支持负速度或360度雷达规格勾选验收。软件旋转初始包络增加0.25m，仍需现场检查滑移。新任务原始CAN和预览写入板端 `~/MobileEss_data/console-runs/<任务ID>.jsonl`，包含最终停车确认。

现场验收完成后使用独立本地配置，设置相应 `accepted_capabilities` 和 `motion_enabled`，并显式 `observation_only:=false` 启动；默认系统服务始终带 `observation_only:=true`，即使误改配置也不能发车。每轮仍须网页现场确认。

全局搜索复用ICP，以全图自由区候选、独立模式比较和新鲜后续帧确认降低误匹配；环境不可辨认时必须拒绝定位。该实现仍需要上述多位置真实数据验收，不能承诺任意环境一定在120秒内成功。

## 离线界面预览

可在宿主机用包含二维和三维地图的配置执行：

```bash
PYTHONPATH=src/mower_console python3 -m mower_console.server --demo --config /tmp/mower-console-demo.yaml --static /home/dyh/MobileEss/src/mower_console/web/dist --host 127.0.0.1 --port 8080
```

`--demo` 不导入ROS、不打开CAN；示例位姿有显著离线标记，始终禁止运动。
