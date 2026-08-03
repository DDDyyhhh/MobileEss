## 执行 Ticket 的硬性规则

### 最小任务原则
- 每个 ticket 只能完成一个可观察行为变化。
- 如果 ticket 同时包含建图、定位、导航、参数调优、UI、日志、部署，必须先拆分。
- 优先选择能在本机、仿真、rosbag replay 或单节点启动中验证的最小改动。
- 不为了“未来扩展”新增抽象层、框架、复杂配置。

### 编码前必须输出
在修改代码前，先写出：
- 本 ticket 的成功标准。
- 最小复现步骤。
- 预期可观察结果。
- 计划修改的文件范围。
- 不确定点和需要用户确认的问题。

如果无法写出“复现步骤”和“预期结果”，暂停并询问用户，不能直接编码。

### ROS2 项目验证要求
每个 ROS2 ticket 完成时，必须提供至少一种可手动执行的验证方式：

- 构建验证：例如 `colcon build --packages-select <pkg>`
- 单测验证：例如 `colcon test --packages-select <pkg>`
- 节点启动验证：例如 `ros2 launch <pkg> <file.launch.py>`
- 话题验证：例如 `ros2 topic echo /xxx`、`ros2 topic hz /xxx`
- TF 验证：例如 `ros2 run tf2_tools view_frames`
- rosbag 复现：例如 `ros2 bag play <bag>`
- RViz 观察项：明确应该看到什么变化
- 实物验证步骤：明确部署命令、启动顺序、安全停止方式、预期运动或传感器表现

不能只写“测试通过”，必须写清楚用户如何复现。

### 完成标准
只有满足以下条件才允许标记完成：
- 改动与 ticket 直接相关。
- 已运行最小必要验证。
- 输出实际验证结果。
- 给出用户可复制执行的复现命令。
- 说明预期现象。
- 如果某项验证无法运行，必须明确写出原因和剩余风险。

### 禁止行为
- 禁止一次 ticket 改多个无关问题。
- 禁止引入与当前目标无关的安全、防御、fallback、兼容层。
- 禁止用 mock 或前端模拟冒充真实业务验证。
- 禁止没跑 e2e 就声称 e2e 通过。
- 禁止每改一个小点就默认跑全量测试；应先跑最小相关测试，只有影响公共接口、构建配置、核心算法时才跑更大范围测试。
- 禁止发现需求矛盾后自行猜测实现，必须暂停询问。

## Agent skills

### Issue tracker

Issues and specs live as markdown files under `.scratch/<feature>/`. See `docs/agents/issue-tracker.md`.

### Triage labels

Use the default labels: `needs-triage`, `needs-info`, `ready-for-agent`, `ready-for-human`, and `wontfix`. See `docs/agents/triage-labels.md`.

### Domain docs

This is a single-context repo: read root `CONTEXT.md` and `docs/adr/` when relevant. See `docs/agents/domain.md`.
