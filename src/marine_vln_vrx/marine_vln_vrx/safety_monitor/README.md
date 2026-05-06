# safety_monitor

职责：
- 检测语义地图中的最近障碍风险
- 对 `controller` 原始控制输出执行减速/急停覆盖
- 输出最终推进器控制到 WAM-V 话题

输入：
- `/vln/semantic_map`
- `/vln/cmd_vel_raw`
- `/vln/raw/*`

输出：
- `/vln/cmd_vel_safe`
- `/wamv/thrusters/left|right/thrust`
- `/wamv/thrusters/left|right/pos`
- `/vln/replan_required`
