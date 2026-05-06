# subgoal_planner

职责：
- 订阅结构化任务 `/vln/task_json` 和语义地图 `/vln/semantic_map`
- 生成语义子目标序列（`PoseArray`）
- 支持 `go_to/pass_between/circle/avoid/stop_near`

发布：
- `/vln/subgoals` (`geometry_msgs/PoseArray`)
- `/vln/subgoals_json` (`std_msgs/String`)

参数：
- `circle_radius`, `circle_points`
- `stop_near_distance`, `avoid_distance`
- `replan_topic`
- `log_root`
