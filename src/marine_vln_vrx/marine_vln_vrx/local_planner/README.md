# local_planner

职责：
- 订阅子目标序列与语义地图
- 生成局部路径 `nav_msgs/Path`
- 使用线性插值 + 障碍物斥力做最小可运行避障

输入：
- `/vln/subgoals`
- `/vln/semantic_map`
- `/wamv/sensors/position/ground_truth_odometry`

输出：
- `/vln/local_path`
