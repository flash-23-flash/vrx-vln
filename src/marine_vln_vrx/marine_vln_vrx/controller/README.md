# controller

职责：
- 跟踪 `/vln/local_path`
- 使用 Pure Pursuit 生成速度/角速度
- 用 PI 速度环 + 差动推力映射得到左右推进器原始指令

输出：
- `/vln/cmd_vel_raw`
- `/vln/raw/left_thrust`, `/vln/raw/right_thrust`
- `/vln/raw/left_pos`, `/vln/raw/right_pos`

参数：
- `lookahead_distance`, `goal_tolerance`
- `kp_yaw`, `kp_speed`, `ki_speed`
- `kff_thrust`, `yaw_to_thrust_gain`
- `max_thrust`, `min_thrust`
