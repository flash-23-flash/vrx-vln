# 节点 I/O 话题与消息设计

## instruction_manager

- 订阅：`/vln/instruction_text` (`std_msgs/String`)
- 可选订阅：
  - `/vln/semantic_map` (`std_msgs/String`)：用于把场景上下文喂给 VLM
  - `/wamv/sensors/cameras/front_camera_sensor/optical/image_raw/compressed` (`sensor_msgs/CompressedImage`)：可选图像输入
- 发布：`/vln/task_json` (`std_msgs/String`, JSON)

## scene_parser

- 订阅：
  - `/model/wamv/odometry` (`nav_msgs/Odometry`)：主 odom
  - `/wamv/sensors/position/ground_truth_odometry` (`nav_msgs/Odometry`)：兼容回退 odom
  - `/wamv/sensors/imu/imu/data` (`sensor_msgs/Imu`)
  - `/wamv/sensors/gps/gps/fix` (`sensor_msgs/NavSatFix`)
  - `/wamv/sensors/cameras/front_camera_sensor/optical/image_raw` (`sensor_msgs/Image`)
  - `/wamv/sensors/lidars/lidar_wamv_sensor/points` (`sensor_msgs/PointCloud2`)
- 发布：`/vln/scene_state` (`std_msgs/String`, JSON)

## semantic_mapper

- 订阅：`/vln/scene_state` (`std_msgs/String`)
- 发布：`/vln/semantic_map` (`std_msgs/String`)

## subgoal_planner

- 订阅：
  - `/vln/task_json` (`std_msgs/String`)
  - `/vln/semantic_map` (`std_msgs/String`)
  - `/vln/replan_required` (`std_msgs/Bool`)
- 发布：
  - `/vln/subgoals` (`geometry_msgs/PoseArray`)
  - `/vln/subgoals_json` (`std_msgs/String`)

## local_planner

- 订阅：
  - `/vln/subgoals` (`geometry_msgs/PoseArray`)
  - `/vln/semantic_map` (`std_msgs/String`)
  - `/model/wamv/odometry` (`nav_msgs/Odometry`)：主 odom
  - `/wamv/sensors/position/ground_truth_odometry` (`nav_msgs/Odometry`)：兼容回退 odom
- 发布：`/vln/local_path` (`nav_msgs/Path`)

## controller

- 订阅：
  - `/vln/local_path` (`nav_msgs/Path`)
  - `/model/wamv/odometry` (`nav_msgs/Odometry`)：主 odom
  - `/wamv/sensors/position/ground_truth_odometry` (`nav_msgs/Odometry`)：兼容回退 odom
- 发布：
  - `/vln/cmd_vel_raw` (`geometry_msgs/Twist`)
  - `/vln/raw/left_thrust` (`std_msgs/Float64`)
  - `/vln/raw/right_thrust` (`std_msgs/Float64`)
  - `/vln/raw/left_pos` (`std_msgs/Float64`)
  - `/vln/raw/right_pos` (`std_msgs/Float64`)

## safety_monitor

- 订阅：
  - `/vln/semantic_map` (`std_msgs/String`)
  - `/vln/cmd_vel_raw` (`geometry_msgs/Twist`)
  - `/vln/raw/left_thrust` (`std_msgs/Float64`)
  - `/vln/raw/right_thrust` (`std_msgs/Float64`)
  - `/vln/raw/left_pos` (`std_msgs/Float64`)
  - `/vln/raw/right_pos` (`std_msgs/Float64`)
- 发布：
  - `/vln/cmd_vel_safe` (`geometry_msgs/Twist`)
  - `/vln/replan_required` (`std_msgs/Bool`)
  - `/wamv/thrusters/left/thrust` (`std_msgs/Float64`)
  - `/wamv/thrusters/right/thrust` (`std_msgs/Float64`)
  - `/wamv/thrusters/left/pos` (`std_msgs/Float64`)
  - `/wamv/thrusters/right/pos` (`std_msgs/Float64`)
