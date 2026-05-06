# scene_parser

职责：
- 订阅 WAM-V 状态与传感器：`odom/imu/gps/camera/lidar`
- 读取简化语义对象真值（YAML）
- 生成局部场景 JSON 并发布 `/vln/scene_state`

参数：
- `odom_topic`, `imu_topic`, `gps_topic`, `camera_topic`, `lidar_topic`
- `object_file`
- `scene_topic`
- `publish_rate_hz`
- `log_root`
