# 最小可运行 Demo

## 1. 构建

```bash
cd /home/pkuiflab/VRX_VLN
./build_vrx_vln.sh
```

## 2. 启动系统

```bash
cd /home/pkuiflab/VRX_VLN
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch marine_vln_vrx marine_vln_system.launch.py headless:=False rviz:=True
```

该 launch 默认加载 `vrx_vln_buoy_test.sdf`，场景内有一对红/绿浮标。

## 3. 发送一条指令

```bash
ros2 topic pub --once /vln/instruction_text std_msgs/msg/String "{data: 'Pass between the red and green buoys'}"
```

或

```bash
ros2 run marine_vln_vrx publish_instruction "Go to waypoint A"
```

## 4. 观察关键话题

```bash
ros2 topic echo /vln/task_json --once
ros2 topic echo /vln/subgoals_json --once
ros2 topic echo /vln/cmd_vel_safe --once
```

## 5. 查看实验日志

日志路径：

`/home/pkuiflab/VRX_VLN/src/marine_vln_vrx/logs/<run_id>/`
