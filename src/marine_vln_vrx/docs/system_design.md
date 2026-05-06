# Marine-VLN 系统设计（最小版本）

## ROS 2 通信图

```text
/vln/instruction_text (String) + [/vln/semantic_map] + [camera/compressed]
        |
        v
instruction_manager ------------------------------> /vln/task_json (String)

/wamv/sensors/* + data/scene_objects.yaml
        |
        v
scene_parser -------------------------------------> /vln/scene_state (String)
        |
        v
semantic_mapper ----------------------------------> /vln/semantic_map (String)

/vln/task_json + /vln/semantic_map
        |
        v
subgoal_planner ----------------------------------> /vln/subgoals (PoseArray)
                                                  -> /vln/subgoals_json (String)

/vln/subgoals + /vln/semantic_map + odom
        |
        v
local_planner ------------------------------------> /vln/local_path (Path)

/vln/local_path + odom
        |
        v
controller ---------------------------------------> /vln/cmd_vel_raw (Twist)
                                                  -> /vln/raw/left_thrust (Float64)
                                                  -> /vln/raw/right_thrust (Float64)
                                                  -> /vln/raw/left_pos (Float64)
                                                  -> /vln/raw/right_pos (Float64)

/vln/semantic_map + /vln/raw/* + /vln/cmd_vel_raw
        |
        v
safety_monitor -----------------------------------> /vln/cmd_vel_safe (Twist)
                                                  -> /vln/replan_required (Bool)
                                                  -> /wamv/thrusters/left/right/thrust (Float64)
                                                  -> /wamv/thrusters/left/right/pos (Float64)
```
