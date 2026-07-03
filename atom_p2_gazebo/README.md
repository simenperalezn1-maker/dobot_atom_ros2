# Atom P2 Gazebo

Gazebo simulation for Atom P2 legged humanoid robot. / Atom P2 足式人形机器人 Gazebo 仿真。

## Launch / 启动

```bash
# Full / 全身
ros2 launch atom_p2_gazebo atom_p2_gazebo.launch.py

# Arms only / 仅手臂
ros2 launch atom_p2_gazebo atom_p2_arms_only.launch.py
```

## Structure / 目录

| Path | Description |
|------|-------------|
| `launch/` | Launch files / 启动文件 |
| `scripts/` | Arm control scripts / 手臂控制脚本 |
| `config/` | RViz config / RViz 配置 |

## Dependencies / 依赖

atom_urdf, gazebo_ros, ros2_control, gazebo_ros2_control
