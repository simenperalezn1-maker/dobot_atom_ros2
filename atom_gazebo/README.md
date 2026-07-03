# Atom Gazebo (default)

Gazebo simulation for Atom W-P3 wheeled humanoid robot. / Atom W-P3 轮式人形机器人 Gazebo 仿真（默认）。

## Launch / 启动

```bash
# Full body / 全身
ros2 launch atom_gazebo atom_gazebo.launch.py

# Arms only / 仅手臂
ros2 launch atom_gazebo atom_arms_only.launch.py
```

## Parameters / 启动参数

| Param | Default | Description |
|-------|---------|-------------|
| `use_sim_time` | `true` | Use simulation clock / 仿真时钟 |
| `paused` | `false` | Pause on start / 启动暂停 |
| `gui` | `true` | Show GUI / 显示界面 |
| `world` | empty | Gazebo world / 世界文件 |

## Structure / 目录

| Path | Description |
|------|-------------|
| `launch/` | Launch files / 启动文件 |
| `urdf/` | Xacro overlay / Xacro 叠加层 |
| `config/` | Controller config / 控制器配置 |
| `worlds/` | Gazebo worlds / 世界文件 |

## Dependencies / 依赖

atom_urdf, gazebo_ros, ros2_control, gazebo_ros2_control
