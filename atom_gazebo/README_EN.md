# Atom Gazebo (default)

Default Gazebo simulation for Atom W-P3 wheeled humanoid robot.

## Launch

```bash
# Full body
ros2 launch atom_gazebo atom_gazebo.launch.py

# Arms only
ros2 launch atom_gazebo atom_arms_only.launch.py
```

## Parameters

| Param | Default | Description |
|-------|---------|-------------|
| `use_sim_time` | `true` | Use simulation clock |
| `paused` | `false` | Pause on start |
| `gui` | `true` | Show GUI |
| `world` | empty | Gazebo world file |

## Structure

| Path | Description |
|------|-------------|
| `launch/` | Launch files |
| `urdf/` | Xacro overlay |
| `config/` | Controller config |
| `worlds/` | Gazebo worlds |

## Dependencies

atom_urdf, gazebo_ros, ros2_control, gazebo_ros2_control
