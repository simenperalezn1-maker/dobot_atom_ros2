# Dobot Atom Robot ROS2 Supporting Packages

This project provides comprehensive ROS2 support for the Dobot Atom humanoid robot, including robot modeling, simulation environments, control interfaces, and demonstration applications.

**Last Updated**: 2026-7-3
**Development & Maintenance**: dobot_futingxing

## 📋 Directory

- [Environment Setup](#environment-setup)
- [Installation](#installation)
- [Quick Start](#quick-start)
- [Package Overview](#package-overview)
- [Troubleshooting](#troubleshooting)

## 🚀 Environment Setup

### System Requirements

**Operating System & ROS2 Distribution:**

| System         | ROS2 Distribution |
| ------------ | --------- |
| Ubuntu 20.04 | Foxy      |
| Ubuntu 22.04 | Humble    |

### Dependencies Installation

#### Installing ROS 2

##### 1. Set Locale

```bash
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

##### **2. Add the ROS 2 Repository**

```bash
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

##### 3. Install ROS 2

```bash
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop
```

##### 4. Set Up the Environment

```bash
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

##### 5. Run the Turtlesim Demo

Open two terminals and run the following commands respectively:

```bash
ros2 run turtlesim turtlesim_node
ros2 run turtlesim turtle_teleop_key
```

- The first command launches the Turtlesim simulator with a blue background.
- The second command launches the teleoperation node, allowing you to control the turtle's movement using the arrow keys.

![turtlesim](/image/image2544.png)

---

#### Install Gazebo

##### Installation

```bash
sudo apt install ros-humble-gazebo-*
```

##### Set Up Environment Variables

```bash
echo "source /usr/share/gazebo/setup.bash" >> ~/.bashrc
```

##### **Run**

```bash
ros2 launch gazebo_ros gazebo.launch.py
```

---

#### **Install MoveIt**

##### **Installation**

```bash
sudo apt-get install ros-humble-moveit
```

#### 2. Install Dependencies

```bash
# ROS2 Control
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers
# URDF & Visualization Tools
sudo apt install ros-humble-urdf ros-humble-xacro
sudo apt install ros-humble-robot-state-publisher ros-humble-joint-state-publisher
sudo apt install ros-humble-joint-state-publisher-gui

# Build Tools
sudo apt install python3-colcon-common-extensions
sudo apt install python3-rosdep python3-vcstool

# DDS Implementations
sudo apt install ros-humble-rmw-cyclonedds-cpp

```

## 📦 Installation

### 1. Clone the Repository

```bash
# Create a workspace
mkdir -p ~/atom_ros2_ws/src
cd ~/atom_ros2_ws/src

# Clone this repository
git clone https://github.com/Dobot-Arm/dobot_atom_ros2.git
```

### 2. Install Dependencies

```bash
cd ~/atom_ros2_ws

# Initialize rosdep (if this is your first time using it)
sudo rosdep init
rosdep update

# Install package dependencies
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Build the Workspace

```bash
cd ~/atom_ros2_ws
colcon build

# Set up environment variables
source install/setup.bash
```

### 4. Configure the Environment

```bash
# Copy configuration files to the user directory
cp src/atom_ros2/setup*.sh ~/

# Choose the configuration file as needed:
# 1. Connect to the real robot
source ~/setup.sh

# 2. Local Simulation Environment
source ~/setup_local.sh

# 3. Default Configuration
source ~/setup_default.sh

# To make these changes permanent, add them to your environment variables
sudo gedit ~/.bashrc
```

## 🔧 Configuration

### Configure Network 

If connecting to the real robot, configure the network interface:

   1.Set the IP address to the `192.168.8.xx` subnet:
   ![IP](/image/IP.jpg)

2. Check the network interface name:

```bash
ifconfig
# or
ip addr show
```

3. Update the network interface name in `setup.sh`:

```bash
# Edit configuration file
gedit ~/setup.sh

# Modify "eth0" in this line to match your actual network interface name
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
    <NetworkInterface name="eth0" priority="default" multicast="default" />
</Interfaces></General></Domain></CycloneDDS>'
```

### Connection Test

After completing the configuration above, restart the ros2 daemon: run ros2 daemon stopfollowed by ros2 daemon start.
```bash
source ~/setup.sh
ros2 topic list
```

You should see the following topics:

![topic](/image/topic.jpg)

Open a terminal and inspect any topic ：e.g., ros2 topic echo /xxxx. If data is streaming, the communication is working correctly:

![topic_info.](/image/topic_info.jpg)

## 🎯 Quick Start

Please refer to the README.md files within each functional module for details.

## 📁 Package Overview
### atom_urdf

- **Description**: Main robot model file containing complete geometry and joint definitions.
- **Contents**:
  - `urdf/atom.urdf`: Main robot model file containing complete geometry and joint definitions.
  - `urdf/atom_gazebo.xacro`: Gazebo Main robot model file containing complete geometry and joint definitions.
  - `meshes/`: Robot 3D mesh files used for visualization and collision detection.
  - `config/`: Robot parameter configuration files.

### atom_gazebo

- **Description**: (Default) Gazebo simulation for the Atom W-P3 wheeled humanoid robot .
- **Content**:
  - `launch/atom_gazebo.launch.py`: Full-body simulation launch file.
  - `launch/atom_arms_only.launch.py`: Arms-only simulation launch file.
  - `urdf/atom_gazebo_control.xacro`: ros2_control simulation overlay.
  - `config/atom_w_p3_controllers.yaml`:Controller parameter configuration.
  - `worlds/empty.world`: Empty world file.
  - Supports ros2_control framework，provide wheeled base + 22-DOF body control.

### atom_p2_gazebo

- **Description**: Gazebo simulation for the Atom P2 legged humanoid robot.
- **Content**:
  - `launch/atom_p2_gazebo.launch.py`: Full-body simulation launch file.
  - `launch/atom_p2_arms_only.launch.py`: Arms-only simulation launch file.
  - `scripts/control_arms.py`: Dual-arm control script.

### dobot_atom

- **Description**: Message definitions for the Atom robot.
- **Content**:
  - `msg/`: Custom message types.
  - Definitions for robot state and control messages.

    ![interface](/image/interface.jpg)

### dobot_atom_rviz

- **Description**: RViz visualization configuration
- **Content**:
  - `launch/dobot_rviz.launch.py`: RViz launch files
  - `rviz/`: RViz configuration files

    ![rviz](/image/rviz.jpg)

### atom_control_examples

- **Description**: Control example programs (see atom_control_examples/README.mdfor details)
- **Content**:
  - Various control algorithm examples
  - Motion planning examples
  - Sensor data processing examples

## 🐛 Troubleshooting

### Troubleshooting

#### 1. Gazebo Fails to Launch

```bash
# Verify Gazebo installation
gazebo --version

# Reinstall Gazebo
sudo apt install gazebo
```

#### 2. Controller Fails to Load

```bash
# Check controller status
ros2 control list_controllers

# Manually load controller
ros2 control load_controller joint_state_broadcaster
ros2 control set_controller_state joint_state_broadcaster active
```

#### 3.  Network Connectivity Issues

```bash
# 检查 DDS 发现
ros2 daemon stop
ros2 daemon start

# List available topics
ros2 topic list
```

#### 4. Build Errors

```bash
# Clean and rebuild
cd ~/atom_ros2_ws
rm -rf build install log
colcon build
```

### Performance Optimization

#### 1. Gazebo Performance

- Close unnecessary GUI panels
- Lower the physics engine update rate
- Use headless mode：`gui:=false`

#### 2. DDS Optimization (Configure Appropriate Network Interface)

- Adjust the DDS Domain ID

## 📚 References

- [ROS2 Official Documentation](https://docs.ros.org/en/humble/)
- [Gazebo Simulation Tutorials](http://gazebosim.org/tutorials)
- [ros2_control Documentation](https://control.ros.org/)
- [URDFTutorials](http://wiki.ros.org/urdf/Tutorials)

## 📞 Restart ROS 2 daemon

If you encounter issues, please:

1. Review the Troubleshooting section of this README.
2. Search existing Issues.
3. Create a new Issue with detailed information.

---

**Note**:Please adjust the configuration files and sample code according to your actual robot hardware interface and communication protocols.
