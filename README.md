# 🤖 ROS2 Robot Control System

A comprehensive robotics control system built with ROS2 Jazzy, featuring autonomous navigation, mapping, camera integration, and remote operation capabilities.

## 📋 Table of Contents
- [Prerequisites](#prerequisites)
- [Quick Start](#quick-start)
- [System Configuration](#system-configuration)
- [Robot Operations](#robot-operations)
- [PC Operations](#pc-operations)
- [Advanced Features](#advanced-features)
- [Troubleshooting](#troubleshooting)

## 🛠 Prerequisites

- **ROS2 Jazzy** installed and configured
- **Ubuntu 24.04** (recommended)
- Robot hardware with camera, LIDAR, and motor controllers
- Network connectivity between robot and control PC

## 🚀 Quick Start

### 1. Environment Setup

- **Source ROS2 environment**  
- **Check USB ports**  
- **Synchronize system time**  

```bash
source ~/robot/install/setup.bash
ls /dev/ttyUSB*
sudo systemctl restart chrony


### 2. Basic Robot Launch
```bash
# Camera only
ros2 launch create_bringup create_2.py camera:=true navigation:=false foxglove:=false

# Full system on Raspberry Pi
ros2 launch create_bringup create_2.py camera:=true navigation:=true foxglove:=true map:=map_file.yaml

# Full system with AI integration
ros2 launch create_bringup create_2.py camera:=true navigation:=true foxglove:=true rosbridge:=true map:=map_file.yaml
```

## 🌐 Network Configuration

### WiFi Connection
```bash
sudo nmcli device wifi connect "<SSID>" password "<PASSWORD>"
```

### 🌍 Remote Access Points

You can connect to a remote server using either **Ubuntu File Manager (SFTP)** or the **terminal (SSH)**:

| Method | Command Example |
|--------|-----------------|
| **SFTP** | `sftp://user@<IP_ADDRESS>` |
| **SSH**  | `ssh user@<IP_ADDRESS>` |

## 🎮 Robot Control

### Manual Control
```bash
# Keyboard control
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Joystick control
ros2 launch create_bringup joy_teleop.launch.py
```

### Hardware Control
```bash
# Main brush motor
ros2 topic pub --once /main_brush_motor create_msgs/msg/MotorSetpoint "{duty_cycle: 1.0}"

# Side brush motor
ros2 topic pub --once /side_brush_motor create_msgs/msg/MotorSetpoint "{duty_cycle: 1.0}"

# Vacuum motor
ros2 topic pub --once /vacuum_motor create_msgs/msg/MotorSetpoint "{duty_cycle: 1.0}"
```

### ⚡ Docking Operations
```bash
# Dock the robot
ros2 launch create_bringup docking.py

# Undock the robot
ros2 launch create_bringup undocking.py
📌 The dock position coordinates are specified inside the configuration file.

## 🗺 Navigation & Mapping

### Real-time Navigation
```bash
# With RViz
ros2 launch create_bringup visual.py

# Without RViz
ros2 launch create_bringup visual.py use_rviz:=false
```

### Simulation Navigation
```bash
ros2 launch create_bringup visual.py map:=/home/simone/robot/src/create_robot/create_bringup/map/map_simulazione.yaml use_sim_time:=true
```

### Mapping Operations

#### Real-time Mapping
```bash
# With RViz
ros2 launch create_bringup visualM.py

# Without RViz
ros2 launch create_bringup visualM.py use_rviz:=false

# Start exploration
ros2 run custom_explorer explorer
```

#### Simulation Mapping
```bash
ros2 launch create_bringup visualM.py use_sim_time:=true
ros2 run custom_explorer explorer
```

### Map Management
**Maps Directory:** `/home/simone/robot/install/create_bringup/share/create_bringup`

## 🛡 Zone Management

### Exclusion Zones
```bash
ros2 launch create_bringup zone_maker.py
```

### Speed Limit Zones
```bash
ros2 launch create_bringup zone_maker.py node:=speed slow_value:=40
```
*Speed values range from 1% to 99%*

## 🔧 Advanced Features

### Gazebo Simulation

#### Full Simulation
```bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(ros2 pkg prefix create_description)/share
export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:$(ros2 pkg prefix create_description)/share
ros2 launch gazebo robot.launch.py
```

#### Headless Simulation
```bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(ros2 pkg prefix create_description)/share
export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:$(ros2 pkg prefix create_description)/share
ros2 launch gazebo robot.launch.py headless:=true
```

### Foxglove Integration
```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765 host:=0.0.0.0
```

### Localization
```bash
ros2 launch create_bringup localization.py
```

### LIDAR Integration
```bash
ros2 run xv11_lidar_python xv11_lidar --ros-args -p port:=/dev/ttyUSB...
```

### AI Integration with LM Studio
```bash
# Navigate to LM Studio directory
cd /Documents/squashfs-root
./lm-studio

# Launch ROS bridge for AI communication
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

## 📁 Important File Locations

| Component | Path |
|-----------|------|
| SLAM Config | `/opt/ros/jazzy/share/slam_toolbox/config/mapper_params_online_async.yaml` |
| Navigation Config | `/opt/ros/jazzy/share/nav2_bringup/params/nav2_params.yaml` |
| Gazebo Worlds | `/opt/ros/jazzy/opt/gz_sim_vendor/share/gz/gz-sim8/worlds` |
| Robot Maps | `/home/simone/robot/install/create_bringup/share/create_bringup` |

## 📷 Camera Setup

For Raspberry Pi Camera 3 with Ubuntu 24.04, follow the setup guide:
[Raspberry Pi 5 Camera Setup](https://www.reddit.com/r/Ubuntu/comments/1ddpnto/raspberry_pi_5_running_2404_with_a_pi_camera_3/)

## 🔍 Troubleshooting

### Common Issues
- **USB Ports:** Check available ports with `ls /dev/ttyUSB*`
- **Time Sync:** Restart chrony service for accurate timestamps
- **Network:** Verify robot and PC are on the same network
- **Permissions:** Ensure proper permissions for USB devices

### Debugging Commands
```bash
# Check ROS2 topics
ros2 topic list

# Monitor specific topics
ros2 topic echo /topic_name

# Check node status
ros2 node list
```

## 📝 License

This project is open source. Please check the LICENSE file for details.

## 🤝 Contributing

Contributions are welcome! Please feel free to submit pull requests or open issues for bugs and feature requests.

---

**Made with ❤️ for robotics enthusiasts**
