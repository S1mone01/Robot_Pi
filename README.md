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
```bash
source ~/robot/install/setup.bash
```
- **Check USB ports**
```bash
ls /dev/ttyUSB*
```
- **Synchronize system time**  
```bash
sudo systemctl restart chrony
```
### 2. Basic Robot Launch
- **Camera only**
```bash
ros2 launch create_bringup create_2.py camera:=true navigation:=false foxglove:=false
```
- **Full system on Raspberry Pi**
```bash
ros2 launch create_bringup create_2.py camera:=true navigation:=true foxglove:=true map:=map_file.yaml
```
- **Full system with AI integration**
```bash
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

### Manual Teleop

- **Keyboard control**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
- **Joystick control**
```bash
ros2 launch create_bringup joy_teleop.launch.py
```

### Hardware Control
- **Main brush motor**
```bash
ros2 topic pub --once /main_brush_motor create_msgs/msg/MotorSetpoint "{duty_cycle: 1.0}"
```
- **Side brush motor**
```bash
ros2 topic pub --once /side_brush_motor create_msgs/msg/MotorSetpoint "{duty_cycle: 1.0}"
```
- **Vacuum motor**
```bash
ros2 topic pub --once /vacuum_motor create_msgs/msg/MotorSetpoint "{duty_cycle: 1.0}"
```

### ⚡ Docking Operations
📌 The dock position coordinates are specified inside the launch file.
- **Dock the robot**
```bash
ros2 launch create_bringup docking.py
```
- **Undock the robot**
```bash
ros2 launch create_bringup undocking.py
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


### Foxglove Integration
```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765 host:=0.0.0.0
```
### AI Integration with LM Studio
# Launch ROS bridge for AI communication
```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

### Localization
```bash
ros2 launch create_bringup localization.py
```

## 📷 Camera Setup

For Raspberry Pi Camera with Ubuntu 24.04, follow the setup guide:
[Raspberry Pi 5 Camera Setup](https://www.reddit.com/r/Ubuntu/comments/1ddpnto/raspberry_pi_5_running_2404_with_a_pi_camera_3/)

## 🔍 Troubleshooting

### Debugging Commands

- **Check ROS2 topics**
```bash
ros2 topic list
```
- **Monitor specific topics**
```bash
ros2 topic echo /topic_name
```
- **Check node status**
```bash
ros2 node list
```

