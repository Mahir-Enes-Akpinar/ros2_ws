# 🦾 ROS 2 Workspace Template

This repository contains a basic **ROS 2 (Jazzy)** workspace structure.  
You can use it as a starting point for your robot projects.

---

## 🚀 How to Build and Run

```bash
# Clone this repo
git clone https://github.com/Mahir-Enes-Akpinar/ros2_ws.git
cd ros2_ws

# Build
colcon build

# Source the setup file
source install/setup.bash

# Run your node
ros2 run my_robot_pkg move_robot


⚙️ Notes

ROS 2 version: Jazzy

Compatible with Ubuntu 24.04

Don’t commit build, install, or log directories.