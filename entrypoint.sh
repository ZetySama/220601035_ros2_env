#!/bin/bash
set -e

# ROS 2 temel kurulumunu kaynak göster [cite: 206]
source /opt/ros/humble/setup.bash

# Kendi çalışma alanımızı (derlenen kodlar) kaynak göster [cite: 206]
source /ws/install/setup.bash

# Launch dosyasını çalıştır [cite: 208]
ros2 launch launch/my_project.launch.py
