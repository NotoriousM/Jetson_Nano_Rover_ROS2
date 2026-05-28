#!/usr/bin/env bash
# env_jetson.sh — переменные окружения ROS2 для Jetson Nano
# Использование:  source env_jetson.sh

# ── ROS2 базовое окружение ───────────────────────────────────────────────────
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0

# ── FastDDS profile (если присутствует) ──────────────────────────────────────
if [[ -f /etc/rover/fastdds_profile.xml ]]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE=/etc/rover/fastdds_profile.xml
fi

# ── Source ROS2 (Foxy на Jetson Nano с Ubuntu 20.04) ─────────────────────────
if [[ -f /opt/ros/foxy/setup.bash ]]; then
    source /opt/ros/foxy/setup.bash
fi

# ── Source workspace ─────────────────────────────────────────────────────────
if [[ -f ~/ros2_ws/install/setup.bash ]]; then
    source ~/ros2_ws/install/setup.bash
fi

echo "[env_jetson] ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "[env_jetson] RMW=$RMW_IMPLEMENTATION"
echo "[env_jetson] Jetson IP: 192.168.10.2  PC IP: 192.168.10.1"
