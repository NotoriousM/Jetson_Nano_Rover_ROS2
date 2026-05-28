#!/usr/bin/env bash
# env_pc.sh — переменные окружения ROS2 для PC
# Использование:  source env_pc.sh
#
# Эти же значения автоматически создаются скриптом setup_network.sh в
# /etc/rover/env.sh — этот файл оставлен для случая когда setup ещё не запускался.

# ── ROS2 базовое окружение ───────────────────────────────────────────────────
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0

# ── FastDDS profile (если присутствует) ──────────────────────────────────────
if [[ -f /etc/rover/fastdds_profile.xml ]]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE=/etc/rover/fastdds_profile.xml
fi

# ── Source ROS2 distro ───────────────────────────────────────────────────────
# Подстроить под свою установку:
if [[ -f /opt/ros/humble/setup.bash ]]; then
    source /opt/ros/humble/setup.bash
elif [[ -f /opt/ros/foxy/setup.bash ]]; then
    source /opt/ros/foxy/setup.bash
fi

# ── Source workspace (если собран) ───────────────────────────────────────────
if [[ -f ~/ros2_ws/install/setup.bash ]]; then
    source ~/ros2_ws/install/setup.bash
fi

echo "[env_pc] ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "[env_pc] RMW=$RMW_IMPLEMENTATION"
echo "[env_pc] PC IP: 192.168.10.1  Jetson IP: 192.168.10.2"
