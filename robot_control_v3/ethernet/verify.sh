#!/usr/bin/env bash
# verify.sh — диагностика сетевой связи между PC и Jetson через ROS2
# Использование:  ./verify.sh
# Запускать на ОБОИХ хостах после setup_network.sh

# Цвета (упрощены для совместимости)
green() { printf "\033[32m%s\033[0m\n" "$*"; }
red()   { printf "\033[31m%s\033[0m\n" "$*"; }
yellow(){ printf "\033[33m%s\033[0m\n" "$*"; }
blue()  { printf "\033[36m%s\033[0m\n" "$*"; }

PEER_IP_PC="192.168.10.1"
PEER_IP_JETSON="192.168.10.2"

# Определяем кто мы
MY_IP=$(ip addr show | grep -oP '(?<=inet\s)192\.168\.10\.\d+' | head -1)

if [[ "$MY_IP" == "$PEER_IP_PC" ]]; then
    ROLE="pc"
    PEER="$PEER_IP_JETSON"
    PEER_NAME="Jetson"
elif [[ "$MY_IP" == "$PEER_IP_JETSON" ]]; then
    ROLE="jetson"
    PEER="$PEER_IP_PC"
    PEER_NAME="PC"
else
    red "ERROR: this host has no IP from rover subnet 192.168.10.0/24"
    echo "Run: sudo ./setup_network.sh {pc|jetson}"
    exit 1
fi

blue "=============================================================="
blue "  Network verification — robot_control_v3"
blue "  Role:   $ROLE"
blue "  My IP:  $MY_IP"
blue "  Peer:   $PEER ($PEER_NAME)"
blue "=============================================================="

# ── Тест 1: ping ─────────────────────────────────────────────────────────────
echo ""
blue "[1] Ping peer..."
if ping -c 3 -W 2 "$PEER" > /dev/null 2>&1; then
    green "    ✓ Peer reachable"
else
    red "    ✗ Cannot ping $PEER"
    echo "      • Cable not connected?"
    echo "      • Other host not powered on?"
    echo "      • IP not configured on peer? (run setup_network.sh there)"
fi

# ── Тест 2: ROS2 окружение ───────────────────────────────────────────────────
echo ""
blue "[2] ROS2 environment..."
[[ -n "$ROS_DOMAIN_ID" ]] && green "    ✓ ROS_DOMAIN_ID=$ROS_DOMAIN_ID" || red "    ✗ ROS_DOMAIN_ID not set!"
[[ -n "$RMW_IMPLEMENTATION" ]] && green "    ✓ RMW=$RMW_IMPLEMENTATION" || yellow "    ! RMW not set (default will be used)"
[[ -n "$FASTRTPS_DEFAULT_PROFILES_FILE" ]] && green "    ✓ FastDDS profile: $FASTRTPS_DEFAULT_PROFILES_FILE" || yellow "    ! No FastDDS profile (DDS will use all interfaces)"

# ── Тест 3: firewall ─────────────────────────────────────────────────────────
echo ""
blue "[3] Firewall (UFW)..."
if command -v ufw >/dev/null 2>&1; then
    if ufw status | grep -q "7400"; then
        green "    ✓ DDS ports 7400-7500/udp open"
    else
        yellow "    ! DDS ports may be blocked. Run:"
        echo "      sudo ufw allow 7400:7500/udp"
    fi
else
    yellow "    ! ufw not installed — cannot check"
fi

# ── Тест 4: ROS2 узлы видят друг друга ───────────────────────────────────────
echo ""
blue "[4] ROS2 nodes visible in DDS network..."
if command -v ros2 >/dev/null 2>&1; then
    NODES=$(timeout 5 ros2 node list 2>/dev/null | wc -l)
    if [[ "$NODES" -gt 0 ]]; then
        green "    ✓ $NODES node(s) visible"
        ros2 node list 2>/dev/null | head -10 | sed 's/^/      /'
    else
        yellow "    ! No ROS2 nodes visible yet"
        echo "      Start a node and rerun this test:"
        echo "        ros2 launch rover_nodes robot_bringup.launch.py"
    fi
else
    red "    ✗ ros2 command not found"
    echo "      source /opt/ros/{distro}/setup.bash"
fi

# ── Тест 5: топики на peer'е ─────────────────────────────────────────────────
echo ""
blue "[5] Topics in DDS network..."
if command -v ros2 >/dev/null 2>&1; then
    TOPICS=$(timeout 5 ros2 topic list 2>/dev/null | wc -l)
    if [[ "$TOPICS" -gt 0 ]]; then
        green "    ✓ $TOPICS topic(s) visible"
        echo ""
        echo "      Rover topics:"
        ros2 topic list 2>/dev/null | grep -E '^/(odom|wheels|wheel|motion|safety|rover|trajectory)' | sed 's/^/      /'
    fi
fi

# ── Итог ─────────────────────────────────────────────────────────────────────
echo ""
blue "=============================================================="
blue "  Recommendations:"
blue "=============================================================="
echo ""
echo "  • If topics from peer are missing — check ROS_DOMAIN_ID matches"
echo "    on both sides. Currently here: $ROS_DOMAIN_ID"
echo ""
echo "  • Check /odom rate (should be ~50 Hz over Ethernet):"
echo "      ros2 topic hz /odom"
echo ""
echo "  • Check round-trip latency:"
echo "      ros2 topic delay /odom"
echo ""
echo "  • Sniff DDS multicast traffic:"
echo "      sudo tcpdump -i \$(ip route get 192.168.10.1 | awk '{print \$3; exit}') udp port 7400"
echo ""
