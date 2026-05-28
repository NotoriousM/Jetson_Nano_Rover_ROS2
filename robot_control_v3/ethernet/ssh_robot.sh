#!/usr/bin/env bash
# ssh_robot.sh — удобная команда для подключения к Jetson по SSH
# Использование:
#   ./ssh_robot.sh                       # просто открыть shell
#   ./ssh_robot.sh "ros2 topic list"     # выполнить команду удалённо
#   ./ssh_robot.sh -L                    # с проброcом X-сервера (для GUI)

# Можно переопределить через окружение:
#   ROBOT_HOST=192.168.1.42 ./ssh_robot.sh
ROBOT_USER="${ROBOT_USER:-rover}"
ROBOT_HOST="${ROBOT_HOST:-192.168.10.2}"   # сначала Ethernet
ROBOT_HOST_WIFI="${ROBOT_HOST_WIFI:-rover.local}"   # mDNS через WiFi (если настроен avahi)

# Если флаг -L → пробросить X11
if [[ "$1" == "-L" ]]; then
    SSH_FLAGS="-Y"
    shift
else
    SSH_FLAGS=""
fi

# Сначала пробуем Ethernet, потом WiFi
if ping -c 1 -W 1 "$ROBOT_HOST" > /dev/null 2>&1; then
    echo "[ssh_robot] Connecting via Ethernet: $ROBOT_USER@$ROBOT_HOST"
    HOST="$ROBOT_HOST"
elif ping -c 1 -W 1 "$ROBOT_HOST_WIFI" > /dev/null 2>&1; then
    echo "[ssh_robot] Ethernet down, falling back to WiFi: $ROBOT_USER@$ROBOT_HOST_WIFI"
    HOST="$ROBOT_HOST_WIFI"
else
    echo "ERROR: cannot reach Jetson on either Ethernet ($ROBOT_HOST) or WiFi ($ROBOT_HOST_WIFI)" >&2
    echo "Check: 1) Ethernet cable plugged in; 2) Jetson powered on; 3) WiFi configured" >&2
    exit 1
fi

if [[ -n "$1" ]]; then
    # Выполнить команду удалённо
    ssh $SSH_FLAGS "$ROBOT_USER@$HOST" "source /etc/rover/env.sh; $*"
else
    # Интерактивный shell
    ssh $SSH_FLAGS "$ROBOT_USER@$HOST"
fi
