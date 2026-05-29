#!/bin/bash
# =============================================================================
# start_robot.sh — One-command startup for the G1 autonomy stack
#
# Run from workstation:   ./start_robot.sh
# Stop the robot stack:   ./start_robot.sh stop
# Check status:           ./start_robot.sh status
# =============================================================================

set -e

ROBOT_IP="192.168.123.164"
ROBOT_USER="unitree"
ROBOT_PASS="123"
LOCAL_IF="eth0"
LOCAL_IP="192.168.123.100"
SSH="sshpass -p $ROBOT_PASS ssh -o ConnectTimeout=10 -o StrictHostKeyChecking=no -o ServerAliveInterval=5 $ROBOT_USER@$ROBOT_IP"
BODY_IP="192.168.123.161"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

ok()   { echo -e "  ${GREEN}✓${NC} $1"; }
fail() { echo -e "  ${RED}✗${NC} $1"; }
info() { echo -e "  ${CYAN}→${NC} $1"; }
warn() { echo -e "  ${YELLOW}!${NC} $1"; }

# ─── STOP ────────────────────────────────────────────────────────────────────
do_stop() {
    echo -e "\n${CYAN}Stopping G1 stack...${NC}"
    $SSH '
        # Kill python3 (bridge, init_lidar, wanderer)
        killall -9 python3 2>/dev/null
        # Kill ROS nodes launched by ros2 launch
        pkill -9 -f "g1_tf_bridge|slam_toolbox|controller_server|planner_server|bt_navigator|lifecycle_manager|robot_state_pub|async_slam|recoveries_server" 2>/dev/null
        # Kill any ros2 launch or run_full wrapper
        pkill -9 -f "ros2.launch|run_full" 2>/dev/null
        # Kill wanderer
        pkill -9 -f "g1_wander" 2>/dev/null
        sleep 1
        # Verify
        REMAIN=$(pgrep -cf "g1_tf_bridge|init_lidar|slam_toolbox|controller_server|g1_wander" 2>/dev/null)
        echo "remain=$REMAIN"
    ' 2>/dev/null | while read line; do
        case "$line" in
            remain=0) ok "All processes stopped" ;;
            remain=*) warn "Some processes may still be running" ;;
        esac
    done
}

# ─── STATUS ──────────────────────────────────────────────────────────────────
do_status() {
    echo -e "\n${CYAN}G1 Stack Status${NC}"
    $SSH '
        echo "=== Uptime ==="
        uptime
        echo ""
        echo "=== Key Processes ==="
        pgrep -af "init_lidar|g1_tf_bridge|slam_toolbox|controller_server|g1_wander" 2>/dev/null || echo "(none running)"
        echo ""
        echo "=== Port 56301 (LiDAR) ==="
        ss -ulnp sport = 56301 2>/dev/null || echo "(clear)"
        echo ""
        echo "=== Recent bridge log ==="
        tail -5 /home/unitree/g1_nav_ws/launch.log 2>/dev/null || echo "(no log)"
    ' 2>/dev/null || fail "Cannot reach robot"
}

# ─── START ───────────────────────────────────────────────────────────────────
do_start() {
    echo -e "\n${CYAN}═══════════════════════════════════════${NC}"
    echo -e "${CYAN}  G1 Autonomy Stack — Startup Sequence  ${NC}"
    echo -e "${CYAN}═══════════════════════════════════════${NC}\n"

    # ── Step 1: Local network ─────────────────────────────────────────────
    echo -e "${YELLOW}[1/5] Checking local network...${NC}"

    if ip addr show "$LOCAL_IF" 2>/dev/null | grep -q "$LOCAL_IP"; then
        ok "eth0 already has $LOCAL_IP"
    else
        warn "eth0 missing $LOCAL_IP — adding it (may need sudo password)"
        sudo ip addr add "$LOCAL_IP/24" dev "$LOCAL_IF" 2>/dev/null && \
            ok "Added $LOCAL_IP to $LOCAL_IF" || \
            fail "Could not add IP (run: sudo ip addr add $LOCAL_IP/24 dev $LOCAL_IF)"
    fi

    # ── Step 2: Robot reachable ───────────────────────────────────────────
    echo -e "\n${YELLOW}[2/5] Waiting for robot head unit...${NC}"

    # Quick check if body controller is up (validates cable/network)
    if ping -c 1 -W 2 "$BODY_IP" >/dev/null 2>&1; then
        ok "Body controller ($BODY_IP) reachable"
    else
        warn "Body controller not responding — is the robot powered on and ethernet connected?"
    fi

    # Wait for head unit (may take a minute after power-on)
    ATTEMPTS=0
    MAX_ATTEMPTS=30
    while ! ping -c 1 -W 2 "$ROBOT_IP" >/dev/null 2>&1; do
        ATTEMPTS=$((ATTEMPTS + 1))
        if [ "$ATTEMPTS" -ge "$MAX_ATTEMPTS" ]; then
            fail "Robot head ($ROBOT_IP) not reachable after ${MAX_ATTEMPTS} attempts"
            exit 1
        fi
        info "Waiting for head unit... (attempt $ATTEMPTS/$MAX_ATTEMPTS)"
        sleep 2
    done
    ok "Head unit ($ROBOT_IP) reachable"

    # Wait for SSH to be ready
    ATTEMPTS=0
    while ! $SSH 'echo ok' >/dev/null 2>&1; do
        ATTEMPTS=$((ATTEMPTS + 1))
        if [ "$ATTEMPTS" -ge 10 ]; then
            fail "SSH not available after 10 attempts"
            exit 1
        fi
        info "Waiting for SSH... (attempt $ATTEMPTS/10)"
        sleep 3
    done
    ok "SSH connection established"

    # ── Step 3: Kill stale processes ──────────────────────────────────────
    echo -e "\n${YELLOW}[3/5] Cleaning up stale processes...${NC}"

    $SSH '
        killall -9 python3 2>/dev/null
        pkill -9 -f "g1_tf_bridge|slam_toolbox|controller_server|planner_server|bt_navigator|lifecycle_manager|robot_state_pub|async_slam|recoveries_server" 2>/dev/null
        pkill -9 -f "ros2.launch|run_full" 2>/dev/null
        pkill -9 -f "g1_wander" 2>/dev/null
        sleep 2
        # Double-tap — some ros2 nodes respawn
        killall -9 python3 2>/dev/null
        pkill -9 -f "g1_tf_bridge|slam_toolbox|controller_server" 2>/dev/null
        sleep 1
        PORTS=$(ss -ulnp sport = 56301 2>/dev/null | grep -c UNCONN)
        echo "ports=$PORTS"
    ' 2>/dev/null | while read line; do
        case "$line" in
            ports=0) ok "All clear — no stale processes or port bindings" ;;
            ports=*)  warn "Port 56301 still has bindings — may need manual cleanup" ;;
        esac
    done

    # ── Step 4: Start LiDAR ───────────────────────────────────────────────
    echo -e "\n${YELLOW}[4/5] Initializing LiDAR...${NC}"

    LIDAR_OUT=$($SSH '
        cd /home/unitree
        python3 init_lidar.py > init_lidar.log 2>&1 & disown
        sleep 3
        if grep -q "LivoxLidarSdkInit returned: True" init_lidar.log 2>/dev/null; then
            echo "SDK_OK"
        else
            echo "SDK_FAIL"
            cat init_lidar.log
        fi
    ' 2>/dev/null)

    if echo "$LIDAR_OUT" | grep -q "SDK_OK"; then
        ok "Livox Mid-360 SDK initialized — streaming on port 56301"
    else
        fail "LiDAR init failed:"
        echo "$LIDAR_OUT"
        exit 1
    fi

    # ── Step 5: Launch ROS stack ──────────────────────────────────────────
    echo -e "\n${YELLOW}[5/5] Launching ROS stack...${NC}"
    info "Starting bridge + SLAM + Nav2..."

    $SSH '
        cd /home/unitree/g1_nav_ws
        bash run_full.sh > launch.log 2>&1 & disown
        echo LAUNCHED
    ' >/dev/null 2>&1

    # Wait for bridge to receive data
    info "Waiting for LiDAR data flow..."
    ATTEMPTS=0
    while true; do
        ATTEMPTS=$((ATTEMPTS + 1))
        if [ "$ATTEMPTS" -ge 30 ]; then
            fail "Bridge did not receive LiDAR data after 30s"
            warn "Check: ssh unitree@$ROBOT_IP 'tail -30 /home/unitree/g1_nav_ws/launch.log'"
            exit 1
        fi
        RESULT=$($SSH 'grep -a "First cloud" /home/unitree/g1_nav_ws/launch.log 2>/dev/null' 2>/dev/null || true)
        if echo "$RESULT" | grep -q "First cloud"; then
            POINTS=$(echo "$RESULT" | grep -oP '\d+ points' | head -1)
            ok "Bridge receiving LiDAR data ($POINTS)"
            break
        fi
        sleep 2
    done

    # Final summary
    echo ""
    echo -e "${GREEN}═══════════════════════════════════════${NC}"
    echo -e "${GREEN}  Stack is UP and running!              ${NC}"
    echo -e "${GREEN}═══════════════════════════════════════${NC}"
    echo ""
    echo -e "  Robot head:  ${CYAN}$ROBOT_IP${NC}"
    echo -e "  ROS domain:  ${CYAN}42${NC}"
    echo -e "  Topics:      ${CYAN}/scan /odom /cmd_vel /map /tf${NC}"
    echo ""
    echo -e "  ${YELLOW}Next steps:${NC}"
    echo -e "    Run wanderer:  ${CYAN}./start_robot.sh wander${NC}"
    echo -e "    Check status:  ${CYAN}./start_robot.sh status${NC}"
    echo -e "    Stop stack:    ${CYAN}./start_robot.sh stop${NC}"
    echo ""
}

# ─── WANDER ──────────────────────────────────────────────────────────────────
do_wander() {
    DURATION=${2:-3}
    echo -e "\n${CYAN}Starting autonomous wandering (${DURATION}s)...${NC}"

    # Kill any existing wanderer first
    $SSH 'pkill -9 -f g1_wander 2>/dev/null' 2>/dev/null
    sleep 1

    # Verify stack is running
    BRIDGE_OK=$($SSH 'pgrep -f g1_tf_bridge >/dev/null 2>&1 && echo yes || echo no' 2>/dev/null)
    if [ "$BRIDGE_OK" != "yes" ]; then
        fail "Stack is not running — run './start_robot.sh' first"
        exit 1
    fi

    # Check scan topic has data
    SCAN_OK=$($SSH '
        source /opt/ros/foxy/setup.bash
        export ROS_DOMAIN_ID=42
        export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
        timeout 3 ros2 topic hz /scan 2>&1 | grep -c "average rate"
    ' 2>/dev/null)
    if [ "$SCAN_OK" = "0" ] || [ -z "$SCAN_OK" ]; then
        warn "No /scan data detected — wanderer may not work"
    else
        ok "/scan topic is active"
    fi

    sshpass -p "$ROBOT_PASS" scp -o ConnectTimeout=10 -o StrictHostKeyChecking=no \
        "$(dirname "$0")/g1_wander.py" "$ROBOT_USER@$ROBOT_IP:/tmp/g1_wander.py" 2>/dev/null && \
        ok "Wanderer deployed" || { fail "Could not copy wanderer"; exit 1; }

    $SSH "
        source /opt/ros/foxy/setup.bash
        export ROS_DOMAIN_ID=42
        export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
        timeout $DURATION python3 /tmp/g1_wander.py > /tmp/wander.log 2>&1 & disown
        echo STARTED
    " >/dev/null 2>&1

    ok "Wanderer running for ${DURATION}s"
    echo ""
    echo -e "  ${YELLOW}Monitor:${NC}  ./start_robot.sh wander-log"
    echo -e "  ${YELLOW}Stop:${NC}     ./start_robot.sh stop-wander"
    echo ""
}

# ─── WANDER-LOG ──────────────────────────────────────────────────────────────
do_wander_log() {
    echo -e "\n${CYAN}Wanderer log:${NC}"
    $SSH 'cat /tmp/wander.log 2>/dev/null || echo "(no log found)"' 2>/dev/null
}

# ─── STOP-WANDER ─────────────────────────────────────────────────────────────
do_stop_wander() {
    $SSH 'pkill -f g1_wander 2>/dev/null' 2>/dev/null && ok "Wanderer stopped" || warn "Wanderer was not running"
}

# ─── MAIN ────────────────────────────────────────────────────────────────────
case "${1:-start}" in
    start)       do_start ;;
    stop)        do_stop ;;
    status)      do_status ;;
    wander)      do_wander "$@" ;;
    wander-log)  do_wander_log ;;
    stop-wander) do_stop_wander ;;
    *)           echo "Usage: $0 {start|stop|status|wander [seconds]|wander-log|stop-wander}" ;;
esac
