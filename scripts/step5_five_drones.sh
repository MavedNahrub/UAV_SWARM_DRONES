#!/usr/bin/env bash
###############################################################################
# step5_five_drones.sh — Scale to 5 UAVs in one Gazebo world
#
# Spawns 5 PX4 SITL instances with XRCE-DDS agent.
# Drones are placed in a pentagon formation, 3m apart.
#
# Usage:
#   ./step5_five_drones.sh          # Default 5 drones
#   ./step5_five_drones.sh 4        # Custom count (2-10)
set -eo pipefail

set +u 2>/dev/null || true
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash 2>/dev/null || true
set -u

PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
PX4_BIN="$PX4_DIR/build/px4_sitl_default/bin/px4"
NUM_DRONES="${1:-5}"
SPACING=3  # meters between drones

echo "══════════════════════════════════════════════════════════════"
echo "  Step 5: Scaling to $NUM_DRONES UAVs"
echo "══════════════════════════════════════════════════════════════"

if [ "$NUM_DRONES" -lt 2 ] || [ "$NUM_DRONES" -gt 10 ]; then
    echo "❌ Drone count must be between 2 and 10"
    exit 1
fi

SESSION="px4_swarm"
tmux kill-session -t "$SESSION" 2>/dev/null || true
tmux new-session -d -s "$SESSION" -x 200 -y 50

# ── Pane 0: XRCE-DDS Agent ──
tmux send-keys -t "$SESSION" "MicroXRCEAgent udp4 -p 8888" Enter
sleep 2

# ── Launch drones ──
for i in $(seq 0 $((NUM_DRONES - 1))); do
    # Calculate position in a line formation
    X_POS=$((i * SPACING))
    Y_POS=0

    if [ "$i" -eq 0 ]; then
        # First drone uses the initial pane (split from agent)
        tmux split-window -v -t "$SESSION"
    else
        tmux split-window -v -t "$SESSION"
        tmux select-layout -t "$SESSION" tiled  # Keep layout manageable
    fi

    INSTANCE_DIR="$PX4_DIR/build/px4_sitl_default/instance_$i"
    mkdir -p "$INSTANCE_DIR"

    tmux send-keys -t "$SESSION" \
        "cd $INSTANCE_DIR && export HEADLESS=1 && PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 PX4_GZ_MODEL_POSE='${X_POS},${Y_POS}' $PX4_BIN -i $i" Enter

    echo "  🚁 Drone $i spawning at position ($X_POS, $Y_POS)..."

    # Wait for each drone to initialize
    if [ "$i" -eq 0 ]; then
        echo "  ⏳ Waiting 20s for Gazebo + first drone..."
        sleep 20
    else
        echo "  ⏳ Waiting 8s for drone $i..."
        sleep 8
    fi
done

# ── Verify all topics ──
echo ""
echo "══════════════════════════════════════════════════════════════"
echo "  Verifying ROS 2 topics for $NUM_DRONES drones..."
echo "══════════════════════════════════════════════════════════════"

sleep 5  # Give XRCE-DDS a moment to sync

TOPIC_LIST=$(ros2 topic list 2>/dev/null || echo "")
ALL_PASS=true

# Instance 0: no namespace prefix (/fmu/out/vehicle_status or /fmu/out/vehicle_status_v1)
if echo "$TOPIC_LIST" | grep -Eq "^/fmu/out/vehicle_status(_v1)?$"; then
    echo "  ✅ Drone 0: /fmu/out/vehicle_status(_v1)"
else
    echo "  ❌ Drone 0: /fmu/out/vehicle_status NOT found"
    ALL_PASS=false
fi

# Instances 1+: /px4_N/ namespace
for i in $(seq 1 $((NUM_DRONES - 1))); do
    EXPECTED="/px4_${i}/fmu/out/vehicle_status"
    if echo "$TOPIC_LIST" | grep -Eq "^${EXPECTED}(_v1)?$"; then
        echo "  ✅ Drone $i: ${EXPECTED}(_v1)"
    else
        echo "  ❌ Drone $i: $EXPECTED NOT found"
        ALL_PASS=false
    fi
done

# Count total vehicle_status topics
VS_COUNT=$(echo "$TOPIC_LIST" | grep -c "vehicle_status" || echo "0")
echo ""
echo "  Total vehicle_status topics found: $VS_COUNT (expected: $((NUM_DRONES * 1))+ )"

echo ""
if [ "$ALL_PASS" = true ]; then
    echo "══════════════════════════════════════════════════════════════"
    echo "  ✅ Step 5 PASSED — All $NUM_DRONES UAVs spawned with"
    echo "     properly namespaced ROS 2 topics!"
    echo "══════════════════════════════════════════════════════════════"
else
    echo "══════════════════════════════════════════════════════════════"
    echo "  ❌ Step 5 FAILED — Some drones missing"
    echo "  Check tmux: tmux attach -t $SESSION"
    echo "══════════════════════════════════════════════════════════════"
fi

echo ""
echo "  Full topic list:"
echo "$TOPIC_LIST" | grep -E "(vehicle_status|vehicle_local_position)" | sort
echo ""
echo "  tmux session: $SESSION"
echo "    Attach: tmux attach -t $SESSION"
echo "    Kill:   tmux kill-session -t $SESSION"
