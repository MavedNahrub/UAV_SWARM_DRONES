#!/bin/bash
# launch_sim_gui.sh — Launch the full PUSHPAK UAV-X swarm simulation with Gazebo GUI
# Run from inside the px4_swarm_dev container with DISPLAY=:0
# set -e deliberately removed — Gazebo GUI exit should NOT stop the simulation

echo "============================================"
echo "  PUSHPAK UAV-X Swarm Simulation Launcher"
echo "============================================"

# ── Environment ──────────────────────────────────────────────
export LIBGL_ALWAYS_SOFTWARE=1
export QT_X11_NO_MITSHM=1
export PX4_HOME="${HOME}/PX4-Autopilot"
export GZ_SIM_RESOURCE_PATH="${PX4_HOME}/Tools/simulation/gz/models:${PX4_HOME}/Tools/simulation/gz/worlds"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${PX4_HOME}/build/px4_sitl_default/src/modules/simulation/gz_plugins"
export GZ_SIM_SERVER_CONFIG_PATH="${PX4_HOME}/src/modules/simulation/gz_bridge/server.config"

NUM_UAVS=${1:-2}
SCENARIO=${2:-/home/px4user/uav_swarm/scenarios/scenario_basic.yaml}
LOG_PATH=${3:-/home/px4user/uav_swarm/logs/log_live.jsonl}

echo "[INFO] UAVs: ${NUM_UAVS}"
echo "[INFO] Scenario: ${SCENARIO}"
echo "[INFO] Log path: ${LOG_PATH}"

# ── Cleanup stale processes and locks ───────────────────────
pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -x "MicroXRCEAgent" 2>/dev/null || true
pkill -9 -x "px4" 2>/dev/null || true
pkill -9 -f "build/px4_sitl_default/bin/px4" 2>/dev/null || true
pkill -9 -f "mission_bringup" 2>/dev/null || true
pkill -9 -f "px4_vehicle_bridge_node" 2>/dev/null || true
pkill -9 -f "scenario_orchestrator" 2>/dev/null || true
pkill -9 -f "comm_gateway" 2>/dev/null || true
pkill -9 -f "cbba_node" 2>/dev/null || true
pkill -9 -f "relay_node" 2>/dev/null || true
sleep 1

for i in $(seq 0 $((NUM_UAVS - 1))); do
    rm -f "/tmp/px4_lock-${i}" "/tmp/px4-sock-${i}" 2>/dev/null || true
done

# ── Ensure Python dependencies are installed ─────────────────
pip3 install -q -r /home/px4user/uav_swarm/requirements.txt 2>/dev/null || true

# ── Source ROS 2 workspaces ──────────────────────────────────
source /opt/ros/humble/setup.bash
source /home/px4user/ros2_ws/install/setup.bash 2>/dev/null || true
source /home/px4user/uav_swarm/ros2_ws/install/setup.bash 2>/dev/null || true

# ── 1. Start Gazebo Sim (server + GUI in same process, restartable) ──
WORLD_SDF=$(ros2 pkg prefix mission_bringup --share)/worlds/disaster_area.sdf

# Use OGRE2 software renderer — more stable than default under Mesa llvmpipe
export MESA_GL_VERSION_OVERRIDE=3.3
export MESA_GLSL_VERSION_OVERRIDE=330
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe
export LP_NUM_THREADS=2
export QT_X11_NO_MITSHM=1

gz sim -v 2 -r "${WORLD_SDF}" &
GZ_PID=$!
echo "[INFO] Gazebo PID: ${GZ_PID}"

# Wait for the Gazebo world to become ready
ATTEMPTS=30
while [ $ATTEMPTS -gt 0 ]; do
    if gz service -i --service /world/default/scene/info 2>&1 | grep -q "Service providers"; then
        echo "[INFO] Gazebo world is ready!"
        break
    fi
    ATTEMPTS=$((ATTEMPTS-1))
    if [ $ATTEMPTS -eq 0 ]; then
        echo "[ERROR] Timed out waiting for Gazebo world"
        exit 1
    fi
    echo "[INFO] Waiting for Gazebo world... (${ATTEMPTS} attempts left)"
    sleep 2
done

echo "[INFO] Step 2/4: Starting Micro XRCE-DDS Agent..."

# ── 2. Start DDS Agent ───────────────────────────────────────
MicroXRCEAgent udp4 -p 8888 &
echo "[INFO] Verifying Micro XRCE-DDS Agent is listening on UDP port 8888..."
DDS_ATTEMPTS=20
while [ $DDS_ATTEMPTS -gt 0 ]; do
    if grep -q -i ":22B8 " /proc/net/udp 2>/dev/null; then
        echo "[INFO] Micro XRCE-DDS Agent is listening on port 8888!"
        break
    fi
    DDS_ATTEMPTS=$((DDS_ATTEMPTS-1))
    sleep 0.5
done
sleep 1

echo "[INFO] Step 3/4: Spawning ${NUM_UAVS} PX4 SITL instances (staggered)..."

# ── 3. Spawn PX4 instances one at a time ─────────────────────
PX4_BIN="${PX4_HOME}/build/px4_sitl_default/bin/px4"

# Spawn poses (spaced >=10m so min_separation 8m is respected at takeoff)
POSES=("0,0,0,0,0,0" "10,0,0,0,0,0" "20,0,0,0,0,0" "0,10,0,0,0,0" "10,10,0,0,0,0" "20,10,0,0,0,0" "0,20,0,0,0,0" "10,20,0,0,0,0")

for i in $(seq 0 $((NUM_UAVS - 1))); do
    INSTANCE_DIR="${PX4_HOME}/build/px4_sitl_default/instance_${i}"
    mkdir -p "${INSTANCE_DIR}"

    echo "[INFO] Spawning PX4 instance ${i}..."

    HEADLESS=1 \
    PX4_SYS_AUTOSTART=4001 \
    PX4_GZ_MODEL_POSE="${POSES[$i]}" \
    PX4_SIM_MODEL=gz_x500 \
    PX4_GZ_WORLD=default \
    PX4_GZ_MODELS="${PX4_HOME}/Tools/simulation/gz/models" \
    PX4_GZ_WORLDS="${PX4_HOME}/Tools/simulation/gz/worlds" \
    PX4_GZ_PLUGINS="${PX4_HOME}/build/px4_sitl_default/src/modules/simulation/gz_plugins" \
    "${PX4_BIN}" -i "${i}" -d "${PX4_HOME}/ROMFS/px4fmu_common" &

    # Enforce SITL parameters in background once instance starts
    (
        export PX4_SIM_INSTANCE=${i}
        sleep 5
        ${PX4_HOME}/build/px4_sitl_default/bin/px4-param set NAV_DLL_ACT 0 2>/dev/null || true
        ${PX4_HOME}/build/px4_sitl_default/bin/px4-param set COM_RCL_EXCEPT 4 2>/dev/null || true
        ${PX4_HOME}/build/px4_sitl_default/bin/px4-param set COM_ARM_WO_GPS 1 2>/dev/null || true
        ${PX4_HOME}/build/px4_sitl_default/bin/px4-param set NAV_RCL_ACT 0 2>/dev/null || true
        ${PX4_HOME}/build/px4_sitl_default/bin/px4-param set COM_ARM_SDCARD 0 2>/dev/null || true
        ${PX4_HOME}/build/px4_sitl_default/bin/px4-param set CBRK_SUPPLY_CHK 894281 2>/dev/null || true
        ${PX4_HOME}/build/px4_sitl_default/bin/px4-param set CBRK_IO_SAFETY 22027 2>/dev/null || true
        ${PX4_HOME}/build/px4_sitl_default/bin/px4-param set COM_PREARM_MODE 0 2>/dev/null || true
    ) &

    # Wait between instances to avoid sensor timeout
    sleep 5
    
    # Verify the model appeared
    echo "[INFO] Models in Gazebo after instance ${i}:"
    gz model --list 2>/dev/null | grep "x500" || echo "  (waiting for model...)"
done

echo "[INFO] Step 4/4: Starting ROS 2 swarm stack..."

# ── 4. Launch the ROS 2 swarm stack ──────────────────────────
UAV_IDS=""
for i in $(seq 0 $((NUM_UAVS - 1))); do
    if [ -n "${UAV_IDS}" ]; then UAV_IDS="${UAV_IDS},"; fi
    UAV_IDS="${UAV_IDS}uav_$(printf '%02d' ${i})"
done

# Launch per-UAV app stacks (with namespace so topics are /uav_XX/...)
for i in $(seq 0 $((NUM_UAVS - 1))); do
    UAV_ID="uav_$(printf '%02d' ${i})"
    ros2 launch mission_bringup uav_stack.launch.py \
        uav_id:="${UAV_ID}" \
        px4_instance:="${i}" \
        scenario:="${SCENARIO}" \
        namespace:="${UAV_ID}" &
    sleep 1
done

# Launch central nodes
ros2 launch mission_bringup central_nodes.launch.py \
    uav_ids:="${UAV_IDS}" \
    scenario:="${SCENARIO}" \
    log_path:="${LOG_PATH}" &

echo "============================================"
echo "  Simulation is RUNNING!"
echo "  ${NUM_UAVS} UAVs in Gazebo with GUI"
echo "============================================"
echo ""
echo "  Look for the 'Gazebo Sim' window on your desktop."
echo "  If paused, press SPACEBAR or click the Play button."
echo "  Use mouse scroll wheel to zoom in on the drones."
echo ""
echo "  Press Ctrl+C to stop everything."

# Keep script alive — just wait on background processes.
# Do NOT restart Gazebo if it exits — that causes duplicate windows.
wait

