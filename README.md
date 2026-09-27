# PUSHPAK UAV-X — Autonomous Resilient Multi-UAV Swarm System

[![ROS 2](https://img.shields.io/badge/ROS_2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![PX4 Autopilot](https://img.shields.io/badge/PX4-v1.16-red.svg)](https://px4.io/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange.svg)](https://gazebosim.org/)
[![License](https://img.shields.io/badge/License-Apache_2.0-green.svg)](LICENSE)

A fully decentralized, resilient Beyond Visual Line of Sight (BVLOS) multi-UAV swarm simulation system built on **ROS 2 Humble**, **PX4 Autopilot SITL**, and **Gazebo Harmonic**. Developed for disaster response, search & rescue, and extreme-environment inspection.

---

## 🌟 Key Highlights

- **Decentralized Task Allocation (CBBA)**: Consensus-Based Bundle Algorithm enabling fully decentralized, auction-driven task assignment without a single point of failure.
- **Dynamic Need-Based Relay Placement**: Autonomous role switching (Explorer $\leftrightarrow$ Dynamic Relay) to maintain ad-hoc RF mesh connectivity under signal degradation and path loss.
- **Artificial Potential Field (APF) Safety Supervisor**: Real-time obstacle avoidance, inter-UAV collision avoidance, dynamic geofence clamping, and emergency hold maneuvers.
- **Predictive Battery RTH & Health Management**: Role-conditioned energy estimation computing dynamic Return-To-Home thresholds against wind and distance.
- **Fault Recovery & Distributed Leader Election**: Heartbeat monitoring, Bully leader election, failsafe state machines, and task reallocation upon UAV dropouts.
- **Real-Time Web Dashboard**: Interactive browser-based telemetry monitoring, live mission logs, UAV status tracking, and scenario triggers.

---

## 🏗️ System Architecture

```mermaid
graph TD
    subgraph Central / Simulation Layer
        GZ[Gazebo Harmonic World / Disaster SDF]
        WB[world_bridge: Collision, Geofence & Obstacles]
        CG[comm_gateway: Ad-hoc RF Mesh & Distance Attenuation]
        ML[mission_logger: JSONL Telemetry & Metrics Aggregator]
        DASH[Web Dashboard & WebSocket Server]
    end

    subgraph Per-UAV Stack (uav_1 .. uav_N)
        PX4[PX4 Autopilot SITL Instance]
        SS[safety_supervisor: APF & Setpoint Arbiter]
        FM[fleet_manager: State Machine & Guidance]
        CBBA[task_allocation: CBBA Decentralized Consensus]
        RP[relay_placement: Adaptive Mesh Relay]
        BATT[battery_rth: Virtual Battery & RTH Estimator]
        FR[fault_recovery: Heartbeat & Distributed Leader Election]
    end

    GZ <--> WB
    WB <--> SS
    CG <--> RP
    CG <--> CBBA
    CG <--> FR
    SS --> PX4
    FM --> SS
    BATT --> FM
    ML --> DASH
```

For full architectural contracts, topic names, and module specifications, see [ARCHITECTURE.md](ARCHITECTURE.md).

---

## 📦 ROS 2 Packages Overview

All custom packages reside in [`ros2_ws/src`](ros2_ws/src):

| Package | Role |
|---|---|
| **`swarm_msgs`** | Custom ROS 2 interfaces (`UAVState`, `TaskBid`, `Heartbeat`, `RelayCommand`, `SafetyStatus`, etc.) |
| **`fleet_manager`** | Top-level UAV mission state machine (IDLE, TAKEOFF, EXPLORING, RELAY, RTH, LANDED) |
| **`task_allocation`** | Decentralized Consensus-Based Bundle Algorithm (CBBA) for dynamic mission task bidding |
| **`safety_supervisor`** | Artificial Potential Field (APF) collision avoidance & 3D geofence clamping setpoint arbiter |
| **`relay_placement`** | Dynamic bridge-relay positioning between disconnected swarm clusters and base station |
| **`battery_rth`** | Predictive role-conditioned discharge model with dynamic Return-To-Home trigger |
| **`fault_recovery`** | Distributed heartbeat tracking, Bully leader election, and automated failsafe triggers |
| **`comm_gateway`** | Mesh topology simulator enforcing signal attenuation, latency, packet drop, and link breakage |
| **`world_bridge`** | Bridges Gazebo world state, dynamic geofences, obstacles, and virtual battery telemetry |
| **`mission_bringup`** | Launch files, world models (`disaster_area.sdf`), multi-vehicle orchestration |
| **`mission_logger`** | High-throughput structured JSONL mission logging and real-time KPI aggregator |
| **`swarm_viz`** | RViz2 markers and swarm visualization tools |

---

## 🚀 Getting Started

### Prerequisites

- **Ubuntu 22.04 LTS (Jammy Jellyfish)**
- **ROS 2 Humble Desktop**
- **Gazebo Harmonic** (`ros-humble-ros-gzharmonic` and `ros-humble-ros-gzharmonic-bridge`)
- **PX4 Autopilot v1.16+**
- **Micro XRCE-DDS Agent**

See [docs/ENVIRONMENT_SETUP.md](docs/ENVIRONMENT_SETUP.md) for detailed toolchain setup and verification steps.

### Building the Workspace

```bash
# Clone the repository
git clone https://github.com/MavedNahrub/UAV_SWARM_DRONES.git
cd UAV_SWARM_DRONES

# Install Python requirements
pip install -r requirements.txt

# Source ROS 2 Humble
source /opt/ros/humble/setup.bash

# Build the ROS 2 packages
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
cd ..
```

---

## 🎮 Running Missions & Scenarios

### 1. Preflight Verification
Verify environment, dependencies, and node configurations:
```bash
./scripts/test_preflight.sh
```

### 2. Multi-UAV Simulation Bringup
Launch the disaster scenario with 5 autonomous UAVs:
```bash
ros2 launch mission_bringup multi_uav_mission.launch.py scenario:=scenario_basic.yaml
```

Available mission scenarios in [`scenarios/`](scenarios/):
- **`scenario_basic.yaml`**: Standard 5-UAV perimeter search and waypoint reconnaissance.
- **`scenario_failure_injection.yaml`**: Injects simulated motor failure, triggering automatic cluster leader election and task re-auctioning.
- **`scenario_priority_interrupt.yaml`**: High-priority search-and-rescue target appears, dynamically pre-empting existing bundles.
- **`scenario_stress_full.yaml`**: High-stress multi-failure, wind gust, and comms degradation stress test.

### 3. Real-Time Web Dashboard
Launch the monitoring dashboard and server:
```bash
python3 scripts/dashboard_server.py
```
Open `http://localhost:8080` (or `scripts/dashboard.html`) in your browser to view live UAV states, battery levels, RF mesh connections, and mission metrics.

---

## 🐳 Docker Deployment

A complete containerized environment is provided under [`docker/`](docker/):

```bash
cd docker
./build_and_run.sh
```

This builds an isolated Ubuntu 22.04 image containing ROS 2 Humble, Gazebo Harmonic, PX4 SITL, and the complete swarm stack with GPU/GUI forwarding.

---

## 📊 Mission Evaluation & Logs

All mission execution data is logged in structured JSONL format under [`logs/`](logs/):
- `run.jsonl`: Timestamped events, state transitions, task allocations, safety violations, and link metrics.
- Summary performance evaluation metrics (mission duration, energy consumed per UAV, communication outages prevented, collision near-misses: 0).

---

## 👥 Contributors & Maintainers

- **MavedNahrub** ([@MavedNahrub](https://github.com/MavedNahrub))
