# UAV Swarm — Environment Setup & Locked Versions

## System Requirements

| Requirement | Value |
|---|---|
| Host OS | Ubuntu 26.04 LTS (Resolute) |
| Docker Target | Ubuntu 22.04 LTS (Jammy) |
| GPU | NVIDIA with driver 535+ |
| RAM | 16 GB minimum (32 GB recommended for 5+ drones) |
| Disk | ~25 GB for Docker image |

## Locked Version Matrix

| Component | Version | Source | Pin Method |
|---|---|---|---|
| **Ubuntu (container)** | 22.04 (Jammy) | `ubuntu:22.04` Docker image | Dockerfile `FROM` |
| **PX4-Autopilot** | v1.17.0 | `git tag v1.17.0` | `--branch v1.17.0` in Dockerfile |
| **Gazebo** | Harmonic (8.x) | Installed by PX4 `ubuntu.sh` | PX4 script pins it |
| **ROS 2** | Humble Hawksbill | `ros-humble-desktop` apt | Ubuntu 22.04 apt repo |
| **Micro-XRCE-DDS-Agent** | v2.4.2 | `git tag v2.4.2` | `--branch v2.4.2` in Dockerfile |
| **px4_msgs** | `release/1.17` branch | GitHub | Branch name in Dockerfile |
| **px4_ros_com** | `main` branch | GitHub | Latest at build time |
| **Python** | 3.10 (system) | Ubuntu 22.04 default | System package |
| **empy** | 3.3.4 | pip | `pip install empy==3.3.4` |
| **Docker** | 24.x+ | docker.com apt repo | Host install |
| **NVIDIA Container Toolkit** | Latest | nvidia.github.io | Host install |

## Architecture

```
┌─────────────────────────── Host: Ubuntu 26.04 ───────────────────────────┐
│                                                                           │
│  ┌──────────────────── Docker: Ubuntu 22.04 ──────────────────────────┐  │
│  │                                                                     │  │
│  │  ┌─────────────┐  ┌─────────────┐       ┌─────────────┐           │  │
│  │  │ PX4 SITL #0 │  │ PX4 SITL #1 │  ...  │ PX4 SITL #N │           │  │
│  │  │ (instance 0)│  │ (instance 1)│       │ (instance N)│           │  │
│  │  └──────┬──────┘  └──────┬──────┘       └──────┬──────┘           │  │
│  │         │ uXRCE-DDS      │                      │                  │  │
│  │         └────────────────┼──────────────────────┘                  │  │
│  │                          ▼                                         │  │
│  │              ┌─────────────────────┐                               │  │
│  │              │ MicroXRCEAgent      │                               │  │
│  │              │ (UDP port 8888)     │                               │  │
│  │              └──────────┬──────────┘                               │  │
│  │                         ▼                                          │  │
│  │              ┌─────────────────────┐                               │  │
│  │              │ ROS 2 Humble DDS    │                               │  │
│  │              │ Topic Namespaces:   │                               │  │
│  │              │  /fmu/...           │  ← Drone 0                   │  │
│  │              │  /px4_1/fmu/...     │  ← Drone 1                   │  │
│  │              │  /px4_2/fmu/...     │  ← Drone 2                   │  │
│  │              │  /px4_N/fmu/...     │  ← Drone N                   │  │
│  │              └─────────────────────┘                               │  │
│  │                                                                     │  │
│  │  ┌─────────────────────────────────┐                               │  │
│  │  │ Gazebo Harmonic (single world)  │ ← GPU via nvidia-container   │  │
│  │  │ All drone models rendered here  │                               │  │
│  │  └─────────────────────────────────┘                               │  │
│  └─────────────────────────────────────────────────────────────────────┘  │
│                                                                           │
│  X11 forwarding (Gazebo GUI) ←──→ Host display                           │
└───────────────────────────────────────────────────────────────────────────┘
```

## Quick Start

```bash
# 1. Install Docker (if not installed)
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
sudo usermod -aG docker $USER
# Log out and back in for group membership

# 2. Install NVIDIA Container Toolkit (for GPU in Docker)
chmod +x docker/install_nvidia_docker.sh
./docker/install_nvidia_docker.sh

# 3. Build the Docker image (takes ~20-30 min first time)
chmod +x docker/build_and_run.sh
./docker/build_and_run.sh build

# 4. Run the container
./docker/build_and_run.sh run

# 5. Inside the container, run verification steps:
bash ~/uav_swarm/scripts/step1_verify_ros2.sh       # Step 1
bash ~/uav_swarm/scripts/step2_single_drone.sh       # Step 2
bash ~/uav_swarm/scripts/step3_two_drones.sh         # Step 3
bash ~/uav_swarm/scripts/step4_ros2_bridge.sh        # Step 4
bash ~/uav_swarm/scripts/step5_five_drones.sh        # Step 5
```

## MAVLink Port Mapping

| Instance | MAVLink UDP (GCS) | MAVLink UDP (API) | XRCE-DDS Key | ROS 2 Namespace |
|---|---|---|---|---|
| 0 | 14540 | 14580 | 1 | `/fmu/...` (root) |
| 1 | 14541 | 14581 | 2 | `/px4_1/fmu/...` |
| 2 | 14542 | 14582 | 3 | `/px4_2/fmu/...` |
| 3 | 14543 | 14583 | 4 | `/px4_3/fmu/...` |
| 4 | 14544 | 14584 | 5 | `/px4_4/fmu/...` |

## Troubleshooting

### Gazebo doesn't open / no GUI
```bash
# On the HOST, allow X11 connections:
xhost +local:docker

# Verify DISPLAY is set inside container:
echo $DISPLAY
```

### "No GPU detected" in Gazebo
```bash
# Test GPU inside container:
nvidia-smi

# If it fails, ensure nvidia-container-toolkit is installed on host:
./docker/install_nvidia_docker.sh
```

### PX4 instances don't connect to XRCE-DDS agent
```bash
# Check agent is running:
ps aux | grep MicroXRCE

# Check PX4 parameter:
# In PX4 console: param show UXRCE_DDS_KEY
# Should be instance_id + 1
```

### Topics missing for a specific drone
```bash
# Check if PX4 instance is running:
ps aux | grep px4

# Restart the agent (it auto-discovers clients):
MicroXRCEAgent udp4 -p 8888
```
