# ARCHITECTURE.md — Resilient BVLOS Swarm Simulation (PUSHPAK UAV-X, Stage 1)

**Revision 2** — migrated from the standalone Python simulation to **ROS 2 Humble + PX4 SITL + Gazebo Harmonic** on **Ubuntu 22.04**. This supersedes Revision 1 in full. Module *responsibilities* and the mission logic (CBBA, relay heuristic, fault recovery, safety enforcement) are unchanged from Revision 1 — what changes is that every module is now a ROS 2 node/package talking over topics/services with custom `.msg` types, and physical UAV motion is delegated to PX4 + Gazebo physics instead of a hand-rolled kinematics function.

Status: implementation spec for an AI coding agent (Antigravity). Any deviation must be logged in `DEVIATIONS.md` — do not silently change topic names, message fields, or node boundaries, since modules are being implemented independently against this contract.

---

## 0. What Changed From Revision 1, and Why It Matters

| Revision 1 (custom Python sim) | Revision 2 (ROS 2 + PX4 + Gazebo) |
|---|---|
| `WorldModel.step()` computed point-mass kinematics | PX4 flight controller + Gazebo physics compute real 6-DOF dynamics; our code only sends velocity/position setpoints and reads back state |
| Battery drain was an in-process pure function | Becomes a per-UAV ROS 2 node maintaining a *virtual* role-conditioned battery, decoupled from Gazebo's own battery plugin (flagged in Assumptions) |
| `LinkModel`/`MeshGraph`/`Router` were plain Python classes called each tick | Becomes a single `comm_gateway` node that gates message topics between UAV namespaces — Gazebo/PX4 have no concept of a degraded network, so this is entirely our own middleware layer (Section 3) |
| `UAVLocalView` was a dataclass built in-process | Becomes an architectural property enforced by **topic/namespace isolation**: a UAV's app nodes may only subscribe to its own namespace's PX4 topics plus whatever `comm_gateway` delivers to its `/uavN/comm_rx` topic — never another UAV's raw pose |
| `SafetySupervisor.enforce()` returned corrected commands to a caller | Becomes the **sole node allowed to publish to PX4's setpoint topics** — topic ownership is the enforcement mechanism now, not a function return value |
| Simulation loop = one Python `for tick in range(N)` | Becomes PX4's own SITL lockstep loop + ROS 2 node timers; there is no single "tick" object anymore, so Section 5 is rewritten as a **causal topic/timer graph** instead of a numbered loop |

The mission-logic novelty points from Revision 1 (decentralization enforced by construction, need-based relay placement, predictive RTH, etc.) all still hold — they're just now enforced by ROS 2 namespace/topic boundaries instead of Python function signatures, which is arguably a *stronger* proof of decentralization since it also holds if you literally ran each UAV's nodes on separate physical machines.

---

## 1. Environment Setup

**This exact combination was checked against current PX4 documentation at the time of writing; verify against `docs.px4.io` and `gazebosim.org` before a from-scratch install, since PX4's supported Gazebo version has changed twice in the last two years.**

| Component | Version | Why this exact one |
|---|---|---|
| OS | Ubuntu 22.04 LTS (Jammy) | required for ROS 2 Humble binary packages |
| ROS 2 | **Humble Hawksbill** (`ros-humble-desktop`) | LTS, and the version PX4's current `main`/v1.16 ROS 2 bridge targets |
| Gazebo | **Gazebo Harmonic** (NOT Fortress, NOT Garden) | PX4 v1.15+ made Gazebo Harmonic the default/only actively supported simulator; Fortress is what ROS 2 Humble ships *by default*, which is the #1 version-mismatch trap for this exact stack — see gotcha below |
| PX4-Autopilot | **v1.16.x stable**, pinned tag (not `main`) | v1.16 is the current stable release line as of this writing; pin a tag (e.g. `git checkout v1.16.0`) rather than tracking `main`, which moves against Gazebo/ROS versions independently and will break silently |
| ROS↔Gazebo bridge | `ros-humble-ros-gzharmonic` + `ros-humble-ros-gzharmonic-bridge` (from `packages.osrfoundation.org`, **not** the default `ros-humble-ros-gz`) | see gotcha below |
| PX4↔ROS 2 bridge | **Micro XRCE-DDS Agent** (build the version pinned in `PX4-Autopilot/Tools/setup`, do not `apt install` an arbitrary one) | PX4 uses uXRCE-DDS, not MAVROS, as of v1.14+; the Agent version must match the client baked into the PX4 firmware you build |
| Ground control (optional, for judges/demo monitoring) | QGroundControl (latest stable AppImage) | not required for the sim to run, useful for the demo video |

### 1.1 The critical gotcha (flag this prominently in the README)

`sudo apt install ros-humble-ros-gz` installs the **default** Humble↔Gazebo bridge, which targets **Gazebo Fortress** — not Harmonic. PX4 v1.15+ Gazebo integration (`make px4_sitl gz_x500`) only speaks Harmonic. Installing the default package results in a simulation that *appears* to start (Gazebo opens, PX4 boots) but silently has **no working `/clock` bridge and no working pose/sensor bridge** — symptoms are ROS 2 nodes hanging waiting for sim time, or `ros2 topic echo /clock` returning nothing.

**Fix:** explicitly remove the wrong package and install the Harmonic-specific one:
```bash
sudo apt remove ros-humble-ros-gz
sudo apt install ros-humble-ros-gzharmonic ros-humble-ros-gzharmonic-bridge
```
Verify before writing any application code: launch `make px4_sitl gz_x500` once, then in a second terminal confirm `ros2 topic echo /clock` is actually publishing. **Do not proceed with node development until this passes** — it is the single most common failure point in PX4+ROS2+Gazebo setups and wastes days if caught late.

### 1.2 Per-machine setup checklist

1. Install ROS 2 Humble desktop (`ros-humble-desktop`).
2. Install Gazebo Harmonic + the Harmonic-specific ROS bridge (1.1 above) — do this *before* building PX4.
3. Clone `PX4-Autopilot` at tag `v1.16.0` (or latest v1.16.x patch), `git submodule update --init --recursive`, run `Tools/setup/ubuntu.sh`.
4. Build once headless to confirm toolchain: `make px4_sitl gz_x500`.
5. Build and install Micro XRCE-DDS Agent from the commit/tag referenced in `PX4-Autopilot/Tools/setup` (do not use a `apt`/`pip` version pulled independently — version drift here is the second most common failure point).
6. `colcon build` this repo's ROS 2 workspace (Section 2) with `ros-humble-*` sourced.
7. Smoke test: run `mission_bringup`'s single-vehicle launch file (Section 4) and confirm PX4, Gazebo, and the XRCE-DDS Agent all report connected before attempting the multi-vehicle launch.

---

## 2. Repository / Workspace Layout

```
pushpak-uav-x/
├── README.md
├── .gitignore
├── docs/
│   ├── ARCHITECTURE.md          # this file
│   └── ASSUMPTIONS.md
├── ros2_ws/
│   └── src/
│       ├── swarm_msgs/                    # all .msg/.srv definitions (Section 3)
│       ├── world_bridge/                  # virtual battery, static geofence/obstacle publishers, collision/geofence monitor
│       ├── comm_gateway/                  # the degraded-mesh network simulator (Section 5)
│       ├── task_allocation/               # per-UAV CBBA node
│       ├── relay_placement/               # per-UAV relay heuristic node
│       ├── fault_recovery/                # per-UAV fault_manager_node + central fleet_manager package's supervisor node
│       ├── battery_rth/                   # per-UAV battery/RTH node
│       ├── safety_supervisor/             # central safety node — sole owner of PX4 setpoint topics
│       ├── fleet_manager/                 # central node: reserve-UAV arm/takeoff, fault aggregation
│       ├── mission_logger/                # JSONL logging node (Section 7)
│       ├── swarm_viz/                     # RViz2 config + MarkerArray publisher node
│       └── mission_bringup/               # launch files, scenario configs (Section 4)
├── scenarios/
│   ├── scenario_basic.yaml
│   ├── scenario_failure_injection.yaml
│   ├── scenario_priority_interrupt.yaml
│   └── scenario_stress_full.yaml
├── logs/                                   # gitignored, JSONL output
└── requirements.txt                        # non-ROS Python deps (networkx, pyyaml, jsonschema for tests)
```

`run_demo.sh` (repo root, replaces Revision 1's `run_demo.py` as the single-command entry point, since this now spans multiple processes a Python script can't cleanly own):
```bash
./run_demo.sh --scenario scenarios/scenario_basic.yaml --num-uavs 6 [--headless] [--out logs/run_001.jsonl]
```
Internally this sources the ROS 2 workspace and calls `ros2 launch mission_bringup multi_uav_mission.launch.py ...` (Section 4) — this is the one file judges run.

---

## 3. Shared Message/Service Definitions — `swarm_msgs`

All cross-node communication uses these types. No node may invent an ad-hoc topic message type outside this package — that defeats the purpose of a shared contract.

### 3.1 Messages (`swarm_msgs/msg/`)

**`UAVState.msg`** — a UAV's own believed state, published by each UAV's own namespace (bridged from PX4, not fabricated):
```
string uav_id
builtin_interfaces/Time stamp
geometry_msgs/Point position          # local ENU, meters, from PX4 local position
float32 heading_rad
float32 speed_mps
float32 battery_frac                  # 0.0-1.0, from world_bridge's virtual battery node
uint8 role                            # enum: SURVEYOR=0 RELAY=1 RTH=2 RESERVE=3 GROUNDED=4 IDLE=5
uint8 health                          # enum: NOMINAL=0 DEGRADED=1 FAILED=2
string assigned_poi_id                # "" if none
string[] bundle                       # ordered PoI ids, CBBA bundle
```

**`PoI.msg`**
```
string poi_id
geometry_msgs/Point position
uint8 priority                        # ROUTINE=1 URGENT=2 CRITICAL=3
uint8 status                          # PENDING=0 ASSIGNED=1 IN_PROGRESS=2 SURVEYED=3 EXPIRED=4
builtin_interfaces/Time created_stamp
builtin_interfaces/Time deadline_stamp   # zero/unset if no deadline
float32 survey_duration_s
string assigned_uav_id
```

**`LinkState.msg`** — published as an array by `comm_gateway` every gateway tick:
```
string uav_a
string uav_b                          # canonical: uav_a < uav_b lexicographically
float32 distance_m
float32 rssi_dbm
bool connected
float32 packet_loss_prob
float32 latency_s
bool is_relay_edge
```
`LinkStateArray.msg`: `LinkState[] links` plus `builtin_interfaces/Time stamp` and `float32 connectivity_fraction`.

**`SwarmMessage.msg`** — the generic application-message envelope routed through `comm_gateway`. Kept generic (not one .msg per app message type) so the gateway can route without understanding payload semantics — exactly mirroring how a real radio doesn't parse application content:
```
string msg_id
string src_uav_id
string dst_uav_id                     # "GCS" is a valid logical destination
string msg_type                       # TASK_BID | TASK_AWARD | TASK_RELEASE | HEARTBEAT | POI_REPORT | POI_ALERT | RELAY_UPDATE | RTH_NOTICE
string payload_json                   # JSON-encoded payload, app-specific per msg_type
builtin_interfaces/Time origin_stamp
uint8 hop_count
uint8 ttl
```

**`FaultEvent.msg`**
```
builtin_interfaces/Time detected_stamp
builtin_interfaces/Time recovered_stamp   # zero/unset if not yet recovered
string uav_id
string fault_type                     # LINK_LOSS | UAV_FAILURE | BATTERY_CRITICAL
```

**`RelayDecision.msg`**
```
builtin_interfaces/Time stamp
string[] relay_uav_ids
bool connectivity_ok
bool reallocation_triggered
string trigger_reason                 # FAULT | TASK_CHANGE | PERIODIC | ""
```

**`Geofence.msg`**: `geometry_msgs/Point[] polygon` (latched/transient-local). **`Obstacle.msg`**: `geometry_msgs/Point center`, `float32 radius_m`; `ObstacleArray.msg`: `Obstacle[] obstacles` (latched).

### 3.2 Services (`swarm_msgs/srv/`)

**`RequestReserveSwapIn.srv`**
```
string departing_uav_id
string reason                         # RTH | FAILURE
---
bool accepted
string activated_uav_id               # "" if no reserve was available
```
Called by a departing UAV's `battery_rth` or `fault_recovery` node against `fleet_manager`'s `/swarm/request_reserve_swap_in` service — this is the one place Revision 1's "reserve pool" bookkeeping becomes a real cross-process call instead of an in-memory list pop.

**`GetGeofence.srv`** / **`GetObstacles.srv`**: trivial request-empty/response-array services as an alternative to the latched-topic approach above, for nodes that start after `world_bridge` and need the current value immediately rather than waiting for the next latched publish. (Implementer's choice which mechanism to use; document it if you pick services over latched topics.)

---

## 4. Multi-Vehicle PX4 SITL + Gazebo Launch Structure — `mission_bringup`

### 4.1 Principles

- **One Gazebo Harmonic world, N PX4 SITL instances.** Gazebo is launched once with a single world file (`mission_bringup/worlds/disaster_area.sdf`) containing the geofence-region terrain and static obstacle models. Each PX4 SITL instance is a separate OS process (`-i <n>` instance flag), each spawning its own vehicle model into the *same* running Gazebo world at a distinct pose — this is PX4's standard supported multi-vehicle-in-one-world pattern.
- **One Micro XRCE-DDS Agent process**, shared by all vehicles — each PX4 instance's uXRCE-DDS client connects to it on a distinct UDP port (PX4 auto-offsets ports per instance number), and the Agent multiplexes all vehicles' topics into the ROS 2 graph, disambiguated by PX4's per-instance topic namespacing (`/px4_<n>/fmu/...`).
- **Our own app nodes are grouped per UAV under `/uavN/` namespaces**, remapped from PX4's `/px4_<n>/fmu/...` topics via a thin per-vehicle bridge/remap layer so the rest of our stack never has to know about PX4's internal instance-numbering scheme.
- **Shared/central nodes** (`comm_gateway`, `safety_supervisor`, `fleet_manager`, `mission_logger`, `swarm_viz`) are launched once, unnamespaced (or under `/swarm/`), and interact with all `/uavN/` namespaces.

### 4.2 Launch file tree

```
mission_bringup/
├── launch/
│   ├── multi_uav_mission.launch.py      # top-level entry point (what run_demo.sh calls)
│   ├── gazebo_world.launch.py           # launches Gazebo Harmonic with the world sdf, once
│   ├── px4_vehicle.launch.py            # launches ONE PX4 SITL instance + its bridges; included N times
│   ├── uav_stack.launch.py              # launches ONE UAV's app nodes (task_allocation, relay_placement, fault_recovery, battery_rth) under a namespace; included N times
│   └── central_nodes.launch.py          # comm_gateway, safety_supervisor, fleet_manager, mission_logger, swarm_viz
├── worlds/
│   └── disaster_area.sdf
├── config/
│   ├── vehicle_spawn_poses.yaml         # per-instance spawn (x, y, z, yaw)
│   └── node_params/*.yaml               # per-package ROS 2 parameter files
└── xrce_agent.launch.py                 # single Micro XRCE-DDS Agent process
```

### 4.3 `multi_uav_mission.launch.py` — structural skeleton

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import GroupAction

def launch_setup(context, *args, **kwargs):
    num_uavs = int(LaunchConfiguration('num_uavs').perform(context))
    scenario = LaunchConfiguration('scenario').perform(context)

    actions = []

    # 1. Gazebo Harmonic, once, with the mission world
    actions.append(IncludeLaunchDescription('gazebo_world.launch.py'))

    # 2. Micro XRCE-DDS Agent, once, shared by all PX4 instances
    actions.append(ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'], output='screen'))

    # 3. N PX4 SITL instances into the same Gazebo world, each at its own spawn pose
    for i in range(num_uavs):
        actions.append(IncludeLaunchDescription(
            'px4_vehicle.launch.py',
            launch_arguments={'instance': str(i), 'uav_id': f'uav_{i:02d}'}.items()))

    # 4. Per-UAV app node stacks, namespaced, remapped onto that instance's bridged topics
    for i in range(num_uavs):
        actions.append(GroupAction([
            PushRosNamespace(f'uav_{i:02d}'),
            IncludeLaunchDescription(
                'uav_stack.launch.py',
                launch_arguments={'uav_id': f'uav_{i:02d}', 'px4_instance': str(i)}.items()),
        ]))

    # 5. Central/shared nodes, once — depend on knowing the full UAV id list
    uav_ids = ','.join(f'uav_{i:02d}' for i in range(num_uavs))
    actions.append(IncludeLaunchDescription(
        'central_nodes.launch.py',
        launch_arguments={'uav_ids': uav_ids, 'scenario': scenario}.items()))

    return actions

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('num_uavs', default_value='6'),
        DeclareLaunchArgument('scenario', default_value='scenarios/scenario_basic.yaml'),
        DeclareLaunchArgument('headless', default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])
```

`OpaqueFunction` is required (not a plain static `LaunchDescription`) because the number of included launch files depends on the runtime `num_uavs` argument — a static launch description can't loop over a launch-time value.

### 4.4 `px4_vehicle.launch.py` — per-instance skeleton

Spawns one PX4 SITL process at a given pose into the already-running Gazebo world, plus its `ros_gz_bridge` topic bridge and a remap node that republishes PX4's `/px4_<n>/fmu/out/...` onto our `/uavN/...` namespace convention:

```python
ExecuteProcess(
    cmd=['PX4-Autopilot/build/px4_sitl_default/bin/px4',
         '-i', instance, '-d',
         f'PX4-Autopilot/ROMFS/px4fmu_common',
         ],
    additional_env={
        'PX4_GZ_MODEL_NAME': 'x500',
        'PX4_GZ_MODEL_POSE': spawn_pose_str,   # from vehicle_spawn_poses.yaml
        'PX4_SIM_MODEL': 'gz_x500',
    },
    output='screen')
```
followed by a `ros_gz_bridge` `parameter_bridge` node for that vehicle's Gazebo-native topics (camera/lidar if used later; Stage 1 needs pose only, which comes via uXRCE-DDS not the Gazebo bridge) and a small `topic_tools relay`/remap node mapping `/px4_<n>/fmu/out/vehicle_local_position` → `/uav_<n>/pose`, `/px4_<n>/fmu/out/battery_status` → raw input for `world_bridge`'s virtual battery node, and `/uav_<n>/cmd_vel` (published only by `safety_supervisor`, see Section 6.7) → `/px4_<n>/fmu/in/trajectory_setpoint`.

### 4.5 `uav_stack.launch.py` — per-UAV app nodes

```python
Node(package='task_allocation', executable='cbba_node', name='cbba_node',
     parameters=[{'uav_id': uav_id, 'bundle_capacity': 3}]),
Node(package='relay_placement', executable='relay_node', name='relay_node',
     parameters=[{'uav_id': uav_id}]),
Node(package='fault_recovery', executable='fault_manager_node', name='fault_manager_node',
     parameters=[{'uav_id': uav_id, 'timeout_ticks': 10}]),
Node(package='battery_rth', executable='battery_rth_node', name='battery_rth_node',
     parameters=[{'uav_id': uav_id, 'critical_frac': 0.2}]),
Node(package='world_bridge', executable='virtual_battery_node', name='virtual_battery_node',
     parameters=[{'uav_id': uav_id}]),
```

### 4.6 `central_nodes.launch.py`

```python
Node(package='comm_gateway', executable='comm_gateway_node', parameters=[{'uav_ids': uav_ids}]),
Node(package='safety_supervisor', executable='safety_supervisor_node', parameters=[{'uav_ids': uav_ids}]),
Node(package='fleet_manager', executable='fleet_manager_node', parameters=[{'uav_ids': uav_ids, 'scenario': scenario}]),
Node(package='mission_logger', executable='mission_logger_node', parameters=[{'out_path': 'logs/run.jsonl'}]),
Node(package='swarm_viz', executable='viz_bridge_node'),
```

---

## 5. Communication Layer — `comm_gateway` (the degraded-mesh simulator)

This is the module that requires the most explanation, since **Gazebo and PX4 have no native concept of a lossy, range-limited mesh network** — Gazebo simulates physics, and PX4/uXRCE-DDS/ROS 2's DDS transport all assume a reliable, unlimited-range local network by default. All degraded-network behavior is therefore our own application-layer middleware, not a network-stack-level simulation. This is a deliberate and documented simplification (see Assumptions §9) — it models *application-message* delivery realistically without needing OS-level network namespaces, `tc`/`netem` traffic shaping, or DDS QoS trickery.

### 5.1 Mechanism

- Every UAV's app nodes never publish `SwarmMessage` directly to another UAV's namespace. They **only ever publish to their own `/uavN/comm_tx` topic** and **only ever subscribe to their own `/uavN/comm_rx` topic**. This is the topic-level equivalent of "a radio only talks to the air, not directly to another radio's antenna."
- `comm_gateway_node` is the only subscriber to every `/uavN/comm_tx` and the only publisher to every `/uavN/comm_rx`. It is, in effect, the simulated RF channel.
- On a fixed timer (`gateway_tick_hz`, e.g. 10 Hz — independent of PX4's own physics rate):
  1. Subscribe to `/uavN/pose` for all N (bridged ground-truth PX4 local position — legitimate, since a real RF channel's propagation genuinely depends on real physical positions, not on any UAV's belief state).
  2. Compute pairwise `LinkState` via the same log-distance path-loss model as Revision 1 (`rssi = tx_power_dbm - (PL0 + 10*n*log10(d/d0)) + shadowing_sample`, `connected = rssi >= threshold`, `packet_loss_prob` derived from RSSI margin), publish `LinkStateArray` on `/swarm/link_states`.
  3. Build the connectivity graph (networkx, `connected=True` edges + a virtual `GCS` node) from step 2.
  4. For every in-flight `SwarmMessage` (queued internally, one queue per message) and every newly received `/uavN/comm_tx` message: advance it one hop along the shortest path (Dijkstra by `latency_s`) toward `dst_uav_id`; draw against `packet_loss_prob` for that hop (drop and stop forwarding on failure); decrement `ttl` (drop at 0); on reaching `dst_uav_id`, publish it to that UAV's `/uavN/comm_rx`.
- This node owns **all** cross-UAV message flow. If two UAVs are within range and a message is queued, it takes real simulated hops and real simulated latency to arrive — never instant, never bypassing the graph.

### 5.2 Why a central node doesn't secretly re-centralize the decentralized design

`comm_gateway` never makes *decisions* — it only computes physical link quality and forwards opaque envelopes. It plays exactly the role a real radio/mesh-routing layer plays in an actual multi-UAV system: the physical/link layer is naturally a shared, position-dependent medium in reality too. Decision-making (CBBA bids, relay choice, fault response) still happens per-UAV, only ever consuming what arrived on that UAV's own `/comm_rx` — this is unchanged from Revision 1's `UAVLocalView` principle, just relocated from a Python dataclass boundary to a ROS 2 topic/namespace boundary.

---

## 6. Modules as ROS 2 Nodes

Same responsibility split as Revision 1 (Section 2's table there still holds); this section only maps each responsibility onto nodes/topics/services.

### 6.1 World Bridge — package `world_bridge`

- **`virtual_battery_node`** (one instance per UAV, namespaced): subscribes to `/uavN/pose` and `/uavN/task_state` (role) for its own UAV, applies the same linear-with-role-multiplier drain model as Revision 1 to a *virtual* `battery_frac` (decoupled from Gazebo's own battery plugin, see Assumptions §9), publishes `/uavN/battery_state` (`UAVState.battery_frac`, or a dedicated lightweight message if you prefer not to overload `UAVState`).
- **`geofence_obstacle_publisher_node`** (once, central): reads the scenario YAML, publishes `Geofence`/`ObstacleArray` on latched (`transient_local` QoS) topics `/swarm/geofence` and `/swarm/obstacles`.
- **`collision_geofence_monitor_node`** (once, central): subscribes to all `/uavN/pose`, publishes `logging-only` `CollisionEvent`/`GeofenceViolation` topics for `mission_logger` — read-only, mirrors Revision 1's `check_collisions`/`check_geofence`, and must never disagree with `safety_supervisor`'s own violation count (test this explicitly, Section 8).
- **NOT responsible for:** actual UAV motion (PX4/Gazebo physics does that now), enforcing geofence (Safety Supervisor does).

### 6.2 Task Allocation — package `task_allocation`

- **`cbba_node`** (one per UAV, namespaced `/uavN/`):
  - Subscribes: `/uavN/comm_rx` (filters `TASK_BID`/`TASK_AWARD`/`TASK_RELEASE`/`POI_ALERT` message types), `/uavN/pose`, `/uavN/battery_state`.
  - Calls: `battery_rth`'s reachability check — exposed as a lightweight service `/uavN/estimate_reachability` (`EstimateReachability.srv`: `geometry_msgs/Point poi_position` → `bool reachable`) rather than a Python callback, since these are now separate processes.
  - Publishes: `/uavN/comm_tx` (outgoing `TASK_BID`/`TASK_AWARD` `SwarmMessage`s), `/uavN/task_state` (`UAVState.assigned_poi_id` + `bundle`, consumed by command-synthesis logic within this same node — see 6.2.1).
  - **NOT responsible for:** relay roles, fault detection, sending velocity commands directly to PX4 (all commands go through Safety Supervisor).

  **6.2.1 Command synthesis** lives inside `cbba_node` (for surveyors) and `battery_rth_node` (for RTH) and `relay_node` (for relays) — each publishes a *proposed* `/uavN/cmd_vel_proposed` (`geometry_msgs/Twist`) based on its own role logic; exactly one of these three nodes is "active" for a given UAV at a given time based on its current `role`, and only the active one's proposal is meaningful (Safety Supervisor consumes whichever was published most recently — implementer should add a `role` field check or a simple mutex-by-role convention, documented in that package's README).

### 6.3 Relay Placement — package `relay_placement`

- **`relay_node`** (one per UAV, namespaced): subscribes to `/uavN/comm_rx` (filters `RELAY_UPDATE`/gossiped `TASK_STATE`-equivalent info) to build a belief of current surveyor positions and available UAVs, runs the same greedy minimal-connectivity heuristic from Revision 1 against that belief, publishes `/uavN/comm_tx` (`RELAY_UPDATE` gossip messages so other UAVs' beliefs converge) and, if this UAV is itself selected as a relay, a `/uavN/cmd_vel_proposed` steering it to the computed relay waypoint. One instance also publishes the aggregate `/swarm/relay_decisions` (`RelayDecision`) for logging — since every UAV computes the same deterministic heuristic against a converged belief, any single instance's output is representative once gossip has propagated; document which instance's publish is treated as canonical for logging (e.g., lowest UAV id) to avoid duplicate log records.
- **NOT responsible for:** task allocation, fault detection.

### 6.4 Fault Recovery — packages `fault_recovery` + `fleet_manager`

- **`fault_manager_node`** (one per UAV, namespaced): subscribes to `/uavN/comm_rx` (HEARTBEAT messages from neighbors) and its own PX4 status bridge (`/uavN/vehicle_status` for hard-failure flags), publishes `HEARTBEAT` messages on its own `/uavN/comm_tx` on a fixed period, and publishes `/swarm/fault_events` (`FaultEvent`) when it detects a neighbor timeout or its own hard failure.
- **`fleet_manager_node`** (once, central, package `fleet_manager`): the one node allowed to arm/command a `RESERVE` UAV into active service, since spawning/activating a grounded physical airframe is inherently a fleet-level action no peer UAV can unilaterally do (this is the one place Revision 1's fully-decentralized story has a necessary real-world analogue: a ground station or designated leader typically holds this authority too — flag this honestly in the proposal rather than overclaiming decentralization here). Subscribes `/swarm/fault_events`, serves `/swarm/request_reserve_swap_in` (`RequestReserveSwapIn.srv`), and on acceptance issues PX4 arm + takeoff commands (via `VehicleCommand` uXRCE-DDS messages) to the chosen reserve UAV's PX4 instance.
- **NOT responsible for:** deciding *what task* the swapped-in UAV takes on (that's `cbba_node`'s next bidding round) or relay placement.

### 6.5 Battery / RTH — package `battery_rth`

- **`battery_rth_node`** (one per UAV, namespaced): subscribes `/uavN/battery_state`, `/uavN/pose`, `/swarm/link_states` (for its own reachability distance-to-GCS-over-mesh, not straight-line-only if you want to upgrade fidelity beyond Revision 1 — optional). Serves `/uavN/estimate_reachability` for `cbba_node`. Publishes `/uavN/rth_command` (bool) and, when RTH is decided, calls `/swarm/request_reserve_swap_in`, and publishes `/uavN/cmd_vel_proposed` steering toward GCS.
- **NOT responsible for:** arming the reserve UAV itself (delegates to `fleet_manager`).

### 6.6 Safety Supervisor — package `safety_supervisor` (unchanged authority, new mechanism)

- **`safety_supervisor_node`** (once, central): subscribes to every `/uavN/cmd_vel_proposed`, every `/uavN/pose`, `/swarm/geofence`, `/swarm/obstacles`. Runs on a fixed high-rate timer (e.g. 20 Hz, faster than the gateway/decision nodes, since safety needs to be lower-latency than mission logic). For each UAV: simulates one step of the proposed command, checks geofence/obstacle/min-separation exactly as Revision 1, and publishes the corrected `/uavN/cmd_vel` — **and this node is the ONLY node in the entire graph with permission to publish to the topic that is bridged into `/px4_<n>/fmu/in/trajectory_setpoint`.** This is enforced by convention (no other package's launch file wires anything to that bridge topic) — flag in code review / tests that no other node ever publishes to `/uavN/cmd_vel`.
- Publishes violation records to `/swarm/collision_events` and `/swarm/geofence_violations` for `mission_logger`.
- **NOT responsible for:** task/relay/battery decisions — only clamps whatever velocity it's handed.

---

## 7. Logging — `mission_logger` (same schema, new inputs)

**The JSONL schema, record types, and rubric-metric mapping from Revision 1 are unchanged** — reproduced here for completeness with the ROS 2 topic each field now comes from instead of an in-process function call.

`mission_logger_node` (once, central) subscribes to:
- `/swarm/link_states` (`LinkStateArray`) → feeds `tick_summary.connectivity_fraction`, and PDR/latency via message delivery counters it also gets from...
- every `/uavN/comm_tx` and `/uavN/comm_rx` (to count `messages_sent`/`messages_delivered`/`messages_dropped` and per-message `latency_s`, since the gateway doesn't itself publish a delivery-count topic — the logger derives PDR by diffing tx vs. rx counts per `msg_id`)
- every `/uavN/task_state` (`UAVState`) → PoI status transitions → `poi_event` records
- `/swarm/fault_events` (`FaultEvent`) → `fault_event` records
- `/swarm/relay_decisions` (`RelayDecision`) → `relay_event` records
- `/swarm/collision_events`, `/swarm/geofence_violations` → `collision_event`/`geofence_violation` records
- every `/uavN/battery_state` → `tick_summary.battery_min_frac`/`battery_mean_frac`

It maintains its own internal tick clock (subscribing to `/clock` from the Gazebo bridge, so log timestamps are simulation time, not wall-clock — important for reproducibility) and emits one `tick_summary` record per fixed logging interval (does not need to match the safety or gateway rate — e.g. 2 Hz is enough for a mission-level log).

### 7.1 Schema tables (unchanged from Revision 1 — reproduced for a self-contained doc)

**`tick_summary`**: `tick, sim_time_s, num_active_uavs, num_connected_uavs, connectivity_fraction` (→ connectivity availability), `messages_sent, messages_delivered, messages_dropped` (→ PDR), `avg_latency_s` (→ latency), `active_relay_count`, `poi_pending, poi_in_progress, poi_surveyed, poi_expired` (→ completion rate), `battery_min_frac, battery_mean_frac`.

**`poi_event`**: `tick, poi_id, event, priority, priority_weight` (→ priority-weighted mission score).

**`fault_event`**: `tick, uav_id, fault_type, detected_tick, recovered_tick, recovery_time_s` (→ recovery time).

**`relay_event`**: `tick, relay_uavs, reallocation_triggered, trigger_reason` (→ relay reallocations).

**`collision_event`**: `tick, uav_a, uav_b, distance_m, min_separation_m` (→ collision count).

**`geofence_violation`**: `tick, uav_id, position, corrected_position`.

**`mission_summary`** (one record, final): `completion_rate, priority_weighted_score, pdr, avg_latency_s, connectivity_availability, total_downtime_s, relay_reallocations_count, avg_recovery_time_s, max_recovery_time_s, collision_count, mission_duration_s, seed, scenario_name`.

---

## 8. Testing Strategy Notes

- **Per-package unit tests** (`ament_pytest` per ROS 2 package): pure logic (CBBA bid math, relay heuristic, path-loss formula) tested as plain Python functions, decoupled from ROS message I/O where possible — keep the math in a plain module, wrap it thinly in the node's callback.
- **`comm_gateway` integration test**: spin up the node with two fake `/uavN/pose` publishers at known distances, assert published `LinkState.rssi_dbm`/`connected` match hand-computed values.
- **Full-stack smoke test**: launch `mission_bringup` with `num_uavs=2` and a 30-second scenario, assert the log file contains at least one `tick_summary` and terminates cleanly.
- **Safety Supervisor exclusivity test**: statically grep all launch files / run `ros2 topic info /uav_00/cmd_vel -v` at runtime to assert exactly one publisher exists (must be `safety_supervisor_node`).
- **Determinism test**: same scenario + same PX4/Gazebo seed (`PX4_SIM_SPEED_FACTOR` fixed, Gazebo's physics is otherwise deterministic given fixed initial conditions) should produce log files with matching `mission_summary` completion/collision counts across two runs — full byte-identical JSONL is **not** guaranteed anymore (Revision 1's stronger determinism claim doesn't survive the move to real physics + DDS timing), so flag this relaxation explicitly in the proposal rather than overclaiming reproducibility.

---

## 9. Assumptions to Flag in the Technical Proposal

Carry over all of Revision 1's assumptions that still apply (PoI generation schedule, fault injection being scenario-scripted, no adversarial jamming, min-separation/geofence placeholder values, CBBA multi-tick convergence being intentional) and add these, specific to the ROS 2/PX4/Gazebo migration:

1. **Virtual battery is decoupled from Gazebo's own battery simulation.** We maintain our own role-conditioned `battery_frac` per UAV rather than relying on PX4/Gazebo's generic battery plugin, so that relay/surveyor/RTH roles drain at different rates as the mission design requires. This means the number shown in QGroundControl (if used for monitoring) will not match our logged `battery_frac` — a deliberate simplification, not a bug.
2. **The degraded mesh network is an application-layer simulation, not a network-stack-level one.** `comm_gateway` gates `SwarmMessage` topic delivery; it does not throttle DDS discovery, PX4/Gazebo's own topics, or any other ROS 2 traffic. A sufficiently adversarial reading of "multi-hop mesh" could argue this doesn't model real RF hardware constraints (e.g., actual radio interference, bandwidth caps) — flagged as an assumption to keep Stage 1 tractable without OS-level network namespacing per UAV process.
3. **PX4 SITL lockstep determinism does not extend to byte-identical logs across runs**, unlike the pure-Python Revision 1 sim — physics + DDS message timing introduce small nondeterminism even with a fixed seed. Reproducibility is claimed at the mission-outcome level (completion rate, collision count, etc.), not the exact-log-bytes level.
4. **Relay placement is computed redundantly per-UAV against a gossip-converged belief**, not by a single authoritative node, matching the decentralization philosophy — but this means during the gossip-convergence window after a topology change, different UAVs may briefly compute different relay sets. This transient disagreement is expected and should show up honestly in `relay_event` records rather than being masked by a canonical override.
5. **Reserve UAV activation (`fleet_manager`) is centralized by necessity**, not simulated as peer-to-peer — flagged honestly in Section 6.4 rather than overclaiming full decentralization for this one action.
6. **PX4 instance count is assumed to comfortably fit one machine's CPU/GPU budget for Gazebo Harmonic** (Stage 1 target: up to ~8-10 vehicles); scaling further would need headless rendering tuning or multi-machine Gazebo, out of scope until organizers confirm the arena's expected swarm size.
7. **All version numbers in Section 1 should be re-verified against `docs.px4.io` and `gazebosim.org` immediately before the team's from-scratch install**, since this is exactly the kind of fast-moving compatibility matrix that has already changed (Fortress → Garden → Harmonic) within PX4's recent release history.

(Duplicated into `docs/ASSUMPTIONS.md` for direct use in the proposal.)
