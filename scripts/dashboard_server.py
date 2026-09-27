#!/usr/bin/env python3
"""
dashboard_server.py — Live WebSocket bridge for UAV Swarm Dashboard.

Subscribes to ROS 2 swarm topics and pushes JSON state to any browser
connected on ws://0.0.0.0:8765.

Usage (inside container):
    python3 scripts/dashboard_server.py --uav-ids uav_00,uav_01,uav_02,uav_03,uav_04,uav_05
"""

import argparse
import asyncio
import json
import math
import sys
import threading
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Point
from swarm_msgs.msg import (
    UAVState, LinkStateArray, FaultEvent, RelayDecision,
    Geofence, ObstacleArray
)

import websockets

# ---------------------------------------------------------------------------
# Shared state (updated by ROS thread, read by WS thread)
# ---------------------------------------------------------------------------

ROLE_NAMES = {0: 'SURVEYOR', 1: 'RELAY', 2: 'RTH', 3: 'RESERVE', 4: 'GROUNDED', 5: 'IDLE'}
HEALTH_NAMES = {0: 'NOMINAL', 1: 'DEGRADED', 2: 'FAILED'}

state = {
    'uavs': {},          # uav_id -> {pos, role, health, battery, assigned_poi, bundle}
    'links': [],         # [{uav_a, uav_b, connected, rssi_dbm, is_relay_edge}]
    'connectivity_fraction': 0.0,
    'geofence': [],      # [{x,y}]
    'obstacles': [],     # [{cx,cy,r}]
    'fault_log': deque(maxlen=50),   # recent fault events
    'relay_log': deque(maxlen=30),   # recent relay decisions
    'mission': {
        'poi_pending': 0, 'poi_in_progress': 0, 'poi_surveyed': 0,
        'poi_expired': 0, 'active_relays': 0, 'active_uavs': 0,
        'messages_sent': 0, 'messages_delivered': 0, 'messages_dropped': 0,
    },
    'log_tail': deque(maxlen=80),    # last N lines from JSONL log
    'ts': 0.0,
}
_state_lock = threading.Lock()

LOG_PATH = '/home/px4user/uav_swarm/logs/log_live.jsonl'


# ---------------------------------------------------------------------------
# ROS 2 subscriber node
# ---------------------------------------------------------------------------

class DashboardBridge(Node):
    def __init__(self, uav_ids):
        super().__init__('dashboard_bridge')
        self.uav_ids = uav_ids

        be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        durability=DurabilityPolicy.VOLATILE,
                        history=HistoryPolicy.KEEP_LAST, depth=1)
        rel = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)

        # Per-UAV
        for uid in uav_ids:
            with _state_lock:
                state['uavs'][uid] = {
                    'id': uid, 'x': 0.0, 'y': 0.0, 'z': 0.0,
                    'role': 'IDLE', 'health': 'NOMINAL',
                    'battery': 1.0, 'assigned_poi': '', 'bundle': []
                }
            self.create_subscription(Point, f'/{uid}/pose',
                lambda msg, u=uid: self._pose(u, msg), be)
            self.create_subscription(UAVState, f'/{uid}/task_state',
                lambda msg, u=uid: self._task(u, msg), rel)
            self.create_subscription(UAVState, f'/{uid}/battery_state',
                lambda msg, u=uid: self._battery(u, msg), be)

        # Global
        self.create_subscription(LinkStateArray, '/swarm/link_states', self._links, rel)
        self.create_subscription(Geofence, '/swarm/geofence', self._geofence, latched)
        self.create_subscription(ObstacleArray, '/swarm/obstacles', self._obstacles, latched)
        self.create_subscription(FaultEvent, '/swarm/fault_events', self._fault, rel)
        self.create_subscription(RelayDecision, '/swarm/relay_decisions', self._relay, rel)

        self.get_logger().info(f'DashboardBridge started for {len(uav_ids)} UAVs')

    def _pose(self, uid, msg):
        with _state_lock:
            u = state['uavs'].setdefault(uid, {'id': uid})
            u['x'] = round(msg.x, 2)
            u['y'] = round(msg.y, 2)
            u['z'] = round(msg.z, 2)
            state['ts'] = time.time()

    def _task(self, uid, msg):
        with _state_lock:
            u = state['uavs'].setdefault(uid, {'id': uid})
            u['role'] = ROLE_NAMES.get(msg.role, 'UNKNOWN')
            u['health'] = HEALTH_NAMES.get(msg.health, 'UNKNOWN')
            u['assigned_poi'] = msg.assigned_poi_id
            u['bundle'] = list(msg.bundle)
            if msg.battery_frac > 0:
                u['battery'] = round(msg.battery_frac, 3)

    def _battery(self, uid, msg):
        with _state_lock:
            u = state['uavs'].setdefault(uid, {'id': uid})
            if msg.battery_frac >= 0:
                u['battery'] = round(msg.battery_frac, 3)

    def _links(self, msg):
        links = []
        for l in msg.links:
            links.append({
                'a': l.uav_a, 'b': l.uav_b,
                'connected': l.connected,
                'rssi': round(l.rssi_dbm, 1),
                'relay': l.is_relay_edge,
                'loss': round(l.packet_loss_prob, 3),
            })
        with _state_lock:
            state['links'] = links
            state['connectivity_fraction'] = round(msg.connectivity_fraction, 3)

    def _geofence(self, msg):
        pts = [{'x': p.x, 'y': p.y} for p in msg.polygon]
        with _state_lock:
            state['geofence'] = pts

    def _obstacles(self, msg):
        obs = [{'cx': o.center.x, 'cy': o.center.y, 'r': o.radius_m} for o in msg.obstacles]
        with _state_lock:
            state['obstacles'] = obs

    def _fault(self, msg):
        ev = {
            'ts': time.strftime('%H:%M:%S'),
            'uav': msg.uav_id,
            'type': msg.fault_type,
            'recovered': msg.recovered_stamp.sec > 0,
            'recovery_s': round(getattr(msg, 'recovery_time_s', 0.0), 1),
        }
        with _state_lock:
            state['fault_log'].appendleft(ev)

    def _relay(self, msg):
        ev = {
            'ts': time.strftime('%H:%M:%S'),
            'relays': list(msg.relay_uav_ids),
            'trigger': msg.trigger_reason,
            'ok': msg.connectivity_ok,
        }
        with _state_lock:
            state['relay_log'].appendleft(ev)


# ---------------------------------------------------------------------------
# JSONL log tailer (runs in background thread)
# ---------------------------------------------------------------------------

def tail_log():
    last_size = 0
    while True:
        try:
            with open(LOG_PATH, 'r') as f:
                f.seek(0, 2)
                size = f.tell()
                if size < last_size:
                    last_size = 0
                if size > last_size:
                    f.seek(last_size)
                    for raw in f:
                        raw = raw.strip()
                        if not raw:
                            continue
                        try:
                            rec = json.loads(raw)
                            rtype = rec.get('type', '')
                            with _state_lock:
                                state['log_tail'].appendleft(rec)
                                m = state['mission']
                                if rtype == 'tick_summary':
                                    m['poi_pending'] = rec.get('poi_pending', 0)
                                    m['poi_in_progress'] = rec.get('poi_in_progress', 0)
                                    m['poi_surveyed'] = rec.get('poi_surveyed', 0)
                                    m['poi_expired'] = rec.get('poi_expired', 0)
                                    m['active_relays'] = rec.get('active_relay_count', 0)
                                    m['active_uavs'] = rec.get('num_active_uavs', 0)
                                    m['messages_sent'] = rec.get('messages_sent', 0)
                                    m['messages_delivered'] = rec.get('messages_delivered', 0)
                                    m['messages_dropped'] = rec.get('messages_dropped', 0)
                                    m['battery_min'] = round(rec.get('battery_min_frac', 0.0), 3)
                                    m['battery_mean'] = round(rec.get('battery_mean_frac', 0.0), 3)
                                    m['sim_time_s'] = rec.get('sim_time_s', 0.0)
                                    m['connectivity'] = round(rec.get('connectivity_fraction', 0.0), 3)
                        except Exception:
                            pass
                    last_size = size
        except FileNotFoundError:
            pass
        time.sleep(0.3)


# ---------------------------------------------------------------------------
# WebSocket server
# ---------------------------------------------------------------------------

async def handler(websocket):
    print(f'[WS] Client connected: {websocket.remote_address}')
    try:
        while True:
            with _state_lock:
                payload = {
                    'uavs': list(state['uavs'].values()),
                    'links': state['links'],
                    'connectivity': state['connectivity_fraction'],
                    'geofence': state['geofence'],
                    'obstacles': state['obstacles'],
                    'fault_log': list(state['fault_log'])[:20],
                    'relay_log': list(state['relay_log'])[:10],
                    'mission': dict(state['mission']),
                    'log_tail': list(state['log_tail'])[:30],
                }
            await websocket.send(json.dumps(payload))
            await asyncio.sleep(0.4)   # ~2.5 Hz refresh
    except websockets.exceptions.ConnectionClosed:
        print(f'[WS] Client disconnected')


async def ws_main(host, port):
    print(f'[WS] Serving on ws://{host}:{port}')
    async with websockets.serve(handler, host, port):
        await asyncio.Future()  # run forever


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def ros_thread(uav_ids):
    rclpy.init()
    node = DashboardBridge(uav_ids)
    try:
        rclpy.spin(node)
    except Exception:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--uav-ids', default='uav_00,uav_01,uav_02,uav_03,uav_04,uav_05')
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8765)
    args = parser.parse_args()

    uav_ids = [x.strip() for x in args.uav_ids.split(',') if x.strip()]
    print(f'[DASH] Starting for UAVs: {uav_ids}')

    # Start log tailer
    t = threading.Thread(target=tail_log, daemon=True)
    t.start()

    # Start ROS in background thread
    rt = threading.Thread(target=ros_thread, args=(uav_ids,), daemon=True)
    rt.start()

    time.sleep(1.5)  # let ROS init
    asyncio.run(ws_main(args.host, args.port))


if __name__ == '__main__':
    main()
