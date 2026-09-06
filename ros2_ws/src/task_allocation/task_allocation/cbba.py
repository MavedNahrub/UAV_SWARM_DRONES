"""
cbba.py — Consensus-Based Bundle Algorithm (CBBA) pure logic.

No ROS dependencies — unit-testable independently.

CBBA works in two phases per round:
1. Bundle phase: each agent greedily builds a bundle of tasks by computing
   marginal score improvements (score = priority_weight / distance).
2. Consensus phase: agents exchange bids and resolve conflicts using
   a simple highest-bid-wins rule with timestamps.

Convergence: after enough rounds (typically N_agents), all agents agree
on a conflict-free assignment.
"""

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple


@dataclass
class TaskInfo:
    """A task (PoI) as seen by a CBBA agent."""
    task_id: str
    position: Tuple[float, float, float]
    priority: int = 1           # ROUTINE=1, URGENT=2, CRITICAL=3
    survey_duration_s: float = 20.0
    status: int = 0             # PENDING=0, ASSIGNED=1, IN_PROGRESS=2, SURVEYED=3
    deadline_s: float = 0.0     # 0 = no deadline


@dataclass
class BidInfo:
    """A bid on a task by an agent."""
    agent_id: str
    task_id: str
    bid_value: float
    timestamp: float            # sim time of bid


# Priority weight mapping
PRIORITY_WEIGHTS = {
    1: 1.0,   # ROUTINE
    2: 3.0,   # URGENT
    3: 10.0,  # CRITICAL
}


def compute_distance(pos_a: Tuple[float, float, float],
                     pos_b: Tuple[float, float, float]) -> float:
    """Euclidean distance between two 3D positions."""
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(pos_a, pos_b)))


def compute_bid_value(agent_pos: Tuple[float, float, float],
                      task: TaskInfo,
                      battery_frac: float = 1.0,
                      speed_mps: float = 10.0) -> float:
    """
    Compute bid value for a task.

    Score = priority_weight / (distance / speed + survey_duration)
    Higher is better. Battery multiplier penalizes low-battery agents.

    Args:
        agent_pos: Agent's current position (x, y, z).
        task: Task information.
        battery_frac: Agent's current battery fraction [0, 1].
        speed_mps: Agent's cruise speed in m/s.

    Returns:
        Bid value (higher = more suitable).
    """
    dist = compute_distance(agent_pos, task.position)
    travel_time = dist / max(speed_mps, 0.1)
    total_time = travel_time + task.survey_duration_s

    priority_weight = PRIORITY_WEIGHTS.get(task.priority, 1.0)

    # Battery penalty: linear reduction for low battery
    battery_factor = max(0.1, battery_frac)

    bid = (priority_weight * battery_factor) / max(total_time, 0.1)
    return bid


class CBBAAgent:
    """
    CBBA agent logic for a single UAV.

    Maintains:
    - bundle: ordered list of task_ids this agent is bidding on
    - winning_bids: dict[task_id] → BidInfo (the agent's belief of the
      winning bid for each task)
    """

    def __init__(self, agent_id: str, position: Tuple[float, float, float],
                 bundle_capacity: int = 3, battery_frac: float = 1.0,
                 speed_mps: float = 10.0):
        self.agent_id = agent_id
        self.position = position
        self.bundle_capacity = bundle_capacity
        self.battery_frac = battery_frac
        self.speed_mps = speed_mps

        # Agent's bundle: ordered list of task IDs this agent is bidding on
        self.bundle: List[str] = []

        # Agent's path: ordered list of task IDs for execution order
        # (same as bundle for this simple implementation)
        self.path: List[str] = []

        # Agent's belief of winning bids: task_id → BidInfo
        self.winning_bids: Dict[str, BidInfo] = {}

        # Available tasks
        self.tasks: Dict[str, TaskInfo] = {}

    def update_position(self, pos: Tuple[float, float, float]):
        self.position = pos

    def update_battery(self, battery_frac: float):
        self.battery_frac = battery_frac

    def set_tasks(self, tasks: List[TaskInfo]):
        """Set/update the task list."""
        for task in tasks:
            self.tasks[task.task_id] = task

    def add_task(self, task: TaskInfo):
        """Add a single new task (e.g., priority interrupt)."""
        self.tasks[task.task_id] = task

    def bundle_phase(self, current_time: float) -> List[BidInfo]:
        """
        Phase 1: Greedily add tasks to bundle up to capacity.

        Returns list of new bids generated this round.
        """
        new_bids = []

        while len(self.bundle) < self.bundle_capacity:
            best_task_id = None
            best_bid_value = -1.0

            for task_id, task in self.tasks.items():
                # Skip already-bundled tasks
                if task_id in self.bundle:
                    continue

                # Skip completed/expired tasks
                if task.status in (3, 4):  # SURVEYED, EXPIRED
                    continue

                # Compute bid value
                bid_value = compute_bid_value(
                    self.position, task,
                    self.battery_frac, self.speed_mps
                )

                # Check if we can outbid the current winner
                current_winner = self.winning_bids.get(task_id)
                if current_winner and current_winner.agent_id != self.agent_id:
                    if bid_value <= current_winner.bid_value:
                        continue

                if bid_value > best_bid_value:
                    best_bid_value = bid_value
                    best_task_id = task_id

            if best_task_id is None:
                break  # No more tasks to bid on

            # Add to bundle
            self.bundle.append(best_task_id)
            self.path.append(best_task_id)

            # Record our bid
            bid = BidInfo(
                agent_id=self.agent_id,
                task_id=best_task_id,
                bid_value=best_bid_value,
                timestamp=current_time,
            )
            self.winning_bids[best_task_id] = bid
            new_bids.append(bid)

        return new_bids

    def consensus_phase(self, received_bids: List[BidInfo]) -> bool:
        """
        Phase 2: Process bids from other agents, resolve conflicts.

        Simple rule: highest bid wins. On tie, earlier timestamp wins.
        On tie in both, lower agent_id wins (deterministic).

        Returns True if any changes were made (re-run bundle phase).
        """
        changed = False

        for incoming_bid in received_bids:
            task_id = incoming_bid.task_id
            current = self.winning_bids.get(task_id)

            if current is None:
                # No existing bid — accept
                self.winning_bids[task_id] = incoming_bid
                changed = True
                continue

            # Compare: highest bid wins
            if incoming_bid.bid_value > current.bid_value:
                winner = incoming_bid
            elif incoming_bid.bid_value < current.bid_value:
                winner = current
            elif incoming_bid.timestamp < current.timestamp:
                winner = incoming_bid
            elif incoming_bid.timestamp > current.timestamp:
                winner = current
            elif incoming_bid.agent_id < current.agent_id:
                winner = incoming_bid
            else:
                winner = current

            if winner != current:
                self.winning_bids[task_id] = winner
                changed = True

                # If we lost a task in our bundle, remove it
                if (current.agent_id == self.agent_id and
                        task_id in self.bundle):
                    self.bundle.remove(task_id)
                    if task_id in self.path:
                        self.path.remove(task_id)

        return changed

    def get_assigned_tasks(self) -> List[str]:
        """Get task IDs assigned to this agent (where this agent wins)."""
        assigned = []
        for task_id in self.bundle:
            winner = self.winning_bids.get(task_id)
            if winner and winner.agent_id == self.agent_id:
                assigned.append(task_id)
        return assigned

    def get_current_target(self) -> Optional[str]:
        """Get the first assigned task (next to execute)."""
        assigned = self.get_assigned_tasks()
        return assigned[0] if assigned else None

    def release_task(self, task_id: str, current_time: float):
        """
        Release a task from this agent's bundle (e.g., on fault/RTH).
        Clears the bid so other agents can pick it up.
        """
        if task_id in self.bundle:
            self.bundle.remove(task_id)
        if task_id in self.path:
            self.path.remove(task_id)
        if task_id in self.winning_bids:
            if self.winning_bids[task_id].agent_id == self.agent_id:
                del self.winning_bids[task_id]

    def handle_priority_interrupt(self, new_task: TaskInfo,
                                  current_time: float) -> List[BidInfo]:
        """
        Handle a new high-priority task injected mid-mission.

        Only re-bids on the new task + any tasks it displaces.
        Does NOT clear the entire bundle (partial re-auction per spec).
        """
        self.add_task(new_task)

        # Compute bid for the new task
        bid_value = compute_bid_value(
            self.position, new_task,
            self.battery_frac, self.speed_mps
        )

        bid = BidInfo(
            agent_id=self.agent_id,
            task_id=new_task.task_id,
            bid_value=bid_value,
            timestamp=current_time,
        )

        # If bundle is full, check if new task outbids the weakest
        if len(self.bundle) >= self.bundle_capacity:
            # Find weakest task in bundle
            weakest_id = None
            weakest_value = float('inf')
            for tid in self.bundle:
                w = self.winning_bids.get(tid)
                if w and w.bid_value < weakest_value:
                    weakest_value = w.bid_value
                    weakest_id = tid

            if weakest_id and bid_value > weakest_value:
                self.release_task(weakest_id, current_time)
            else:
                return []  # Can't fit the new task

        # Add new task to bundle
        self.bundle.append(new_task.task_id)
        self.path.append(new_task.task_id)
        self.winning_bids[new_task.task_id] = bid

        return [bid]
