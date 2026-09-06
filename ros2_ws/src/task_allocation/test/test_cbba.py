"""
test_cbba.py — Unit tests for CBBA algorithm logic.
"""

import pytest
from task_allocation.cbba import (
    CBBAAgent, TaskInfo, BidInfo, compute_bid_value, compute_distance,
    PRIORITY_WEIGHTS,
)


@pytest.fixture
def sample_tasks():
    """Create a set of 3 PoIs at different positions and priorities."""
    return [
        TaskInfo(task_id='poi_01', position=(80.0, 120.0, 0.0),
                 priority=1, survey_duration_s=30.0),
        TaskInfo(task_id='poi_02', position=(200.0, -50.0, 0.0),
                 priority=1, survey_duration_s=25.0),
        TaskInfo(task_id='poi_03', position=(-100.0, 200.0, 0.0),
                 priority=2, survey_duration_s=20.0),
    ]


class TestComputeDistance:
    def test_zero_distance(self):
        assert compute_distance((0, 0, 0), (0, 0, 0)) == 0.0

    def test_known_distance(self):
        d = compute_distance((0, 0, 0), (3, 4, 0))
        assert abs(d - 5.0) < 0.001

    def test_3d_distance(self):
        d = compute_distance((1, 2, 3), (4, 6, 3))
        assert abs(d - 5.0) < 0.001


class TestComputeBidValue:
    def test_closer_agent_bids_higher(self, sample_tasks):
        """An agent closer to the task should bid higher."""
        task = sample_tasks[0]
        bid_close = compute_bid_value((70.0, 110.0, 0.0), task)
        bid_far = compute_bid_value((300.0, 300.0, 0.0), task)
        assert bid_close > bid_far

    def test_higher_priority_bids_higher(self):
        """Higher priority tasks should get higher bids."""
        routine = TaskInfo('t1', (100, 100, 0), priority=1, survey_duration_s=20)
        urgent = TaskInfo('t2', (100, 100, 0), priority=2, survey_duration_s=20)
        critical = TaskInfo('t3', (100, 100, 0), priority=3, survey_duration_s=20)

        pos = (0, 0, 0)
        b1 = compute_bid_value(pos, routine)
        b2 = compute_bid_value(pos, urgent)
        b3 = compute_bid_value(pos, critical)
        assert b3 > b2 > b1

    def test_low_battery_bids_lower(self, sample_tasks):
        """An agent with low battery should bid lower."""
        task = sample_tasks[0]
        bid_full = compute_bid_value((0, 0, 0), task, battery_frac=1.0)
        bid_low = compute_bid_value((0, 0, 0), task, battery_frac=0.3)
        assert bid_full > bid_low

    def test_bid_always_positive(self, sample_tasks):
        """Bid value must always be positive."""
        for task in sample_tasks:
            bid = compute_bid_value((0, 0, 0), task)
            assert bid > 0


class TestCBBAAgent:
    def test_bundle_phase_assigns_tasks(self, sample_tasks):
        """Agent should bid on available tasks."""
        agent = CBBAAgent('uav_00', (0, 0, 0), bundle_capacity=3)
        agent.set_tasks(sample_tasks)

        bids = agent.bundle_phase(current_time=0.0)
        assert len(bids) > 0
        assert len(agent.bundle) > 0

    def test_bundle_respects_capacity(self, sample_tasks):
        """Bundle should not exceed capacity."""
        agent = CBBAAgent('uav_00', (0, 0, 0), bundle_capacity=2)
        agent.set_tasks(sample_tasks)

        agent.bundle_phase(current_time=0.0)
        assert len(agent.bundle) <= 2

    def test_consensus_resolves_conflict(self, sample_tasks):
        """Higher bid should win in consensus."""
        agent_a = CBBAAgent('uav_00', (0, 0, 0), bundle_capacity=3)
        agent_b = CBBAAgent('uav_01', (200, 200, 0), bundle_capacity=3)

        agent_a.set_tasks(sample_tasks)
        agent_b.set_tasks(sample_tasks)

        bids_a = agent_a.bundle_phase(current_time=0.0)
        bids_b = agent_b.bundle_phase(current_time=0.0)

        # Cross-process consensus
        agent_a.consensus_phase(bids_b)
        agent_b.consensus_phase(bids_a)

        # No task should be assigned to both agents
        assigned_a = set(agent_a.get_assigned_tasks())
        assigned_b = set(agent_b.get_assigned_tasks())
        overlap = assigned_a & assigned_b
        assert len(overlap) == 0, f"Overlap detected: {overlap}"

    def test_three_agents_converge(self, sample_tasks):
        """3 agents, 3 tasks → each agent gets exactly 1 task."""
        agents = [
            CBBAAgent('uav_00', (0, 0, 0), bundle_capacity=1),
            CBBAAgent('uav_01', (100, 0, 0), bundle_capacity=1),
            CBBAAgent('uav_02', (0, 100, 0), bundle_capacity=1),
        ]

        for a in agents:
            a.set_tasks(sample_tasks)

        # Run multiple rounds until convergence
        for round_num in range(10):
            all_bids = []
            for a in agents:
                bids = a.bundle_phase(current_time=float(round_num))
                all_bids.extend(bids)

            # Cross-consensus
            for a in agents:
                other_bids = [b for b in all_bids if b.agent_id != a.agent_id]
                a.consensus_phase(other_bids)

        # Each agent should have exactly 1 task
        all_assigned = []
        for a in agents:
            assigned = a.get_assigned_tasks()
            all_assigned.extend(assigned)

        # No duplicates
        assert len(all_assigned) == len(set(all_assigned)), \
            f"Duplicate assignments: {all_assigned}"

        # All tasks assigned
        assert len(all_assigned) == 3, \
            f"Not all tasks assigned: {all_assigned}"

    def test_release_task(self, sample_tasks):
        """Releasing a task should remove it from bundle."""
        agent = CBBAAgent('uav_00', (0, 0, 0), bundle_capacity=3)
        agent.set_tasks(sample_tasks)
        agent.bundle_phase(current_time=0.0)

        first_task = agent.bundle[0]
        agent.release_task(first_task, current_time=1.0)
        assert first_task not in agent.bundle
        assert first_task not in agent.get_assigned_tasks()

    def test_priority_interrupt(self, sample_tasks):
        """New high-priority task should be added to bundle."""
        agent = CBBAAgent('uav_00', (0, 0, 0), bundle_capacity=3)
        agent.set_tasks(sample_tasks)
        agent.bundle_phase(current_time=0.0)

        critical_task = TaskInfo(
            task_id='poi_critical',
            position=(10, 10, 0),
            priority=3,
            survey_duration_s=10.0,
        )

        new_bids = agent.handle_priority_interrupt(critical_task, current_time=1.0)
        assert len(new_bids) > 0
        assert 'poi_critical' in agent.bundle
