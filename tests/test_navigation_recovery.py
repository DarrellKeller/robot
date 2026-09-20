"""Fast telemetry replay: no serial, network, microphone or model loading."""
import tempfile
import unittest
from pathlib import Path

from jev_control import Controller
from mission_store import MissionStore
from navigation_recovery import Recovery
from test_jev_control import telemetry


class RecoveryReplay(unittest.TestCase):
    def setUp(self):
        self.r = Recovery()
        self.now = 100.0

    def feed(self, distance=250, *, motion='stop', dt=.1, vision=None, **changes):
        self.now += dt
        sensors = telemetry(motion=motion, tof_mm=[None, 393, distance, 299 if distance == 250 else distance, None], **changes)
        return self.r.update(sensors, vision or {}, self.now, enabled=True)

    def clear(self):
        for _ in range(5):
            self.feed(450, motion='backward')
        self.assertEqual(self.r.phase, 'turn')

    def test_wall_retreat_turn_fresh_view_preserves_order(self):
        self.feed()
        self.assertEqual(self.r.phase, 'retreat')
        self.assertEqual(self.r.context()['blocking_sensors'], ['F', 'R45'])
        # Hardware permits forward again at 31 cm; recovery still needs turning room.
        self.feed(310, motion='backward')
        self.assertEqual(self.r.allowed(['stop', 'forward', 'left', 'right', 'backward']), ['stop', 'backward'])
        self.clear()
        self.assertEqual(self.r.allowed(['stop', 'forward', 'left', 'right', 'backward']), ['stop', 'left', 'right'])
        self.feed(450, motion='left', heading_deg=0)
        self.feed(450, motion='left', heading_deg=22)
        self.assertEqual(self.r.phase, 'observe')
        self.assertEqual(self.r.allowed(['stop', 'forward', 'left']), ['stop'])
        old_capture = self.now - 1
        self.feed(450, vision={'fresh': True, 'captured_at': old_capture})
        self.assertEqual(self.r.phase, 'observe')
        stopped_at = self.r.stopped_at
        event = self.feed(450, vision={'fresh': True, 'captured_at': stopped_at + .01})
        self.assertIsNone(self.r.phase)
        self.assertEqual(event['from'], 'observe')

    def test_missing_blocker_or_stale_packet_does_not_prove_clearance(self):
        self.feed()
        for _ in range(8):
            self.feed(None)
        self.assertEqual(self.r.phase, 'retreat')
        self.feed(450)
        self.feed(450, age_s=.4)
        self.feed(450)
        self.feed(450)
        self.assertEqual(self.r.phase, 'retreat')
        self.assertEqual(self.r.reverse_s, 0)

    def test_continuous_reverse_cannot_evade_attempt_limit(self):
        self.feed()
        for _ in range(26):
            self.feed(motion='backward')
        self.assertEqual(self.r.phase, 'help')
        self.assertGreaterEqual(self.r.bad_attempts, 4)
        self.assertEqual(self.r.allowed(['stop', 'backward']), ['stop'])

    def test_missing_ranges_do_not_count_as_failure_but_reverse_stays_bounded(self):
        self.feed()
        for _ in range(25):
            self.feed(None, motion='backward')
        self.assertEqual(self.r.phase, 'retreat')
        self.assertEqual(self.r.bad_attempts, 0)
        for _ in range(20):
            self.feed(None, motion='backward')
        self.assertEqual(self.r.phase, 'help')
        self.assertEqual(self.r.bad_attempts, 0)

    def test_reacquired_range_can_confirm_progress(self):
        self.feed()
        for _ in range(8):
            self.feed(None, motion='backward')
        for _ in range(6):
            self.feed(350, motion='backward')
        self.assertEqual(self.r.trend, 'improving')
        self.assertEqual(self.r.bad_attempts, 0)
        self.assertEqual(self.r.phase, 'retreat')

    def test_slow_pivot_has_time_to_reach_measured_heading(self):
        self.feed()
        self.clear()
        for i in range(23):
            self.feed(450, motion='left', heading_deg=i)
        self.assertEqual(self.r.phase, 'observe')

    def test_improvement_does_not_allow_unbounded_reverse(self):
        self.feed(100)
        for i in range(25):
            self.feed(100 + i * 5, motion='backward')
        self.assertEqual(self.r.phase, 'retreat')
        for i in range(20):
            self.feed(225 + i * 5, motion='backward')
        self.assertEqual(self.r.phase, 'help')
        self.assertGreaterEqual(self.r.reverse_s, 4)

    def test_stops_do_not_count_as_reverse_attempts_and_timeout_asks_help(self):
        self.feed()
        for _ in range(305):
            self.feed()
        self.assertEqual(self.r.reverse_s, 0)
        self.assertEqual(self.r.bad_attempts, 0)
        self.assertEqual(self.r.phase, 'help')

    def test_new_obstacle_during_turn_returns_to_retreat(self):
        self.feed()
        self.clear()
        self.feed()
        self.assertEqual(self.r.phase, 'retreat')
        self.assertGreater(self.r.reverse_s, 0)  # No fresh reverse budget.

    def test_paused_mission_does_not_start_or_advance_recovery(self):
        self.r.update(telemetry(tof_mm=[250] * 5), {}, self.now, enabled=False)
        self.assertIsNone(self.r.phase)
        self.feed()
        for i in range(5):
            self.r.update(telemetry(tof_mm=[450] * 5), {}, self.now + i, enabled=False)
        self.assertEqual(self.r.phase, 'retreat')

    def test_accepted_steering_resumes_a_mission_paused_for_clarification(self):
        with tempfile.TemporaryDirectory() as d:
            c = Controller.__new__(Controller)
            c.store = MissionStore(Path(d) / 'state.json')
            c.store.install('Find an objective', [{'kind': 'goal', 'instruction': 'Find an objective', 'completion': 'Found'}])
            c.store.data['status'] = 'screening'
            c.prior_status, c.epoch = 'paused', 6
            c.pending_transcript = {'text': 'Go right that way'}
            c.goal_request = None
            c.route_user('steer')
            self.assertEqual(c.store.data['status'], 'active')
            self.assertEqual(c.store.data['steering_advice'], 'Go right that way')
            self.assertEqual(c.store.data['goal'], 'Find an objective')

    def test_steering_preserves_task_and_new_goal_resets_recovery(self):
        with tempfile.TemporaryDirectory() as d:
            c = Controller.__new__(Controller)
            c.store = MissionStore(Path(d) / 'state.json')
            steps = [{'kind': 'goal', 'instruction': 'Deliver the item to a girl', 'completion': 'Delivered'}]
            c.store.install('Deliver the item to a girl', steps)
            c.pending_transcript = {'text': 'You might want to turn left'}
            c.prior_status, c.epoch = 'active', 1
            c.goal_request = None
            c.route_user('steer')
            self.assertEqual(c.store.data['goal'], 'Deliver the item to a girl')
            self.assertEqual(c.store.data['steps'], steps)
            self.assertIsNone(c.goal_request)
            self.assertEqual(c.store.context()['steering_advice'], 'You might want to turn left')
            c.store.recovery = self.r
            self.feed()
            c.store.install('Find a table', steps)
            self.assertIsNone(c.store.context()['recovery'])
            self.assertIsNone(c.store.context()['steering_advice'])
