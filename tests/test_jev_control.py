"""Offline checks for the physical-control boundary; no API, camera or motors."""
import copy
import json
from pathlib import Path
import tempfile
import time
import unittest
from concurrent.futures import Future
from types import SimpleNamespace
from unittest.mock import Mock, patch

from jev_client import questions, validate_answers
from jev_control import Controller, choose_movement
from mission_store import MissionStore, validate_plan
from robot_link import RobotLink, allowed_movements, parse_telemetry
from audio_controller import AudioController


def telemetry(**changes):
    value = dict(protocol=2, uptime_ms=1000, command_id=1, motion="stop", stop_reason="commanded",
                 tof_mm=[900, 900, 900, 900, 900], heading_deg=72.0, yaw_rate_dps=0.0,
                 imu_valid=True, imu_age_ms=5, imu_model="MPU-6050", age_s=0.01)
    value.update(changes)
    return value


def answers():
    return {name: ({"type": "noul", "noul": 0.1} if q["type"] == "noul" else
                   {"type": "choice", "choice": next(iter(q["criteria"])), "confidence": 0.95})
            for name, q in questions().items()}


class WakeAcknowledgement(unittest.TestCase):
    def test_acknowledgement_stops_first_and_does_not_complete_talk(self):
        audio = AudioController(enabled=False)
        tts = Mock()
        def speak(text):
            self.assertEqual(text, 'Huh?')
            self.assertEqual(audio.status(), 'listening')
            self.assertEqual(audio.events.get_nowait(), {'kind': 'listening'})
            return True
        tts.speak.side_effect = speak
        self.assertTrue(audio._handle_wake('Mauricio!', True, tts))
        self.assertTrue(audio.events.empty())

    def test_same_utterance_command_is_preserved(self):
        audio = AudioController(enabled=False)
        self.assertFalse(audio._handle_wake('Robot, stop.', True, Mock()))
        self.assertEqual(audio.events.get_nowait()['kind'], 'listening')
        self.assertEqual(audio.events.get_nowait(), {'kind': 'user', 'text': 'stop'})
        self.assertTrue(audio.events.empty())

    def test_unrelated_speech_does_not_acknowledge(self):
        audio = AudioController(enabled=False)
        tts = Mock()
        self.assertFalse(audio._handle_wake('Hello there', True, tts))
        tts.speak.assert_not_called()
        self.assertTrue(audio.events.empty())


class Boundaries(unittest.TestCase):
    def test_bad_or_missing_sensors_cannot_move(self):
        for data in ({}, telemetry(age_s=0.3), telemetry(imu_valid=False), telemetry(imu_age_ms=101),
                     telemetry(tof_mm=[900, None, 900, 900, 900])):
            self.assertEqual(allowed_movements(data), ["stop"])
        self.assertEqual(allowed_movements(telemetry(tof_mm=[100, 900, 900, 900, 900])), ["stop", "forward"])
        self.assertEqual(allowed_movements(telemetry(tof_mm=[900, 900, 100, 900, 900])), ["stop"])

    def test_reject_legacy_and_malformed_packets(self):
        for packet in ('1,2,3,4,5', '{}', '[]', json.dumps(telemetry(heading_deg=float('nan'))),
                       json.dumps(telemetry(tof_mm=[900, 900, -1, 900, 900]))):
            self.assertIsNone(parse_telemetry(packet))
        self.assertIsNotNone(parse_telemetry(json.dumps(telemetry())))

    def test_dispatch_rechecks_sensors(self):
        link = RobotLink(dry_run=True)
        link.serial = Mock()
        link.latest, link.received_at = telemetry(), time.monotonic()
        self.assertEqual(link.command('forward'), 'forward')
        link.latest['tof_mm'][2] = 100
        self.assertEqual(link.command('forward'), 'stop')
        self.assertIn(b',stop,600\n', link.serial.write.call_args.args[0])

    def test_no_movement_on_old_decisions_or_missing_visual_context(self):
        a = answers()
        state = dict(status='active', awaiting_user_answer=False, audio_state='wake_listening',
                     vision={'fresh': True}, allowed_movements=['stop', 'forward'])
        self.assertEqual(choose_movement(a, state, 0.2), 'forward')
        self.assertEqual(choose_movement(a, state, 0.6), 'stop')
        for changes in ({'status': 'paused'}, {'awaiting_user_answer': True},
                        {'audio_state': 'listening'}, {'vision': {'fresh': False}},
                        {'allowed_movements': ['stop']}):
            self.assertEqual(choose_movement(a, state | changes, 0.2), 'stop')
        a['movement']['confidence'] = 0.4
        self.assertEqual(choose_movement(a, state, 0.2), 'stop')

    def test_jev_schema_validation(self):
        a = answers()
        self.assertEqual(validate_answers({'answers': a}, questions()), a)
        for field, value in [('choice', 'reverse'), ('confidence', float('nan'))]:
            bad = copy.deepcopy(a)
            bad['movement'][field] = value
            with self.assertRaises(ValueError):
                validate_answers({'answers': bad}, questions())
        bad = copy.deepcopy(a)
        bad['goal_complete']['noul'] = True
        with self.assertRaises(ValueError):
            validate_answers({'answers': bad}, questions())

    def test_minimal_parallel_schema(self):
        self.assertNotIn('next_mode', questions())
        self.assertFalse(any('recovery' in k for k in questions()))
        self.assertIn('none', questions()['lfm_speech_tool']['criteria'])


class Persistence(unittest.TestCase):
    def test_restart_does_not_resume_and_route_is_measured(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'mission.json'
            store = MissionStore(path)
            store.install('Find table', [{'kind': 'navigate', 'instruction': 'Find table', 'completion': 'Table visible'}])
            store.observe_motion(telemetry(motion='left', heading_deg=170))
            store.observe_motion(telemetry(motion='left', heading_deg=190))
            store.observe_motion(telemetry(motion='stop', heading_deg=190, stop_reason='clearance_or_sensor'))
            self.assertEqual(store.data['recent_route'][0]['heading_change_deg'], 20)
            self.assertEqual(store.data['recent_route'][0]['translation'], 'unmeasured')
            store.save()
            reloaded = MissionStore(path)
            self.assertEqual(reloaded.data['status'], 'paused')
            self.assertEqual(reloaded.step['instruction'], 'Find table')

    def test_plan_validation(self):
        with self.assertRaises(ValueError):
            validate_plan({'intent': 'new_goal', 'steps': []})
        with self.assertRaises(ValueError):
            validate_plan({'intent': 'new_goal', 'steps': [{'kind': 'raw_motor', 'instruction': 'go', 'completion': 'done'}]})
        self.assertEqual(validate_plan({'intent': 'cancel'})['steps'], [])

    def test_subgoal_progress_does_not_skip(self):
        with tempfile.TemporaryDirectory() as d:
            store = MissionStore(Path(d) / 'mission.json')
            steps = [{'kind': 'navigate', 'instruction': 'Find table', 'completion': 'Visible'},
                     {'kind': 'talk', 'instruction': 'Ask', 'completion': 'Spoken'}]
            store.install('Find then ask', steps)
            store.advance()
            self.assertEqual(store.step['kind'], 'talk')
            self.assertEqual(store.data['status'], 'active')
            store.advance()
            self.assertEqual(store.data['status'], 'completed')


class DecisionLifecycle(unittest.TestCase):
    def bare_controller(self, tmp):
        c = Controller.__new__(Controller)
        c.link = Mock()
        c.store = MissionStore(Path(tmp) / 'mission.json')
        c.epoch = 2
        c.request_epoch = 1
        c.request_at = time.monotonic()
        c.future = Future()
        c.future.set_result({'answers': answers(), 'model': 'test'})
        c.last_sent = 'forward'
        return c

    def test_previous_goal_response_cannot_restart_motors(self):
        with tempfile.TemporaryDirectory() as d:
            c = self.bare_controller(d)
            c.handle_decision(time.monotonic())
            c.link.command.assert_called_once_with('stop')
            self.assertIsNone(c.future)

    def test_late_response_cannot_restart_motors(self):
        with tempfile.TemporaryDirectory() as d:
            c = self.bare_controller(d)
            c.request_epoch = c.epoch
            c.request_at -= 1
            c.handle_decision(time.monotonic())
            c.link.command.assert_called_once_with('stop')

    def test_failed_request_stops_and_backs_off(self):
        with tempfile.TemporaryDirectory() as d:
            c = self.bare_controller(d)
            c.future = Future()
            c.future.set_exception(TimeoutError())
            c.api_failures = 0
            now = time.monotonic()
            c.handle_decision(now)
            c.link.command.assert_called_once_with('stop')
            self.assertGreater(c.api_retry_at, now)

    def test_stop_still_allows_parallel_vision_and_question(self):
        with tempfile.TemporaryDirectory() as d:
            c = self.bare_controller(d)
            c.request_epoch = c.epoch
            a = answers()
            a['movement']['choice'] = 'stop'
            a['need_fresh_vision']['noul'] = 0.99
            a['lfm_vision_tool']['choice'] = 'find_goal'
            a['should_ask_person']['noul'] = 0.99
            c.future = Future()
            c.future.set_result({'answers': a, 'model': 'test'})
            c.store.install('Find table', [{'kind': 'navigate', 'instruction': 'Find table', 'completion': 'Visible'}])
            state = c.store.context() | dict(sensors=telemetry(), vision={'fresh': True},
                audio_state='idle', allowed_movements=['stop'])
            c.context = Mock(return_value=state)
            c.tools = Mock()
            c.tools.status.return_value = {}
            c.link.command.side_effect = lambda action: action
            c.request_speech = Mock()
            c.audit = Mock()
            c.request_state = state
            c.handle_decision(time.monotonic())
            c.link.command.assert_called_once_with('stop')
            self.assertEqual(c.desired_vision, 'find_goal')
            self.assertTrue(c.request_speech.call_args.args[2])

    def test_actual_playback_completes_talk_step_and_retains_question(self):
        with tempfile.TemporaryDirectory() as d:
            c = self.bare_controller(d)
            c.audio = Mock()
            c.audio.events = __import__('queue').Queue()
            c.audio.events.put({'kind': 'spoken', 'revision': c.epoch, 'text': 'Which room?', 'ask': True})
            c.store.install('Ask and listen', [
                {'kind': 'talk', 'instruction': 'Ask room', 'completion': 'Spoken'},
                {'kind': 'listen', 'instruction': 'Hear answer', 'completion': 'Answer captured'}])
            c.speech_completes_step = True
            c.handle_audio()
            self.assertEqual(c.store.step['kind'], 'listen')
            self.assertEqual(c.store.data['pending_question']['text'], 'Which room?')
            self.assertEqual(c.store.data['dialogue'][-1]['role'], 'assistant')

    def test_failed_playback_does_not_complete_talk_step(self):
        with tempfile.TemporaryDirectory() as d:
            c = self.bare_controller(d)
            c.audio = Mock()
            c.audio.events = __import__('queue').Queue()
            c.audio.events.put({'kind': 'speech_failed', 'revision': c.epoch})
            c.store.install('Say hello', [{'kind': 'talk', 'instruction': 'Hello', 'completion': 'Spoken'}])
            c.handle_audio()
            self.assertEqual(c.store.data['step_index'], 0)

    def test_plan_failure_asks_for_retry_without_advancing(self):
        with tempfile.TemporaryDirectory() as d:
            c = self.bare_controller(d)
            c.tools = Mock()
            c.tools.results = __import__('queue').Queue()
            c.tools.results.put({'kind': 'plan', 'revision': c.epoch, 'error': 'ValueError'})
            c.audio = Mock()
            c.audio.speak.return_value = True
            c.speech_completes_step = True
            c.handle_tools()
            self.assertEqual(c.store.data['status'], 'paused')
            self.assertFalse(c.speech_completes_step)
            self.assertTrue(c.speech_pending)
            self.assertEqual(c.audio.speak.call_args.args[1:], (True, c.epoch))


if __name__ == '__main__':
    unittest.main()
