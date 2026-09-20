import sys
import unittest
import time
from types import SimpleNamespace
from unittest.mock import Mock, patch

from lfm_tools import LFMTools, Job


class SpeechGeneration(unittest.TestCase):
    def test_queued_vision_uses_current_frame_and_capture_time(self):
        worker = LFMTools.__new__(LFMTools)
        current_frame = object()
        captured_at = time.monotonic()
        worker.frame_provider = Mock(return_value=(current_frame, captured_at, 42.0))
        worker._generate = Mock(return_value='A doorway is ahead')
        job = Job(0, 1, 'vision', 'describe_scene', {}, 0, object(), captured_at - 10, 0)
        worker._execute(job, None, None, {})
        self.assertIs(job.frame, current_frame)
        self.assertEqual(job.captured_at, captured_at)
        self.assertEqual(job.heading, 42.0)

    def test_dead_camera_does_not_send_an_old_queued_frame(self):
        worker = LFMTools.__new__(LFMTools)
        worker.frame_provider = Mock(return_value=(object(), time.monotonic() - 2, 0))
        worker._generate = Mock()
        with self.assertRaises(ValueError):
            worker._execute(Job(0, 1, 'vision', 'describe_scene', {}, 0), None, None, {})
        worker._generate.assert_not_called()

    def test_history_reaches_template_and_empty_candidate_preserves_others(self):
        formatted = []
        def template(processor, config, prompt, **kwargs):
            formatted.append(prompt)
            return 'formatted'
        generate = Mock(side_effect=[SimpleNamespace(text=''),
                                    SimpleNamespace(text='Watch these wheels'),
                                    SimpleNamespace(text='You got it')])
        modules = {'mlx_vlm': SimpleNamespace(generate=generate),
                   'mlx_vlm.prompt_utils': SimpleNamespace(apply_chat_template=template)}
        state = {'dialogue': [{'role': 'user', 'text': 'Dance for me'}], 'sensors': {},
                 'vision': {}, 'current_step': {'kind': 'goal', 'instruction': 'Dance'}}
        with patch.dict(sys.modules, modules):
            result = LFMTools.__new__(LFMTools)._generate(
                Job(0, 1, 'speech', 'answer_user', state, 0), None, None, {})
        self.assertEqual(result, ['Watch these wheels', 'You got it'])
        self.assertIn({'role': 'user', 'content': 'Dance for me'}, formatted[0])
        self.assertIn('Current request: Dance', formatted[0][-1]['content'])
