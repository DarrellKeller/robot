import sys
import unittest
import time
from types import SimpleNamespace
from unittest.mock import Mock, patch

from lfm_tools import LFMTools, Job, scene_description, language_context


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

    def test_history_reaches_single_reply_generation(self):
        formatted = []
        def template(prompt, **kwargs):
            formatted.append(prompt)
            return 'formatted'
        generate = Mock(return_value='Watch these wheels')
        modules = {'mlx_lm': SimpleNamespace(generate=generate),
                   'mlx_lm.sample_utils': SimpleNamespace(make_sampler=Mock(), make_logits_processors=Mock())}
        tokenizer = SimpleNamespace(apply_chat_template=template)
        state = {'dialogue': [{'role': 'user', 'text': 'Dance for me'}], 'sensors': {},
                 'vision': {}, 'current_step': {'kind': 'goal', 'instruction': 'Dance'}}
        with patch.dict(sys.modules, modules):
            result = LFMTools.__new__(LFMTools)._generate(
                Job(0, 1, 'speech', 'answer_user', state, 0), None, tokenizer, {})
        self.assertEqual(result, 'Watch these wheels')
        generate.assert_called_once()
        self.assertIn({'role': 'user', 'content': 'Dance for me'}, formatted[0])
        self.assertIn('Current request: Dance', formatted[0][-1]['content'])

    def test_refusals_are_not_scene_descriptions(self):
        for text in ["I'm sorry, but I can't assist with that request.",
                     "I'm unable to assist with this request. The image contains inappropriate content.",
                     "I cannot help with that request"]:
            with self.assertRaises(ValueError):
                scene_description(text)
        self.assertEqual(scene_description("I see a sign saying I cannot help with that request."),
                         "I see a sign saying I cannot help with that request.")
        self.assertEqual(scene_description('A chair is ahead. An opening is to the right.'),
                         'A chair is ahead. An opening is to the right.')

    def test_vision_prompt_is_independent_of_raw_goal(self):
        generate = Mock(return_value=SimpleNamespace(text='A chair is ahead.'))
        template = Mock(return_value='formatted image prompt')
        modules = {'mlx_vlm': SimpleNamespace(generate=generate),
                   'mlx_vlm.prompt_utils': SimpleNamespace(apply_chat_template=template)}
        frame = Mock()
        with patch.dict(sys.modules, modules):
            LFMTools.__new__(LFMTools)._generate(
                Job(0, 1, 'vision', 'describe_scene', {'goal': 'RAW REQUEST'}, 0, frame), None, None, {})
        self.assertNotIn('RAW REQUEST', str(template.call_args))

    def test_full_shared_dialogue_window_is_available(self):
        dialogue = [{'role': 'user', 'text': str(i)} for i in range(30)]
        context = language_context({'dialogue': dialogue})
        self.assertEqual(len(context['dialogue']), 24)
        self.assertEqual(context['dialogue'][0]['content'], '6')
