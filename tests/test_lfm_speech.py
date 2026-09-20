import sys
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from lfm_tools import LFMTools, Job


class SpeechGeneration(unittest.TestCase):
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
        self.assertEqual(formatted[0][-1], {'role': 'user', 'content': 'Dance for me'})
