import json
from pathlib import Path
import tempfile
import unittest
from runtime_log import start_trace, record, close_trace
from speech_text import plain_speech


class RuntimeLogging(unittest.TestCase):
    def test_separate_session_files_preserve_events(self):
        with tempfile.TemporaryDirectory() as directory:
            handler, first = start_trace(Path(directory))
            record('speech', approved_text='Hi!', tts_text='Hi')
            close_trace(handler)
            handler, second = start_trace(Path(directory))
            record('movement', action='stop')
            close_trace(handler)
            self.assertNotEqual(first, second)
            rows = [json.loads(line) for line in first.read_text().splitlines()]
            self.assertEqual([r['event'] for r in rows], ['speech', 'session_end'])
            self.assertEqual(rows[0]['tts_text'], 'Hi')

    def test_punctuation_does_not_merge_words_or_reach_tts(self):
        self.assertEqual(plain_speech('I’m Mauricio—ready! What’s next?'),
                         'I am Mauricio ready Whats next')
        self.assertEqual(plain_speech('left/right **now** 🤖'), 'left right now')
