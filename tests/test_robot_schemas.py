import unittest
from pydantic import ValidationError
from robot_schemas import GoalDraft, SpeechCandidates, Transcript, JevDecisions
from lfm_tools import language_context
from audio_controller import transcript_quality
from jev_client import questions


class Contracts(unittest.TestCase):
    def test_exactly_three_nonempty_replies(self):
        for candidates in ([], ['one'], ['one', 'two', ''], ['a', 'b', 'c', 'd']):
            with self.assertRaises(ValidationError):
                SpeechCandidates(candidates=candidates)
        self.assertEqual(SpeechCandidates(candidates=[' one ', 'two', 'three']).candidates[0], 'one')

    def test_goals_reject_wrong_types_and_extra_actions(self):
        for value in ({'goal': []}, {'goal': '  '}, {'goal':'Dance', 'motor':'forward'}):
            with self.assertRaises(ValidationError):
                GoalDraft.model_validate(value)

    def test_transcript_quality_is_bounded_and_detects_repetition(self):
        quality = transcript_quality({'text': 'a little bit ' * 20})
        self.assertGreater(quality['max_repeated_trigram'], 10)
        Transcript(text='a little bit', quality=quality)
        with self.assertRaises(ValidationError):
            Transcript(text='Hello', quality={'no_speech_probability': float('nan')})

    def test_unapproved_transcript_never_reaches_language_context(self):
        context = language_context({'pending_transcript': {'text':'garbage'}, 'legacy_dialogue': ['garbage'],
                                    'dialogue':[{'role':'user','text':'hello'}], 'goal':'Find table'})
        self.assertNotIn('garbage', str(context))
        self.assertEqual(context['goal'], 'Find table')

    def test_questions_match_typed_decisions(self):
        self.assertEqual(set(questions()), set(JevDecisions.model_fields))
        self.assertNotIn('lfm_vision_tool', questions())


if __name__ == '__main__':
    unittest.main()
