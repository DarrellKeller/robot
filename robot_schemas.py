"""Validated model boundaries. Shape validation is separate from Jev approval."""
from typing import Annotated, Generic, Literal, TypeVar

from pydantic import BaseModel, ConfigDict, Field, StringConstraints

Probability = Annotated[float, Field(ge=0, le=1, allow_inf_nan=False, strict=True)]
Text = Annotated[str, StringConstraints(strip_whitespace=True, min_length=1, max_length=1000)]
Reply = Annotated[str, StringConstraints(strip_whitespace=True, min_length=1, max_length=500)]


class Contract(BaseModel):
    model_config = ConfigDict(extra='forbid', strict=True)


class GoalDraft(Contract):
    goal: Text


class SpeechCandidates(Contract):
    candidates: Annotated[list[Reply], Field(min_length=1, max_length=3)]


class SceneDescription(Contract):
    sentences: Annotated[list[Reply], Field(min_length=1, max_length=3)]


class TranscriptQuality(Contract):
    word_count: Annotated[int, Field(ge=0)] = 0
    max_repeated_trigram: Annotated[int, Field(ge=0)] = 0
    no_speech_probability: Probability = 0.0
    compression_ratio: Annotated[float, Field(ge=0, allow_inf_nan=False)] = 0.0
    average_log_probability: Annotated[float, Field(allow_inf_nan=False)] = 0.0


class Transcript(Contract):
    text: Text
    quality: TranscriptQuality = Field(default_factory=TranscriptQuality)


Choice = TypeVar('Choice', bound=str)


class ChoiceAnswer(Contract, Generic[Choice]):
    type: Literal['choice']
    choice: Choice
    confidence: Probability
    probabilities: dict[str, Probability] = Field(default_factory=dict)


class NoulAnswer(Contract):
    type: Literal['noul']
    noul: Probability


class JevDecisions(Contract):
    user_route: ChoiceAnswer[Literal['ignore', 'clarify', 'chat', 'goal', 'steer', 'answer', 'cancel', 'resume']]
    approve_goal: NoulAnswer
    activity: ChoiceAnswer[Literal['wait', 'navigate', 'dance', 'talk', 'listen']]
    movement: ChoiceAnswer[Literal['forward', 'backward', 'left', 'right', 'stop']]
    need_fresh_vision: NoulAnswer
    lfm_speech_tool: ChoiceAnswer[Literal['none', 'ask_person_about_situation', 'answer_user', 'status_update', 'celebrate']]
    speech_choice: ChoiceAnswer[Literal['wait', 'reject', '1', '2', '3']]
    goal_complete: NoulAnswer
    should_remember: NoulAnswer
    speech_1_ok: NoulAnswer
    speech_2_ok: NoulAnswer
    speech_3_ok: NoulAnswer
