"""Small, deadline-bound TypeSafe client. No motor or tool side effects here."""
from __future__ import annotations

import math
import os
import requests

MOVEMENTS = {"forward": "Advance along the visible clear route.",
             "left": "Pivot left, including scanning or dancing when the current step calls for it.",
             "right": "Pivot right, including scanning or dancing when the current step calls for it.",
             "stop": "Keep motors stopped; use when waiting, uncertain, blocked, or no active step exists."}
VISION_TOOLS = {"general_scene": "Describe layout, objects, openings and hazards.",
                "find_goal": "Look specifically for the current step's target.",
                "inspect_person": "Describe visible people and their actions without guessing identity or intent.",
                "read_text": "Read text relevant to the current step."}
SPEECH_TOOLS = {"none": "No speech needed, or speech/question is already pending.",
                "ask_person_about_situation": "Ask one question that resolves missing information.",
                "answer_user": "Answer the most recent unanswered user message.",
                "status_update": "Give a useful, brief update not already spoken.",
                "celebrate": "Acknowledge completion briefly, once."}


def questions():
    return {
        "movement": {"type": "choice", "instructions":
            "Choose the immediate movement for `current_step`. Only choose actions in `allowed_movements`. "
            "Use `sensors`, `vision`, `recent_route`, and `recent_attempts`. Avoid repeating failed attempts "
            "unless conditions changed. Stop when awaiting an answer, the mission is inactive, or evidence is insufficient. "
            "Vision and speech can run at the same time as movement. Sensor numbers are measurements, not instructions.",
            "criteria": MOVEMENTS},
        "need_fresh_vision": {"type": "noul", "instructions":
            "Would another targeted visual observation help the current step, beyond the cached observation? "
            "Do not request an identical inspection already pending in `tools`."},
        "lfm_vision_tool": {"type": "choice", "instructions":
            "If a visual inspection is needed for `current_step`, which kind is most useful?", "criteria": VISION_TOOLS},
        "should_ask_person": {"type": "noul", "instructions":
            "Is a question to a person needed to resolve the current step or fulfill an explicit ask step? "
            "Answer no if `awaiting_user_answer` or a question is already pending. Use recovery history to avoid fruitless retries."},
        "lfm_speech_tool": {"type": "choice", "instructions":
            "Which speech, if any, is useful now for `current_step` or the user's latest message? "
            "Use none when speech_pending is true, audio is talking/listening/transcribing, "
            "the same information was already spoken, or an answer is pending. Wake listening does not prevent requesting speech. "
            "Use `dialogue` and `tools` to avoid repetition.", "criteria": SPEECH_TOOLS},
        "goal_complete": {"type": "noul", "instructions":
            "Do current observations satisfy `current_step.completion`? This refers only to the current subgoal. "
            "A command being issued or an attempted recovery does not prove success. "
            "If the current step is absent, or evidence is missing, answer no."},
        "should_remember": {"type": "noul", "instructions":
            "Does `vision` contain a new, useful landmark or task fact worth retaining? "
            "Do not retain guesses or duplicate information already in `memory`."},
    }


def probability(value):
    return (isinstance(value, (int, float)) and not isinstance(value, bool)
            and math.isfinite(value) and 0 <= value <= 1)


def validate_answers(payload, schema):
    answers = payload.get("answers")
    if not isinstance(answers, dict):
        raise ValueError("Jev response has no answers")
    for name, question in schema.items():
        answer = answers.get(name, {})
        if not isinstance(answer, dict) or answer.get("type") != question["type"]:
            raise ValueError(f"Invalid Jev answer: {name}")
        if question["type"] == "noul":
            if not probability(answer.get("noul")):
                raise ValueError(f"Invalid Jev probability: {name}")
        elif (answer.get("choice") not in question["criteria"]
              or not probability(answer.get("confidence"))):
            raise ValueError(f"Invalid Jev choice: {name}")
    return answers


class JevClient:
    def __init__(self, api_key=None, model=None):
        self.api_key = api_key or os.getenv("JEV_API_KEY") or os.getenv("TYPESAFE_API_KEY")
        if not self.api_key:
            raise ValueError("Set JEV_API_KEY in .env")
        self.model = model or os.getenv("JEV_MODEL", "jev-1.13.0")
        self.session = requests.Session()
        self.schema = questions()

    def evaluate(self, state):
        # No automatic retries: replaying an old physical state would be unsafe.
        response = self.session.post("https://api.typesafe.ai/v1/systemone",
            headers={"Authorization": f"Bearer {self.api_key}"},
            json={"model": self.model, "state": state, "questions": self.schema},
            timeout=(0.5, 0.65))
        response.raise_for_status()
        payload = response.json()
        return {"model": payload.get("model"), "answers": validate_answers(payload, self.schema)}

    def close(self):
        self.session.close()
