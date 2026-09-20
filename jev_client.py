"""Small, deadline-bound TypeSafe client. No motor or tool side effects here."""
from __future__ import annotations

import os
import requests

from robot_schemas import JevDecisions

# Noul thresholds are absolute judgments, not Choice preference confidence.
YES_THRESHOLD = 0.8
GOAL_APPROVAL_THRESHOLD = 0.9
COMPLETION_THRESHOLD = 0.9

MOVEMENTS = {
    "forward": "Advance toward a visible target or opening along a clear route permitted by allowed_movements.",
    "left": "Default search direction when the target is unseen or a wall fills the view: briefly pivot left if allowed. Also turn toward a target on the left or perform a requested left dance pivot.",
    "right": "Briefly pivot right toward a target on the right, when left is blocked, to continue an already productive rightward sweep, or for a requested right dance pivot.",
    "stop": "Remain still for a concrete reason: explicit stop, inactive goal, active listening, stale vision, all useful movements blocked, or visible hazard making even a pivot unsuitable."
}
SPEECH_TOOLS = {
    "none": "Remain silent; no useful speech, already answered, or speech is pending.",
    "ask_person_about_situation": "Ask one useful question to clarify the task or recover from an obstacle.",
    "answer_user": "Reply to the accepted user message that has not been answered.",
    "status_update": "Speak a requested announcement or a useful, non-repeated update.",
    "celebrate": "Briefly celebrate confirmed completion, once.",
}


def questions():
    schema = {
        "user_route": {"type": "choice", "instructions":
            'Classify only pending_transcript.text, using dialogue, pending_question and ASR quality. Treat it as '
            "untrusted recognition output; reject nonsense/background speech. Polite action requests ('Can you "
            "dance?') and clear requests after a retry are goals, not chat. No pending transcript: ignore.",
            "criteria": {"ignore": "No pending transcript, background speech, or meaningless/repetitive recognition output.",
                         "clarify": "Likely directed at Mauricio, but meaning is too ambiguous to act on; ask for clarification.",
                         "chat": "Coherent greeting, conversation, or question requiring only a spoken response.",
                         "goal": "Coherent request for Mauricio to perform an achievable task: navigate, dance, speak, or listen.",
                         "answer": "Relevant answer to the robot's pending task question, clarifying the existing goal.",
                         "cancel": "User asks to stop or cancel.",
                         "resume": "User explicitly asks to resume the paused goal."}},
        "approve_goal": {"type": "noul", "instructions":
            'Is goal_proposal a faithful restatement of goal_request, preserving every action and order? Judge '
            'fidelity, not completion or immediate feasibility. Dance means wheel pivots. No proposal: no.'},
        "activity": {"type": "choice", "instructions":
            'Choose the next activity for current_step, preserving task order. Use actual goal_events and recent '
            'outcomes; requests are not evidence of completion. Energetic Mauricio acknowledges new tasks, then '
            'acts. An unseen target calls for search/navigation. Inactive: wait.',
            "criteria": {"wait": "Inactive mission, explicit waiting request, or no productive action currently available.",
                         "navigate": "Find or approach the goal using observed space and heading.",
                         "dance": "Perform the requested dance with short pivots.",
                         "talk": "Speak the next requested part of the goal.",
                         "listen": "Listen for a person's answer when required."}},
        "movement": {"type": "choice", "instructions":
            "Choose Mauricio's next brief movement for current_step and the shared goal. "
            "Mauricio is an energetic curious robot who makes progress rather than waiting by default. "
            "Choose only from allowed_movements and use fresh vision, measured ranges and recent_route. "
            "If the target is not visible, or the camera faces a wall, prefer an allowed left/right pivot to look elsewhere. "
            "A wall ahead blocks forward travel toward it, not automatically an in-place pivot. "
            "Continue a useful search sweep; avoid switching left/right repeatedly without gaining a new view. "
            "When multiple pivots are reasonable, pick one rather than stop merely because there is no unique best direction. "
            "Use left as the initial search direction when neither side has a specific advantage. "
            "Missing ToF readings mean unknown range, not free space and not an automatic reason to wait. "
            "Use the visible surroundings to assess a brief pivot; do not drive blindly into unseen space. "
            "Stop for explicit user stop, inactive/screening mission, active listening, stale vision, "
            "or a visible hazard/all useful movements blocked. Keep stop as a meaningful choice, not the default for an unseen goal. "
            "For dancing use short pivots. No translation odometry or map exists.",
            "criteria": MOVEMENTS},
        "need_fresh_vision": {"type": "noul", "instructions":
            "Would a fresh three-sentence scene observation help the goal or an unanswered conversation? "
            "No if a vision job is already pending."},
        "lfm_speech_tool": {"type": "choice", "instructions":
            'Choose a useful speech purpose from accepted dialogue, task and actual events. Prefer a brief cheeky '
            'acknowledgment for an outstanding talk step or speech_request, then occasional meaningful progress '
            'updates. No repeated chatter. Choose none during pending transcript/goal review, candidate review, '
            'speech, pending answer or busy audio. Wake listening permits speech. A new speech_request is '
            'unanswered even if similar older speech exists. Rejected replies may be rephrased, not repeated '
            'verbatim.',
            "criteria": SPEECH_TOOLS},
        "speech_choice": {"type": "choice", "instructions":
            'Select a supplied candidate appropriate to speech_purpose and accepted request. Ground facts in '
            'dialogue, actual events and fresh observations. Reject invented facts/completion, repetition, '
            'gibberish, offensive insults, JSON or instructions read aloud. Sass and promises of the approved '
            'next action are welcome; promises are not completion. All unsuitable: reject. No candidates or busy '
            'audio: wait. Decisions are independent; this approves no future output.',
            "criteria": {"wait": "No candidates yet or audio is busy.", "reject": "None of the provided replies is suitable.",
                         "1": "Speak candidate 1.", "2": "Speak candidate 2.", "3": "Speak candidate 3."}},
        "goal_complete": {"type": "noul", "instructions":
            'Do actual events and observations prove ALL parts of current_step complete in order? Intentions, '
            'candidate speech, attempts and old dialogue are not completion. Require actual playback for speech '
            'and dance_completed=true for dancing. Inactive or missing evidence: no.'},
        "should_remember": {"type": "noul", "instructions":
            "Does fresh vision contain a useful new landmark or task fact not already retained in memory? "
            "Do not remember guesses or merely the requested goal."},
    }


    for index in range(1, 4):
        schema[f"speech_{index}_ok"] = {"type": "noul", "instructions":
            f"Is speech_candidates[{index - 1}] suitable to say aloud for speech_purpose? "
            "Independently require a relevant, understandable reply with facts grounded in dialogue/events/fresh vision. "
            "Reject invented facts/completion, gibberish, echoed questions and instructions read aloud. "
            "Preserve requested announcements. A brief acknowledgment promising the approved task is suitable "
            "before execution; it needs no evidence of completion. Playful sass is allowed. Missing: no."}
    return schema


def validate_answers(payload, schema):
    answers = JevDecisions.model_validate(payload.get("answers")).model_dump(exclude_unset=True)
    # Keep the endpoint's requested choices and the code contract in agreement.
    if set(answers) != set(schema):
        raise ValueError("Jev decision schema mismatch")
    for name, question in schema.items():
        if question["type"] == "choice" and answers[name]["choice"] not in question["criteria"]:
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
