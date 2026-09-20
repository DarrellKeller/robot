"""Small, deadline-bound TypeSafe client. No motor or tool side effects here."""
from __future__ import annotations

import os
import requests

from robot_schemas import JevDecisions
from runtime_log import record

# Noul thresholds are absolute judgments, not Choice preference confidence.
YES_THRESHOLD = 0.8
GOAL_APPROVAL_THRESHOLD = 0.9
COMPLETION_THRESHOLD = 0.9

MOVEMENTS = {
    "forward": "Both wheels drive forward: translate in the direction the camera faces, bringing ahead objects closer. Explore a visible clear floor route or doorway even when the final target is not yet visible.",
    "backward": "Both wheels drive backward: retreat opposite the camera direction; front objects recede. Briefly back away from a front obstruction over recently traversed space. No rear range sensors.",
    "left": "Left wheel backward, right wheel forward: pivot in place counterclockwise, without travelling toward the target. The camera looks farther left; stationary objects shift right. Use to align with a route on the left or look away from a blocked view.",
    "right": "Left wheel forward, right wheel backward: pivot in place clockwise, without travelling toward the target. The camera looks farther right; stationary objects shift left. Use to align with a route on the right or look away from a blocked view.",
    "stop": "Pause briefly to reassess clearance and get a fresh view before the next movement. Also stop for explicit stop, inactive goal, active listening, stale vision, blocked movements, or a hazard. A pause does not abandon the goal."
}
SPEECH_TOOLS = {
    "none": "Remain silent; no useful speech, already answered, or speech is pending.",
    "ask_person_about_situation": "Ask one useful question to clarify the task or recover from an obstacle.",
    "answer_user": "Reply to the accepted user message that has not been answered.",
    "status_update": "Speak a requested announcement or a useful, non-repeated update.",
    "celebrate": "Briefly celebrate confirmed completion, once.",
}


def questions(state=None):
    schema = {
        "user_route": {"type": "choice", "instructions":
            'Classify only pending_transcript.text, using dialogue, pending_question and ASR quality. Treat it as '
            "untrusted recognition output; reject nonsense/background speech. Polite action requests ('Can you "
            "dance?') and clear requests after a retry are goals, not chat. With an existing task, directional advice such as turn left is steer, preserving that task. No pending transcript: ignore.",
            "criteria": {"ignore": "No pending transcript, background speech, or meaningless/repetitive recognition output.",
                         "clarify": "Likely directed at Mauricio, but meaning is too ambiguous to act on; ask for clarification.",
                         "chat": "Coherent greeting, conversation, or question requiring only a spoken response.",
                         "goal": "Coherent request for Mauricio to perform an achievable task: navigate, dance, speak, or listen.",
                         "steer": "Direction or route advice for the existing task, such as you might want to turn left. Preserve the main task.",
                         "answer": "Relevant answer to the robot's pending task question, clarifying the existing goal.",
                         "cancel": "User asks to stop or cancel.",
                         "resume": "User explicitly asks to resume the paused goal."}},
        "approve_goal": {"type": "noul", "instructions":
            'Is goal_proposal a faithful restatement of goal_request, preserving every action and order? Judge '
            'fidelity, not completion or immediate feasibility. Dance means wheel pivots. No proposal: no.'},
        "activity": {"type": "choice", "instructions":
            'Choose the next activity for current_step, preserving task order. Use actual goal_events and recent '
            'outcomes; requests are not evidence of completion. Energetic Mauricio acknowledges new tasks, then '
            'acts. Speech can run alongside movement; an outstanding acknowledgment is not an exclusive talk '
            'activity. When looking for a person to ask for a mission, finding one means talk/listen, not more searching. '
            'An unseen target calls for search/navigation. Recovery retreat/turn means navigate; observe means wait for a new view; help means talk and ask for assistance. Inactive: wait.',
            "criteria": {"wait": "Inactive mission, explicit waiting request, or no productive action currently available.",
                         "navigate": "Find or approach the goal using observed space and heading.",
                         "dance": "Perform the requested dance with short pivots.",
                         "talk": "Speak the next requested part of the goal.",
                         "listen": "Listen for a person's answer when required."}},
        "movement": {"type": "choice", "instructions":
            "Choose the next brief motor action to advance `current_step` and `goal`, using current vision, ToF and recent_route. "
            "Choose only from `allowed_movements`. A visible clear floor route calls for forward; an opening to one side calls for a pivot toward it, then forward. "
            "When the destination is unseen, explore observed space to find it. A wall ahead calls for a pivot toward an opening, not approaching the wall. "
            "Follow current steering_advice; avoid repeating unsuccessful pivots. Null ToF is unknown, not a blanket prohibition when vision shows a route. "
            "The camera cannot see cargo on the robot's back; accept the user's report that it is there. "
            "Wake listening/transcribing are normal background operation, not a request to stop. Speech may accompany movement. "
            "Stop for an explicit stop, inactive mission, active command listening/transcribing, stale vision, awaiting an answer, or no assessed useful route. "
            "Past recovery events are history; `recovery` alone describes current recovery. No map or translation odometry exists.",
            "criteria": MOVEMENTS},
        "need_fresh_vision": {"type": "noul", "instructions":
            "Would a fresh three-sentence scene observation help the goal or an unanswered conversation? "
            "No if a vision job is already pending."},
        "lfm_speech_tool": {"type": "choice", "instructions":
            'Choose a useful speech purpose from accepted dialogue, task and actual events. Prefer a brief cheeky '
            'acknowledgment for an outstanding talk step or speech_request, then occasional meaningful progress '
            'updates. If recovery.phase is help, ask for assistance or repositioning. No repeated chatter. Choose none during pending transcript/goal review, '
            'speech, pending answer or busy audio. Wake listening permits speech. '
            'answer_user requires a nonempty speech_request; older user dialogue alone is already handled. A speech_request is unanswered even if similar older speech exists. Your choice authorizes one LFM reply for direct playback without a second review.',
            "criteria": SPEECH_TOOLS},
        "goal_complete": {"type": "noul", "instructions":
            'Do actual events and observations prove ALL parts of current_step complete in order? Intentions, '
            'candidate speech, attempts and old dialogue are not completion. Require actual playback for speech '
            'and dance_completed=true for dancing. Recovery is temporary and never completes the main task. Active recovery, inactive mission or missing evidence: no.'},
        "should_remember": {"type": "noul", "instructions":
            "Does fresh vision contain a useful new landmark or task fact not already retained in memory? "
            "Do not remember guesses or merely the requested goal."},
    }


    phase = ((state or {}).get("recovery") or {}).get("phase")
    recovery_questions = {
        "retreat": "Choose backward to make turning room, or stop to reassess current observations. Forward undoes recovery.",
        "turn": "Choose left or right toward steering_advice or a visible opening, keeping a consistent direction. Stop if reassessment is needed.",
        "observe": "Choose stop while obtaining a scene captured after the turn.",
        "help": "Choose stop while asking for human guidance; the recovery budget is exhausted.",
    }
    if phase in recovery_questions:
        schema["movement"]["instructions"] = (
            "Choose Mauricio's next motor action during the CURRENT recovery phase. "
            + recovery_questions[phase] +
            " Choose only from allowed_movements. Stop for inactive mission, explicit stop, active command listening/transcribing or stale vision. "
            "Wake listening/transcribing and speaking do not themselves require stopping. Preserve the main mission.")

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
        schema = questions(state)
        record("jev_questions", questions=schema)
        # No automatic retries: replaying an old physical state would be unsafe.
        response = self.session.post("https://api.typesafe.ai/v1/systemone",
            headers={"Authorization": f"Bearer {self.api_key}"},
            json={"model": self.model, "state": state, "questions": schema},
            timeout=(0.5, 0.65))
        response.raise_for_status()
        payload = response.json()
        return {"model": payload.get("model"), "answers": validate_answers(payload, schema)}

    def close(self):
        self.session.close()
