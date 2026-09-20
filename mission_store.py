"""Persistent subgoals and compact evidence. All mutation stays on the control thread."""
from __future__ import annotations

import copy
import json
import os
import time

from pathlib import Path

from navigation_recovery import Recovery

KINDS = {"navigate", "dance", "talk", "listen", "goal"}


def validate_plan(value):
    if not isinstance(value, dict) or value.get("intent") not in {"new_goal", "clarification", "conversation", "cancel"}:
        raise ValueError("Invalid plan intent")
    steps = value.get("steps", [])
    if not isinstance(steps, list) or len(steps) > 8:
        raise ValueError("Plan must contain at most eight steps")
    clean = []
    for step in steps:
        if not isinstance(step, dict) or step.get("kind") not in KINDS:
            raise ValueError("Unsupported step kind")
        if any(not isinstance(step.get(k), str) or not 1 <= len(step[k].strip()) <= 400
               for k in ("instruction", "completion")):
            raise ValueError("Missing step instruction or completion criterion")
        clean.append({k: step[k] for k in ("kind", "instruction", "completion")})
    if value["intent"] == "new_goal" and not clean:
        raise ValueError("A new goal needs steps")
    return {"intent": value["intent"], "steps": clean}


class MissionStore:
    def __init__(self, path, *, load_saved=False):
        self.path = Path(path)
        self.data = {"version": 2, "goal": "", "steps": [], "step_index": 0,
                     "status": "idle", "memory": [], "dialogue": [], "recent_attempts": [],
                     "recent_route": [], "goal_events": [], "pending_question": None, "last_outcome": "startup"}
        if load_saved and self.path.exists():
            saved = json.loads(self.path.read_text())
            if saved.get("version") not in {1, 2}:
                raise ValueError("Unsupported mission store version")
            self.data.update(saved)
            if saved["version"] == 1:
                # Preserve pre-screening transcripts on disk, but do not treat them
                # as Jev-approved conversation or feed them back to either model.
                self.data["legacy_dialogue"] = self.data["dialogue"]
                self.data["dialogue"] = []
                self.data["version"] = 2
            # A saved mission never restarts physical movement automatically.
            if self.data["status"] in {"active", "screening", "drafting"}:
                self.data["status"] = "paused"
            self.data["pending_question"] = None
        self.recovery = Recovery()
        self.revision = 0
        self.step_started = time.monotonic()
        self.segment = None
        self.dirty = not load_saved

    @property
    def step(self):
        i = self.data["step_index"]
        return self.data["steps"][i] if i < len(self.data["steps"]) else None

    def changed(self):
        self.dirty = True
        self.revision += 1

    def utterance(self, role, text):
        self.data["dialogue"].append({"role": role, "text": text[:1000]})
        self.data["dialogue"] = self.data["dialogue"][-24:]
        self.changed()

    def install(self, goal, plan):
        self.recovery = Recovery()
        self.data["steering_advice"] = None
        self.data.update(goal=goal[:1000], goal_user_request=goal[:1000], steps=plan, step_index=0, status="active",
                         recent_attempts=[], goal_events=[], pending_question=None, last_outcome="goal_started")
        self.step_started = time.monotonic()
        self.changed()

    def goal_event(self, kind, detail):
        self.data["goal_events"].append({"kind": kind, "detail": detail})
        self.data["goal_events"] = self.data["goal_events"][-24:]
        self.changed()

    def advance(self):
        self.recovery = Recovery()
        self.data["steering_advice"] = None
        self.data["step_index"] += 1
        self.data["recent_attempts"] = []
        self.data["last_outcome"] = "step_completed"
        if self.step is None:
            self.data["status"] = "completed"
        self.step_started = time.monotonic()
        self.changed()

    def outcome(self, action, result):
        entry = {"action": action, "result": result}
        # Record events, not repeated telemetry packets of the same stop.
        self.data["recent_attempts"].append(entry)
        self.data["recent_attempts"] = self.data["recent_attempts"][-6:]
        self.data["last_outcome"] = result
        self.dirty = True

    def observe_motion(self, snapshot):
        if not snapshot or snapshot.get("age_s", 1) > 0.25:
            self.segment = None
            return
        now = time.monotonic()
        action = snapshot["motion"]
        if self.segment and self.segment["action"] == action:
            self.segment["duration_s"] = round(now - self.segment["started"], 2)
            self.segment["heading_change_deg"] = round(snapshot["heading_deg"] - self.segment["heading_start"], 1)
            self.dirty = True
            return
        if self.segment:
            self.segment["duration_s"] = round(now - self.segment["started"], 2)
            self.segment["heading_change_deg"] = round(snapshot["heading_deg"] - self.segment["heading_start"], 1)
            entry = {k: v for k, v in self.segment.items() if k not in {"started", "heading_start"}}
            entry["ended_by"] = snapshot["stop_reason"] if action == "stop" else "new_command"
            self.data["recent_route"].append(entry)
            self.data["recent_route"] = self.data["recent_route"][-8:]
            if action == "stop" and snapshot["stop_reason"] not in {"commanded", "none"}:
                self.outcome(self.segment["action"], snapshot["stop_reason"])
            self.dirty = True
        self.segment = None if action == "stop" else {
            "action": action, "started": now, "heading_start": snapshot["heading_deg"],
            "duration_s": 0, "heading_change_deg": 0, "translation": "unmeasured"}

    def remember(self, observation):
        text = observation.get("text", "")
        if text and not any(m["text"] == text for m in self.data["memory"]):
            self.data["memory"].append({"text": text[:500], "source": "vision_observation",
                                         "heading_deg": observation.get("heading_deg")})
            self.data["memory"] = self.data["memory"][-12:]
            self.dirty = True

    def context(self):
        d = copy.deepcopy(self.data)
        d.pop("legacy_dialogue", None)
        d["recovery"] = self.recovery.context()
        attempts = d["recent_attempts"]
        repeated = len(attempts) >= 2 and attempts[-1] == attempts[-2]
        d.update(current_step=copy.deepcopy(self.step),
                 step_elapsed_s=round(time.monotonic() - self.step_started, 1),
                 repeated_failure=repeated,
                 awaiting_user_answer=d["pending_question"] is not None)
        if self.segment:
            d["current_motion"] = {k: v for k, v in self.segment.items() if k not in {"started", "heading_start"}}
        return d

    def save(self):
        if not self.dirty:
            return
        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.path.with_suffix('.tmp')
        with tmp.open('w') as stream:
            json.dump(self.data, stream, indent=2)
            stream.flush()
            os.fsync(stream.fileno())
        tmp.replace(self.path)
        self.dirty = False
