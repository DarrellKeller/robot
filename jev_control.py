"""Jev decides; local code owns state, parallel jobs and motor side effects."""
from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
import json
import logging
from logging.handlers import RotatingFileHandler
import queue
import signal
import time
from pathlib import Path

from audio_controller import AudioController
from jev_client import JevClient
from lfm_tools import Camera, LFMTools
from mission_store import MissionStore, validate_plan
from robot_link import RobotLink, allowed_movements

ROOT = Path(__file__).resolve().parent
DECISION_MAX_AGE = 0.45
DECISION_CONFIDENCE = 0.75  # Initial settings; validate on recorded robot scenarios.
YES_THRESHOLD = 0.8
VISION_MAX_AGE = 2.0


def drain(q):
    while True:
        try:
            yield q.get_nowait()
        except queue.Empty:
            return


def choose_movement(answers, state, result_age):
    answer = answers["movement"]
    if (result_age > DECISION_MAX_AGE or answer["confidence"] < DECISION_CONFIDENCE
        or state["status"] != "active" or state["awaiting_user_answer"]
        or state["audio_state"] in {"listening", "transcribing"}
        or not state["vision"].get("fresh")):
        return "stop"
    action = answer["choice"]
    return action if action in state["allowed_movements"] else "stop"


class Controller:
    def __init__(self, args):
        self.args = args
        self.store = MissionStore(args.state)
        self.client = JevClient()
        self.link = RobotLink(args.port, dry_run=not args.live)
        self.camera = None if args.no_camera else Camera(args.camera, lambda: self.link.snapshot().get("heading_deg"))
        self.tools = LFMTools()
        self.audio = AudioController(enabled=not args.no_audio)
        self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="jev")
        self.future = None
        self.epoch = 0
        self.closed = False
        self.observation = {}
        self.last_decision_at = 0
        self.next_decision_at = 0
        self.next_vision_at = 0
        self.last_save_at = 0
        self.speech_pending = False
        self.speech_requested_at = 0
        self.speech_completes_step = False
        self.last_speech_at = -100
        self.user_pending = None
        self.prior_status = self.store.data["status"]
        self.request_state = None
        self.request_at = 0
        self.request_epoch = 0
        self.last_sent = "stop"
        self.last_audio_state = "idle"
        self.last_heading = None
        self.dance_seconds = 0.0
        self.previous_tick = time.monotonic()
        self.api_failures = 0
        self.api_retry_at = 0
        self.desired_vision = None
        self.step_overdue_recorded = False
        self.audit = logging.getLogger("jev.audit")
        self.audit.propagate = False
        self.audit.setLevel(logging.INFO)
        args.state.parent.mkdir(parents=True, exist_ok=True)
        handler = RotatingFileHandler(args.state.parent / 'decisions.jsonl', maxBytes=2_000_000, backupCount=2)
        self.audit.addHandler(handler)
        self.audit_handler = handler
        if args.resume and self.store.step:
            self.store.data["status"] = "active"
            self.store.changed()
        if args.plan:
            plan = validate_plan(json.loads(Path(args.plan).read_text()))
            if plan["intent"] != "new_goal":
                raise ValueError("--plan requires a new_goal with at least one step")
            self.store.install(args.goal or "User-supplied plan", plan["steps"])
        elif args.goal:
            self.on_user(args.goal)

    def invalidate(self):
        self.epoch += 1
        self.audio.invalidate(self.epoch)
        self.last_decision_at = 0
        self.speech_pending = False
        self.speech_completes_step = False
        self.desired_vision = None
        self.link.command("stop")
        self.last_sent = "stop"

    def advance(self):
        self.store.advance()
        self.dance_seconds = 0
        self.step_overdue_recorded = False
        self.invalidate()

    def context(self):
        now = time.monotonic()
        sensors = self.link.snapshot()
        state = self.store.context()
        vision = dict(self.observation)
        vision["age_s"] = round(now - vision.get("captured_at", 0), 2)
        heading = sensors.get("heading_deg")
        observed_heading = vision.get("heading_deg")
        heading_change = None if heading is None or observed_heading is None else heading - observed_heading
        vision["heading_change_since_capture_deg"] = heading_change
        vision["fresh"] = (bool(vision.get("text")) and vision["age_s"] <= VISION_MAX_AGE
                           and heading_change is not None and abs(heading_change) <= 25)
        if heading is not None and self.last_heading is not None:
            sensors["heading_change_since_decision_deg"] = round(heading - self.last_heading, 1)
        rate = sensors.get("yaw_rate_dps", 0)
        sensors["rotation"] = "left" if rate > 2 else "right" if rate < -2 else "approximately_stationary"
        state.update(sensors=sensors, allowed_movements=allowed_movements(sensors), vision=vision,
                     audio_state=self.audio.status(), tools=self.tools.status(),
                     speech_pending=self.speech_pending, dance_motion_seconds=round(self.dance_seconds, 2))
        return state

    def on_user(self, text):
        text = text.strip()[:1000]
        if not text:
            return
        self.invalidate()
        self.store.utterance("user", text)
        if text.lower().strip(' .!?') in {"stop", "cancel", "stop moving", "cancel task"}:
            self.store.data.update(status="paused", pending_question=None, last_outcome="user_stopped")
            self.store.changed()
            self.user_pending = None
            return
        if text.lower().strip(' .!?') in {"resume", "continue"} and self.store.step:
            self.store.data.update(status="active", pending_question=None)
            self.store.changed()
            return
        if self.store.data["status"] != "planning":
            self.prior_status = self.store.data["status"]
        self.store.data["status"] = "planning"
        self.store.changed()
        self.user_pending = text

    def request_vision(self, tool, state, priority=2):
        if not self.camera:
            return False
        frame, captured_at, heading = self.camera.latest()
        if frame is None or time.monotonic() - captured_at > 0.25:
            return False
        submitted = self.tools.submit("vision", tool, state, self.epoch, frame, captured_at,
                                      heading, priority=priority)
        if submitted:
            self.next_vision_at = time.monotonic() + 1 / self.args.vision_hz
        return submitted

    def request_speech(self, tool, state, ask=False, completes_step=False):
        if (self.speech_pending or state["awaiting_user_answer"] or not self.audio.enabled
            or self.audio.status() in {"disabled", "listening", "transcribing", "talking"} or time.monotonic() - self.last_speech_at < 3):
            return
        # Keep the same typed tool surface. The ask flag wins when the independent
        # speech choice disagrees with the need for a question.
        tool = "ask_person_about_situation" if ask else tool
        if self.tools.submit("speech", tool, state, self.epoch, priority=0):
            self.speech_pending = True
            self.speech_completes_step = completes_step
            self.speech_requested_at = time.monotonic()

    def handle_tools(self):
        for result in drain(self.tools.results):
            if result["kind"] == "fatal":
                raise RuntimeError("LFM failed to initialize: " + result["error"])
            if result["revision"] != self.epoch:
                continue
            kind = result["kind"]
            if "error" in result:
                self.store.outcome(kind, "tool_failed: " + result["error"])
                if kind == "plan":
                    self.store.data["status"] = "paused"
                    self.speech_completes_step = False
                    self.speech_pending = self.audio.speak(
                        "I heard you, but couldn't work out your request. Could you say it again?",
                        True, self.epoch)
                    self.speech_requested_at = time.monotonic()
                if kind == "speech":
                    self.speech_pending = False
                    self.last_speech_at = time.monotonic()
                continue
            value = result["value"]
            if kind == "vision":
                self.observation = {"text": value, "captured_at": result["captured_at"],
                                    "heading_deg": result["heading_deg"], "tool": result["tool"]}
            elif kind == "plan":
                intent = value["intent"]
                if intent == "new_goal":
                    self.store.install(self.store.data["dialogue"][-1]["text"], value["steps"])
                    self.invalidate()
                elif intent == "cancel":
                    self.store.data.update(status="paused", pending_question=None)
                    self.store.changed()
                elif intent == "clarification":
                    self.store.data.update(status="active" if self.store.step else "idle", pending_question=None)
                    self.store.data["last_outcome"] = "user_clarification_received"
                    if self.store.step and self.store.step["kind"] == "listen":
                        self.advance()
                    self.store.changed()
                else:
                    self.store.data["status"] = self.prior_status
                    self.request_speech("answer_user", self.context())
            elif kind == "speech":
                ask = result["tool"] == "ask_person_about_situation"
                if not self.audio.speak(value, ask, self.epoch):
                    self.speech_pending = False
                    self.store.outcome("speech", "audio_unavailable")

    def handle_audio(self):
        for event in drain(self.audio.events):
            kind = event["kind"]
            if kind == "user":
                self.on_user(event["text"])
            elif kind == "listening":
                self.link.command("stop")
                self.last_sent = "stop"
                self.last_decision_at = 0
            elif kind in {"spoken", "speech_failed"}:
                if event["revision"] != self.epoch:
                    continue
                self.speech_pending = False
                self.last_speech_at = time.monotonic()
                if kind == "speech_failed":
                    self.store.outcome("speech", "playback_failed")
                    continue
                self.store.utterance("assistant", event["text"])
                if event["ask"]:
                    self.store.data["pending_question"] = {"text": event["text"],
                        "step_index": self.store.data["step_index"]}
                if self.speech_completes_step and self.store.data["status"] == "active" and self.store.step and self.store.step["kind"] == "talk":
                    self.advance()
            elif kind == "listen_timeout":
                self.store.data["pending_question"] = None
                self.store.outcome("listen", "no_answer")
                self.store.changed()
            elif kind == "audio_error":
                self.speech_pending = False
                self.store.outcome("audio", event["error"])

    def handle_decision(self, now):
        if self.future is None or not self.future.done():
            return
        future, self.future = self.future, None
        try:
            response = future.result()
        except Exception as exc:
            self.link.command("stop")
            self.last_sent = "stop"
            self.api_failures += 1
            self.api_retry_at = now + min(5, 0.25 * 2 ** min(self.api_failures, 5))
            self.store.outcome("decision", "api_failed: " + type(exc).__name__)
            logging.warning("Jev request failed (%s); motors stopped", type(exc).__name__)
            return
        if self.request_epoch != self.epoch or now - self.request_at > DECISION_MAX_AGE:
            self.link.command("stop")
            self.last_sent = "stop"
            return
        self.api_failures = 0
        self.last_decision_at = now
        state = self.context()  # Fresh sensors and tool state, not request-time state.
        answers = response["answers"]
        action = choose_movement(answers, state, now - self.request_at)
        step = self.store.step
        if step and step["kind"] != "navigate" and step["kind"] != "dance":
            action = "stop"
        self.last_sent = self.link.command(action)
        if action != self.last_sent:
            self.store.outcome(action, "host_clearance_gate")
        if (answers["should_remember"]["noul"] >= YES_THRESHOLD
            and state["vision"].get("fresh")):
            self.store.remember(state["vision"])
        if (answers["need_fresh_vision"]["noul"] >= YES_THRESHOLD
            and answers["lfm_vision_tool"]["confidence"] >= DECISION_CONFIDENCE):
            tool = answers["lfm_vision_tool"]["choice"]
            pending = self.tools.status().get("vision", {})
            if pending.get("tool") != tool:
                self.desired_vision = tool
        if state["status"] == "active":
            ask = answers["should_ask_person"]["noul"] >= YES_THRESHOLD
            speech = answers["lfm_speech_tool"]
            if ask or (speech["choice"] != "none" and speech["confidence"] >= DECISION_CONFIDENCE):
                self.request_speech(speech["choice"], state, ask, completes_step=bool(step and step["kind"] == "talk"))
            # Speech and listening completion come from actual audio events.
            complete = answers["goal_complete"]["noul"] >= 0.9
            if (step and complete and ((step["kind"] == "navigate" and state["vision"].get("fresh"))
                                      or (step["kind"] == "dance" and self.dance_seconds >= 4))):
                self.advance()
        self.audit.info(json.dumps({"time": time.time(), "request": self.request_state,
                                    "response": response, "dispatched_movement": self.last_sent,
                                    "latency_s": now - self.request_at}))
        logging.info("decision model=%s movement=%s step=%s", response["model"], self.last_sent,
                     self.store.data["step_index"])
        self.last_heading = state["sensors"].get("heading_deg")

    def tick(self):
        now = time.monotonic()
        self.handle_audio()
        self.handle_tools()
        state = self.context()
        self.store.observe_motion(state["sensors"])
        if (self.store.step and self.store.step["kind"] == "dance" and
            state["sensors"].get("motion") in {"left", "right"} and state["sensors"].get("age_s", 1) < 0.25):
            self.dance_seconds += min(now - self.previous_tick, 0.1)
        self.previous_tick = now
        # Independent of API completion: stop on stale vision/sensors/audio changes.
        if (self.last_sent != "stop" and (now - self.last_decision_at > 0.5
            or self.last_sent not in state["allowed_movements"] or not state["vision"].get("fresh")
            or state["audio_state"] in {"listening", "transcribing"}
            or state["status"] != "active" or state["awaiting_user_answer"])):
            self.link.command("stop")
            self.last_sent = "stop"
        if self.user_pending and "plan" not in self.tools.status():
            if self.tools.submit("plan", "plan", state, self.epoch, priority=0):
                self.user_pending = None
        if state["status"] == "active":
            if self.desired_vision:
                if self.request_vision(self.desired_vision, state, priority=1):
                    self.desired_vision = None
            elif now >= self.next_vision_at:
                self.request_vision("general_scene", state)
            if state["step_elapsed_s"] > 120 and not self.step_overdue_recorded:
                self.store.outcome("current_step", "No confirmed completion within 120 seconds; reconsider approach or ask person")
                self.step_overdue_recorded = True
            if (self.store.step and self.store.step["kind"] == "listen" and
                not state["awaiting_user_answer"] and not self.speech_pending and
                state["audio_state"] not in {"listening", "transcribing", "talking", "disabled"} and
                now - self.last_speech_at >= 3):
                if self.audio.listen(self.epoch):
                    self.store.data["pending_question"] = {"text": self.store.step["instruction"],
                                                            "step_index": self.store.data["step_index"]}
                    self.store.changed()
        self.handle_decision(now)
        if (self.future is None and now >= max(self.next_decision_at, self.api_retry_at)
            and self.store.data["status"] == "active" and "plan" not in self.tools.status()):
            self.request_state = self.context()
            self.request_at, self.request_epoch = now, self.epoch
            self.future = self.executor.submit(self.client.evaluate, self.request_state)
            self.next_decision_at = now + 1 / self.args.decision_hz
        if self.speech_pending and now - self.speech_requested_at > 30:
            self.invalidate()
            self.store.outcome("speech", "deadline_exceeded")
        if now - self.last_save_at >= 1:
            self.store.save()
            # Headless diagnostics: no camera images or credentials in this snapshot.
            status_path = self.args.state.parent / 'status.json'
            status_tmp = status_path.with_suffix('.tmp')
            status_tmp.write_text(json.dumps({"updated_at": time.time(), **self.context()}, indent=2))
            status_tmp.replace(status_path)
            self.last_save_at = now

    def run(self):
        started = time.monotonic()
        try:
            while not self.closed:
                self.tick()
                if self.args.duration and time.monotonic() - started >= self.args.duration:
                    break
                time.sleep(0.02)
        finally:
            self.close()

    def close(self):
        self.closed = True
        try:
            self.link.close()
        finally:
            self.store.save()
            self.tools.close()
            self.audio.close()
            if self.camera:
                self.camera.close()
            self.executor.shutdown(wait=True, cancel_futures=True)
            self.client.close()
            self.audit.removeHandler(self.audit_handler)
            self.audit_handler.close()


def main():
    from dotenv import load_dotenv
    load_dotenv(ROOT / '.env')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--live', action='store_true', help='Enable serial motor commands; requires protocol-v2 firmware')
    parser.add_argument('--port', default='/dev/tty.usbserial-0001')
    parser.add_argument('--goal')
    parser.add_argument('--plan', help='Validated JSON plan file, bypassing generative planning')
    parser.add_argument('--resume', action='store_true', help='Explicitly resume persisted subgoals')
    parser.add_argument('--state', type=Path, default=ROOT / 'runtime' / 'mission.json')
    parser.add_argument('--camera', type=int, default=0)
    parser.add_argument('--no-camera', action='store_true')
    parser.add_argument('--no-audio', action='store_true')
    parser.add_argument('--vision-hz', type=float, default=1)
    parser.add_argument('--decision-hz', type=float, default=4)
    parser.add_argument('--duration', type=float, default=0, help='Exit after this many seconds; zero runs until interrupted')
    args = parser.parse_args()
    if not 0.1 <= args.vision_hz <= 2 or not 0.1 <= args.decision_hz <= 10 or args.duration < 0:
        parser.error('Use vision 0.1–2 Hz, decisions 0.1–10 Hz, and a nonnegative duration')
    logging.basicConfig(level=logging.INFO, format='%(asctime)s %(message)s')
    logging.info('Mode: %s', 'LIVE' if args.live else 'DRY RUN (no serial connection)')
    controller = Controller(args)
    def stop(*_):
        controller.closed = True
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    controller.run()


if __name__ == '__main__':
    main()
