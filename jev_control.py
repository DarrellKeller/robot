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
from jev_client import JevClient, YES_THRESHOLD, GOAL_APPROVAL_THRESHOLD, COMPLETION_THRESHOLD
from lfm_tools import Camera, LFMTools
from mission_store import MissionStore, validate_plan
from robot_link import RobotLink, allowed_movements, SENSOR_NAMES
from robot_schemas import Transcript, GoalDraft, SpeechCandidates
from runtime_log import start_trace, record, close_trace

ROOT = Path(__file__).resolve().parent
DECISION_MAX_AGE = 0.45
VISION_MAX_AGE = 2.0


def drain(q):
    while True:
        try:
            yield q.get_nowait()
        except queue.Empty:
            return


def choose_movement(answers, state, result_age):
    answer = answers["movement"]
    if (result_age > DECISION_MAX_AGE
        or state["status"] != "active" or state["awaiting_user_answer"]
        or state["audio_state"] in {"listening", "transcribing"}
        or not state["vision"].get("fresh")):
        return "stop"
    action = answer["choice"]
    return action if action in state["allowed_movements"] else "stop"


class Controller:
    def __init__(self, args):
        self.args = args
        self.trace_handler, self.trace_path = start_trace(args.state.parent)
        record("session_start", configuration=vars(args))
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
        self.pending_transcript = None
        self.goal_request = None
        self.goal_proposal = None
        self.speech_candidates = []
        self.speech_purpose = None
        self.speech_request = None
        self.activity = "wait"
        self.dance_recorded = False
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
            if self.store.step["kind"] == "talk":
                self.speech_request = self.store.step["instruction"]
        elif args.goal:
            self.on_user(args.goal)

    def invalidate(self):
        self.epoch += 1
        self.audio.invalidate(self.epoch)
        self.last_decision_at = 0
        self.speech_pending = False
        self.speech_completes_step = False
        self.desired_vision = None
        self.speech_candidates = []
        self.speech_purpose = None
        self.speech_request = None
        self.goal_request = None
        self.goal_proposal = None
        self.pending_transcript = None
        self.activity = "wait"
        self.link.command("stop")
        self.last_sent = "stop"

    def advance(self):
        self.store.advance()
        self.dance_seconds = 0
        self.dance_recorded = False
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
                     hardware_context={
                         "tof_reliability": "ToF readings may be unreliable. Mux sensors reporting out_of_range are not necessarily disconnected; unknown range is not clear space. Compare valid ranges with vision.",
                         "unknown_directions": [name for name, value in zip(SENSOR_NAMES, sensors.get("tof_mm", [None] * 5)) if value is None],
                         "heading_reliability": "Gyro heading is relative and drifts; short-term changes are more useful than absolute heading. Translation is not measured."
                     },
                     audio_state=self.audio.status(), tools=self.tools.status(),
                     speech_pending=self.speech_pending, dance_motion_seconds=round(self.dance_seconds, 2),
                     dance_completed=self.dance_seconds >= 4, activity=self.activity,
                     pending_transcript=self.pending_transcript, goal_request=self.goal_request,
                     goal_proposal=self.goal_proposal, speech_candidates=self.speech_candidates,
                     speech_purpose=self.speech_purpose, speech_request=self.speech_request)
        return state

    def on_user(self, text, quality=None):
        text = text.strip()[:1000]
        if not text:
            return
        prior = self.prior_status if self.store.data["status"] in {"screening", "drafting"} else self.store.data["status"]
        self.invalidate()
        self.prior_status = prior
        # Local emergency stop is never delayed by model approval.
        if text.lower().strip(' .!?') in {"stop", "cancel", "stop moving", "cancel task"}:
            self.store.utterance("user", text)
            self.store.data.update(status="paused", pending_question=None, last_outcome="user_stopped")
            self.store.changed()
            return
        self.pending_transcript = Transcript(text=text, quality=quality or {}).model_dump()
        record("transcript_pending", transcript=self.pending_transcript, epoch=self.epoch)
        self.store.data["status"] = "screening"
        self.store.changed()
        logging.info("TRANSCRIPT awaiting Jev: %s", text)

    def route_user(self, route):
        text = self.pending_transcript["text"]
        self.pending_transcript = None
        self.store.data["status"] = self.prior_status
        logging.info("Jev transcript route: %s", route)
        record("transcript_routed", text=text, route=route, epoch=self.epoch)
        if route == "ignore":
            self.store.outcome("transcript", "rejected_by_jev")
            return
        if route == "clarify":
            # Do not leak questionable recognized words to LFM.
            self.store.data.update(status="paused", pending_question=None)
            self.speech_request = "Ask the user to repeat; the last transcription was unclear."
            self.store.changed()
            return
        self.store.utterance("user", text)
        if route == "cancel":
            self.store.data.update(status="paused", pending_question=None)
        elif route == "resume":
            self.store.data.update(status="active" if self.store.step else "idle", pending_question=None)
        elif route == "goal":
            self.goal_request = text
            self.store.data.update(status="drafting", pending_question=None)
        elif route == "answer":
            self.store.data.update(status="active" if self.store.step else "idle", pending_question=None)
            self.store.goal_event("user_answer", text)
            if self.store.step and self.store.step["kind"] == "listen":
                self.advance()
        else:
            self.store.data["pending_question"] = None
            self.speech_request = text
        self.store.changed()

    def request_vision(self, tool, state, priority=2):
        if not self.camera:
            return False
        frame, captured_at, heading = self.camera.latest()
        if frame is None or time.monotonic() - captured_at > 0.25:
            return False
        visual_state = dict(state)
        step = state.get("current_step") or {}
        if state.get("activity") != "navigate" and step.get("kind") != "navigate":
            visual_state["goal"] = ""
        submitted = self.tools.submit("vision", tool, visual_state, self.epoch, frame, captured_at,
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
        if (state.get("current_step") or {}).get("kind") == "talk" and not ask:
            tool = "status_update"
        if self.tools.submit("speech", tool, state, self.epoch, priority=0):
            self.speech_pending = True
            self.speech_completes_step = completes_step
            self.speech_requested_at = time.monotonic()

    def handle_tools(self):
        for result in drain(self.tools.results):
            record("tool_result", result=result, current_epoch=self.epoch)
            if result["kind"] == "fatal":
                raise RuntimeError("LFM failed to initialize: " + result["error"])
            if result["revision"] != self.epoch:
                continue
            kind = result["kind"]
            if "error" in result:
                self.store.outcome(kind, "tool_failed: " + result["error"])
                logging.warning("LFM %s failed: %s", kind, result["error"])
                if kind == "goal":
                    # A failed rewrite must not strand an already accepted request.
                    # Jev still reviews the verbatim request before installation.
                    self.goal_proposal = self.goal_request
                if kind == "speech":
                    self.speech_pending = False
                    self.last_speech_at = time.monotonic()
                continue
            value = result["value"]
            if kind == "vision":
                self.observation = {"text": value, "captured_at": result["captured_at"],
                                    "heading_deg": result["heading_deg"], "tool": "describe_scene"}
                logging.info("SCENE: %s", value)
            elif kind == "goal":
                self.goal_proposal = GoalDraft(goal=value).goal
                logging.info("GOAL draft awaiting Jev: %s", value)
            elif kind == "speech":
                self.speech_candidates = SpeechCandidates(candidates=value).candidates
                self.speech_purpose = result["tool"]
                logging.info("SPEECH candidates awaiting Jev: %s", json.dumps(value))

    def handle_audio(self):
        for event in drain(self.audio.events):
            record("audio_event", audio_event=event, current_epoch=self.epoch)
            kind = event["kind"]
            if kind == "user":
                self.on_user(event["text"], event.get("quality"))
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
                self.store.goal_event("spoken", event["text"])
                self.speech_request = None
                logging.info("SPOKEN: %s", event["text"])
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
            record("decision_error", error=type(exc).__name__)
            self.link.command("stop")
            self.last_sent = "stop"
            self.api_failures += 1
            self.api_retry_at = now + min(5, 0.25 * 2 ** min(self.api_failures, 5))
            self.store.outcome("decision", "api_failed: " + type(exc).__name__)
            logging.warning("Jev request failed (%s); motors stopped", type(exc).__name__)
            return
        if self.request_epoch != self.epoch or now - self.request_at > DECISION_MAX_AGE:
            record("decision_discarded", request_epoch=self.request_epoch, current_epoch=self.epoch,
                   latency_s=now - self.request_at, response=response)
            self.link.command("stop")
            self.last_sent = "stop"
            return
        self.api_failures = 0
        self.last_decision_at = now
        state = self.context()  # Fresh sensors and tool state, not request-time state.
        answers = response["answers"]
        record("jev_decision", request=self.request_state, response=response,
               latency_s=now - self.request_at, epoch=self.epoch)
        # Decisions refer to the exact pending input/candidates in request_state.
        if self.pending_transcript:
            if self.request_state.get("pending_transcript") == self.pending_transcript:
                route = answers["user_route"]
                self.route_user(route["choice"])
            return
        if self.goal_proposal:
            if self.request_state.get("goal_proposal") == self.goal_proposal:
                if answers["approve_goal"]["noul"] >= GOAL_APPROVAL_THRESHOLD:
                    goal, original = self.goal_proposal, self.goal_request
                    steps = [{"kind": "goal", "instruction": goal,
                        "completion": "All requested actions completed in order, with actual evidence."}]
                    self.store.install(goal, steps)
                    self.store.data["goal_user_request"] = original
                    self.dance_seconds, self.dance_recorded = 0, False
                    self.invalidate()
                    if self.audio.enabled:
                        self.speech_request = "Briefly acknowledge the accepted task in Mauricio's cheeky voice. Say what you will do next without claiming completion."
                        # The approved goal already authorizes this reversible
                        # draft. Jev still selects and approves actual playback.
                        self.request_speech("answer_user", self.context())
                    logging.info("GOAL approved by Jev: %s", goal)
                    record("goal_approved", goal=goal, original_request=original)
                else:
                    if self.goal_proposal != self.goal_request:
                        self.goal_proposal = self.goal_request
                        self.store.outcome("goal", "rewrite_rejected_reviewing_original_request")
                    else:
                        self.goal_proposal = self.goal_request = None
                        self.store.data["status"] = "paused"
                        self.speech_request = "Ask for a clearer achievable task; the goal was rejected."
                        self.store.outcome("goal", "rejected_by_jev")
            return
        activity = answers["activity"]
        self.activity = activity["choice"]
        step = self.store.step
        action = choose_movement(answers, state, now - self.request_at)
        if not step or step["kind"] not in {"navigate", "dance", "goal"} or self.activity not in {"navigate", "dance"}:
            action = "stop"
        if step and step["kind"] == "dance" and action in {"forward", "backward"}:
            action = "stop"
        self.last_sent = self.link.command(action)
        record("motion_dispatch", requested=action, sent=self.last_sent, sensors=state["sensors"])
        if action != self.last_sent:
            self.store.outcome(action, "host_clearance_gate")
        if answers["should_remember"]["noul"] >= YES_THRESHOLD and state["vision"].get("fresh"):
            self.store.remember(state["vision"])
        if answers["need_fresh_vision"]["noul"] >= YES_THRESHOLD and "vision" not in self.tools.status():
            self.desired_vision = "describe_scene"
        if self.speech_candidates and self.request_state.get("speech_candidates") == self.speech_candidates:
            selection = answers["speech_choice"]
            if selection["choice"] != "wait":
                choice = selection["choice"]
                if (choice == "reject" or int(choice) > len(self.speech_candidates)
                    or answers[f"speech_{choice}_ok"]["noul"] < YES_THRESHOLD):
                    self.speech_candidates = []
                    self.speech_pending = False
                    self.last_speech_at = now
                    self.store.outcome("speech", "all_candidates_rejected_by_jev")
                elif state["audio_state"] not in {"talking", "listening", "transcribing", "disabled"}:
                    selected = self.speech_candidates[int(choice) - 1]
                    ask = self.speech_purpose == "ask_person_about_situation"
                    if self.audio.speak(selected, ask, self.epoch):
                        logging.info("Jev approved speech candidate %s: %s", choice, selected)
                        record("speech_approved", choice=choice, text=selected, epoch=self.epoch)
                        self.speech_candidates = []
        elif not self.goal_request:
            speech = answers["lfm_speech_tool"]
            if speech["choice"] != "none":
                self.request_speech(speech["choice"], state,
                    completes_step=bool(step and step["kind"] == "talk"))
        if state["status"] == "active":
            if (self.activity == "listen" and not state["awaiting_user_answer"] and not self.speech_pending
                and state["audio_state"] not in {"talking", "listening", "transcribing", "disabled"}
                and now - self.last_speech_at >= 3):
                if self.audio.listen(self.epoch):
                    self.store.data["pending_question"] = {"text": step["instruction"],
                                                          "step_index": self.store.data["step_index"]}
                    self.store.changed()
            complete = answers["goal_complete"]["noul"] >= COMPLETION_THRESHOLD
            if (step and complete and not self.speech_pending and
                ((step["kind"] in {"navigate", "goal"} and state["vision"].get("fresh"))
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
        if (self.store.step and self.activity == "dance" and
            state["sensors"].get("motion") in {"left", "right"} and state["sensors"].get("age_s", 1) < 0.25):
            self.dance_seconds += min(now - self.previous_tick, 0.1)
        if self.dance_seconds >= 4 and not self.dance_recorded:
            self.store.goal_event("dance", "At least four seconds of measured pivot motion completed")
            self.dance_recorded = True
        self.previous_tick = now
        # Independent of API completion: stop on stale vision/sensors/audio changes.
        if (self.last_sent != "stop" and (now - self.last_decision_at > 0.5
            or self.last_sent not in state["allowed_movements"] or not state["vision"].get("fresh")
            or state["audio_state"] in {"listening", "transcribing"}
            or state["status"] != "active" or state["awaiting_user_answer"])):
            self.link.command("stop")
            self.last_sent = "stop"
        if self.goal_request and not self.goal_proposal and "goal" not in self.tools.status():
            self.tools.submit("goal", "draft_goal", state, self.epoch, priority=0)
        # Keep observations current during conversation as well as navigation.
        if self.desired_vision:
            if self.request_vision("describe_scene", state, priority=1):
                self.desired_vision = None
        elif now >= self.next_vision_at:
            self.request_vision("describe_scene", state)
        if state["status"] == "active" and state["step_elapsed_s"] > 120 and not self.step_overdue_recorded:
            self.store.outcome("current_step", "No confirmed completion within 120 seconds; reconsider approach or ask person")
            self.step_overdue_recorded = True
        self.handle_decision(now)
        if (self.future is None and now >= max(self.next_decision_at, self.api_retry_at)
            and "goal" not in self.tools.status()):
            self.request_state = self.context()
            self.request_at, self.request_epoch = now, self.epoch
            self.future = self.executor.submit(self.client.evaluate, self.request_state)
            busy = self.store.data["status"] == "active" or self.pending_transcript or self.speech_pending or self.goal_proposal
            self.next_decision_at = now + (1 / self.args.decision_hz if busy else 1)
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
            record("state_snapshot", state=self.context())
            self.last_save_at = now

    def run(self):
        started = time.monotonic()
        try:
            while not self.closed:
                self.tick()
                if self.args.duration and time.monotonic() - started >= self.args.duration:
                    break
                time.sleep(0.02)
        except Exception as exc:
            record("controller_error", error=type(exc).__name__, message=str(exc))
            raise
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
            close_trace(self.trace_handler)


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
