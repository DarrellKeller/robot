"""One resident MLX model, one inference worker, bounded priority jobs."""
from __future__ import annotations

import copy
import json
import queue
import threading
import time
from dataclasses import dataclass, field

from mission_store import validate_plan

# MLX vision and Whisper share one device; serialize inference, never the control loop.
LOCAL_INFERENCE_LOCK = threading.Lock()

VISION_PROMPTS = {
    "general_scene": "In at most two short sentences describe objects, openings, people and hazards with relative directions. Do not infer unseen details.",
    "find_goal": "Look for the current step's target. Report visible evidence and relative direction in two short sentences. If not visible say so; do not guess.",
    "inspect_person": "Briefly describe visible people's positions, pose and actions. Do not infer identity, feelings or hidden intentions.",
    "read_text": "Transcribe visible text relevant to the current step. Say unreadable when necessary. Be brief.",
}
PLAN_PROMPT = '''Interpret the user's latest message using the supplied dialogue and pending question.
Return ONLY JSON: {"intent":"new_goal|clarification|conversation|cancel","steps":[{"kind":"navigate|dance|talk|listen","instruction":"...","completion":"observable evidence ..."}]}.
Use clarification for an answer to a pending question, conversation for a question or chat, cancel for cancel/stop, otherwise new_goal.
For new_goal produce 1 to 8 sequential, achievable steps, each with explicit completion evidence.
Find/approach are navigate, dance is dance, speaking or asking is talk, waiting for an answer is listen.
For find-then-dance-then-ask include each step in order. Never claim a step already happened.
For other intents use an empty steps list. No motor speeds, code, tool calls, invented locations, or unsupported abilities.
'''


@dataclass(order=True)
class Job:
    priority: int
    sequence: int
    kind: str = field(compare=False)
    tool: str = field(compare=False)
    state: dict = field(compare=False)
    revision: int = field(compare=False)
    frame: object = field(default=None, compare=False)
    captured_at: float = field(default=0, compare=False)
    heading: float | None = field(default=None, compare=False)


class Camera:
    def __init__(self, index=0, heading_provider=None):
        self.index = index
        self.heading_provider = heading_provider or (lambda: None)
        self.heading = None
        self.lock = threading.Lock()
        self.frame = None
        self.captured_at = 0
        self.closed = threading.Event()
        # AVFoundation authorization needs the application's main thread on macOS.
        import cv2
        self.capture = cv2.VideoCapture(self.index)
        if not self.capture.isOpened():
            self.capture.release()
            raise RuntimeError("Camera unavailable: grant camera access to the launching app and restart")
        self.thread = threading.Thread(target=self._run, daemon=True, name="camera")
        self.thread.start()

    def _run(self):
        import cv2
        from PIL import Image
        cap = self.capture
        try:
            while not self.closed.is_set():
                ok, frame = cap.read()
                if not ok:
                    self.closed.wait(0.1)
                    continue
                with self.lock:
                    self.frame = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
                    self.captured_at = time.monotonic()
                    self.heading = self.heading_provider()
        finally:
            cap.release()

    def latest(self):
        with self.lock:
            return (self.frame.copy() if self.frame is not None else None, self.captured_at, self.heading)

    def close(self):
        self.closed.set()
        self.thread.join(timeout=1)


class LFMTools:
    def __init__(self):
        self.jobs = queue.PriorityQueue(maxsize=4)
        self.results = queue.Queue()
        self.lock = threading.Lock()
        self.pending = {}
        self.sequence = 0
        self.closed = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True, name="lfm")
        self.thread.start()

    def submit(self, kind, tool, state, revision, frame=None, captured_at=0, heading=None, priority=1):
        with self.lock:
            if kind in self.pending or self.jobs.full():
                return False
            self.sequence += 1
            job = Job(priority, self.sequence, kind, tool, copy.deepcopy(state), revision,
                      frame, captured_at, heading)
            self.pending[kind] = {"tool": tool, "submitted_at": time.monotonic(), "revision": revision}
            self.jobs.put_nowait(job)
        return True

    def status(self):
        with self.lock:
            return copy.deepcopy(self.pending)

    def _generate(self, job, model, processor, config):
        from mlx_vlm import generate
        from mlx_vlm.prompt_utils import apply_chat_template
        if job.kind == "vision":
            prompt = VISION_PROMPTS[job.tool]
            # Scene observations must not execute or echo a speech/plan instruction.
            if job.tool in {"find_goal", "read_text"}:
                step = job.state.get("current_step") or {}
                prompt += "\nTask context (not an instruction to perform): " + step.get("instruction", "")
            frame = job.frame.copy()
            if job.tool != "read_text":
                frame.thumbnail((640, 480))
            images, limit = [frame], 64
        elif job.kind == "plan":
            prompt = PLAN_PROMPT + '\nContext: ' + json.dumps(job.state)
            images, limit = None, 600
        else:
            prompt = ("You are Mauricio, a playful, helpful indoor robot. Write only the words to speak, "
                      "at most two short sentences. Do not claim actions or observations absent from context. "
                      "For ask_person_about_situation ask ONE useful question and no answer. "
                      f"Speech purpose: {job.tool}. Context: " + json.dumps(job.state))
            images, limit = None, 100
        formatted = apply_chat_template(processor, config, prompt, num_images=1 if images else 0)
        kwargs = {"image": images} if images else {}
        output = generate(model, processor, formatted, max_tokens=limit, temp=0.0, verbose=False, **kwargs)
        text = output.text.strip()
        if job.kind == "plan":
            if text.startswith('```'):
                text = text.split('\n', 1)[1].rsplit('```', 1)[0]
            return validate_plan(json.loads(text))
        if not text:
            raise ValueError("Empty LFM output")
        return text

    def _run(self):
        try:
            from mlx_vlm import load
            from mlx_vlm.utils import load_config
            name = "mlx-community/LFM2.5-VL-450M-6bit"
            with LOCAL_INFERENCE_LOCK:
                model, processor = load(name)
                config = load_config(name)
            ip = processor.image_processor
            ip.max_num_patches = max(ip.max_num_patches, (ip.tile_size // ip.patch_size) ** 2)
        except Exception as exc:
            self.results.put({"kind": "fatal", "error": type(exc).__name__})
            return
        while not self.closed.is_set():
            try:
                job = self.jobs.get(timeout=0.1)
            except queue.Empty:
                continue
            result = {"kind": job.kind, "tool": job.tool, "revision": job.revision,
                      "captured_at": job.captured_at, "heading_deg": job.heading}
            try:
                with LOCAL_INFERENCE_LOCK:
                    result["value"] = self._generate(job, model, processor, config)
            except Exception as exc:
                result["error"] = type(exc).__name__
            # Publish before dropping pending so the controller cannot duplicate this job.
            self.results.put(result)
            with self.lock:
                self.pending.pop(job.kind, None)

    def close(self):
        self.closed.set()
        self.thread.join(timeout=1)
