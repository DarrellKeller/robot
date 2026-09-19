#!/usr/bin/env python3
"""Test retained-chat memory during short, teacher-forced apartment errands.

The full errand appears only in the opening user message. Later user messages
contain just the environment update and legal next rooms; both the model's
prior assistant reply and every user update remain in the chat context. The
fixed environment path isolates task-memory failures from cascading movement
mistakes.
"""

from __future__ import annotations

import argparse
import gc
import json
import re
import time
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

import mlx.core as mx
from mlx_vlm import generate, load
from mlx_vlm.prompt_utils import apply_chat_template
from mlx_vlm.utils import load_config

LFM_450M_MODEL = "mlx-community/LFM2.5-VL-450M-6bit"
LFM_1_6B_MODEL = "mlx-community/LFM2.5-VL-1.6B-6bit"
MODELS = (("LFM2.5-VL-450M 6-bit", LFM_450M_MODEL), ("LFM2.5-VL-1.6B 6-bit", LFM_1_6B_MODEL))


@dataclass(frozen=True)
class MoveProbe:
    current_room: str
    allowed_rooms: tuple[str, ...]
    expected_room: str
    update: str


@dataclass(frozen=True)
class Episode:
    name: str
    opening: str
    probes: tuple[MoveProbe, ...]
    final_question: str
    final_keywords: tuple[str, ...]
    final_any_keywords: tuple[str, ...] = ()


EPISODES = (
    Episode(
        "coffee delivery",
        "A resident asked: fetch coffee from the kitchen, then deliver it to the office. The robot starts in the entry.",
        (
            MoveProbe("entry", ("hall",), "hall", "This is the first move."),
            MoveProbe("hall", ("entry", "kitchen", "office", "bedroom"), "kitchen", "The low-level safety layer moved the robot to the hall."),
            MoveProbe("kitchen", ("hall", "pantry"), "hall", "The robot reached the kitchen and picked up the coffee."),
            MoveProbe("hall", ("entry", "kitchen", "office", "bedroom"), "office", "The robot is back in the hall, carrying the coffee."),
        ),
        "The robot arrived at the office. In one friendly sentence, tell the resident what was completed.",
        ("coffee", "office"),
    ),
    Episode(
        "charger run",
        "A resident asked: pick up a charger in the office, then bring it to the bedroom. The robot starts in the entry.",
        (
            MoveProbe("entry", ("hall",), "hall", "This is the first move."),
            MoveProbe("hall", ("entry", "kitchen", "office", "bedroom"), "office", "The safety layer moved the robot to the hall."),
            MoveProbe("office", ("hall", "living room"), "hall", "The robot reached the office and picked up the charger."),
            MoveProbe("hall", ("entry", "kitchen", "office", "bedroom"), "bedroom", "The robot is back in the hall, carrying the charger."),
        ),
        "The robot arrived at the bedroom. In one friendly sentence, tell the resident what was completed.",
        ("charger", "bedroom"),
    ),
    Episode(
        "laundry errand",
        "A resident asked: get detergent from the laundry room, then store it in the pantry. The robot starts in the office.",
        (
            MoveProbe("office", ("hall", "living room"), "living room", "This is the first move."),
            MoveProbe("living room", ("office", "bathroom", "laundry"), "laundry", "The safety layer moved the robot to the living room."),
            MoveProbe("laundry", ("living room",), "living room", "The robot reached the laundry room and picked up detergent."),
            MoveProbe("living room", ("office", "bathroom", "laundry"), "office", "The robot is back in the living room, carrying detergent."),
            MoveProbe("office", ("hall", "living room"), "hall", "The robot reached the office, still carrying detergent."),
            MoveProbe("hall", ("entry", "kitchen", "office", "bedroom"), "kitchen", "The robot reached the hall, still carrying detergent."),
            MoveProbe("kitchen", ("hall", "pantry"), "pantry", "The robot reached the kitchen, still carrying detergent."),
        ),
        "The robot arrived at the pantry. In one friendly sentence, tell the resident what was completed.",
        ("detergent", "pantry"),
    ),
    Episode(
        "three-stop patrol",
        "A resident asked: check the pantry, then the bathroom, then finish at the bedroom. The robot starts in the entry.",
        (
            MoveProbe("entry", ("hall",), "hall", "This is the first move."),
            MoveProbe("hall", ("entry", "kitchen", "office", "bedroom"), "kitchen", "The safety layer moved the robot to the hall."),
            MoveProbe("kitchen", ("hall", "pantry"), "pantry", "The robot reached the kitchen."),
            MoveProbe("pantry", ("kitchen",), "kitchen", "The robot completed the pantry check. Continue the original order."),
            MoveProbe("kitchen", ("hall", "pantry"), "hall", "The robot is back in the kitchen."),
            MoveProbe("hall", ("entry", "kitchen", "office", "bedroom"), "office", "The robot is back in the hall."),
            MoveProbe("office", ("hall", "living room"), "living room", "The robot reached the office."),
            MoveProbe("living room", ("office", "bathroom", "laundry"), "bathroom", "The robot reached the living room."),
        ),
        "The robot arrived at the bathroom. In one friendly sentence, state the remaining destination.",
        ("bedroom",),
        ("next", "remaining", "destination"),
    ),
)


@dataclass
class TurnResult:
    episode: str
    turn: int
    current_room: str
    expected_room: str
    response: str
    chosen_room: str | None
    correct: bool
    latency_seconds: float


@dataclass
class FinalResult:
    episode: str
    response: str
    correct: bool
    latency_seconds: float


@dataclass
class ModelResult:
    name: str
    model: str
    move_accuracy_percent: float
    early_accuracy_percent: float
    late_accuracy_percent: float
    final_memory_percent: float
    mean_turn_seconds: float
    task_index: float
    turns: list[TurnResult]
    finals: list[FinalResult]


def configure_lfm_450m_processor(model_id: str, processor: Any) -> None:
    if model_id == LFM_450M_MODEL:
        image_processor = processor.image_processor
        image_processor.max_num_patches = max(
            image_processor.max_num_patches,
            (image_processor.tile_size // image_processor.patch_size) ** 2,
        )


def choice_prompt(probe: MoveProbe) -> str:
    return f"""{probe.update}
Current room: {probe.current_room}.
Allowed next rooms: {", ".join(probe.allowed_rooms)}.
Choose the next room. Reply with exactly one line: MOVE: <one allowed room>."""


def parse_move(response: str, allowed_rooms: tuple[str, ...]) -> str | None:
    match = re.search(r"(?im)^\s*move\s*:\s*([^\n]+)", response)
    candidate = match.group(1) if match else response
    candidate = re.sub(r"[^a-z ]", "", candidate.lower()).strip()
    return candidate if candidate in allowed_rooms else None


def generate_reply(model: Any, processor: Any, config: Any, messages: list[dict[str, str]], max_tokens: int) -> tuple[str, float]:
    prompt = apply_chat_template(processor, config, messages, num_images=0)
    started = time.perf_counter()
    output = generate(model, processor, prompt, max_tokens=max_tokens, temp=0.0, verbose=False)
    return output.text.strip(), time.perf_counter() - started


def run_episode(model: Any, processor: Any, config: Any, episode: Episode, max_tokens: int) -> tuple[list[TurnResult], FinalResult]:
    messages: list[dict[str, str]] = [{
        "role": "user",
        "content": f"""You are the high-level planning and speaking layer for an apartment robot. Keep the resident's original order in memory across this conversation. For each movement request, choose only from the allowed rooms.\n\n{episode.opening}\n\nYou will receive the first movement request next.""",
    }]
    turns = []
    for number, probe in enumerate(episode.probes, start=1):
        messages.append({"role": "user", "content": choice_prompt(probe)})
        response, latency = generate_reply(model, processor, config, messages, max_tokens)
        chosen = parse_move(response, probe.allowed_rooms)
        turns.append(TurnResult(episode.name, number, probe.current_room, probe.expected_room, response, chosen, chosen == probe.expected_room, latency))
        messages.append({"role": "assistant", "content": response})
    messages.append({"role": "user", "content": episode.final_question})
    response, latency = generate_reply(model, processor, config, messages, max_tokens)
    lower_response = response.lower()
    final_correct = all(keyword in lower_response for keyword in episode.final_keywords)
    if episode.final_any_keywords:
        final_correct = final_correct and any(keyword in lower_response for keyword in episode.final_any_keywords)
    final = FinalResult(episode.name, response, final_correct, latency)
    return turns, final


def run_model(name: str, model_id: str, max_tokens: int) -> ModelResult:
    model = processor = config = None
    try:
        model, processor = load(model_id)
        config = load_config(model_id)
        configure_lfm_450m_processor(model_id, processor)
        warmup = apply_chat_template(processor, config, "Reply: ready", num_images=0)
        generate(model, processor, warmup, max_tokens=8, temp=0.0, verbose=False)
        turns: list[TurnResult] = []
        finals: list[FinalResult] = []
        for episode in EPISODES:
            episode_turns, final = run_episode(model, processor, config, episode, max_tokens)
            turns.extend(episode_turns)
            finals.append(final)
        accuracy = 100 * sum(turn.correct for turn in turns) / len(turns)
        early = [turn for turn in turns if turn.turn <= 2]
        late = [turn for turn in turns if turn.turn >= 5]
        early_accuracy = 100 * sum(turn.correct for turn in early) / len(early)
        late_accuracy = 100 * sum(turn.correct for turn in late) / len(late)
        final_memory = 100 * sum(final.correct for final in finals) / len(finals)
        mean_turn_seconds = sum(turn.latency_seconds for turn in turns) / len(turns)
        task_index = 0.60 * accuracy + 0.25 * final_memory + 0.15 * late_accuracy
        return ModelResult(name, model_id, round(accuracy, 2), round(early_accuracy, 2), round(late_accuracy, 2), round(final_memory, 2), round(mean_turn_seconds, 3), round(task_index, 2), turns, finals)
    finally:
        del model, processor, config
        gc.collect()
        mx.clear_cache()


def markdown_report(results: list[ModelResult]) -> str:
    lines = [
        "# LFM retained-context apartment benchmark", "",
        "Each episode retains every user and assistant turn. The errand appears only in the opening message; later prompts disclose the current room and legal moves but not the original order or required next stop.", "",
        "## Summary", "",
        "| Model | Task index / 100 | All route turns | Turns 1–2 | Turns 5+ | Final memory answer | Mean turn (s) |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for result in results:
        lines.append(f"| {result.name} | {result.task_index:.2f} | {result.move_accuracy_percent:.1f}% | {result.early_accuracy_percent:.1f}% | {result.late_accuracy_percent:.1f}% | {result.final_memory_percent:.1f}% | {result.mean_turn_seconds:.3f} |")
    lines.append("\nTask index = 60% all route turns + 25% final memory answer + 15% turns 5+. Teacher-forced location updates prevent one bad move from hiding later memory performance.")
    for result in results:
        lines.extend(["", f"## {result.name}", "", "| Episode | Turn | Current | Expected | Model choice | Correct | Actual response |", "| --- | ---: | --- | --- | --- | --- | --- |"])
        for turn in result.turns:
            response = turn.response.replace("|", "\\|")
            lines.append(f"| {turn.episode} | {turn.turn} | {turn.current_room} | {turn.expected_room} | {turn.chosen_room or '—'} | {'yes' if turn.correct else 'no'} | {response} |")
        lines.extend(["", "### Final memory answers", "", "| Episode | Correct | Actual response |", "| --- | --- | --- |"])
        for final in result.finals:
            response = final.response.replace("|", "\\|")
            lines.append(f"| {final.episode} | {'yes' if final.correct else 'no'} | {response} |")
    lines.extend(["", "## Interpretation", "", "This tests retained chat context, not hidden chain-of-thought. The assistant outputs are intentionally stored as context so the model also has to cope with its own earlier mistakes."])
    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--max-tokens", type=int, default=32)
    parser.add_argument("--output-dir", type=Path, default=Path(__file__).resolve().parent / "results" / "memory")
    args = parser.parse_args()
    output_dir = args.output_dir / datetime.now().strftime("%Y%m%d-%H%M%S")
    output_dir.mkdir(parents=True, exist_ok=False)
    results = []
    for name, model_id in MODELS:
        print(f"Running {name}...")
        results.append(run_model(name, model_id, args.max_tokens))
    report = markdown_report(results)
    (output_dir / "report.md").write_text(report)
    (output_dir / "results.json").write_text(json.dumps([asdict(result) for result in results], indent=2) + "\n")
    print(report)
    print(f"Saved report: {output_dir / 'report.md'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
