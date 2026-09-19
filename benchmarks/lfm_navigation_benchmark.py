#!/usr/bin/env python3
"""Compare LFM models on constrained apartment-navigation and speech decisions.

This benchmark gives each model a currently legal set of room-level moves, like
a safety layer would. It measures shortest-path choices for multi-stop errands
and useful spoken acknowledgements. Saved notes are model-produced audit text,
not hidden reasoning traces.
"""

from __future__ import annotations

import argparse
import gc
import json
import re
import time
from collections import deque
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
MODELS = (
    ("LFM2.5-VL-450M 6-bit", LFM_450M_MODEL),
    ("LFM2.5-VL-1.6B 6-bit", LFM_1_6B_MODEL),
)

APARTMENT = {
    "entry": ("hall",),
    "hall": ("entry", "kitchen", "office", "bedroom"),
    "kitchen": ("hall", "pantry"),
    "pantry": ("kitchen",),
    "office": ("hall", "living room"),
    "living room": ("office", "bathroom", "laundry"),
    "bathroom": ("living room",),
    "laundry": ("living room",),
    "bedroom": ("hall",),
}


@dataclass(frozen=True)
class NavigationCard:
    story: str
    order: str
    current_room: str
    completed_stops: tuple[str, ...]
    next_stop: str


@dataclass(frozen=True)
class SpeechCard:
    story: str
    order: str
    next_stop: str


NAVIGATION_CARDS = (
    NavigationCard("coffee delivery", "Fetch coffee from the kitchen, then deliver it to the office.", "entry", (), "kitchen"),
    NavigationCard("coffee delivery", "Fetch coffee from the kitchen, then deliver it to the office.", "hall", (), "kitchen"),
    NavigationCard("coffee delivery", "Fetch coffee from the kitchen, then deliver it to the office.", "kitchen", ("kitchen",), "office"),
    NavigationCard("coffee delivery", "Fetch coffee from the kitchen, then deliver it to the office.", "hall", ("kitchen",), "office"),
    NavigationCard("charger run", "Pick up a charger in the office, then bring it to the bedroom.", "entry", (), "office"),
    NavigationCard("charger run", "Pick up a charger in the office, then bring it to the bedroom.", "hall", (), "office"),
    NavigationCard("charger run", "Pick up a charger in the office, then bring it to the bedroom.", "office", ("office",), "bedroom"),
    NavigationCard("charger run", "Pick up a charger in the office, then bring it to the bedroom.", "hall", ("office",), "bedroom"),
    NavigationCard("laundry errand", "Get detergent from the laundry room, then store it in the pantry.", "office", (), "laundry"),
    NavigationCard("laundry errand", "Get detergent from the laundry room, then store it in the pantry.", "living room", (), "laundry"),
    NavigationCard("laundry errand", "Get detergent from the laundry room, then store it in the pantry.", "laundry", ("laundry",), "pantry"),
    NavigationCard("laundry errand", "Get detergent from the laundry room, then store it in the pantry.", "living room", ("laundry",), "pantry"),
    NavigationCard("laundry errand", "Get detergent from the laundry room, then store it in the pantry.", "office", ("laundry",), "pantry"),
    NavigationCard("laundry errand", "Get detergent from the laundry room, then store it in the pantry.", "hall", ("laundry",), "pantry"),
    NavigationCard("laundry errand", "Get detergent from the laundry room, then store it in the pantry.", "kitchen", ("laundry",), "pantry"),
    NavigationCard("night patrol", "Check the pantry, then the bathroom, then finish at the bedroom.", "entry", (), "pantry"),
    NavigationCard("night patrol", "Check the pantry, then the bathroom, then finish at the bedroom.", "hall", (), "pantry"),
    NavigationCard("night patrol", "Check the pantry, then the bathroom, then finish at the bedroom.", "kitchen", (), "pantry"),
    NavigationCard("night patrol", "Check the pantry, then the bathroom, then finish at the bedroom.", "pantry", ("pantry",), "bathroom"),
    NavigationCard("night patrol", "Check the pantry, then the bathroom, then finish at the bedroom.", "kitchen", ("pantry",), "bathroom"),
    NavigationCard("night patrol", "Check the pantry, then the bathroom, then finish at the bedroom.", "hall", ("pantry",), "bathroom"),
    NavigationCard("night patrol", "Check the pantry, then the bathroom, then finish at the bedroom.", "office", ("pantry",), "bathroom"),
)
SPEECH_CARDS = (
    SpeechCard("coffee delivery", "Fetch coffee from the kitchen, then deliver it to the office.", "kitchen"),
    SpeechCard("charger run", "Pick up a charger in the office, then bring it to the bedroom.", "office"),
    SpeechCard("laundry errand", "Get detergent from the laundry room, then store it in the pantry.", "laundry"),
    SpeechCard("night patrol", "Check the pantry, then the bathroom, then finish at the bedroom.", "pantry"),
)


@dataclass
class NavigationResult:
    card: NavigationCard
    expected_move: str
    response: str
    parsed_move: str | None
    legal: bool
    shortest_path_choice: bool
    visible_note: str | None
    latency_seconds: float


@dataclass
class SpeechResult:
    card: SpeechCard
    response: str
    mentions_stop: bool
    concise: bool
    latency_seconds: float


@dataclass
class ModelResult:
    name: str
    model: str
    load_seconds: float
    navigation_accuracy_percent: float
    legal_move_percent: float
    speech_compliance_percent: float
    concise_speech_percent: float
    mean_navigation_seconds: float
    mean_speech_seconds: float
    task_index: float
    navigation: list[NavigationResult]
    speech: list[SpeechResult]


def configure_lfm_450m_processor(model_id: str, processor: Any) -> None:
    if model_id == LFM_450M_MODEL:
        image_processor = processor.image_processor
        tile_patch_count = (image_processor.tile_size // image_processor.patch_size) ** 2
        image_processor.max_num_patches = max(image_processor.max_num_patches, tile_patch_count)


def shortest_distance(start: str, target: str) -> int:
    queue = deque([(start, 0)])
    seen = {start}
    while queue:
        room, distance = queue.popleft()
        if room == target:
            return distance
        for neighbor in APARTMENT[room]:
            if neighbor not in seen:
                seen.add(neighbor)
                queue.append((neighbor, distance + 1))
    raise ValueError(f"No route from {start} to {target}")


def expected_move(card: NavigationCard) -> str:
    return min(APARTMENT[card.current_room], key=lambda room: shortest_distance(room, card.next_stop))


def navigation_prompt(card: NavigationCard) -> str:
    completed = ", ".join(card.completed_stops) if card.completed_stops else "none"
    allowed = ", ".join(APARTMENT[card.current_room])
    return f"""You are the slow, high-level language layer for an apartment robot.
Choose the one next room that best continues this errand. A safety layer only
allows the listed moves, so never name another room.

Resident order: {card.order}
Already completed: {completed}.
Robot is currently in: {card.current_room}.
Next required stop: {card.next_stop}.
Allowed next rooms: {allowed}.

Reply on exactly two lines:
MOVE: one room copied exactly from Allowed next rooms
NOTE: one short sentence explaining the choice
"""


def speech_prompt(card: SpeechCard) -> str:
    return f"""You are the speaking layer for an apartment robot.
The resident said: {card.order}
The robot's immediate next stop is {card.next_stop}.
Reply with one friendly sentence of at most 14 words that confirms the immediate next stop.
Do not add facts or claim the errand is already complete."""


def clean_room(value: str) -> str:
    return re.sub(r"[^a-z ]", "", value.lower()).strip()


def parse_navigation(response: str, allowed_rooms: tuple[str, ...]) -> tuple[str | None, str | None]:
    move = note = None
    for line in (line.strip() for line in response.splitlines() if line.strip()):
        if line.lower().startswith("move:"):
            candidate = clean_room(line.split(":", 1)[1])
            if candidate in allowed_rooms:
                move = candidate
        elif line.lower().startswith("note:"):
            note = line.split(":", 1)[1].strip()
    normalized = clean_room(response)
    if move is None and normalized in allowed_rooms:
        move = normalized
    return move, note


def run_navigation_card(model: Any, processor: Any, config: Any, card: NavigationCard, max_tokens: int) -> NavigationResult:
    prompt = apply_chat_template(processor, config, navigation_prompt(card), num_images=0)
    started = time.perf_counter()
    output = generate(model, processor, prompt, max_tokens=max_tokens, temp=0.0, verbose=False)
    latency = time.perf_counter() - started
    move, note = parse_navigation(output.text.strip(), APARTMENT[card.current_room])
    expected = expected_move(card)
    return NavigationResult(card, expected, output.text.strip(), move, move is not None, move == expected, note, latency)


def run_speech_card(model: Any, processor: Any, config: Any, card: SpeechCard, max_tokens: int) -> SpeechResult:
    prompt = apply_chat_template(processor, config, speech_prompt(card), num_images=0)
    started = time.perf_counter()
    output = generate(model, processor, prompt, max_tokens=max_tokens, temp=0.0, verbose=False)
    latency = time.perf_counter() - started
    response = output.text.strip()
    return SpeechResult(card, response, card.next_stop in response.lower(), len(response.split()) <= 14, latency)


def run_model(name: str, model_id: str, max_tokens: int) -> ModelResult:
    model = processor = config = None
    try:
        started = time.perf_counter()
        model, processor = load(model_id)
        config = load_config(model_id)
        configure_lfm_450m_processor(model_id, processor)
        load_seconds = time.perf_counter() - started
        warmup = apply_chat_template(processor, config, "Reply with: ready", num_images=0)
        generate(model, processor, warmup, max_tokens=8, temp=0.0, verbose=False)
        navigation = [run_navigation_card(model, processor, config, card, max_tokens) for card in NAVIGATION_CARDS]
        speech = [run_speech_card(model, processor, config, card, max_tokens) for card in SPEECH_CARDS]
        nav_accuracy = 100 * sum(item.shortest_path_choice for item in navigation) / len(navigation)
        legal_moves = 100 * sum(item.legal for item in navigation) / len(navigation)
        speech_compliance = 100 * sum(item.mentions_stop for item in speech) / len(speech)
        concise_speech = 100 * sum(item.concise for item in speech) / len(speech)
        task_index = 0.70 * nav_accuracy + 0.15 * legal_moves + 0.10 * speech_compliance + 0.05 * concise_speech
        return ModelResult(name, model_id, round(load_seconds, 3), round(nav_accuracy, 2), round(legal_moves, 2), round(speech_compliance, 2), round(concise_speech, 2), round(sum(item.latency_seconds for item in navigation) / len(navigation), 3), round(sum(item.latency_seconds for item in speech) / len(speech), 3), round(task_index, 2), navigation, speech)
    finally:
        del model, processor, config
        gc.collect()
        mx.clear_cache()


def markdown_report(results: list[ModelResult]) -> str:
    lines = [
        "# LFM apartment-navigation benchmark", "",
        "Text-only, constrained room-level decisions. This is a proxy for supervised high-level planning and speech, not proof of driving safety.", "",
        "## Summary", "",
        "| Model | Task index / 100 | Shortest-path decisions | Legal moves | Speech names next stop | Concise speech | Navigation decision (s) | Speech response (s) |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for result in results:
        lines.append(f"| {result.name} | {result.task_index:.2f} | {result.navigation_accuracy_percent:.1f}% | {result.legal_move_percent:.1f}% | {result.speech_compliance_percent:.1f}% | {result.concise_speech_percent:.1f}% | {result.mean_navigation_seconds:.3f} | {result.mean_speech_seconds:.3f} |")
    lines.append("\nTask index = 70% shortest-path choice + 15% legal move + 10% speech naming the immediate stop + 5% concise speech.")
    for result in results:
        lines.extend(["", f"## {result.name}", "", "### Navigation decisions", "", "| Story | Current | Next stop | Allowed | Expected | Model move | Correct | Visible decision note |", "| --- | --- | --- | --- | --- | --- | --- | --- |"])
        for item in result.navigation:
            note = (item.visible_note or "—").replace("|", "\\|")
            lines.append(f"| {item.card.story} | {item.card.current_room} | {item.card.next_stop} | {', '.join(APARTMENT[item.card.current_room])} | {item.expected_move} | {item.parsed_move or '—'} | {'yes' if item.shortest_path_choice else 'no'} | {note} |")
        lines.extend(["", "### Spoken acknowledgements", "", "| Story | Immediate stop | Response | Names stop | ≤14 words |", "| --- | --- | --- | --- | --- |"])
        for item in result.speech:
            response = item.response.replace("|", "\\|")
            lines.append(f"| {item.card.story} | {item.card.next_stop} | {response} | {'yes' if item.mentions_stop else 'no'} | {'yes' if item.concise else 'no'} |")
    lines.extend(["", "## Interpretation", "", "The visible decision notes are generated audit text, not hidden reasoning traces. Evaluate those notes for obvious confusion, but use shortest-path and legal-move rates for numerical comparison."])
    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--max-tokens", type=int, default=48)
    parser.add_argument("--output-dir", type=Path, default=Path(__file__).resolve().parent / "results" / "navigation")
    args = parser.parse_args()
    if args.max_tokens < 8:
        parser.error("--max-tokens must be at least 8.")
    output_dir = args.output_dir / datetime.now().strftime("%Y%m%d-%H%M%S")
    output_dir.mkdir(parents=True, exist_ok=False)
    results = []
    for name, model_id in MODELS:
        print(f"Running {name}...")
        results.append(run_model(name, model_id, args.max_tokens))
    report = markdown_report(results)
    (output_dir / "report.md").write_text(report)
    (output_dir / "results.json").write_text(json.dumps([asdict(item) for item in results], indent=2) + "\n")
    print(report)
    print(f"Saved report: {output_dir / 'report.md'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
