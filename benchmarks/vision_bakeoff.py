#!/usr/bin/env python3
"""Compare the robot's local vision models on one identical image.

The script captures one webcam frame (or accepts --image), then runs the frame
through the two MLX VLMs used by this project. It
writes a Markdown report beside the captured image so qualitative answers and
speed measurements remain tied to the exact frame being judged.
"""

from __future__ import annotations

import argparse
import gc
import json
import shutil
import time
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

import cv2
import mlx.core as mx
from PIL import Image
from mlx_vlm import generate, load
from mlx_vlm.prompt_utils import apply_chat_template
from mlx_vlm.utils import load_config

LFM_450M_MODEL = "mlx-community/LFM2.5-VL-450M-6bit"
LFM_MODEL = "mlx-community/LFM2.5-VL-1.6B-6bit"
DEFAULT_PROMPT = (
    "Describe this camera image accurately and concretely. Identify visible "
    "objects, their relative positions, any readable text, and anything that "
    "would matter to a small indoor robot. Do not speculate about unseen details."
)


@dataclass
class Result:
    name: str
    model: str
    answer: str | None
    error: str | None
    model_load_seconds: float | None
    request_seconds: float | None
    prompt_tokens: int | None
    generated_tokens: int | None
    prompt_tokens_per_second: float | None
    generated_tokens_per_second: float | None
    peak_memory_gb: float | None


def seconds_from_ns(value: Any) -> float | None:
    return round(float(value) / 1_000_000_000, 3) if value else None


def rounded(value: Any) -> float | None:
    return round(float(value), 2) if value is not None else None


def capture_frame(camera_index: int, output_path: Path) -> Path:
    camera = cv2.VideoCapture(camera_index)
    if not camera.isOpened():
        raise RuntimeError(
            f"Could not open camera {camera_index}. Grant camera permission to the "
            "terminal/application, or re-run with --image PATH."
        )
    try:
        # Avoid saving an old frame that was sitting in the camera buffer.
        for _ in range(5):
            camera.read()
        ok, frame = camera.read()
    finally:
        camera.release()
    if not ok:
        raise RuntimeError(f"Could not read a frame from camera {camera_index}.")
    if not cv2.imwrite(str(output_path), frame):
        raise RuntimeError(f"Could not save captured frame to {output_path}.")
    return output_path


def configure_mlx_processor(model_id: str, processor: Any) -> None:
    """Apply model-repository compatibility settings before MLX inference."""
    if model_id != LFM_450M_MODEL:
        return

    # This MLX conversion declares `max_num_patches=256`, but LFM's visual
    # encoder receives pre-pixel-unshuffle patches. A 512px tile has
    # (512 / 16)^2 = 1,024 of those patches; limiting it to 256 truncates the
    # image while retaining the original spatial dimensions, causing a shape
    # mismatch in mlx-vlm. Preserve the complete tile instead.
    image_processor = processor.image_processor
    tile_patch_count = (image_processor.tile_size // image_processor.patch_size) ** 2
    image_processor.max_num_patches = max(
        image_processor.max_num_patches, tile_patch_count
    )


def run_mlx_vlm(name: str, model_id: str, image_path: Path, prompt: str, max_tokens: int) -> Result:
    model = processor = config = None
    try:
        load_started = time.perf_counter()
        model, processor = load(model_id)
        config = load_config(model_id)
        configure_mlx_processor(model_id, processor)
        model_load_seconds = time.perf_counter() - load_started
        formatted_prompt = apply_chat_template(processor, config, prompt, num_images=1)
        # Load weights and run an unreported first inference before timing the
        # result shown to the user. That matches a robot process which keeps
        # its vision model resident in memory.
        generate(
            model,
            processor,
            formatted_prompt,
            image=[str(image_path)],
            max_tokens=max_tokens,
            temp=0.0,
            verbose=False,
        )
        request_started = time.perf_counter()
        output = generate(
            model,
            processor,
            formatted_prompt,
            image=[str(image_path)],
            max_tokens=max_tokens,
            temp=0.0,
            verbose=False,
        )
        request_seconds = time.perf_counter() - request_started
        return Result(
            name=name,
            model=model_id,
            answer=output.text.strip(),
            error=None,
            model_load_seconds=rounded(model_load_seconds),
            request_seconds=rounded(request_seconds),
            prompt_tokens=output.prompt_tokens,
            generated_tokens=output.generation_tokens,
            prompt_tokens_per_second=rounded(output.prompt_tps),
            generated_tokens_per_second=rounded(output.generation_tps),
            peak_memory_gb=rounded(output.peak_memory),
        )
    except Exception as error:  # Keep other models running if one backend fails.
        return Result(name, model_id, None, str(error), None, None, None, None, None, None, None)
    finally:
        # Evaluate serially so measurements are meaningful on unified memory.
        del model, processor, config
        gc.collect()
        mx.clear_cache()


def markdown_report(image_path: Path, prompt: str, results: list[Result]) -> str:
    lines = [
        "# Vision bake-off",
        "",
        f"Image: `{image_path.name}`",
        "",
        f"Prompt: {prompt}",
        "",
        "## Speed",
        "",
        "| Model | Startup (s) | Warm request (s) | Prompt tok/s | Generated tok/s | Peak memory (GB) |",
        "| --- | ---: | ---: | ---: | ---: | ---: |",
    ]
    for result in results:
        lines.append(
            "| {name} | {load} | {request} | {prompt_tps} | {generation_tps} | {memory} |".format(
                name=result.name,
                load=result.model_load_seconds if result.model_load_seconds is not None else "—",
                request=result.request_seconds if result.request_seconds is not None else "—",
                prompt_tps=result.prompt_tokens_per_second if result.prompt_tokens_per_second is not None else "—",
                generation_tps=result.generated_tokens_per_second if result.generated_tokens_per_second is not None else "—",
                memory=result.peak_memory_gb if result.peak_memory_gb is not None else "—",
            )
        )
    lines.extend(["", "## Answers", ""])
    for result in results:
        lines.extend([f"### {result.name}", "", f"Model: `{result.model}`", ""])
        if result.error:
            lines.extend([f"**Error:** {result.error}", ""])
        else:
            lines.extend([result.answer or "(No text returned.)", ""])
    lines.append(
        "Speed notes: `Startup` is model initialization time. Each model then performs "
        "one unreported warm-up on the same image. `Warm request` is the second, end-to-end "
        "inference time and is the meaningful latency for steady robot operation. Token speeds "
        "are backend-reported."
    )
    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image", type=Path, help="Use an existing image instead of capturing the webcam.")
    parser.add_argument("--camera-index", type=int, default=0, help="OpenCV camera index (default: 0).")
    parser.add_argument("--prompt", default=DEFAULT_PROMPT)
    parser.add_argument("--max-tokens", type=int, default=128)
    parser.add_argument("--output-dir", type=Path, default=Path(__file__).resolve().parent / "results" / "vision")
    args = parser.parse_args()

    run_dir = args.output_dir / datetime.now().strftime("%Y%m%d-%H%M%S")
    run_dir.mkdir(parents=True, exist_ok=False)
    image_path = run_dir / "camera_frame.jpg"
    try:
        if args.image:
            if not args.image.is_file():
                parser.error(f"Image does not exist: {args.image}")
            shutil.copy2(args.image, image_path)
        else:
            capture_frame(args.camera_index, image_path)
    except Exception:
        # Do not leave an empty, ignored result directory after a permission or
        # hardware failure before a frame exists.
        if run_dir.exists() and not any(run_dir.iterdir()):
            run_dir.rmdir()
        raise

    # Verify the image once before giving the identical file to every backend.
    with Image.open(image_path) as image:
        image.verify()

    print(f"Using frame: {image_path}")
    results = []
    print("Running LFM2.5-VL-450M 6-bit (MLX)...")
    results.append(run_mlx_vlm("LFM2.5-VL-450M 6-bit (MLX)", LFM_450M_MODEL, image_path, args.prompt, args.max_tokens))
    print("Running LFM2.5-VL-1.6B 6-bit (MLX)...")
    results.append(run_mlx_vlm("LFM2.5-VL-1.6B 6-bit (MLX)", LFM_MODEL, image_path, args.prompt, args.max_tokens))

    report_path = run_dir / "report.md"
    report_path.write_text(markdown_report(image_path, args.prompt, results))
    (run_dir / "results.json").write_text(json.dumps([asdict(result) for result in results], indent=2) + "\n")
    print(markdown_report(image_path, args.prompt, results))
    print(f"Saved report: {report_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
