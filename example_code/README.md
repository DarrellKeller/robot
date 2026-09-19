# Standalone examples

These scripts are historical experiments, not the Jev runtime. Read each script's
configuration before running it; some use a webcam, microphone, speaker, local
model server, or a serial port. Additional packages such as `openai` may be needed
for individual examples.

- `esp32_control.py`: old single-character serial controls. Requires legacy firmware;
  it cannot drive the new protocol-v2 firmware.
- `mlx_webcam_vision.py`: local MLX webcam experiment.
- `webcam_vision.py`: webcam experiment using an OpenAI-compatible local server.
- `scene_analyzer.py`, `test_original_format.py`: older scene/model-format experiments.
- `piper_with_playback.py`, `download_voice.py`: speech playback and voice download helpers.
- `wakeword_detector.py`: standalone wakeword experiment. Do not run beside the
  Jev controller, which owns microphone access.

For maintained model comparisons, use [benchmarks](../benchmarks/README.md).
For robot operation, use `autonomous_control.py` at the repository root.
