# Mauricio

An indoor robot using **Jev** for typed decisions, **LFM2.5-VL-450M** for local
vision, **LFM2.5-1.2B Instruct** for speech and goal wording, **Whisper/Piper** for conversation, and an **ESP32** for
motor control and continuous ToF/MPU feedback.

Movement, vision and speech run independently. Recovery uses the same Jev
questions with recent outcomes as context. Jev screens transcriptions, approves
shared goals and chooses when to speak. A single LFM reply plays directly. One vision prompt describes the
scene, people and text in three sentences. Code owns persistence,
command expiry and sensor checks.

## Getting started

Install the dependencies in your Python environment:

```sh
python -m pip install -r requirements.txt
```

The runtime and benchmarks use `requirements.txt`. Additional packages for the
retained legacy controller and historical examples are optional:

```sh
python -m pip install -r requirements/legacy.txt
python -m pip install -r requirements/examples.txt
```

Each optional file includes the runtime dependencies. Use the project's interpreter
(e.g. `.conda/bin/python`) consistently when installing and running scripts.

Copy `.env.example` to `.env` **only if you do not already have a `.env`**, then
set `JEV_API_KEY`. Keep the Piper voice model `en_US-ryan-high.onnx` and its JSON
config at the repository root. Local MLX models download on first use.

```sh
python autonomous_control.py --goal "Find the dining table"
```

The default is a **dry run with no serial connection**. Camera, audio and model
calls still run. Use `--help` for options, including `--no-audio`, `--no-camera`,
`--duration`, `--plan`, and `--resume`.

Live operation requires the matching protocol-v2 firmware:

```sh
python autonomous_control.py --live --port /dev/tty.usbserial-0001 --goal "Find the dining table"
```

Do not run the legacy `wakeword_server.py` alongside the Jev controller; the new
controller owns the microphone. Saved missions do not resume motion automatically.

See the [controller guide](docs/jev-controller.md) for firmware wiring, model
setup, voice interaction, planning, timing, protocol details and current limits.
Hardware calibration and end-to-end robot trials remain to be done.

## Repository layout

| Location | Contents |
| --- | --- |
| Root Python modules | Jev runtime and retained legacy modules |
| `motor_control/` | ESP32 firmware |
| `tests/` | Offline control-boundary checks |
| `benchmarks/` | Vision, navigation and memory experiments |
| `scripts/` | Benchmark launchers, including macOS Finder `.command` files |
| `example_code/` | Standalone historical experiments; not the active controller |
| `docs/` | Detailed controller documentation |
| `runtime/` | Private mission state and decision logs; ignored by Git |
| `benchmarks/results/` | Local reports and captured images; ignored by Git |

## Development checks

```sh
python -m unittest discover -s tests -v
arduino-cli compile --fqbn esp32:esp32:esp32 motor_control
```

These checks do not move or flash the robot. See [benchmarks](benchmarks/README.md)
for model comparisons and [examples](example_code/README.md) for standalone tools.

The old controller is available through `python autonomous_control.py --legacy`.
It requires the old firmware and separate wakeword server. It does not support
protocol v2.
