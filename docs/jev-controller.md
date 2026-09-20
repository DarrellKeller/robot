# Mauricio: Jev-controlled indoor robot

Jev makes short, typed decisions from ToF, MPU heading, local vision, dialogue,
subgoals and recent outcomes. Python owns execution and persistence. The ESP32
owns motor output, continuous sensor acquisition, clearance checks and a command
watchdog. LFM2.5-VL-450M supplies scene descriptions, candidate replies and short goal drafts.
Jev screens user input, approves goals, and selects every generated reply before playback.
With audio enabled, a newly approved task requests a brief cheeky acknowledgment.
Acknowledgment generation and playback do not block valid movement; speech
and movement can run concurrently. Explicit `talk` steps still require actual playback to complete.
Spoken replies become shared event history; the accepted request remains the goal.
Without audio, the task starts directly. LFM drafts up to three replies; a failed
candidate does not discard the usable ones, and Jev can only play an existing,
independently approved candidate. Local inference yields between candidates to
allow microphone transcription.

## Architecture

- `autonomous_control.py`: entry point; runs the Jev controller by default.
- `jev_control.py`: independent decision, vision, audio and telemetry scheduling.
- `jev_client.py`: TypeSafe HTTP client and typed decisions and input/output approval gates.
- `robot_link.py`: single serial reader and protocol-v2 movement dispatch.
- `mission_store.py`: atomic persistence of subgoals, dialogue, observed motion,
  recovery outcomes and selected visual memories.
- `lfm_tools.py`: resident MLX model, bounded priority queue and persistent camera.
- `audio_controller.py`: sole microphone/playback owner, Whisper wake detection,
  question/answer capture and Piper playback.
- `motor_control/motor_control.ino`: protocol-v2 ESP32 firmware.
- `legacy_autonomous_control.py`: previous controller, preserved separately.

There is no exclusive `next_mode`. Movement, vision and speech can proceed
independently. Speaking and microphone capture are mutually exclusive. A pending
question holds motion until answered or listening times out.

| Jev question | Type | Purpose |
| --- | --- | --- |
| `movement` | Choice | `forward`, `backward`, `left`, `right`, `stop` |
| `need_fresh_vision` | Noul | Gate an additional visual inspection |
| `user_route` | Choice | Reject noise, clarify, chat, goal, answer, cancel, resume |
| `approve_goal` | Noul | Check a goal draft against the accepted request |
| `activity` | Choice | Navigate, dance, talk, listen or wait within the shared goal |
| `speech_choice` | Choice | Select candidate 1/2/3, reject all, or wait |
| `speech_1_ok` / `speech_2_ok` / `speech_3_ok` | Noul | Independently check each candidate for relevance and grounding |
| `lfm_speech_tool` | Choice | None, question, answer, update, celebration |
| `goal_complete` | Noul | Completion evidence for the current subgoal |
| `should_remember` | Noul | Retain the current observation as a sourced fact |

Recovery adds no questions: recent attempts, interruption reasons, repeated
failures and a step-overdue event become input to these same questions. Firmware
sensor/clearance stops never depend on a model's answer.

Choice answers select among actions; their confidence is logged but is not a
blanket permission-to-act threshold. Noul approval thresholds live in
`jev_client.py`: 0.8 for speech grounding and other yes/no gates, and 0.9 for
goal approval and completion. Movement still requires an active mission, fresh
vision and decision, no pending answer or active listening, and live hardware
clearance. An unseen target favors a brief search pivot (initially left when
neither side has an advantage); forward travel requires a visually assessed
route. Stop remains available for concrete reasons to remain still.

## Setup

Install Python requirements in the project's environment:

```sh
python -m pip install -r requirements.txt
```

`requirements.txt` covers the Jev runtime and benchmarks. For the old controller,
install `requirements/legacy.txt`; for all historical examples, install
`requirements/examples.txt`. These optional files include the runtime requirements.

Create `.env` (ignored by Git):

```dotenv
JEV_API_KEY=your_key
JEV_MODEL=jev-1.13.0
```

The version is pinned because model changes can affect calibrated thresholds.
The client sends state and typed questions to `https://api.typesafe.ai/v1/systemone`.
It sends the question definitions, not the Python source file. Static movement
policy lives in those questions; hardware context contains reliability facts
rather than repeating the navigation instructions.
It does not retry an old physical snapshot; the loop backs off and submits fresh
state after errors. Never put keys into tracked files or CLI arguments.

Local models:

- Vision, speech text and plan proposals: `mlx-community/LFM2.5-VL-450M-6bit`.
- Whisper: `mlx-community/whisper-base.en-mlx`.
- Piper: place `en_US-ryan-high.onnx` and its JSON config beside `tts_module.py`.

Grant camera and microphone access to the terminal/app used to run Python before
disconnecting the display. On macOS, check Privacy & Security → Camera and
Microphone for Terminal. Camera authorization is requested on the main thread.
**Do not run `wakeword_server.py` alongside the Jev controller.** Audio is now
owned by the controller itself. The old server remains for the legacy path.

## Firmware compatibility

Compile/upload `motor_control/motor_control.ino` for your ESP32 with the Pololu
VL53L0X library. This firmware and the new controller must be deployed together.
The host refuses movement without protocol-v2 telemetry. Old one-character
movement commands cannot start motors on the new firmware; `x` still stops them.

The existing wiring is retained: five ToF sensors at left 90°, left 45°, front,
right 45°, right 90°, via TCA channels 7, 6, 5, 4, 3. Motor pins are unchanged.
The gyro driver retains the existing MPU-6050-compatible register map and Y-axis
rotation convention. It reports the detected model. **MPU-3050 compatibility has
not been established**; verify the actual module and mounting before driving.
No fabricated support or absolute compass heading is assumed.

## Running

Default is a dry run: **no serial port is opened and no motor commands are sent**.
The camera, local models, audio and Jev API still operate unless disabled. Without
sensor telemetry the only permitted movement is stop; dry run is not a simulator.

```sh
python autonomous_control.py --goal "Find the dining table, dance beside it, then ask whether people liked it."
```

A live run requires an explicit flag and the matching firmware:

```sh
python autonomous_control.py --live --port /dev/tty.usbserial-0001 --goal "Find the dining table"
```

Other options:

```sh
python autonomous_control.py --help
python autonomous_control.py --no-audio --no-camera --duration 10
python autonomous_control.py --live --resume
python autonomous_control.py --vision-hz 2
```

Use a wake word such as “robot” or “Mauricio.” A command can follow the wake word
in the same utterance, or the robot listens for a follow-up. After the robot asks
a question, it listens automatically without requiring another wake word.
“Stop,” “cancel,” “stop moving,” and “cancel task” stop and pause the task.
“Resume” or “continue” explicitly resumes a retained plan. Voice capture does not
interrupt TTS; Ctrl-C and the serial `x` stop are independent of audio.

Active missions load paused on restart. Use `--resume` explicitly. Runtime state
and bounded decision logs live under ignored `runtime/`. A once-per-second
`status.json` snapshot reports sensors, audio, tools and task state for headless
diagnostics; `--state PATH` selects
another store. Decision logs include the sent state, answers and dispatched action.
They can contain private dialogue and observations; keep them local.

Each run also writes `runtime/sessions/<UTC timestamp>-<PID>.jsonl` without
rotating away earlier runs. It records raw transcripts and quality, input routing,
exact Jev request/response pairs (including discarded decisions), LFM prompts and
outputs, goal approval, speech selection, punctuation-free TTS text, playback
results, motion dispatch and one-second state snapshots. The camera frames sent
to LFM are saved as JPEGs in that session's `frames/` directory for visual review.
No microphone recordings or API credentials are stored. The live Terminal log
prints the session path at startup. Session files remain local until removed.

The personality startup plan is `config/startup-plan.json`. Run it with
`--plan config/startup-plan.json --goal "Introduce Mauricio and invite a task"`.
Punctuation and symbols are removed before speech synthesis; the trace retains
both the approved text and the words actually sent to TTS.


## Goals and execution

Raw Whisper output and recognition quality go to Jev first. Rejected text never
enters accepted dialogue or LFM prompts. The local exact stop command remains
immediate; the fixed wake acknowledgement "Huh?" is the only ungated spoken cue.

There is no LFM JSON planner. After Jev approves an action request, LFM may rewrite
it as a short plain-text goal. Jev checks the draft before code installs it. If the
rewrite fails or changes the request, the original accepted request is presented
to Jev for approval instead. Both models share the approved goal. Jev selects the
next activity and motor primitive directly, including the order of compound tasks.
Actual spoken words, captured answers and measured dance motion are retained as
goal events; proposed replies are never completion evidence. Dance evidence requires
four seconds of reported pivot motion. This is reactive navigation, not a mapped
path planner.

One vision prompt requests three short sentences covering scene geometry, obstacles,
people's visible appearance/actions, and readable text, with the shared goal as
search context. The same observation runs during idle conversation and active goals.
There are no separate goal-search, person-inspection or text-reading tools.

Jev authorizes speech generation. LFM proposes three replies at temperature 0.8,
with Mauricio's playful, sassy personality. Jev chooses a candidate and independently checks its grounding, or
rejects all of them; only its selected reply reaches Piper. Candidates are tied
to the current input revision and cannot be approved by an older request.

Jev gets up to 24 accepted messages (roughly 12 exchanges), the shared goal,
current observation, ToF/MPU feedback, eight route segments, six outcomes, twelve
remembered observations and up to 24 goal events. LFM speech gets the last 12 accepted
messages, the same goal/current task, scene freshness, measured motor state, six
recent goal events and three outcomes. Goal rewriting uses the last six accepted
messages plus the approved request. Vision uses the image and shared goal. Pydantic validates goal drafts, exactly three nonempty candidate replies, transcription
quality and every returned Jev decision. Validation enforces shape, not truth; Jev
approval is a separate gate. Neither
model keeps hidden conversation memory between calls. Version-1 unscreened dialogue
is archived in the mission file but excluded from both model contexts.

Manually reviewed legacy step lists remain supported with `--plan PATH`:

```json
{
  "intent": "new_goal",
  "steps": [
    {
      "kind": "navigate",
      "instruction": "Find the dining table",
      "completion": "The dining table is visible in a fresh observation"
    },
    {
      "kind": "talk",
      "instruction": "Tell the user that the table was found",
      "completion": "The announcement finished playing"
    }
  ]
}
```

Repeated identical movement segments are merged. Route entries distinguish
reported motor activity from measured heading change: **translation is
unmeasured**. Gyro heading is continuous relative to startup and can drift. These
are useful breadcrumbs, not position localization or a reliable return route.
Stored scene descriptions retain their source; they are model observations,
not independently verified map facts.

## Timing and command protocol

Initial settings are deliberately explicit and need chassis testing:

- Jev: up to 4 requests/second during tasks/input handling, 1/second while idle; one request in flight. Discard responses older
  than 450 ms or belonging to an earlier goal/subgoal.
- Vision: 1 observation/second by default, configurable up to 2. Explicit
  inspections take priority over the next background job. One pending job per
  kind prevents frame and speech backlogs; one model worker owns LFM inference.
  LFM and Whisper serialize device inference; Jev and motor control remain independent.
- Vision becomes stale after 2 seconds or 25° of heading change. Motion then
  stops until a fresh view is available. Camera capture itself runs continuously.
- Host: rechecks fresh sensors at dispatch; stops when telemetry is older than
  250 ms. Stops after 500 ms without a usable decision.
- Firmware: each motion lease is 600 ms; maximum accepted lease is 650 ms.
  Renewing an action continues it smoothly. Expiry stops motors; it does not
  grant permission to keep moving for an entire subgoal.
- Firmware takes one single-shot ToF measurement per loop, selecting exactly one
  PCA/TCA9548A channel. Other sensors retain their last readings while awaiting
  their turn. Commands, watchdog and MPU are serviced between measurements.
  Each library polling phase has a 30 ms timeout (two phases per measurement).
  Telemetry is emitted at the first loop opportunity after 50 ms, in every mode.
  Invalid/stale ranges are `null`, never “clear.” `tof_status` distinguishes
  `ok`, `out_of_range`, `timeout`, `i2c_error`, `mux_error`, `init_failed`, and `stale`.
  Startup sensor timeout matches main at 200 ms; runtime polls remain bounded
  at 30 ms per phase. MPU is polled during turns too.
- Motion requires fresh telemetry and a fresh calibrated MPU. Missing ToF returns
  no longer veto movement: they remain unknown, and Jev must assess the route from
  fresh vision. Detected obstacles still stop motion at 300 mm: forward checks the
  three forward-facing sensors; pivots check valid returns from all five.
  This cannot guarantee obstacle detection in directions with missing readings.
  Reverse is not exposed because rear coverage is absent. PWM defaults to 100.
- Confidence settings (0.75 choices, 0.8 Nouls, 0.9 completion) are initial
  policy settings, not empirically established reliability guarantees.

Host command (newline terminated):

```text
M,42,left,600
M,43,stop,600
```

Firmware telemetry (one JSON object per line):

```json
{"protocol":2,"uptime_ms":1000,"command_id":42,"motion":"left","stop_reason":"none","tof_mm":[900,900,900,900,900],"heading_deg":72.0,"yaw_rate_dps":20.0,"imu_valid":true,"imu_age_ms":5,"imu_model":"MPU-6050"}
```

## Validation and legacy code

Offline control-boundary tests:

```sh
python -m unittest discover -s tests -v
arduino-cli compile --fqbn esp32:esp32:esp32 motor_control
```

These check software behavior/buildability, not physical braking, sensor coverage,
turn direction, model judgment, conversational quality or sustained throughput.
Live robot tests, PWM/clearance calibration and end-to-end task trials remain.

For the old controller, `python autonomous_control.py --legacy` runs the retained
code. It requires the **old firmware** and the old separate wakeword server; it is
not a compatibility mode for protocol v2. Older benchmark scripts and example
programs remain separate from the Jev runtime.

References: [TypeSafe architecture](https://docs.typesafe.ai/concepts/how-to-build-with-system-one),
[parallel questions](https://docs.typesafe.ai/patterns/fan-out),
[API](https://docs.typesafe.ai/api),
[Jev limitations](https://docs.typesafe.ai/model-jaggedness/jev-1.13),
[Pololu VL53L0X library](https://github.com/pololu/vl53l0x-arduino).

Backward movement uses a 250 ms renewable lease and checks measured side clearance.
The robot has no rear range sensor; Jev is instructed to retreat briefly over
recently traversed space and reassess. Front obstacles do not prevent retreat.
Whisper metadata is normalized before validation; transcription failures are
logged and the listener continues instead of silently terminating.
