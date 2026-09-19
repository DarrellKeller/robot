# Model benchmarks

These are standalone model comparisons, separate from the Jev controller. They
use the same local MLX environment as the runtime. No benchmark sends motor
commands. The vision benchmark captures a camera frame unless `--image` is given.

From the repository root:

```sh
python benchmarks/vision_bakeoff.py --help
python benchmarks/lfm_navigation_benchmark.py --help
python benchmarks/lfm_memory_benchmark.py --help
```

- **Vision bake-off:** compare the 450M and 1.6B models on the same image.
- **Navigation:** compare model decisions on text descriptions of a fixed apartment graph.
- **Memory:** measure retention of errands across teacher-forced dialogue turns.

Navigation and memory results do not measure visual inference or physical navigation.

On macOS, open the corresponding `.command` file in `scripts/` from Finder. The
launchers use `.conda/bin/python`, then `.venv/bin/python`, then `python3`. Set
`ROBOT_PYTHON` to override the interpreter. Command-line arguments are forwarded:

```sh
scripts/run_vision_bakeoff.command --image /absolute/path/to/image.jpg
scripts/run_lfm_navigation_benchmark.command --help
```

Reports are timestamped under `benchmarks/results/{vision,navigation,memory}/`,
regardless of the working directory. Override with `--output-dir PATH` if needed.
Existing local reports have been retained in these directories. Captured images
and reports stay ignored by Git.
