"""Thread-safe, per-run JSONL trace for local robot tuning."""
import json
import logging
import os
import time
from datetime import datetime, timezone
from pathlib import Path

LOGGER = logging.getLogger('robot.events')
LOGGER.propagate = False
LOGGER.setLevel(logging.INFO)
_frame_directory = None


def start_trace(directory):
    global _frame_directory
    directory = Path(directory) / 'sessions'
    directory.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
    path = directory / f'{stamp}-{os.getpid()}.jsonl'
    handler = logging.FileHandler(path, encoding='utf-8')
    handler.setFormatter(logging.Formatter('%(message)s'))
    LOGGER.addHandler(handler)
    _frame_directory = directory / path.stem / 'frames'
    logging.info('Detailed session log: %s', path)
    return handler, path


def capture_frame(frame):
    if _frame_directory is None:
        return None
    _frame_directory.mkdir(parents=True, exist_ok=True)
    path = _frame_directory / f'{time.time_ns()}.jpg'
    frame.save(path, format='JPEG', quality=75)
    record('vision_frame', path=str(path))
    return str(path)


def record(event, **details):
    LOGGER.info(json.dumps({'time': datetime.now(timezone.utc).isoformat(),
                           'monotonic_s': time.monotonic(), 'event': event, **details},
                          ensure_ascii=False, default=str))


def close_trace(handler):
    global _frame_directory
    record('session_end')
    LOGGER.removeHandler(handler)
    handler.close()
    _frame_directory = None
