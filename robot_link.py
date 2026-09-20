"""Single serial reader and expiring protocol-v2 motion commands."""
from __future__ import annotations

import copy
import json
import math
import threading
import time

ACTIONS = {"stop", "forward", "left", "right"}
SENSOR_NAMES = ("L90", "L45", "F", "R45", "R90")
TELEMETRY_MAX_AGE = 0.25
CLEARANCE_MM = 300
LEASE_MS = 600


def number(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def parse_telemetry(line):
    try:
        data = json.loads(line)
        if not isinstance(data, dict) or data.get("protocol") != 2:
            return None
        ranges = data.get("tof_mm")
        if not isinstance(ranges, list) or len(ranges) != 5:
            return None
        if any(v is not None and (not number(v) or not 0 < v < 8190) for v in ranges):
            return None
        if (data.get("motion") not in ACTIONS or not isinstance(data.get("imu_valid"), bool)
            or not number(data.get("heading_deg")) or not number(data.get("yaw_rate_dps"))
            or not number(data.get("imu_age_ms")) or data["imu_age_ms"] < 0
            or not isinstance(data.get("command_id"), int)
            or not isinstance(data.get("stop_reason"), str)):
            return None
        return data
    except (ValueError, TypeError, UnicodeDecodeError):
        return None


def allowed_movements(snapshot):
    if (not snapshot or snapshot.get("age_s", float("inf")) > TELEMETRY_MAX_AGE
        or not snapshot.get("imu_valid") or snapshot.get("imu_age_ms", 1000) >= 100):
        return ["stop"]
    ranges = snapshot["tof_mm"]
    allowed = ["stop"]
    # Missing returns are unknown, not a blanket motor veto. Jev also sees
    # the missing directions and current vision before choosing any movement.
    if not any(v is not None and v < CLEARANCE_MM for v in ranges[1:4]):
        allowed.append("forward")
    if not any(v is not None and v < CLEARANCE_MM for v in ranges):
        allowed.extend(("left", "right"))
    return allowed


class RobotLink:
    def __init__(self, port=None, dry_run=False, serial_factory=None):
        self.dry_run = dry_run
        self.lock = threading.Lock()
        self.write_lock = threading.Lock()
        self.latest = None
        self.received_at = 0.0
        self.sequence = 0
        self.closed = threading.Event()
        self.error = None
        self.serial = None
        self.reader = None
        if not dry_run:
            if serial_factory is None:
                import serial
                serial_factory = serial.Serial
            self.serial = serial_factory(port, 115200, timeout=0.05, write_timeout=0.1)
            self.serial.write(b'x\n')
            self.reader = threading.Thread(target=self._read, daemon=True, name="robot-telemetry")
            self.reader.start()

    def _read(self):
        buf = b""
        try:
            while not self.closed.is_set():
                buf += self.serial.read(max(1, min(self.serial.in_waiting, 4096)))
                while b'\n' in buf:
                    line, buf = buf.split(b'\n', 1)
                    data = parse_telemetry(line)
                    if data:
                        with self.lock:
                            self.latest, self.received_at = data, time.monotonic()
                if len(buf) > 4096:
                    buf = b""
        except Exception as exc:
            self.error = type(exc).__name__
            with self.lock:
                self.latest = None

    def snapshot(self):
        with self.lock:
            if self.latest is None:
                return {}
            result = copy.deepcopy(self.latest)
            result["age_s"] = round(time.monotonic() - self.received_at, 3)
        result["tof_cm"] = {k: None if v is None else round(v / 10, 1)
                            for k, v in zip(SENSOR_NAMES, result["tof_mm"])}
        result["allowed_movements"] = allowed_movements(result)
        return result

    def command(self, action):
        if action not in ACTIONS:
            raise ValueError("Unknown motion")
        with self.write_lock:
            # Recheck fresh sensor data at the point of dispatch.
            if action not in allowed_movements(self.snapshot()):
                action = "stop"
            self.sequence += 1
            if self.serial:
                self.serial.write(f"M,{self.sequence},{action},{LEASE_MS}\n".encode())
            return action

    def close(self):
        try:
            self.command("stop")
        finally:
            self.closed.set()
            if self.reader:
                self.reader.join(timeout=0.3)
            if self.serial:
                self.serial.close()
