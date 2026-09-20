"""Evidence-based recovery bookkeeping; never sends motor commands."""
from robot_link import SENSOR_NAMES, CLEARANCE_MM

CLEAR_MM = 400
CLEAR_HOLD_S = 0.3
REVERSE_LIMIT_S = 4.0
ATTEMPT_S = 0.5
IMPROVEMENT_MM = 20
TURN_DEGREES = 20
TURN_LIMIT_S = 3.0
RECOVERY_LIMIT_S = 30
FAILED_ATTEMPT_LIMIT = 4


class Recovery:
    def __init__(self):
        self.phase = None
        self.blockers = set()
        self.reverse_s = self.turn_s = self.elapsed_s = 0.0
        self.bad_attempts = 0
        self.chunk_s = 0.0
        self.baseline = None
        self.clear_since = self.turn_heading = self.stopped_at = None
        self.previous = None
        self.trend = "unknown"
        self.reason = None

    def allowed(self, actions):
        permitted = {"retreat": {"stop", "backward"}, "turn": {"stop", "left", "right"},
                     "observe": {"stop"}, "help": {"stop"}}
        return [a for a in actions if a in permitted.get(self.phase, set(actions))]

    def context(self):
        if self.phase is None:
            return None
        objectives = {"retreat": "Back away to make room to turn",
                      "turn": "Turn toward steering advice or a visible opening",
                      "observe": "Stop and get a scene captured after the turn before resuming the main task",
                      "help": "Ask for help or repositioning before retrying recovery"}
        return dict(phase=self.phase, objective=objectives[self.phase],
                    blocking_sensors=[SENSOR_NAMES[i] for i in sorted(self.blockers)],
                    clearance_target_cm=40, clearance_trend=self.trend,
                    reverse_motion_s=round(self.reverse_s, 2),
                    attempts_without_improvement=self.bad_attempts, reason=self.reason)

    def update(self, sensors, vision, now, *, enabled):
        before = self.phase
        fresh = bool(sensors) and sensors.get("age_s", 1) <= 0.25
        # Integrate only consecutive fresh telemetry, never infer travel from a request.
        dt = 0.0
        if fresh and self.previous and now - self.previous[0] <= 0.25:
            dt = max(0, now - self.previous[0])
        previous_motion = self.previous[1] if self.previous else "stop"
        self.previous = (now, sensors.get("motion", "stop")) if fresh else None
        if not enabled or not fresh:
            self.clear_since = None
            return None
        ranges = sensors["tof_mm"]
        close = {i for i, v in enumerate(ranges) if v is not None and v < CLEARANCE_MM}
        if self.phase is None and close:
            self.__init__()
            self.previous = (now, sensors.get("motion", "stop"))
            self.phase = "retreat"
            self.blockers = close
            self.baseline = min(ranges[i] for i in close)
        if self.phase is None:
            return None
        self.elapsed_s += dt
        if self.phase in {"retreat", "turn"}:
            new = close - self.blockers
            self.blockers |= close
            if new:
                self.baseline = None
            if close and self.phase == "turn":
                self.phase = "retreat"
                self.clear_since = None
                self.turn_heading = None
        if self.phase == "retreat":
            values = [ranges[i] for i in self.blockers]
            clearance = min(values) if all(v is not None for v in values) else None
            if previous_motion == "backward":
                self.reverse_s += dt
                self.chunk_s += dt
            if self.chunk_s >= ATTEMPT_S:
                if clearance is None or self.baseline is None:
                    self.trend = "unknown"
                    # Missing data is not evidence of a failed retreat. Preserve
                    # the last valid baseline so reacquisition can show progress.
                    if clearance is not None:
                        self.baseline = clearance
                else:
                    improved = clearance >= self.baseline + IMPROVEMENT_MM
                    self.trend = "improving" if improved else "not improving"
                    self.bad_attempts = 0 if improved else self.bad_attempts + 1
                    self.baseline = clearance
                self.chunk_s = 0.0
            if clearance is not None and clearance > CLEAR_MM:
                if self.clear_since is None:
                    self.clear_since = now
                if now - self.clear_since >= CLEAR_HOLD_S:
                    self.phase = "turn"
            else:
                self.clear_since = None
            if self.phase == "retreat" and (self.reverse_s >= REVERSE_LIMIT_S or self.bad_attempts >= FAILED_ATTEMPT_LIMIT):
                self.phase, self.reason = "help", "Reverse budget exhausted or clearance failed to improve"
        elif self.phase == "turn":
            if (sensors.get("motion") in {"left", "right"} and sensors.get("imu_valid")
                and sensors.get("imu_age_ms", 1000) < 100):
                if self.turn_heading is None:
                    self.turn_heading = sensors["heading_deg"]
                if previous_motion in {"left", "right"}:
                    self.turn_s += dt
                if abs(sensors["heading_deg"] - self.turn_heading) >= TURN_DEGREES:
                    self.phase = "observe"
            if self.turn_s >= TURN_LIMIT_S and self.phase == "turn":
                self.phase, self.reason = "help", "Pivot commands produced insufficient heading change"
        elif self.phase == "observe":
            if sensors.get("motion") == "stop":
                if self.stopped_at is None:
                    self.stopped_at = now
                if vision.get("fresh") and vision.get("captured_at", 0) >= self.stopped_at:
                    self.phase = None
            else:
                self.stopped_at = None
        if self.phase in {"retreat", "turn", "observe"} and self.elapsed_s >= RECOVERY_LIMIT_S:
            self.phase, self.reason = "help", "Recovery made no timely progress"
        if self.phase != before:
            return {"from": before, "to": self.phase, "evidence": self.context()}
        return None
