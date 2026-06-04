import threading
import json

class CalibState:
    """Single source of truth. Thread-safe read/write."""

    COLORS = ["#4A9EE8", "#E87A4A", "#4AE87A", "#D080E0", "#E8D440"]

    def __init__(self, num_cameras, max_captures):
        self._lock         = threading.Lock()
        self._num_cameras  = num_cameras
        self._max_captures = max_captures
        self._data         = self._fresh()

    # ── internal helper ──
    def _fresh(self):
        return {
            "phase":        "capturing",
            "captures":     0,
            "max_captures": self._max_captures,
            "cam_detected": [False] * self._num_cameras,
            "cameras":      [],
            "board":        {},
            "log":          []
        }

    def reset(self):
        """Wipes all data back to initial state. Called by recalibrate."""
        with self._lock:
            self._data = self._fresh()

    def update(self, **kwargs):
        with self._lock:
            self._data.update(kwargs)

    def set_cam_detected(self, idx, val):
        with self._lock:
            self._data["cam_detected"][idx] = val

    def increment_captures(self):
        with self._lock:
            self._data["captures"] += 1
            return self._data["captures"]

    def log(self, msg):
        print(msg)
        with self._lock:
            self._data["log"].append(msg)
            if len(self._data["log"]) > 80:
                self._data["log"] = self._data["log"][-80:]

    def snapshot(self):
        with self._lock:
            return json.dumps(self._data)