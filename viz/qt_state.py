import json
from PyQt5.QtCore import QObject, pyqtSignal, QTimer

class QtStateWatcher(QObject):
    """
    Polls CalibState.snapshot() every 200ms on the main thread.
    Emits state_changed(dict) whenever data updates.
    No changes to CalibState needed.
    """

    state_changed = pyqtSignal(dict)

    def __init__(self, calib_state, interval_ms=200):
        super().__init__()
        self._state      = calib_state
        self._last_snap  = ""

        self._timer = QTimer()
        self._timer.setInterval(interval_ms)
        self._timer.timeout.connect(self._poll)

    def start(self):
        self._timer.start()

    def _poll(self):
        snap = self._state.snapshot()      # thread-safe JSON string
        if snap == self._last_snap:
            return                         # nothing changed, skip
        self._last_snap = snap
        self.state_changed.emit(json.loads(snap))