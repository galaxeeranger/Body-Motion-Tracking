from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QHBoxLayout,
    QVBoxLayout, QLabel
)
from PyQt5.QtCore import Qt

from viz.viewport import Viewport3D
from viz.sidebar  import Sidebar
from viz.qt_state import QtStateWatcher


class MainWindow(QMainWindow):
    """
    Top-level window.
    Owns viewport + sidebar + wires QtStateWatcher to both.
    Receives AppController to wire recalibrate button.
    """

    def __init__(self, calib_state, controller):
        super().__init__()
        self.setWindowTitle("Multi-Camera Calibration — Live 3D")
        self.resize(1100, 680)
        self.setStyleSheet("background:#111111;")

        self._controller = controller

        # ── core widgets ──
        self.viewport = Viewport3D()
        self.sidebar  = Sidebar(self.viewport)

        # ── wire recalibrate button ──
        # sidebar emits recalibrate_requested signal
        # controller.restart_pipeline() handles it
        self.sidebar.recalibrate_requested.connect(
            self._on_recalibrate
        )

        # ── layout ──
        central = QWidget()
        layout  = QHBoxLayout(central)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        layout.addWidget(self.viewport, stretch=1)
        layout.addWidget(self.sidebar)

        # ── header bar ──
        header = self._build_header()

        # ── root layout ──
        root_widget = QWidget()
        root_layout = QVBoxLayout(root_widget)
        root_layout.setContentsMargins(0, 0, 0, 0)
        root_layout.setSpacing(0)
        root_layout.addWidget(header)
        root_layout.addWidget(central)

        self.setCentralWidget(root_widget)

        # ── state watcher ──
        self._watcher = QtStateWatcher(calib_state, interval_ms=200)
        self._watcher.state_changed.connect(self.viewport.on_state)
        self._watcher.state_changed.connect(self.sidebar.on_state)
        self._watcher.start()

    # ────────────────────────────────────────────────
    # Recalibrate handler
    # ────────────────────────────────────────────────
    def _on_recalibrate(self):
        """
        Disables button while running,
        delegates to controller,
        sidebar log will show progress automatically
        via QtStateWatcher.
        """
        self.sidebar.set_recalibrate_enabled(False)
        self._controller.restart_pipeline()

    # ────────────────────────────────────────────────
    # Re-enable recalibrate button when done/error
    # ────────────────────────────────────────────────
    def _on_pipeline_done(self):
        self.sidebar.set_recalibrate_enabled(True)

    # ────────────────────────────────────────────────
    # Header bar
    # ────────────────────────────────────────────────
    def _build_header(self):
        bar = QWidget()
        bar.setFixedHeight(38)
        bar.setStyleSheet(
            "background:#1a1a1a;"
            "border-bottom:1px solid #333333;"
        )

        lay = QHBoxLayout(bar)
        lay.setContentsMargins(18, 0, 18, 0)
        lay.setSpacing(14)

        title = QLabel("Multi-Camera Calibration — Live 3D")
        title.setStyleSheet(
            "color:#ffffff; font-size:14px; font-weight:600;"
        )

        hint = QLabel("Drag · Scroll to zoom")
        hint.setStyleSheet("color:#555555; font-size:11px;")

        lay.addWidget(title)
        lay.addStretch()
        lay.addWidget(hint)

        return bar

    # ────────────────────────────────────────────────
    # Clean shutdown
    # ────────────────────────────────────────────────
    def closeEvent(self, event):
        self._watcher._timer.stop()
        self._controller.stop()
        event.accept()