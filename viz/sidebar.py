from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QCheckBox, QPushButton, QTextEdit, QFrame, QSizePolicy
)
from PyQt5.QtCore  import Qt, pyqtSignal
from PyQt5.QtGui   import QColor, QFont

COLORS = ["#4A9EE8", "#E87A4A", "#4AE87A", "#D080E0", "#E8D440"]

PHASE_STYLE = {
    "capturing":   ("Capturing",   "#2a3a4a", "#6da8de"),
    "calibrating": ("Calibrating", "#3a3a1a", "#e8d440"),
    "done":        ("Done ✓",      "#1a3a1a", "#6de06d"),
    "error":       ("Error ✗",     "#3a1a1a", "#e06d6d"),
}


class Sidebar(QWidget):
    """
    Right panel — mirrors <aside> in ui.html.
    Adds recalibrate_requested signal for recalibrate button.
    """

    # ── signal emitted when recalibrate button clicked ──
    recalibrate_requested = pyqtSignal()

    def __init__(self, viewport, parent=None):
        super().__init__(parent)
        self._viewport      = viewport
        self._cam_rows      = []
        self._last_log_n    = 0

        self.setFixedWidth(255)
        self.setStyleSheet("background:#161616; color:#dddddd;")

        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        # ── phase badge + counter ──
        root.addWidget(self._make_header())
        root.addWidget(self._hline())

        # ── cameras ──
        root.addWidget(self._section_label("Cameras"))
        self._cam_status_layout = QVBoxLayout()
        self._cam_status_layout.setContentsMargins(10, 0, 10, 6)
        self._cam_status_layout.setSpacing(2)
        cam_wrap = QWidget()
        cam_wrap.setLayout(self._cam_status_layout)
        root.addWidget(cam_wrap)
        root.addWidget(self._hline())

        # ── controls ──
        root.addWidget(self._section_label("Controls"))
        root.addWidget(self._make_controls())
        root.addWidget(self._hline())

        # ── recalibrate button ──
        root.addWidget(self._make_recalibrate())
        root.addWidget(self._hline())

        # ── log ──
        root.addWidget(self._section_label("Log"))
        self._log = QTextEdit()
        self._log.setReadOnly(True)
        self._log.setStyleSheet("""
            QTextEdit {
                background: transparent;
                color: #777777;
                font-family: monospace;
                font-size: 11px;
                border: none;
            }
        """)
        self._log.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        root.addWidget(self._log)
        # add this with other toggle flags
        self.show_cam_board_dist = True
    # ────────────────────────────────────────────────
    # Public slot
    # ────────────────────────────────────────────────
    def on_state(self, data: dict):
        self._update_header(data)
        self._update_cam_status(data)
        self._update_log(data)

        # re-enable recalibrate button when pipeline is done or error
        phase = data.get("phase", "capturing")
        if phase in ("done", "error"):
            self.set_recalibrate_enabled(True)

    # ────────────────────────────────────────────────
    # Header
    # ────────────────────────────────────────────────
    def _make_header(self):
        w = QWidget()
        w.setStyleSheet("background:#1a1a1a;")
        lay = QHBoxLayout(w)
        lay.setContentsMargins(10, 8, 10, 8)

        self._badge = QLabel("Capturing")
        self._badge.setStyleSheet(
            "background:#2a3a4a; color:#6da8de;"
            "border-radius:10px; padding:2px 8px; font-size:11px;"
        )

        self._counter = QLabel("0 / 25")
        self._counter.setStyleSheet("color:#888888; font-size:12px;")

        lay.addWidget(self._badge)
        lay.addStretch()
        lay.addWidget(self._counter)
        return w

    def _update_header(self, data):
        phase = data.get("phase", "capturing")
        label, bg, fg = PHASE_STYLE.get(phase, ("…", "#2a2a2a", "#aaaaaa"))
        self._badge.setText(label)
        self._badge.setStyleSheet(
            f"background:{bg}; color:{fg};"
            "border-radius:10px; padding:2px 8px; font-size:11px;"
        )
        cap = data.get("captures", 0)
        mx  = data.get("max_captures", 25)
        self._counter.setText(f"{cap} / {mx}")

    # ────────────────────────────────────────────────
    # Camera status rows
    # ────────────────────────────────────────────────
    def _update_cam_status(self, data):
        cams     = data.get("cameras", [])
        detected = data.get("cam_detected", [])
        N        = max(len(detected), len(cams), 1)

        while len(self._cam_rows) < N:
            row, dot, status = self._make_cam_row(len(self._cam_rows))
            self._cam_status_layout.addWidget(row)
            self._cam_rows.append((dot, status))

        for i, (dot, status_lbl) in enumerate(self._cam_rows):
            color      = COLORS[i % len(COLORS)]
            calibrated = any(c.get("idx") == i for c in cams)
            det        = detected[i] if i < len(detected) else False

            if calibrated:
                dot.setStyleSheet(
                    f"background:{color}; border-radius:4px;"
                    "min-width:9px; max-width:9px;"
                    "min-height:9px; max-height:9px;"
                )
                status_lbl.setText("calibrated")
                status_lbl.setStyleSheet("color:#555555; font-size:11px;")
            elif det:
                dot.setStyleSheet(
                    "background:#e8a840; border-radius:4px;"
                    "min-width:9px; max-width:9px;"
                    "min-height:9px; max-height:9px;"
                )
                status_lbl.setText("detected")
                status_lbl.setStyleSheet("color:#555555; font-size:11px;")
            else:
                dot.setStyleSheet(
                    "background:#333333; border-radius:4px;"
                    "min-width:9px; max-width:9px;"
                    "min-height:9px; max-height:9px;"
                )
                status_lbl.setText("waiting")
                status_lbl.setStyleSheet("color:#444444; font-size:11px;")

    def _make_cam_row(self, idx):
        color = COLORS[idx % len(COLORS)]
        row   = QWidget()
        lay   = QHBoxLayout(row)
        lay.setContentsMargins(4, 3, 4, 3)
        lay.setSpacing(7)

        dot = QLabel()
        dot.setFixedSize(9, 9)
        dot.setStyleSheet(
            "background:#333333; border-radius:4px;"
            "min-width:9px; max-width:9px;"
            "min-height:9px; max-height:9px;"
        )

        name = QLabel(f"Cam {idx}")
        name.setStyleSheet(
            f"color:{color}; font-weight:600; font-size:12px;"
        )

        status = QLabel("waiting")
        status.setStyleSheet("color:#444444; font-size:11px;")
        status.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

        lay.addWidget(dot)
        lay.addWidget(name)
        lay.addStretch()
        lay.addWidget(status)
        return row, dot, status

    # ────────────────────────────────────────────────
    # Controls (checkboxes + view presets)
    # ────────────────────────────────────────────────
    def _make_controls(self):
        w   = QWidget()
        lay = QVBoxLayout(w)
        lay.setContentsMargins(14, 6, 14, 10)
        lay.setSpacing(4)

        checks = [
            ("Frustums",      "show_frustums"),
            ("ChArUco board", "show_board"),
            ("Axes",          "show_axes"),
            ("Sight lines",   "show_sightlines"),
            ("Cam→Board dist",     "show_cam_board_dist"),  # ← add this
        ]
        for label, attr in checks:
            cb = QCheckBox(label)
            cb.setChecked(True)
            cb.setStyleSheet("font-size:12px; color:#cccccc;")
            cb.toggled.connect(lambda val, a=attr: self._toggle(a, val))
            lay.addWidget(cb)

        btn_row = QHBoxLayout()
        btn_row.setSpacing(5)
        for name in ["Front", "Top", "Iso", "Reset"]:
            btn = QPushButton(name)
            btn.setStyleSheet("""
                QPushButton {
                    font-size:11px; padding:4px 8px;
                    border:1px solid #444; background:#222;
                    color:#cccccc; border-radius:4px;
                }
                QPushButton:hover { background:#333; }
            """)
            btn.clicked.connect(
                lambda _, n=name.lower(): self._viewport.set_view(n)
            )
            btn_row.addWidget(btn)
        lay.addLayout(btn_row)
        return w

    def _toggle(self, attr, val):
        setattr(self._viewport, attr, val)
        self._viewport.update()

    # ────────────────────────────────────────────────
    # Recalibrate button
    # ────────────────────────────────────────────────
    def _make_recalibrate(self):
        w   = QWidget()
        lay = QVBoxLayout(w)
        lay.setContentsMargins(14, 8, 14, 8)

        self._recalib_btn = QPushButton("🔄  Recalibrate")
        self._recalib_btn.setStyleSheet("""
            QPushButton {
                font-size:12px;
                font-weight:600;
                padding:7px;
                border:1px solid #4A9EE8;
                background:#1a2a3a;
                color:#4A9EE8;
                border-radius:5px;
            }
            QPushButton:hover {
                background:#2a3a4a;
                color:#6db8ff;
            }
            QPushButton:disabled {
                border:1px solid #333;
                background:#1a1a1a;
                color:#444444;
            }
        """)
        self._recalib_btn.clicked.connect(self._on_recalibrate_clicked)
        lay.addWidget(self._recalib_btn)
        return w

    def _on_recalibrate_clicked(self):
        # disable immediately to prevent double click
        self.set_recalibrate_enabled(False)
        # reset log counter so new log appears fresh
        self._last_log_n = 0
        self._log.clear()
        # emit signal — window.py handles the rest
        self.recalibrate_requested.emit()

    def set_recalibrate_enabled(self, val: bool):
        """Called by window.py to enable/disable button."""
        self._recalib_btn.setEnabled(val)
        if val:
            self._recalib_btn.setText("🔄  Recalibrate")
        else:
            self._recalib_btn.setText("⏳  Running...")

    # ────────────────────────────────────────────────
    # Log
    # ────────────────────────────────────────────────
    def _update_log(self, data):
        lines = data.get("log", [])
        if len(lines) <= self._last_log_n:
            return
        for line in lines[self._last_log_n:]:
            if "✅" in line or "COMPLETE" in line:
                color = "#6de06d"
            elif "❌" in line:
                color = "#e05d5d"
            elif any(x in line for x in ["🔧", "📷", "🎯", "🟢", "🔄"]):
                color = "#5b9cf6"
            else:
                color = "#777777"
            self._log.append(
                f'<span style="color:{color}">{line}</span>'
            )
        self._log.verticalScrollBar().setValue(
            self._log.verticalScrollBar().maximum()
        )
        self._last_log_n = len(lines)

    # ────────────────────────────────────────────────
    # Helpers
    # ────────────────────────────────────────────────
    def _section_label(self, text):
        lbl = QLabel(text.upper())
        lbl.setStyleSheet(
            "color:#666666; font-size:10px; font-weight:600;"
            "letter-spacing:1px; padding:8px 14px 4px 14px;"
        )
        return lbl

    def _hline(self):
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setStyleSheet("color:#2a2a2a;")
        return line
