import math
import numpy as np
from PyQt5.QtWidgets import QWidget, QToolTip
from PyQt5.QtCore    import Qt, QPoint
from PyQt5.QtGui     import QPainter, QPen, QColor, QFont, QBrush

COLORS = ["#4A9EE8", "#E87A4A", "#4AE87A", "#D080E0", "#E8D440"]


class Viewport3D(QWidget):
    """
    Pure QPainter 3D viewport.
    Same projection math as ui.html — ported 1:1 from JS.
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(400, 300)
        self.setCursor(Qt.OpenHandCursor)
        self.setMouseTracking(True)

        # ── view state (mirrors JS globals) ──
        self.yaw   =  0.45
        self.pitch = -0.25
        self.vd    =  0.55

        # ── interaction ──
        self._drag     = False
        self._last_pos = QPoint()
        self._hov      = -1          # hovered camera index

        # ── data from CalibState ──
        self._cameras      = []
        self._cam_detected = []
        self._board        = {}
        self._phase        = "capturing"
        # ── add this line ──
        self._board_world = None    # set after calibration via on_state()
        # ── toggle flags (controlled by sidebar checkboxes) ──
        self.show_frustums   = True
        self.show_board      = True
        self.show_axes       = True
        self.show_sightlines = True
        self.show_cam_board_dist = True    # ← this line was missing
    # ────────────────────────────────────────────────
    # Public slot — called by QtStateWatcher.state_changed
    # ────────────────────────────────────────────────
    # def on_state(self, data: dict):
    #     self._phase        = data.get("phase", "capturing")
    #     self._cam_detected = data.get("cam_detected", [])
    #     self._board        = data.get("board", {})

    #     # compute cam.pos = R^T @ -T  (same as JS)
    #     cams = []
    #     for c in data.get("cameras", []):
    #         R   = np.array(c["R"])
    #         T   = np.array(c["T"])
    #         pos = (R.T @ -np.array(T).flatten()).tolist()
    #         cams.append({**c, "pos": pos})
    #     self._cameras = cams
    #     self.update()   # trigger paintEvent
    # def on_state(self, data: dict):
    #     self._phase        = data.get("phase", "capturing")
    #     self._cam_detected = data.get("cam_detected", [])
    #     self._board        = data.get("board", {})

    #     # compute cam.pos = R^T @ -T
    #     cams = []
    #     for c in data.get("cameras", []):
    #         R   = np.array(c["R"])
    #         T   = np.array(c["T"])
    #         pos = (R.T @ -np.array(T).flatten()).tolist()
    #         cams.append({**c, "pos": pos})
    #     self._cameras = cams

    #     # ── new: compute real board center in world space ──
    #     # average board_world across all cameras for display
    #     board_positions = [
    #         c["board_world"] for c in self._cameras
    #         if "board_world" in c
    #     ]
    #     if board_positions:
    #         self._board_world = np.mean(board_positions, axis=0).tolist()
    #     else:
    #         self._board_world = None   # fallback — not yet calibrated

    #     self.update()
    def on_state(self, data: dict):
        self._phase        = data.get("phase", "capturing")
        self._cam_detected = data.get("cam_detected", [])
        self._board        = data.get("board", {})

        # compute cam.pos = R^T @ -T
        cams = []
        for c in data.get("cameras", []):
            R   = np.array(c["R"])
            T   = np.array(c["T"])
            pos = (R.T @ -np.array(T).flatten()).tolist()
            cams.append({**c, "pos": pos})
        self._cameras = cams

        # ── compute real board center in world space ──
        # use median instead of mean — ignores outlier cameras (e.g bad Cam1)
        board_positions = [
            c["board_world"] for c in self._cameras
            if "board_world" in c
        ]
        if board_positions:
            self._board_world = np.median(           # ← changed from np.mean
                board_positions, axis=0
            ).tolist()
        else:
            self._board_world = None

        self.update()

    # ────────────────────────────────────────────────
    # Preset views
    # ────────────────────────────────────────────────
    def set_view(self, name):
        if name == "front": self.yaw, self.pitch, self.vd = 0,    0,     0.55
        elif name == "top": self.yaw, self.pitch, self.vd = 0,   -math.pi/2+0.05, 0.55
        elif name == "iso": self.yaw, self.pitch, self.vd = 0.6, -0.3,   0.55
        else:               self.yaw, self.pitch, self.vd = 0.45,-0.25,  0.55
        self.update()

    # ────────────────────────────────────────────────
    # Projection — identical to JS proj()
    # ────────────────────────────────────────────────
    def proj(self, p):
        x, y, z = p
        x1 =  x * math.cos(self.yaw) + z * math.sin(self.yaw)
        z1 = -x * math.sin(self.yaw) + z * math.cos(self.yaw)
        y2 =  y * math.cos(self.pitch) - z1 * math.sin(self.pitch)
        z2 =  y * math.sin(self.pitch) + z1 * math.cos(self.pitch)
        f  = 560 / self.vd
        cx, cy = self.width() / 2, self.height() / 2
        dz = z2 + 2.5
        return (cx + x1 * f / dz, cy - y2 * f / dz)

    # ────────────────────────────────────────────────
    # Mouse interaction
    # ────────────────────────────────────────────────
    def mousePressEvent(self, e):
        if e.button() == Qt.LeftButton:
            self._drag = True
            self._last_pos = e.pos()
            self.setCursor(Qt.ClosedHandCursor)

    def mouseReleaseEvent(self, e):
        self._drag = False
        self.setCursor(Qt.OpenHandCursor)

    def mouseMoveEvent(self, e):
        if self._drag:
            dx = e.x() - self._last_pos.x()
            dy = e.y() - self._last_pos.y()
            self.yaw   += dx * 0.008
            self.pitch += dy * 0.008
            self.pitch  = max(-1.52, min(1.52, self.pitch))
            self._last_pos = e.pos()
            self.update()
            return

        # hover detection
        mx, my = e.x(), e.y()
        hov = -1
        for i, c in enumerate(self._cameras):
            px, py = self.proj(c["pos"])
            if abs(px - mx) < 14 and abs(py - my) < 14:
                hov = i
                break

        if hov != self._hov:
            self._hov = hov
            self.update()

        if hov >= 0:
            c  = self._cameras[hov]
            T  = c["T"]
            tm = [f"{v*1e3:.1f}" for v in (T if isinstance(T[0], float) else [t[0] for t in T])]
            pos = c["pos"]
            pm  = [f"{v*1e3:.1f}" for v in pos]
            QToolTip.showText(
                e.globalPos(),
                f"<b>{c['name']}</b><br>T: [{', '.join(tm)}] mm<br>Pos: [{', '.join(pm)}] mm"
            )
        else:
            QToolTip.hideText()

    def wheelEvent(self, e):
        self.vd *= 1 + e.angleDelta().y() * 0.001
        self.vd  = max(0.12, min(2.5, self.vd))
        self.update()

    def leaveEvent(self, e):
        self._hov = -1
        self.update()

    # ────────────────────────────────────────────────
    # Paint
    # ────────────────────────────────────────────────
    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor("#0d0d0d"))

        self._draw_grid(p)
        if self.show_axes:       self._draw_axes(p)
        if self.show_board:      self._draw_board(p)

        if not self._cameras:
            self._draw_pending(p)
            return

        if self.show_sightlines: self._draw_sightlines(p)
        if self.show_frustums:   self._draw_frustums(p)
        self._draw_cam_bodies(p)
        self._draw_dist_labels(p)

    # ── grid ──
    def _draw_grid(self, p):
        pen = QPen(QColor(80, 80, 80, 60))
        pen.setWidthF(0.5)
        p.setPen(pen)
        for i in range(-5, 6):
            ax, ay = self.proj([i * .05, 0, -.25])
            bx, by = self.proj([i * .05, 0,  .25])
            p.drawLine(int(ax), int(ay), int(bx), int(by))
            ax, ay = self.proj([-.25, 0, i * .05])
            bx, by = self.proj([ .25, 0, i * .05])
            p.drawLine(int(ax), int(ay), int(bx), int(by))

    # ── axes ──
    def _draw_axes(self, p):
        L = 0.045
        for end, col, label in [
            ([L, 0, 0], "#ff4444", "X"),
            ([0, L, 0], "#44ff44", "Y"),
            ([0, 0, L], "#4488ff", "Z"),
        ]:
            ox, oy = self.proj([0, 0, 0])
            ex, ey = self.proj(end)
            pen = QPen(QColor(col)); pen.setWidthF(1.5)
            p.setPen(pen)
            p.drawLine(int(ox), int(oy), int(ex), int(ey))
            lx, ly = self.proj([e * 1.2 for e in end])
            p.setPen(QColor(col))
            p.setFont(QFont("system-ui", 8))
            p.drawText(int(lx) - 5, int(ly) + 4, label)

    # ── charuco board ──
    # def _draw_board(self, p):
    #     bd = self._board
    #     if not bd or not bd.get("cols"):
    #         return
    #     bw = bd["cols"] * bd["square_length"]
    #     bh = bd["rows"] * bd["square_length"]
    #     bz = 0.3
    #     corners = [
    #         [-.5*bw, -.5*bh, bz], [.5*bw, -.5*bh, bz],
    #         [.5*bw,  .5*bh, bz],  [-.5*bw, .5*bh, bz],
    #     ]
    #     pts = [self.proj(c) for c in corners]

    #     from PyQt5.QtGui import QPolygonF
    #     from PyQt5.QtCore import QPointF
    #     poly = QPolygonF([QPointF(x, y) for x, y in pts])
    #     p.setBrush(QBrush(QColor(204, 170, 68, 25)))
    #     p.setPen(QPen(QColor("#CCAA44"), 1.5))
    #     p.drawPolygon(poly)

    #     # grid lines
    #     pen = QPen(QColor(204, 170, 68, 60)); pen.setWidthF(0.5)
    #     p.setPen(pen)
    #     for r in range(bd["rows"] + 1):
    #         t = r / bd["rows"]
    #         ax, ay = self.proj([-.5*bw, -.5*bh + t*bh, bz])
    #         bx, by = self.proj([ .5*bw, -.5*bh + t*bh, bz])
    #         p.drawLine(int(ax), int(ay), int(bx), int(by))
    #     for c in range(bd["cols"] + 1):
    #         t = c / bd["cols"]
    #         ax, ay = self.proj([-.5*bw + t*bw, -.5*bh, bz])
    #         bx, by = self.proj([-.5*bw + t*bw,  .5*bh, bz])
    #         p.drawLine(int(ax), int(ay), int(bx), int(by))

    #     cx, cy = self.proj([0, 0, bz])
    #     p.setPen(QColor("#CCAA44"))
    #     p.setFont(QFont("Arial", 8, QFont.Bold))
    #     p.drawText(int(cx) - 40, int(cy) - 13, "ChArUco Board")
    def _draw_board(self, p):
        bd = self._board
        if not bd or not bd.get("cols"):
            return

        bw = bd["cols"] * bd["square_length"]
        bh = bd["rows"] * bd["square_length"]

        # ── use real board world position if available,
        #    fallback to bz=0.3 during capture phase ──
        if self._board_world is not None:
            bx = self._board_world[0]
            by = self._board_world[1]
            bz = self._board_world[2]
        else:
            bx, by, bz = 0, 0, 0.3    # fallback during capture

        corners = [
            [bx - .5*bw, by - .5*bh, bz],
            [bx + .5*bw, by - .5*bh, bz],
            [bx + .5*bw, by + .5*bh, bz],
            [bx - .5*bw, by + .5*bh, bz],
        ]
        pts = [self.proj(c) for c in corners]

        from PyQt5.QtGui import QPolygonF
        from PyQt5.QtCore import QPointF
        poly = QPolygonF([QPointF(x, y) for x, y in pts])
        p.setBrush(QBrush(QColor(204, 170, 68, 25)))
        p.setPen(QPen(QColor("#CCAA44"), 1.5))
        p.drawPolygon(poly)

        # grid lines
        pen = QPen(QColor(204, 170, 68, 60))
        pen.setWidthF(0.5)
        p.setPen(pen)
        for r in range(bd["rows"] + 1):
            t = r / bd["rows"]
            ax, ay = self.proj([bx - .5*bw, by - .5*bh + t*bh, bz])
            bx2, by2 = self.proj([bx + .5*bw, by - .5*bh + t*bh, bz])
            p.drawLine(int(ax), int(ay), int(bx2), int(by2))
        for c in range(bd["cols"] + 1):
            t = c / bd["cols"]
            ax, ay = self.proj([bx - .5*bw + t*bw, by - .5*bh, bz])
            bx2, by2 = self.proj([bx - .5*bw + t*bw, by + .5*bh, bz])
            p.drawLine(int(ax), int(ay), int(bx2), int(by2))

        # label
        cx, cy = self.proj([bx, by, bz])
        p.setPen(QColor("#CCAA44"))
        p.setFont(QFont("Arial", 8, QFont.Bold))
        p.drawText(int(cx) - 40, int(cy) - 13, "ChArUco Board")

    # ── pending (before calibration) ──
    def _draw_pending(self, p):
        N = max(len(self._cam_detected), 1)
        for i in range(N):
            det = self._cam_detected[i] if i < len(self._cam_detected) else False
            th  = (i / N) * math.pi * 2 - math.pi / 2
            pos = [.12 * math.cos(th), 0, .12 * math.sin(th) + .3]
            px, py = self.proj(pos)
            col = QColor(COLORS[i % len(COLORS)]) if det else QColor("#444444")
            pen = QPen(col, 1, Qt.DashLine if not det else Qt.SolidLine)
            p.setPen(pen)
            p.setBrush(Qt.NoBrush)
            p.drawRect(int(px) - 8, int(py) - 5, 16, 10)
            p.setPen(QColor(COLORS[i % len(COLORS)]) if det else QColor("#555555"))
            p.setFont(QFont("Arial", 8, QFont.Bold))
            p.drawText(int(px) - 15, int(py) - 12, f"Cam {i}")

    # ── sight lines ──
    def _draw_sightlines(self, p):
        for c in self._cameras:
            col = QColor(c["color"])
            col.setAlpha(50)
            pen = QPen(col, 1, Qt.DashLine)
            p.setPen(pen)
            ax, ay = self.proj(c["pos"])
            bx, by = self.proj([0, 0, 0.3])
            p.drawLine(int(ax), int(ay), int(bx), int(by))

    # ── frustums ──
    def _draw_frustums(self, p):
        for i, c in enumerate(self._cameras):
            hov = (i == self._hov)
            K   = c["K"]
            fx, fy = K[0][0], K[1][1]
            cx_, cy_ = K[0][2], K[1][2]
            w   = c.get("imgW", 640)
            h   = c.get("imgH", 480)
            d   = 0.06
            R   = np.array(c["R"])
            T   = np.array(c["T"]).flatten()

            def iw(u, v, z):
                xc = (u - cx_) / fx * z
                yc = (v - cy_) / fy * z
                pc = np.array([xc, yc, z])
                return (R.T @ (pc - T)).tolist()

            apex = c["pos"]
            fps  = [iw(u, v, d) for u, v in [(0,0),(w,0),(w,h),(0,h)]]
            al   = 255 if hov else 150
            col  = QColor(c["color"]); col.setAlpha(al)
            lw   = 1.5 if hov else 0.7

            # apex → corners
            p.setPen(QPen(col, lw))
            ax, ay = self.proj(apex)
            for fp in fps:
                fx2, fy2 = self.proj(fp)
                p.drawLine(int(ax), int(ay), int(fx2), int(fy2))

            # rect edges
            for j in range(4):
                x1, y1 = self.proj(fps[j])
                x2, y2 = self.proj(fps[(j+1) % 4])
                p.drawLine(int(x1), int(y1), int(x2), int(y2))

            # fill face
            from PyQt5.QtGui import QPolygonF
            from PyQt5.QtCore import QPointF
            face_col = QColor(c["color"]); face_col.setAlpha(30)
            p.setBrush(QBrush(face_col))
            p.setPen(Qt.NoPen)
            poly = QPolygonF([QPointF(*self.proj(fp)) for fp in fps])
            p.drawPolygon(poly)
            p.setBrush(Qt.NoBrush)

    # ── camera bodies ──
    def _draw_cam_bodies(self, p):
        for i, c in enumerate(self._cameras):
            hov = (i == self._hov)
            px, py = self.proj(c["pos"])
            sz  = 11 if hov else 8
            col = QColor(c["color"])

            # body rect
            border = QColor("#ffffff") if hov else QColor(c["color"] + "aa")
            p.setPen(QPen(border, 2 if hov else 1))
            p.setBrush(QBrush(col))
            p.drawRect(int(px) - sz, int(py) - int(sz * .65), sz * 2, int(sz * 1.3))

            # lens circle
            p.setBrush(QBrush(QColor("#111111")))
            p.setPen(Qt.NoPen)
            p.drawEllipse(int(px) + sz, int(py) - int(sz * .42), int(sz * .84), int(sz * .84))

            # label
            p.setPen(col)
            p.setFont(QFont("Arial", 8, QFont.Bold))
            p.drawText(int(px) - 15, int(py) - sz - 5, c["name"])

    # ── distance labels ──
    # def _draw_dist_labels(self, p):
    #     if len(self._cameras) < 2:
    #         return
    #     c0 = self._cameras[0]["pos"]
    #     p.setFont(QFont("Arial", 8))
    #     for c in self._cameras[1:]:
    #         ci  = c["pos"]
    #         d   = math.sqrt(sum((a - b)**2 for a, b in zip(ci, c0)))
    #         mid = [(a + b) / 2 for a, b in zip(c0, ci)]
    #         mx, my = self.proj(mid)
    #         p.setPen(QColor(160, 160, 160, 220))
    #         p.drawText(int(mx) - 25, int(my) - 5, f"{d*1000:.1f} mm")
    def _draw_dist_labels(self, p):
        if len(self._cameras) < 2:
            return

        p.setFont(QFont("Arial", 8))

        # ── existing: cam0 → camN distances (grey) ──
        c0 = self._cameras[0]["pos"]
        for c in self._cameras[1:]:
            ci  = c["pos"]
            d   = math.sqrt(sum((a-b)**2 for a,b in zip(ci, c0)))
            mid = [(a+b)/2 for a,b in zip(c0, ci)]
            mx, my = self.proj(mid)
            p.setPen(QColor(160, 160, 160, 220))
            p.drawText(int(mx)-25, int(my)-5, f"{d*1000:.1f} mm")

        # ── new: each camera → its own real board position (yellow) ──
        if self.show_cam_board_dist:
            for c in self._cameras:
                if "board_world" not in c:
                    continue
                cam_pos    = c["pos"]
                board_pos  = c["board_world"]
                d = math.sqrt(
                    sum((a-b)**2 for a,b in zip(cam_pos, board_pos))
                )
                mid = [(a+b)/2 for a,b in zip(cam_pos, board_pos)]
                mx, my = self.proj(mid)
                p.setPen(QColor(200, 180, 80, 220))
                p.drawText(
                    int(mx)-25, int(my)-5,
                    f"{c['name']}→Board: {d*1000:.1f} mm"
                )