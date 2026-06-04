
"""
visualizer_3d.py  —  Real-time 3D Human Pose Visualizer (OpenGL)
=================================================================
Drop-in replacement for Simple3DVisualizer (matplotlib).

Requirements:
    pip install PyOpenGL PyOpenGL_accelerate glfw numpy

Usage:
    viz = Simple3DVisualizer()
    alive = viz.update(points_3d)   # (33,3) array, NaN = missing
    if not alive: break

Controls:
    Left-drag  → orbit
    Scroll     → zoom
    ESC        → quit
"""

import math
import numpy as np
import glfw
from OpenGL.GL import *
from OpenGL.GLU import *


# ─────────────────────────────────────────────────────────────────
# MediaPipe 33-joint skeleton connections
# ─────────────────────────────────────────────────────────────────
POSE_CONNECTIONS = [
    (0,1),(1,2),(2,3),(3,7),
    (0,4),(4,5),(5,6),(6,8),
    (11,12),(11,23),(12,24),(23,24),
    (11,13),(13,15),(12,14),(14,16),
    (15,17),(15,19),(15,21),(16,18),(16,20),(16,22),
    (23,25),(25,27),(24,26),(26,28),
    (27,29),(29,31),(28,30),(30,32),
]

FACE_IDS  = {0,1,2,3,4,5,6,7,8}
TORSO_IDS = {11,12,23,24}
ARM_L     = {11,13,15,17,19,21}
ARM_R     = {12,14,16,18,20,22}
LEG_L     = {23,25,27,29,31}
LEG_R     = {24,26,28,30,32}

def _bone_color(i, j):
    if i in FACE_IDS  and j in FACE_IDS:  return (0.90, 0.75, 0.55)
    if i in TORSO_IDS and j in TORSO_IDS: return (0.95, 0.55, 0.20)
    if i in ARM_L     or  j in ARM_L:     return (0.85, 0.40, 0.15)
    if i in ARM_R     or  j in ARM_R:     return (0.85, 0.40, 0.15)
    if i in LEG_L     or  j in LEG_L:     return (0.80, 0.35, 0.10)
    if i in LEG_R     or  j in LEG_R:     return (0.80, 0.35, 0.10)
    return (0.90, 0.50, 0.20)


# ─────────────────────────────────────────────────────────────────
# GL drawing primitives
# ─────────────────────────────────────────────────────────────────

def _sphere(cx, cy, cz, r=0.030):
    glPushMatrix()
    glTranslatef(float(cx), float(cy), float(cz))
    q = gluNewQuadric()
    gluSphere(q, r, 14, 14)
    gluDeleteQuadric(q)
    glPopMatrix()


def _cylinder(p1, p2, r=0.025):
    dx = float(p2[0]-p1[0])
    dy = float(p2[1]-p1[1])
    dz = float(p2[2]-p1[2])
    L  = math.sqrt(dx*dx + dy*dy + dz*dz)
    if L < 1e-6:
        return
    glPushMatrix()
    glTranslatef(float(p1[0]), float(p1[1]), float(p1[2]))
    ax = -dy; ay = dx
    ang = math.degrees(math.acos(max(-1.0, min(1.0, dz / L))))
    if abs(ax) < 1e-6 and abs(ay) < 1e-6:
        if dz < 0:
            glRotatef(180.0, 1, 0, 0)
    else:
        glRotatef(ang, ax, ay, 0)
    q = gluNewQuadric()
    gluCylinder(q, r, r, L, 12, 1)
    gluDeleteQuadric(q)
    glPopMatrix()


def _axis_gizmo(L=0.25):
    for color, tip in [
        ((1.0, 0.1, 0.1), (L, 0, 0)),
        ((0.1, 0.9, 0.1), (0, L, 0)),
        ((0.2, 0.4, 1.0), (0, 0, L)),
    ]:
        glColor3f(*color)
        _cylinder((0,0,0), tip, r=0.007)
        _sphere(*tip, r=0.018)


def _checkered_floor(size=4.0, tiles=12):
    step = size / tiles
    half = size / 2.0
    for row in range(tiles):
        for col in range(tiles):
            x0 = -half + col * step
            z0 = -half + row * step
            x1 = x0 + step
            z1 = z0 + step
            if (row + col) % 2 == 0:
                glColor3f(0.82, 0.82, 0.82)
            else:
                glColor3f(0.15, 0.15, 0.15)
            glBegin(GL_QUADS)
            glNormal3f(0, 1, 0)
            glVertex3f(x0, 0.0, z0)
            glVertex3f(x1, 0.0, z0)
            glVertex3f(x1, 0.0, z1)
            glVertex3f(x0, 0.0, z1)
            glEnd()


def _room_walls(size=4.0, height=2.8):
    """
    3 solid walls (back, left, right) + ceiling, each as a filled quad
    with a semi-transparent tint so the room box is clearly visible.
    """
    s = size / 2.0

    # (corners CCW from inside, normal pointing inward)
    faces = [
        # back wall z = -s
        ([(-s,0,-s),(s,0,-s),(s,height,-s),(-s,height,-s)], (0,0,1)),
        # left wall x = -s
        ([(-s,0,s),(-s,0,-s),(-s,height,-s),(-s,height,s)],  (1,0,0)),
        # right wall x = +s
        ([(s,0,-s),(s,0,s),(s,height,s),(s,height,-s)],       (-1,0,0)),
        # ceiling y = height
        ([(-s,height,s),(s,height,s),(s,height,-s),(-s,height,-s)], (0,-1,0)),
    ]

    # ── filled semi-transparent faces ──
    glDisable(GL_CULL_FACE)
    glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
    glEnable(GL_LIGHTING)
    glColor4f(0.82, 0.85, 0.90, 0.22)

    for verts, norm in faces:
        glBegin(GL_QUADS)
        glNormal3f(*norm)
        for v in verts:
            glVertex3f(*v)
        glEnd()

    # ── edge outlines ──
    glDisable(GL_LIGHTING)
    glColor3f(0.55, 0.55, 0.60)
    glLineWidth(1.6)

    for verts, _ in faces:
        glBegin(GL_LINE_LOOP)
        for v in verts:
            glVertex3f(*v)
        glEnd()

    # vertical corner lines
    for (x, z) in [(-s,-s),(s,-s),(s,s),(-s,s)]:
        glBegin(GL_LINES)
        glVertex3f(x, 0,      z)
        glVertex3f(x, height, z)
        glEnd()

    glEnable(GL_LIGHTING)


# ─────────────────────────────────────────────────────────────────
# Main Visualizer
# ─────────────────────────────────────────────────────────────────

class Simple3DVisualizer:

    WIN_W, WIN_H = 960, 720

    def __init__(self):
        if not glfw.init():
            raise RuntimeError("GLFW init failed")

        glfw.window_hint(glfw.SAMPLES, 4)
        self.window = glfw.create_window(
            self.WIN_W, self.WIN_H, "3D Motion — Human Pose", None, None
        )
        if not self.window:
            glfw.terminate()
            raise RuntimeError("GLFW window creation failed")

        glfw.make_context_current(self.window)
        glfw.swap_interval(1)

        self._azim  =  35.0
        self._elev  =  22.0
        self._dist  =   5.0
        self._drag  = False
        self._last_mouse = (0, 0)
        self._debug_n    = 0      # print first 5 frames to terminal

        glfw.set_mouse_button_callback(self.window, self._cb_mouse)
        glfw.set_cursor_pos_callback(self.window,   self._cb_cursor)
        glfw.set_scroll_callback(self.window,        self._cb_scroll)
        glfw.set_key_callback(self.window,           self._cb_key)

        self._init_gl()
        

    # ── GL init ───────────────────────────────
    def _init_gl(self):
        glEnable(GL_DEPTH_TEST)
        glDepthFunc(GL_LEQUAL)
        glEnable(GL_MULTISAMPLE)
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glEnable(GL_COLOR_MATERIAL)
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)
        glShadeModel(GL_SMOOTH)
        glLightfv(GL_LIGHT0, GL_POSITION, [3.0, 7.0, 5.0, 1.0])
        glLightfv(GL_LIGHT0, GL_DIFFUSE,  [1.0, 0.98, 0.95, 1.0])
        glLightfv(GL_LIGHT0, GL_AMBIENT,  [0.45, 0.45, 0.45, 1.0])

    # ── input ─────────────────────────────────
    def _cb_mouse(self, win, btn, act, mods):
        if btn == glfw.MOUSE_BUTTON_LEFT:
            self._drag = (act == glfw.PRESS)
            self._last_mouse = glfw.get_cursor_pos(win)

    def _cb_cursor(self, win, x, y):
        if self._drag:
            dx = x - self._last_mouse[0]
            dy = y - self._last_mouse[1]
            self._azim += dx * 0.4
            self._elev  = max(-89, min(89, self._elev - dy * 0.4))
            self._last_mouse = (x, y)

    def _cb_scroll(self, win, xoff, yoff):
        self._dist = max(0.8, min(15.0, self._dist - yoff * 0.3))

    def _cb_key(self, win, key, sc, act, mods):
        if key == glfw.KEY_ESCAPE and act == glfw.PRESS:
            glfw.set_window_should_close(win, True)

    # ── camera ────────────────────────────────
    def _set_camera(self):
        w, h = glfw.get_framebuffer_size(self.window)
        glViewport(0, 0, w, h)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45.0, w / max(h, 1), 0.05, 200.0)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

        ra = math.radians(self._azim)
        re = math.radians(self._elev)
        d  = self._dist
        ex = d * math.cos(re) * math.sin(ra)
        ey = d * math.sin(re)
        ez = d * math.cos(re) * math.cos(ra)
        look_y = 1.0   # look at hip-height
        gluLookAt(ex, ey + look_y, ez,
                  0,  look_y,      0,
                  0,  1,           0)
        
    # ── skeleton ──────────────────────────────
    def _draw_skeleton(self, pts):
        valid = ~np.isnan(pts).any(axis=1)
        if np.sum(valid) < 4:
            return

        bone_r  = 0.025
        joint_r = 0.032

        for (i, j) in POSE_CONNECTIONS:
            if i >= len(pts) or j >= len(pts): continue
            if not valid[i] or not valid[j]:   continue
            glColor3f(*_bone_color(i, j))
            _cylinder(pts[i], pts[j], r=bone_r)

        for k in range(len(pts)):
            if not valid[k]: continue
            r = joint_r * 1.4 if k in (0, 11, 12, 23, 24) else joint_r
            glColor3f(0.98, 0.88, 0.68)
            _sphere(*pts[k], r=r)

    # ── coordinate normalisation ──────────────
    def _normalise(self, raw):
        """
        Works for any unit (m / cm / mm) and any camera orientation.

        1. Flip Y  (image Y is downward; world Y should be upward)
        2. Auto-scale so person height ≈ 1.7 world units
        3. Shift feet to Y = 0
        4. Centre on X / Z
        """
        pts = np.array(raw, dtype=np.float64)

        # 1. flip Y
        pts[:, 1] = -pts[:, 1]
        # ✅ flip X to fix mirror
        pts[:, 0] = -pts[:, 0]
        pts[:, 2] = -pts[:, 2]

        valid = ~np.isnan(pts).any(axis=1)
        if np.sum(valid) < 4:
            return pts

        # 2. auto-scale
        y_vals = pts[valid, 1]
        h_range = float(np.nanmax(y_vals) - np.nanmin(y_vals))
        if h_range > 1e-4:
            scale = 1.7 / h_range
            pts   = pts * scale

        # re-check valid after scale
        valid = ~np.isnan(pts).any(axis=1)

        # 3. floor feet at Y=0
        foot_ids = [27, 28, 29, 30, 31, 32]
        foot_ok  = [k for k in foot_ids if valid[k]]
        
        
        ref_y    = np.nanmin(pts[foot_ok, 1]) if foot_ok else np.nanmin(pts[valid, 1])
        pts[:, 1] -= ref_y

        # 4. centre X/Z
        pts[:, 0] -= np.nanmean(pts[valid, 0])
        pts[:, 2] -= np.nanmean(pts[valid, 2])

        return pts

    # ── public update ─────────────────────────
    def update(self, points_3d):
        """
        Render one frame.  Call this in your main loop.

        Parameters
        ----------
        points_3d : np.ndarray (33, 3)
            Triangulated joints.  NaN = joint not visible.

        Returns
        -------
        bool — False when the window is closed (break your loop).
        """
        if glfw.window_should_close(self.window):
            self._cleanup()
            return False

        glfw.make_context_current(self.window)
        glfw.poll_events()

        glClearColor(0.95, 0.95, 0.96, 1.0)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        self._set_camera()

        # ── floor (no lighting so checker is full contrast) ──
        glDisable(GL_LIGHTING)
        _checkered_floor(size=4.0, tiles=12)
     
        glEnable(GL_LIGHTING)

        # ── room walls ──
        _room_walls(size=4.0, height=2.8)
        # ── axis gizmo ──
        glDisable(GL_LIGHTING)
        _axis_gizmo(L=0.25)
        glEnable(GL_LIGHTING)

        # ── skeleton ──
        if points_3d is not None:
            # debug first 5 frames
            if self._debug_n < 5:
                v = ~np.isnan(points_3d).any(axis=1)
                if v.any():
                    p = points_3d[v]
                    print(f"[viz dbg {self._debug_n}] valid={v.sum()}/33 "
                          f"X=[{p[:,0].min():.3f},{p[:,0].max():.3f}] "
                          f"Y=[{p[:,1].min():.3f},{p[:,1].max():.3f}] "
                          f"Z=[{p[:,2].min():.3f},{p[:,2].max():.3f}]")
                self._debug_n += 1

            pts = self._normalise(points_3d)
            # add here
            # # ✅ update camera target to follow person
            # valid = ~np.isnan(pts).any(axis=1)
            # if valid.any():
            #     self._target[0] = float(np.nanmean(pts[valid, 0]))
            #     self._target[2] = float(np.nanmean(pts[valid, 2])) 
                
            # # end here
            self._draw_skeleton(pts)

        glfw.swap_buffers(self.window)
        return True

    # ── cleanup ───────────────────────────────
    def _cleanup(self):
        glfw.destroy_window(self.window)
        glfw.terminate()

    def close(self):
        self._cleanup()