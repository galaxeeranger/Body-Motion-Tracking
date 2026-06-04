"""
Temporal smoothing for triangulated 3D skeletons  (SCALING_8_CAMERAS.md §2.4)

Provides:
    - OneEuroFilter   : low-lag adaptive smoothing of a scalar signal
    - SkeletonSmoother: per-joint One-Euro smoothing + physical jump rejection

Why this exists
---------------
With 8+ surround cameras a joint is occasionally triangulated from a bad view
(e.g. a back camera that swapped left/right). RANSAC removes most of these, but
a single-frame outlier still slips through and shows up as a sudden "jump".
This module:
    1. rejects physically impossible jumps between consecutive frames, and
    2. smooths the remaining jitter with a One-Euro filter (fast when the joint
       moves, smooth when it is still).
"""

import numpy as np


class _LowPass:
    """Exponential low-pass with NaN-safe initialisation."""

    def __init__(self):
        self.y = None

    def __call__(self, x, alpha):
        if self.y is None or not np.isfinite(self.y):
            self.y = float(x)
        else:
            self.y = alpha * x + (1.0 - alpha) * self.y
        return self.y


class OneEuroFilter:
    """
    One-Euro filter (Casiez et al. 2012) for a single scalar coordinate.

    mincutoff : lower = smoother but more lag when still
    beta      : higher = less lag during fast motion
    """

    def __init__(self, freq=30.0, mincutoff=1.0, beta=0.3, dcutoff=1.0):
        self.freq      = float(freq)
        self.mincutoff = float(mincutoff)
        self.beta      = float(beta)
        self.dcutoff   = float(dcutoff)
        self._x        = _LowPass()
        self._dx       = _LowPass()
        self._prev     = None

    def _alpha(self, cutoff):
        tau = 1.0 / (2.0 * np.pi * cutoff)
        te  = 1.0 / self.freq
        return 1.0 / (1.0 + tau / te)

    def __call__(self, x):
        if not np.isfinite(x):
            # nothing to update with — return last smoothed value (may be None)
            return self._x.y if self._x.y is not None else np.nan

        prev = self._prev if self._prev is not None else x
        dx   = (x - prev) * self.freq
        edx  = self._dx(dx, self._alpha(self.dcutoff))
        cutoff = self.mincutoff + self.beta * abs(edx)
        out  = self._x(x, self._alpha(cutoff))
        self._prev = x
        return out


class SkeletonSmoother:
    """
    Smooths a (num_joints, 3) world-space skeleton over time.

    update(points_3d) -> smoothed (num_joints, 3)

    NaN joints (seen by too few cameras this frame) are passed through as the
    last known value (or NaN if never seen). Joints that move more than
    `max_jump_m` between frames are treated as outliers and held at the previous
    value for one frame.
    """

    def __init__(self, num_joints, freq=30.0, mincutoff=1.0,
                 beta=0.3, max_jump_m=0.30):
        self.num_joints = num_joints
        self.max_jump   = float(max_jump_m)
        self._filters = [
            [OneEuroFilter(freq, mincutoff, beta) for _ in range(3)]
            for _ in range(num_joints)
        ]
        self._last = np.full((num_joints, 3), np.nan, dtype=np.float64)

    def update(self, points_3d):
        pts = np.asarray(points_3d, dtype=np.float64)
        out = np.full((self.num_joints, 3), np.nan, dtype=np.float64)

        for j in range(self.num_joints):
            p = pts[j]

            # joint not triangulated this frame -> hold last known
            if not np.all(np.isfinite(p)):
                out[j] = self._last[j]
                continue

            last = self._last[j]

            # physical jump rejection
            if np.all(np.isfinite(last)) and \
               np.linalg.norm(p - last) > self.max_jump:
                out[j] = last          # reject outlier, hold previous
                continue

            sm = np.array(
                [self._filters[j][k](p[k]) for k in range(3)],
                dtype=np.float64
            )
            out[j] = sm
            self._last[j] = sm

        return out
