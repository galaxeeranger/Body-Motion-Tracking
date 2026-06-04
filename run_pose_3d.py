
"""
run_pose_3d.py  —  Multi-camera real-time 3D human pose estimation
===================================================================
Works with N cameras (minimum 2).

Controls (3D window):
    Left-drag   → orbit camera
    Scroll      → zoom
    ESC         → quit
"""

import time
import cv2
import numpy as np
import mediapipe as mp

from triangulation.triangulator import Triangulator
from utils.visualizer_3d        import Simple3DVisualizer   # NEW OpenGL visualizer
from utils.smoothing            import SkeletonSmoother
from calib.capture              import _CameraThread        # threaded DSHOW capture (no hang)
from config import (
    USE_RANSAC_TRIANGULATION, RANSAC_REPROJ_PX, RANSAC_MIN_INLIERS,
    SMOOTH_FREQ, SMOOTH_MINCUTOFF, SMOOTH_BETA, SMOOTH_MAX_JUMP_M,
)

# ─────────────────────────────────────────────────────────────────
# 1. Load calibration
# ─────────────────────────────────────────────────────────────────
data = np.load("output/multi_cam_calibration.npz", allow_pickle=True)

K_list    = [np.array(K, dtype=np.float64) for K in data["K_list"]]
R_list    = [np.array(R, dtype=np.float64) for R in data["R_list"]]
T_list    = [np.array(T, dtype=np.float64) for T in data["T_list"]]
# distortion is REQUIRED for correct triangulation on real lenses
dist_list = [np.array(d, dtype=np.float64) for d in data["dist_list"]]

num_cams = len(K_list)
print(f"Loaded calibration for {num_cams} cameras")

# ─────────────────────────────────────────────────────────────────
# 2. Triangulator  (distortion-aware + robust RANSAC)
# ─────────────────────────────────────────────────────────────────
triangulator = Triangulator(
    K_list, R_list, T_list,
    dist_list=dist_list,
    min_views=2,
    conf_thresh=0.3,
    use_ransac=USE_RANSAC_TRIANGULATION,
    ransac_reproj_px=RANSAC_REPROJ_PX,
    ransac_min_inliers=RANSAC_MIN_INLIERS,
)

# ─────────────────────────────────────────────────────────────────
# 3. Open cameras  (threaded + DirectShow, same as calibration capture)
# ─────────────────────────────────────────────────────────────────
# Plain cv2.VideoCapture(i) + back-to-back cap.read() hangs on Windows:
# the default MSMF backend blocks indefinitely, and opening all USB cameras
# at once overloads the bus. _CameraThread uses CAP_DSHOW with a staggered
# start and a background reader loop, so reads never block the main thread.
cam_threads = [_CameraThread(i) for i in range(num_cams)]
for ct in cam_threads:
    ct.start()

# Wait until every camera has decoded enough frames (flushes warmup/black
# frames; cam(N-1) needs ~N seconds of staggered init before it streams).
print("Warming up cameras...")
deadline = time.time() + 20.0
while time.time() < deadline:
    counts = [ct.frame_count for ct in cam_threads]
    if all(c >= 10 for c in counts):
        break
    ready = sum(1 for c in counts if c >= 10)
    print(f"  Cameras ready: {ready}/{num_cams}  frames={counts}")
    time.sleep(0.5)
else:
    counts = [ct.frame_count for ct in cam_threads]
    print(f"WARNING: not all cameras ready after 20s — frames={counts}")
print("Cameras ready.")

# ─────────────────────────────────────────────────────────────────
# 4. MediaPipe pose detectors  (one per camera)
# ─────────────────────────────────────────────────────────────────
mp_pose = mp.solutions.pose
mp_draw  = mp.solutions.drawing_utils

pose_detectors = [
    mp_pose.Pose(
        static_image_mode=False,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5
    )
    for _ in range(num_cams)
]

def extract_keypoints(results, frame_shape):
    """
    Returns (33, 3) → (x_pixel, y_pixel, visibility_confidence).
    Zeros if no detection.
    """
    h, w   = frame_shape[:2]
    kps    = np.zeros((33, 3), dtype=np.float32)
    if results.pose_landmarks:
        for idx, lm in enumerate(results.pose_landmarks.landmark):
            kps[idx] = [lm.x * w, lm.y * h, lm.visibility]
    return kps

# ─────────────────────────────────────────────────────────────────
# 5. Visualizer + temporal smoother
# ─────────────────────────────────────────────────────────────────
viz = Simple3DVisualizer()

smoother = SkeletonSmoother(
    num_joints=33,
    freq=SMOOTH_FREQ,
    mincutoff=SMOOTH_MINCUTOFF,
    beta=SMOOTH_BETA,
    max_jump_m=SMOOTH_MAX_JUMP_M,
)

# ─────────────────────────────────────────────────────────────────
# 6. Main loop
# ─────────────────────────────────────────────────────────────────
print("Running — ESC in 3D window or 'q' in camera view to quit.")

while True:
    frames           = []
    keypoints_2d_all = []

    # ── read latest frame from every camera (non-blocking) ──
    for i, ct in enumerate(cam_threads):
        ret, frame = ct.get_latest()
        if not ret or frame is None:
            print(f"Camera {i} read failed — exiting")
            break
        frames.append(frame)
    else:
        # ── MediaPipe inference on each camera ──
        for i in range(num_cams):
            rgb     = cv2.cvtColor(frames[i], cv2.COLOR_BGR2RGB)
            results = pose_detectors[i].process(rgb)

            kps = extract_keypoints(results, frames[i].shape)
            keypoints_2d_all.append(kps)

            # draw skeleton overlay on camera feed
            mp_draw.draw_landmarks(
                frames[i],
                results.pose_landmarks,
                mp_pose.POSE_CONNECTIONS
            )

        # ── triangulate → 3D joints (robust RANSAC) ──
        points_3d = triangulator.triangulate_frame(keypoints_2d_all)

        # ── temporal smoothing + jump rejection ──
        points_3d = smoother.update(points_3d)

        # ── update 3D visualizer ──
        alive = viz.update(points_3d)
        if not alive:
            break

        # ── show camera feeds ──
        for i, frame in enumerate(frames):
            cv2.imshow(f"Cam {i}", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord('q'):   # ESC or Q
            break
        continue

    break   # inner break propagates here

# ─────────────────────────────────────────────────────────────────
# 7. Cleanup
# ─────────────────────────────────────────────────────────────────
for ct in cam_threads:
    ct.stop()
cv2.destroyAllWindows()
viz.close()
print("Done.")