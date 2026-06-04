# Multi-Camera Calibration & 3D Mocap — Technical Documentation

This document explains **what approach** and **what logic** the `multi_cam_calib`
project uses. It is written so a new engineer can understand the *why* behind
each stage, not just the *what*.

---

## 1. Goal of the Project

The system has two jobs:

1. **Calibrate** an arbitrary number of cameras (default 3) so that we know,
   for every camera:
   - its **intrinsics** `K` (focal length, principal point) and lens
     **distortion** coefficients, and
   - its **extrinsics** `R, T` (where the camera sits and how it is rotated)
     in one shared **world coordinate frame**.

2. **Reconstruct 3D human pose** in real time: each camera runs a 2D pose
   detector (MediaPipe), and the matched 2D joints are **triangulated** into
   a single 3D skeleton using the calibration from step 1.

The whole thing is metric (real-world meters) because calibration is anchored
to a physical **ChArUco board** of known square size.

---

## 2. Why ChArUco (the core design choice)

A **ChArUco board** is a chessboard fused with ArUco markers in the white
squares. It is the foundation of the entire approach because it gives us three
things a plain chessboard cannot:

| Property | Why it matters here |
|---|---|
| **Unique IDs per corner** | Each chessboard corner has a global ID. When two cameras see the board from different angles, we can match *the same physical corner* across cameras by ID — no ambiguous ordering. |
| **Partial-view robustness** | Even if a camera only sees part of the board (occlusion, edge of frame), the visible markers still resolve their IDs. A plain chessboard needs the *whole* board visible. |
| **Known metric geometry** | `SQUARE_LENGTH = 0.04 m` is the physical truth. This injects real-world scale, so the reconstruction is in meters, not arbitrary units. |

The board is defined once in `calib/board.py`:

```python
aruco.CharucoBoard((cols, rows), square_length, marker_length, DICT_4X4_250)
```

`board.getChessboardCorners()` later gives us the **3D position of every corner
in board coordinates** — this is the "ground truth" object geometry used by
stereo calibration and bundle adjustment.

---

## 3. The Calibration Pipeline (6 Stages)

The orchestrator is `calib/pipeline.py → CalibPipeline.run()`. It sequences six
stages. The high-level strategy is:

> **Calibrate each camera alone → calibrate pairs → chain pairs into one world
> frame → (optionally) globally refine → save.**

```
Stage 1  Capture          (collect synchronized ChArUco detections)
Stage 2  Intrinsics       (K + distortion, per camera, independently)
Stage 3  Stereo pairs     (relative R,T between every overlapping camera pair)
Stage 4  Pose graph (BFS) (chain pairwise transforms into ONE world frame)
Stage 5  Bundle adjust    (global refinement — currently DISABLED)
Stage 6  Save             (.npz + push to 3D viewer)
```

### Stage 1 — Capture (`calib/capture.py`)

**Approach: threaded, lag-free, pairwise-aware capture.**

- **One thread per camera** (`_CameraThread`). Each thread sits in a tight
  `grab → decode → store latest frame` loop. The main loop only ever reads the
  *most recent* decoded frame via `get_latest()`.
  - **Why:** OpenCV buffers 10+ frames internally. With long USB cables the
    main thread would otherwise read frames that are 300–500 ms stale. The
    reader thread continuously drains the buffer so the main thread always gets
    a fresh frame.
- **Staggered camera startup** (`time.sleep(1 + cam_idx)`) plus a warm-up gate
  that waits until *every* camera has decoded ≥10 frames.
  - **Why:** Windows DirectShow (`CAP_DSHOW`) returns black/garbage frames
    during negotiation, and all cameras hitting the USB bus simultaneously
    causes dropouts. Staggering + flushing avoids capturing junk.
- **Format is forced once** (MJPG, 640×480, 15 fps, buffersize 1) before the
  thread starts, to stop DSHOW renegotiating mid-stream.

**Capture logic:**
- Every loop, each camera's frame is detected for ChArUco corners.
- **Auto-capture rule:** if **≥2 cameras** see the board *and* `CAPTURE_DELAY`
  (2 s) has passed since the last capture, the frame is stored for **all**
  cameras and the overlap is recorded.
- The 2-second delay forces the user to move the board to a new pose between
  captures → diverse viewpoints → better calibration.

**Key data structures produced:**
```
all_charuco_corners[cam][frame]   # 2D corner pixel positions
all_charuco_ids[cam][frame]       # which corner IDs those are
frame_mask[cam][frame]            # did this cam see the board this frame?
overlap_pairs[(i,j)] = [frame...] # frames where BOTH cam i and j saw the board
```

**Coverage check:** at the end it builds an adjacency graph from camera pairs
that share ≥ `MIN_PAIR_FRAMES` (10) frames and runs a BFS from Cam0 to verify
**every camera is reachable**. If a camera has no overlap path to Cam0, it
cannot be placed in the world frame — this is reported as a warning.

### Stage 2 — Intrinsic Calibration (`calib/intrinsic.py`)

**Approach: calibrate each camera independently** with
`cv2.aruco.calibrateCameraCharuco`.

- For each camera, **filter invalid frames** first: drop any frame where
  corners/ids are `None`, fewer than 4 corners, or corner/id counts mismatch.
  Require ≥5 valid frames or raise an error.
  - **Why:** OpenCV's calibration crashes or returns garbage on malformed
    frames. This guard is the difference between a robust run and a hard crash.
- Output per camera: `K` (3×3 intrinsic matrix) and `dist` (distortion coeffs).

> Note in `pipeline.py`: in a real room you'd run intrinsics **once per camera
> beforehand** and load them. Here they're computed from the same session for
> simplicity.

### Stage 3 — Pairwise Stereo Calibration (`calib/stereo.py`)

**Approach: for every viable camera pair `(i, j)`, compute the rigid transform
that maps camera j's frame into camera i's frame.**

For each pair sharing ≥10 frames:
1. Walk their shared frames. In each frame, find the **common corner IDs** seen
   by *both* cameras (`np.intersect1d`). Require ≥6 common corners.
2. Build three matched lists:
   - `objpoints` — the **3D board position** of each common corner
     (from `board.getChessboardCorners()`),
   - `imgpoints_i`, `imgpoints_j` — the **2D pixel** positions in each camera.
3. Run `cv2.stereoCalibrate(..., flags=CALIB_FIX_INTRINSIC)`.
   - **`CALIB_FIX_INTRINSIC`** = trust the Stage-2 `K`/`dist`, solve *only* for
     the relative `R, T` between the two cameras. This is more stable and is
     why intrinsics are done first.

**Output:** `pair_transforms[(i,j)] = {R, T, rms}` where `R, T` is the pose of
camera j expressed in camera i's coordinate frame, plus the RMS reprojection
error in pixels (the quality metric).

Because matching is done by **corner ID**, the cameras do **not** need to see
the same corners in the same order — the IDs solve the correspondence problem.

### Stage 4 — Pose Graph / BFS (`calib/graph.py`)

**Approach: chain the pairwise transforms into ONE consistent world frame using
a breadth-first spanning tree rooted at Camera 0.**

The problem: Stage 3 gives us *relative* poses (cam0↔cam1, cam1↔cam2, …). We
need *absolute* poses in a single frame. We pick **Camera 0 as the world
origin**:

```
R_world[0] = I,   T_world[0] = 0
```

Then:
1. Build an adjacency map. For each pair `(i,j)` we store both directions —
   the direct transform `i→j` and its **inverse** `j→i`
   (`R_inv = Rᵀ`, `T_inv = -Rᵀ·T`).
2. **BFS from Camera 0.** When we reach a new camera `dst` from an already-placed
   camera `src`, we **compose** transforms to place `dst` in the world:
   ```
   R_world[dst] = R_world[src] @ R_src_dst
   T_world[dst] = R_world[src] @ T_src_dst + T_world[src]
   ```
   This is the standard rigid-transform chain rule:
   `X_world = R_world_k · X_cam_k + T_world_k`.
3. Any camera not reachable from Cam0 is filled with identity (so downstream
   code doesn't crash) and flagged.

**Why BFS specifically:** BFS gives the **shortest chain** from Cam0 to every
camera. Each stereo hop adds a little error, so shortest path = least
accumulated error. The convention is `X_world = R·X_cam + T`, i.e. `R, T` are
**camera-from-world extrinsics** — important because the triangulator builds
`P = K[R|T]` directly with the same convention.

### Stage 5 — Bundle Adjustment (`calib/bundle_adjust.py`) — *DISABLED*

**Approach (when enabled): one global non-linear optimization that refines all
camera poses simultaneously to minimize total reprojection error.**

The BFS result accumulates error down the chain — cameras far from Cam0 drift.
Bundle Adjustment (BA) fixes this by treating all cameras symmetrically:

- **What is optimized:** only the camera poses (cams 1..N as Rodrigues rotation
  vectors + translation; Cam0 stays fixed as the world anchor).
- **What is held fixed:** the 3D board points use their **known geometry**
  (`board.getChessboardCorners()`) rather than being free variables.
  - **Why this is critical:** the file's docstring calls it "Fixed Bundle
    Adjustment." If you let the 3D points float freely, the optimizer can shrink
    or inflate the whole scene (**scale drift / explosion**). Anchoring to the
    physical board keeps metric scale locked.
- **Residual:** for every observed corner,
  `project(K, R, T, X_board) − observed_2d_pixel`. 2D points are undistorted
  first so the projection model needs no distortion term. Points behind the
  camera get a large penalty (1000).
- **Solver:** `scipy.optimize.least_squares` with `method="trf"`
  (Trust Region Reflective — more stable than Levenberg–Marquardt here).

**Current state:** Stage 5 is **enabled** in `pipeline.py`, gated by
`USE_BUNDLE_ADJUST` in `config.py`. It was re-enabled for multi-camera scaling:
BFS chaining accumulates error around a camera ring, and global BA is the only
stage that distributes it evenly. BA falls back to the BFS poses automatically
if it has too few observations, so it is safe at any camera count.

### Stage 6 — Save (`calib/saver.py`)

Writes everything to `output/multi_cam_calibration.npz`:
`K_list, dist_list, R_list (world), T_list (world)`.

It also builds a `cam_data` dict (position in mm, color, image size, etc.) and
pushes it into the shared state so the **3D viewer** can draw the camera
frustums live.

---

## 4. The 3D Reconstruction Path (`run_pose_3d.py`)

This is a **separate, standalone script** run *after* calibration. Its logic:

1. **Load** `multi_cam_calibration.npz` → `K_list, R_list, T_list`.
2. Create a `Triangulator` (see below).
3. Open all cameras, warm up auto-exposure (30 throwaway frames).
4. Create **one MediaPipe Pose detector per camera**.
5. **Main loop, per frame:**
   - Read every camera, run MediaPipe → 33 body landmarks each, stored as
     `(x_pixel, y_pixel, visibility)`.
   - Pass all cameras' keypoints to `triangulator.triangulate_frame(...)`.
   - Update the OpenGL 3D skeleton view; show 2D overlays per camera.
   - ESC / `q` to quit.

### Triangulation logic (`triangulation/triangulator.py`)

**Approach: robust per-joint multi-view triangulation — RANSAC over the camera
views, then a confidence-weighted DLT (Direct Linear Transform) on the inliers,
with distortion correction.**

- **Projection matrices:** `P_i = K_i · [R_i | T_i]` (same camera-from-world
  convention the calibration produced).
- **Undistortion:** every 2D point is run through `cv2.undistortPoints(..., P=K)`
  once per frame. The docstring stresses this is **required** — skipping it
  biases scale and reprojection for any real lens.
- **Per joint:**
  1. Collect observations from cameras where `confidence ≥ conf_thresh` (0.3).
  2. If fewer than `min_views` (2) cameras see it → output `NaN` (can't
     triangulate from one ray).
  3. **RANSAC (3+ views):** every camera *pair* proposes a 3D hypothesis; we
     reproject it into all the other views and count **inliers** (reprojection
     error < `RANSAC_REPROJ_PX`, default 12 px). The hypothesis with the most
     agreeing cameras wins, and the point is **re-solved using only the
     inliers**. This rejects bad views — left/right swaps and occlusion guesses
     — that would otherwise corrupt the solution. With exactly 2 views there is
     nothing to vote on, so it falls back to a direct DLT.
  4. **DLT solve** on the chosen inliers. Each camera contributes two rows:
     ```
     w · (x·P[2] − P[0])
     w · (y·P[2] − P[1])
     ```
     where `w` is the confidence (so confident cameras pull harder). Solve
     `A·X = 0` via **SVD** (right-singular vector of the smallest singular
     value), divide by the homogeneous coordinate → metric `(X, Y, Z)`.

This is robust least-squares multi-view triangulation: each camera gives a ray,
RANSAC discards the rays that disagree, and SVD finds the 3D point closest to
the surviving rays. (Toggle via `USE_RANSAC_TRIANGULATION` in `config.py`.)

### Temporal smoothing (`utils/smoothing.py`)

After triangulation, the live runtime passes the 33-joint skeleton through a
`SkeletonSmoother`:
- **Jump rejection** — if a joint moves more than `SMOOTH_MAX_JUMP_M` (0.30 m)
  between frames it is treated as a single-frame outlier and held at its previous
  value (catches the rare bad joint that survives RANSAC).
- **One-Euro filter** — per-coordinate adaptive smoothing: smooth when the joint
  is still, low-lag when it moves fast. `NaN` joints (seen by too few cameras
  this frame) pass through as the last known value.

---

## 5. The Live UI (`viz/` + PyQt5)

The calibration runs inside a **PyQt5 desktop app** (`viz/app.py`,
`viz/window.py`, `viz/sidebar.py`, `viz/viewport.py`).

- **Threading model:** `CalibPipeline.run()` executes in a background `QThread`
  (`PipelineWorker`) so the heavy OpenCV/calibration work never freezes the UI.
- **`CalibState` (`core/state.py`)** is the single, thread-safe source of truth.
  The pipeline writes progress/logs/camera data into it under a lock; the UI
  polls a JSON `snapshot()` to render the sidebar, log, and 3D viewport.
- **Recalibrate button** → `AppController.restart_pipeline()` safely stops the
  thread, calls `pipeline.reset()`, and starts fresh.

This is a clean **producer/consumer** split: pipeline produces state, UI
consumes it, communication is one shared locked object.

---

## 6. Configuration (`config.py`)

All tunables live in one file:

| Setting | Meaning |
|---|---|
| `NUM_CAMERAS = 3` | how many cameras to calibrate |
| `MAX_CAPTURES = 25` | target number of captured board poses |
| `CAPTURE_DELAY = 2.0` | seconds between auto-captures (forces pose variety) |
| `CHARUCO_ROWS/COLS` | board geometry — **must match the printed board** |
| `SQUARE_LENGTH = 0.04` | physical square size in meters — **sets metric scale** |
| `MIN_PAIR_FRAMES = 10` | min shared frames before a pair is "viable" |
| `BA_MAX_ITER / BA_FTOL` | bundle-adjustment solver limits |

⚠️ The single most important correctness rule: `SQUARE_LENGTH` / `MARKER_LENGTH`
**must equal the real printed board**, or every distance comes out wrong.

---

## 7. End-to-End Data Flow (summary)

```
                    ┌─────────────── CALIBRATION (PyQt app, main.py) ───────────────┐
  ChArUco board →   Capture → Intrinsics → Stereo pairs → BFS pose graph → (BA off) → save .npz
   (known size)      (per cam 2D)  (K,dist)   (rel R,T)     (world R,T)              │
                    └────────────────────────────────────────────────────────────────┘
                                                   │ output/multi_cam_calibration.npz
                                                   ▼
                    ┌──────────────── 3D MOCAP (run_pose_3d.py) ────────────────────┐
  N webcams →   MediaPipe 2D pose → undistort → DLT triangulation (SVD) → 3D skeleton
                    └────────────────────────────────────────────────────────────────┘
```

---

## 8. Key Design Decisions Recap (the "why")

1. **ChArUco over chessboard** — per-corner IDs solve cross-camera
   correspondence and survive partial views.
2. **Intrinsics first, then `CALIB_FIX_INTRINSIC` stereo** — decouples the
   problem; relative pose is much more stable when `K` is fixed.
3. **Pairwise stereo + BFS chaining** — scales to N cameras; you only need
   *overlapping pairs*, not all cameras seeing the board at once. BFS minimizes
   accumulated chain error.
4. **Fixed-geometry bundle adjustment** — refines globally *without* letting
   scale drift, because board points stay at their known metric positions.
   Enabled via `USE_BUNDLE_ADJUST`; essential once the camera chain/ring grows.
5. **Threaded capture** — defeats USB/DSHOW latency and black-frame issues on
   Windows.
6. **Robust RANSAC + DLT triangulation** with undistortion & confidence weights
   — rejects bad views (left/right swaps, occlusion guesses), then smooths the
   result over time, degrading gracefully (NaN) when a joint is seen by too few
   cameras.
```
