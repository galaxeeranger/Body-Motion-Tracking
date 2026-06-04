# Scaling to 8+ Cameras — Implementation Plan & Surround Calibration Guide

This document describes **what must be implemented** to make the system work
reliably with **8 or more cameras** arranged around a room (e.g. 4 corners + the
centre of each of the 4 walls), and **how to calibrate** such a rig by walking a
ChArUco board around the space.

It is a design/roadmap spec. Each section states **the problem at scale**, **what
to implement**, and **why**.

---

## Implementation Status (what is built vs. pending)

| Item | Section | Status | Where |
|---|---|---|---|
| Re-enable global bundle adjustment | §2.1 | ✅ **Done** | `config.USE_BUNDLE_ADJUST`, `calib/pipeline.py` |
| Loop-closure / weak-link diagnostics | §2.2 | ✅ **Done** | `calib/graph.py → _report_graph_health` |
| Robust RANSAC triangulation | §2.3 | ✅ **Done** | `triangulation/triangulator.py` |
| Per-joint temporal smoothing | §2.4 | ✅ **Done** | `utils/smoothing.py`, wired in `run_pose_3d.py` |
| Pass distortion into runtime triangulation | §2.3 | ✅ **Done** (bug fix) | `run_pose_3d.py` |
| Config constants for the above | §5 | ✅ **Done** | `config.py` |
| Feed all loop edges into BA | §2.2 | ⏳ Pending | BA still refines BFS tree poses |
| Sparse Jacobian for BA | §2.1 | ➖ Not needed | poses-only BA has a tiny parameter vector |
| Skeleton / bone-length constraints | §2.5 | ⏳ Pending | hand-off to `24_Mocap_add_ik.py` |
| Soft sync by timestamp window | §2.6 | ⏳ Pending | needs `_CameraThread` timestamps |
| Raise `NUM_CAMERAS` to 8 | §5 | ⏳ Manual | kept at 3 for current hardware |

> The code now **scales** to 8+ cameras (everything derives from `NUM_CAMERAS`),
> but `NUM_CAMERAS` is deliberately left at **3** to match the cameras currently
> connected. Bump it when the extra cameras are physically installed.

The verified behaviour: with a deliberately corrupted ("swapped") camera view,
robust RANSAC triangulation recovers the true joint exactly, whereas the old
plain DLT was pulled ~0.33 m off. Smoothing then rejects single-frame jumps.

---

## 0. Target Setup

```
            Back wall
     C4───────M3───────C3
     │                  │
 M4  │      person      │  M2
     │     (centre)      │
     │                  │
     C1───────M1───────C2
            Front wall

  C1..C4 = corner cameras      M1..M4 = wall-centre cameras
  All 8 cameras point inward at the centre of the room.
```

- **Cameras:** 8 (scalable to 10–12 by raising `NUM_CAMERAS`).
- **Goal:** full-body 360° markerless mocap of a person in the centre.
- **Coordinate frame:** one shared world frame, then re-levelled so the floor is
  Y=0 (optional ground step).

---

## 1. Why the Current Code Does Not Scale As-Is

| Area | Works at 3 cams (one wall) | Breaks at 8 cams (surround) |
|---|---|---|
| **Capture** | hard-coded camera count, USB on one hub | needs many cameras, USB bandwidth + sync |
| **Pose graph (BFS)** | short chain → tiny error | long ring chain → error accumulates badly |
| **Bundle adjustment** | safely **disabled** | **mandatory** — only thing that fixes ring drift |
| **Triangulation** | plain DLT, all views trusted | needs outlier rejection (left/right swaps, occlusion) |
| **Calibration capture** | board static-ish in front of 3 cams | board must be **walked around the whole room** |
| **Sync** | ignorable for slow motion | matters — 8 unsynced webcams blur fast motion |

The architecture (pairwise stereo → graph → BA → triangulate) is **fundamentally
correct** for N cameras. The work is in hardening five specific stages.

---

## 2. What Must Be Implemented

### 2.1 Re-enable Global Bundle Adjustment  *(highest priority)*

**Problem:** BFS (`graph.py`) chains pairwise transforms from Cam0 outward. In a
ring of 8 cameras the camera farthest from Cam0 is 4–5 stereo hops away, and each
hop adds error. The result drifts.

**Implement:**
- Un-comment **Stage 5** in `calib/pipeline.py` so the BFS poses are passed into
  `BundleAdjuster.optimise(...)` and the **refined** poses are saved.
- Keep the existing "fixed geometry" formulation in `calib/bundle_adjust.py`
  (board 3D points stay at their known metric positions; only camera poses are
  optimised). This prevents scale drift/explosion.
- Add a **sparse Jacobian** (scipy `least_squares(jac_sparsity=...)`) so BA stays
  fast with many cameras and thousands of observations.

**Why:** BA is the only stage that distributes error **globally and
symmetrically** across all cameras instead of letting it pile up at the end of
the chain. **For a ring, BFS alone is not accurate enough.**

### 2.2 Loop-Closure–Aware Pose Graph

**Problem:** BFS builds a *spanning tree* — it uses only enough edges to reach
every camera once, and **throws away extra pairs**. In a ring you want those
extra edges (they form loops) because they constrain the solution.

**Implement:**
- Keep BFS for the **initial** poses (it's fine as a starting guess).
- Feed **all** viable pairwise transforms (not just the tree edges) into BA as
  constraints, **including the edge that closes the ring** (e.g. last corner ↔
  first corner).
- Add a `graph` health check: report the number of independent loops and warn if
  any camera is connected by only a **single** edge ("weak link").

**Why:** Loop closure lets BA balance error around the whole ring. A camera held
by only one edge has no redundancy — one bad pair ruins it.

### 2.3 Robust Multi-View Triangulation  *(quality-critical at runtime)*

**Problem:** Today `triangulation/triangulator.py` runs a plain DLT over **every**
camera above `conf_thresh`. With surround cameras, a back camera can **swap
left/right** or **guess an occluded joint**, and that wrong ray corrupts the
least-squares result — producing a joint that "flies off."

**Implement (in `Triangulator`):**
1. **RANSAC triangulation per joint:**
   - Pick random 2-camera subsets, triangulate, then count how many *other*
     cameras agree (reproject the 3D point; inlier if pixel error < threshold,
     e.g. 10–15 px).
   - Keep the hypothesis with the most inliers; **re-triangulate using only the
     inliers**.
2. **Reprojection-error gating:** after the final solve, drop any camera whose
   reprojection error is still large and solve once more.
3. **Adaptive `min_views`:** require ≥2 inliers; prefer ≥3 when available for
   stability.
4. Keep the existing **confidence weighting** and **undistortion**.

**Why:** This is what kills left/right swaps and occlusion guesses. Coverage is
already solved by 8 cameras; **bad views sneaking in** is the real accuracy
limiter, and RANSAC removes them.

### 2.4 Per-Joint Temporal Smoothing

**Problem:** Single-frame outliers (a momentary swap) cause visible jumps.

**Implement:**
- A simple **One-Euro filter** or velocity-gated smoother per joint coordinate.
- A **jump rejector:** if a joint moves more than a physically impossible amount
  between frames, hold the previous value / mark low-confidence.

**Why:** Cleans residual jitter and catches the occasional bad frame that slips
past RANSAC.

### 2.5 Skeleton / Bone-Length Constraints  *(optional, strong upgrade)*

**Problem:** Independently triangulated joints can have impossible bone lengths
(forearm stretching frame to frame).

**Implement:**
- Calibrate each subject's bone lengths once (median over a calm sequence).
- Enforce them each frame (project triangulated joints onto the constraint, or a
  small per-frame optimisation), then hand off to **IK** (`24_Mocap_add_ik.py`).

**Why:** Turns a noisy point cloud of joints into a coherent, rig-ready skeleton.

### 2.6 Capture Scaling & Synchronization

**Problem:** 8 USB webcams overload a single bus, and unsynchronized frames blur
fast motion across views.

**Implement:**
- **Bandwidth:** keep **MJPG**; distribute cameras across **multiple USB
  controllers / PCIe USB cards** (not one hub). Consider lowering per-camera FPS
  during calibration only.
- **Soft sync:** timestamp every grabbed frame in each `_CameraThread`; in the
  main loop, only triangulate frames whose timestamps fall within a small window
  (e.g. < 15 ms). Drop frames that can't be matched.
- **Hard sync (best):** move to genuinely synchronized cameras (global-shutter
  machine-vision cameras with a hardware trigger) for production-grade fast
  motion.
- **Config:** `NUM_CAMERAS` already drives most loops; audit for any hard-coded
  `range(3)` / index assumptions and make everything derive from `NUM_CAMERAS`.

**Why:** Coverage and calibration can be perfect, but unsynced fast motion still
breaks triangulation. Sync is the scaling bottleneck people forget.

### 2.7 Calibration Quality Reporting & Diagnostics

**Implement:**
- Per-pair **RMS** table (already partly there) + a **graph diagram** of which
  pairs connected and which are weak.
- After BA: **per-camera reprojection RMS** and a **scale check** (reconstruct a
  known board length, compare to physical → % error).
- A red/green **"calibration health"** summary so the operator knows whether to
  re-capture before trusting the result.

**Why:** With 8 cameras you cannot eyeball correctness; you need numbers that say
"this rig is good."

---

## 3. How to Calibrate a Surround Rig (ChArUco Board Walk)

This is the **operating procedure**, and the most important practical part.

### 3.1 The Core Principle

A flat ChArUco board is only visible from **one side**, so **no single board
placement is seen by all 8 cameras.** That is fine. The requirement is:

> Every **neighbouring camera pair** must share **enough frames** (≥
> `MIN_PAIR_FRAMES`, raise to ~15–20), and the pair-graph must be **connected**
> and contain **loops**.

The board is **walked around the room** so that, over time, each adjacent pair of
cameras gets a batch of shared views. The cameras far apart never co-observe the
board — they link **transitively** through the chain, and **bundle adjustment**
ties it all together.

### 3.2 The Walk Pattern

```
   Start in front of M1 (front wall) ── visible to C1, M1, C2
        │  move board slowly, tilt at varied angles
        ▼
   Walk toward the right wall ──────── overlap zone: C2 sees board with M2
        │
        ▼
   In front of M2 (right wall) ─────── visible to C2, M2, C3
        │
        ▼
   Continue to back wall ───────────── overlap: C3 with M3
        │
        ▼
   In front of M3 (back wall) ──────── visible to C3, M3, C4
        │
        ▼
   Continue to left wall ───────────── overlap: C4 with M4
        │
        ▼
   In front of M4 (left wall) ──────── visible to C4, M4, C1
        │
        ▼
   Return to front (CLOSE THE LOOP) ── overlap: C1 with M1 again
```

**Rules for the operator:**
1. **Move slowly**, pausing so auto-capture fires (every `CAPTURE_DELAY` seconds
   when ≥2 cameras see the board).
2. **Tilt and rotate** the board at each spot — varied angles give better
   stereo/intrinsic solutions than flat-on views.
3. **Linger in the overlap zones** between two walls — this is where neighbouring
   cameras both see the board and the chain links form.
4. **Close the loop:** finish where you started so the ring's last edge exists.
5. Keep the board **fully inside** each camera's frame when capturing for that
   pair (partial is OK thanks to ChArUco IDs, but fuller = better).
6. Aim for **20–40 captures per neighbouring pair**.

### 3.3 What the Software Checks After the Walk

- **Coverage report** (`capture.py → _report_coverage`): lists each pair's shared
  frame count and flags `INSUFFICIENT` pairs. Re-walk those zones.
- **Connectivity (BFS reachability):** every camera must reach Cam0. If not, a
  wall transition was missed — re-capture that gap.
- **Loop check (new, §2.2):** warns about single-edge "weak link" cameras.

### 3.4 Intrinsics First (recommended for 8 cams)

For a big rig, calibrate **intrinsics per camera once, offline**, by holding the
board close and filling each camera's frame at many angles. Save and **load**
them, then run only the **extrinsic** (stereo + graph + BA) walk above with
`CALIB_FIX_INTRINSIC`. This is more accurate and far less error-prone than
solving intrinsics from the surround walk.

---

## 4. Putting It Together — New Pipeline at Scale

```
OFFLINE (once per camera)
  └─ Intrinsic calibration per camera (board close, many angles) → save K, dist

CALIBRATION (board walk around room)
  1. Capture          neighbouring pairs share frames (loop closed)
  2. Load intrinsics  (K, dist from offline step)
  3. Stereo pairs     ALL viable pairs (CALIB_FIX_INTRINSIC)
  4. Pose graph (BFS) initial world poses + keep extra edges for loops
  5. Bundle adjust    GLOBAL refine over all cams + loop edges  ← RE-ENABLED
  6. Quality report   per-cam RMS, scale %, weak-link warnings
  7. Save             multi_cam_calibration.npz

RUNTIME (live mocap)
  1. Grab 8 cams      soft-sync by timestamp window
  2. MediaPipe 2D     33 joints × conf, per camera
  3. Undistort
  4. Robust triangulate  per joint: RANSAC + reproj gating  ← NEW
  5. Temporal smooth  per-joint filter + jump rejection      ← NEW
  6. Skeleton/IK      bone-length constraints → IK retarget  ← NEW/optional
```

---

## 5. Config Changes Required (`config.py`)

| Setting | Now | For 8+ cams |
|---|---|---|
| `NUM_CAMERAS` | 3 | 8 (or more) |
| `MIN_PAIR_FRAMES` | 10 | 15–20 |
| `MAX_CAPTURES` | 25 | 80–150 (more poses to cover the ring) |
| Bundle adjustment | disabled | **enabled** |
| `CAMERA_COLORS` | 10 colours | already enough; extend if > 10 cams |
| (new) `RANSAC_REPROJ_PX` | – | ~10–15 px inlier threshold |
| (new) `SYNC_WINDOW_MS` | – | ~15 ms frame-match window |

---

## 6. Priority Order (what to build first)

1. **Re-enable bundle adjustment** (§2.1) — without it, nothing else matters for a ring.
2. **Robust RANSAC triangulation** (§2.3) — removes flying joints at runtime.
3. **Loop-closure graph + diagnostics** (§2.2, §2.7) — makes calibration trustworthy.
4. **Soft sync by timestamp** (§2.6) — needed once motion is fast.
5. **Temporal smoothing** (§2.4) — polish.
6. **Skeleton/bone-length + IK** (§2.5) — production rig output.

---

## 7. Summary

- The existing **pairwise → graph → BA → triangulate** architecture **scales to
  8+ cameras** — it does **not** need redesigning.
- **Coverage is solved by surrounding the room**: each joint is triangulated only
  from the cameras that actually see it, so front/back/side gaps cancel out.
- The real work is **hardening five stages**: re-enable **bundle adjustment**,
  add **loop closure**, add **robust (RANSAC) triangulation**, add **soft sync**,
  and add **smoothing/skeleton constraints**.
- Calibration is done by **walking a ChArUco board around the room**, lingering in
  overlap zones between neighbouring cameras and **closing the loop**, so the
  pair-graph is connected with redundant edges for bundle adjustment to refine.
