NUM_CAMERAS   = 5
CAPTURE_DELAY = 2.0
MAX_CAPTURES  = 100    # raised for 5 cameras — more poses to cover all pairs
VIZ_PORT      = 5050
OUTPUT_PATH   = "output/multi_cam_calibration.npz"

CHARUCO_ROWS  = 5  #3
CHARUCO_COLS  = 7  #5
SQUARE_LENGTH =  0.04 #0.04   # ⚠️ must match your SVG
MARKER_LENGTH =   0.02 #0.02   # ⚠️ must match your SVG
# ── bundle adjustment ──────────────────────────
BA_MAX_ITER     = 200        # scipy least_squares max iterations
BA_FTOL         = 1e-6       # function tolerance for convergence

# minimum shared frames required between a camera pair to attempt stereo calibration
# (raise toward 15-20 for an 8+ camera surround ring — see SCALING_8_CAMERAS.md)
MIN_PAIR_FRAMES     = 10

# ── bundle adjustment toggle ───────────────────
# Re-enabled for scaling: BFS chaining accumulates error around a camera ring,
# and global BA is the only stage that distributes it evenly. Safe at any count
# (BA falls back to the BFS poses if it has too few observations).
USE_BUNDLE_ADJUST   = True

# ── robust triangulation (runtime) ─────────────
# RANSAC over per-joint multi-view observations rejects bad views (left/right
# swaps, occlusion guesses) before the final DLT solve.
USE_RANSAC_TRIANGULATION = True
RANSAC_REPROJ_PX         = 12.0   # inlier threshold: max reprojection error (px)
RANSAC_MIN_INLIERS       = 2      # minimum agreeing cameras to accept a joint

# ── temporal smoothing (runtime) ───────────────
SMOOTH_FREQ        = 30.0    # approx capture frame rate (Hz)
SMOOTH_MINCUTOFF   = 1.0     # lower = smoother when still
SMOOTH_BETA        = 0.3     # higher = less lag during fast motion
SMOOTH_MAX_JUMP_M  = 0.30    # reject joint jumps larger than this (metres)

# ── multi-camera soft sync ─────────────────────
SYNC_WINDOW_MS     = 15.0    # match frames across cameras within this window

# ── visualisation ──────────────────────────────
# single source — sidebar.py and saver.py both import from here
CAMERA_COLORS   = ["#4A9EE8", "#E87A4A", "#4AE87A", "#D080E0", "#E8D440",
                   "#E84A9E", "#4AE8D0", "#E8C44A", "#9E4AE8", "#4AE870"]