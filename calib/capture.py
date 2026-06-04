

import cv2
import time
import threading
from config import NUM_CAMERAS, CAPTURE_DELAY, MAX_CAPTURES, MIN_PAIR_FRAMES


class _CameraThread:
    """
    Dedicated thread per camera.

    Continuously grabs frames in a tight loop so the internal OpenCV
    buffer never stalls. The main thread only calls .get_latest() to
    read the most-recent decoded frame — no waiting, no stale buffer.

    Why this matters with a 5 m USB 3.0 cable:
      - USB isochronous transfer latency is higher at cable extremes.
      - OpenCV's default buffer depth is 10+ frames. By the time the
        main thread reads, it may be 300–500 ms behind reality.
      - This thread drains that buffer continuously; the main thread
        always gets the freshest decoded frame.
    """

    def __init__(self, cam_idx: int):
        self.cam_idx = cam_idx
        self._cap    = cv2.VideoCapture(cam_idx, cv2.CAP_DSHOW)

        if not self._cap.isOpened():
            raise RuntimeError(f"Cannot open camera {cam_idx}")

        # ── force format negotiation ONCE before thread starts ──
        # This prevents DSHOW renegotiating mid-stream (causes black frames)
        self._cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self._cap.set(cv2.CAP_PROP_FPS,          15)
        self._cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)

        self._lock        = threading.Lock()
        self._frame       = None
        self._ok          = False
        self._frame_count = 0          # counts successfully decoded frames
        self._stop        = threading.Event()
        self._thread = threading.Thread(
            target=self._reader_loop,
            name=f"cam{cam_idx}_reader",
            daemon=True,          # dies automatically when main exits
        )

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=2.0)
        self._cap.release()

    def get_latest(self):
        """Returns (ok, frame) — always the most-recent decoded frame."""
        with self._lock:
            return self._ok, (self._frame.copy() if self._frame is not None else None)

    @property
    def frame_count(self):
        """Total frames successfully decoded so far."""
        with self._lock:
            return self._frame_count

    @property
    def image_size(self):
        """(width, height) — read once after open."""
        w = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        return (w, h)

    def _reader_loop(self):
        """Hot loop: grab → decode → store. Never pause."""
        # staggered delay: cam0=1s, cam1=2s, cam2=3s, cam3=4s
        # stops all cameras hammering USB at the same time
        # without this, cam1 returns black frames on Windows DSHOW
        time.sleep(1.0 + self.cam_idx * 1.0)

        consecutive_fails = 0
        while not self._stop.is_set():
            ok, frame = self._cap.read()
            if not ok:
                consecutive_fails += 1
                if consecutive_fails > 10:
                    time.sleep(0.5)
                continue
            consecutive_fails = 0
            with self._lock:
                self._ok          = ok
                self._frame       = frame
                self._frame_count += 1


# ══════════════════════════════════════════════════════════════════════════════

class CaptureSession:
    """
    Pairwise-aware capture loop — threaded, lag-free version.

    Key changes vs original
    -----------------------
    1. Each camera runs in its own _CameraThread.
       The main loop calls get_latest() — non-blocking, always fresh.

    2. Detection is still on the main thread but is now the ONLY work
       happening there (no blocking VideoCapture.read() calls).
       For 4 cameras at 640x480 with DICT_4X4_50 this is fast enough.

    3. Display windows are throttled to ~15 fps via a simple timer so
       imshow doesn't eat CPU when detection is fast.

    Storage layout (unchanged — StereoCalibrator contract preserved)
    ----------------------------------------------------------------
    all_charuco_corners[cam_idx]  : list of charuco corner arrays (one per captured frame)
    all_charuco_ids[cam_idx]      : list of charuco id arrays    (one per captured frame)
    frame_mask[cam_idx]           : bool list — True if cam saw board in that frame
    overlap_pairs                 : dict  (i, j) -> [frame_idx, ...]  i < j always
    """

    def __init__(self, board_config, state):
        self.cfg             = board_config
        self.state           = state
        self.num_cameras     = NUM_CAMERAS
        self.capture_delay   = CAPTURE_DELAY
        self.max_captures    = MAX_CAPTURES
        self.min_pair_frames = MIN_PAIR_FRAMES

        # per-camera storage  (indexed [cam][frame])
        self.all_charuco_corners = [[] for _ in range(NUM_CAMERAS)]
        self.all_charuco_ids     = [[] for _ in range(NUM_CAMERAS)]
        self.frame_mask          = [[] for _ in range(NUM_CAMERAS)]

        # overlap_pairs[(i,j)] = list of frame indices shared by cam i and cam j
        self.overlap_pairs = {}

        self.last_grays = []

    # ─────────────────────────────────────────────────────────────
    # Main entry point
    # ─────────────────────────────────────────────────────────────
    def run(self):
        # ── start one thread per camera ──
        cam_threads = []
        for i in range(self.num_cameras):
            ct = _CameraThread(i)
            ct.start()
            cam_threads.append(ct)

        # ── wait until every camera has decoded ≥10 frames ──
        # (flushes DSHOW black/warmup frames; cam3 needs ~4s stagger + init time)
        self.state.log("Waiting for all cameras to initialize...")
        deadline = time.time() + 20.0
        while time.time() < deadline:
            counts = [ct.frame_count for ct in cam_threads]
            # require each camera to have decoded ≥10 frames
            # (flushes DSHOW warmup/black frames before we start)
            if all(c >= 10 for c in counts):
                break
            ready = sum(1 for c in counts if c >= 10)
            self.state.log(f"  Cameras ready: {ready}/{self.num_cameras}  frames={counts}")
            time.sleep(0.5)
        else:
            counts = [ct.frame_count for ct in cam_threads]
            self.state.log(f"WARNING: not all cameras ready after 20s — frames={counts}")
        self.state.log("All cameras ready — starting capture loop")
        last_capture_time = 0.0
        last_display_time = 0.0
        display_interval  = 1.0 / 15.0   # cap display at ~15 fps
        frame_count       = 0

        aruco      = cv2.aruco
        dictionary = self.cfg.dictionary
        board      = self.cfg.board

        self.state.log("Threaded capture started — lag-free mode")
        self.state.log(f"Target: {self.max_captures} frames, min {self.min_pair_frames} per pair")

        try:
            while True:
                now = time.time()

                # ── grab latest frame from every camera (non-blocking) ──
                frames     = []
                grays      = []
                detections = []   # (ok, ch_corners, ch_ids) per camera

                all_ok = True
                for i, ct in enumerate(cam_threads):
                    ok, frame = ct.get_latest()
                    if not ok or frame is None:
                        all_ok = False
                        break
                    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    frames.append(frame)
                    grays.append(gray)

                    # ── detect ──
                    corners, ids, _ = aruco.detectMarkers(gray, dictionary)
                    detected = False
                    ch_corners, ch_ids = None, None

                    if ids is not None and len(ids) > 0:
                        _, ch_corners, ch_ids = aruco.interpolateCornersCharuco(
                            corners, ids, gray, board
                        )
                        if ch_ids is not None and len(ch_ids) > 5:
                            detected = True

                    detections.append((detected, ch_corners, ch_ids))
                    self.state.set_cam_detected(i, detected)

                if not all_ok:
                    time.sleep(0.005)
                    continue

                num_detected = sum(1 for d in detections if d[0])

                # ── auto-capture ──
                if num_detected >= 2 and (now - last_capture_time) > self.capture_delay:
                    for i, (det, ch_corners, ch_ids) in enumerate(detections):
                        self.all_charuco_corners[i].append(ch_corners)
                        self.all_charuco_ids[i].append(ch_ids)
                        self.frame_mask[i].append(det)

                    self._record_overlap(frame_count, detections)
                    last_capture_time = now
                    frame_count      += 1
                    n = self.state.increment_captures()
                    self.state.log(
                        f"Frame {n}: {num_detected}/{self.num_cameras} cams detected"
                        f" — pairs so far: {self._pair_summary()}"
                    )

                # ── display: throttled to 15 fps ──
                if now - last_display_time >= display_interval:
                    for i, (frame, detection) in enumerate(zip(frames, detections)):
                        disp = frame.copy()
                        ok_d, ch_corners, ch_ids = detection

                        if ok_d:
                            corners_raw, ids_raw, _ = aruco.detectMarkers(
                                grays[i], dictionary
                            )
                            if ids_raw is not None:
                                aruco.drawDetectedMarkers(disp, corners_raw, ids_raw)
                            aruco.drawDetectedCornersCharuco(disp, ch_corners, ch_ids)

                        status = "DETECTED" if ok_d else "NOT DETECTED"
                        color  = (0, 255, 0) if ok_d else (0, 0, 255)
                        cv2.putText(disp, f"Cam{i}: {status}", (10, 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
                        cv2.putText(disp, f"Frames: {frame_count}/{self.max_captures}",
                                    (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                                    (200, 200, 200), 1)
                        cv2.imshow(f"Cam{i}", disp)

                    last_display_time = now

                if frame_count >= self.max_captures:
                    self.state.log("Target frames reached")
                    break

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self.state.log("Capture stopped by user")
                    break

                # ── small sleep keeps CPU sane without adding meaningful lag ──
                time.sleep(0.005)

        finally:
            for ct in cam_threads:
                ct.stop()
            cv2.destroyAllWindows()

        self.last_grays = grays
        self._report_coverage()

    # ─────────────────────────────────────────────────────────────
    # Overlap tracking helpers  (unchanged)
    # ─────────────────────────────────────────────────────────────
    def _record_overlap(self, frame_idx, detections):
        for i in range(self.num_cameras):
            for j in range(i + 1, self.num_cameras):
                if detections[i][0] and detections[j][0]:
                    key = (i, j)
                    if key not in self.overlap_pairs:
                        self.overlap_pairs[key] = []
                    self.overlap_pairs[key].append(frame_idx)

    def _pair_summary(self):
        parts = [f"({i},{j}):{len(v)}"
                 for (i, j), v in sorted(self.overlap_pairs.items())]
        return " ".join(parts) if parts else "none yet"

    def _report_coverage(self):
        self.state.log("\n── Overlap coverage report ──")
        viable = []
        for (i, j), frames in sorted(self.overlap_pairs.items()):
            n  = len(frames)
            ok = n >= self.min_pair_frames
            self.state.log(f"  Cam{i} <-> Cam{j}: {n} frames  [{'OK' if ok else 'INSUFFICIENT'}]")
            if ok:
                viable.append((i, j))

        reachable   = self._reachable_cameras(viable)
        unreachable = set(range(self.num_cameras)) - reachable
        if unreachable:
            self.state.log(
                f"WARNING: cameras {unreachable} have no viable overlap path to Cam0"
            )
        else:
            self.state.log("All cameras reachable from Cam0 — graph is connected")

    def _reachable_cameras(self, viable_pairs):
        adj = {i: set() for i in range(self.num_cameras)}
        for (i, j) in viable_pairs:
            adj[i].add(j)
            adj[j].add(i)
        visited, queue = {0}, [0]
        while queue:
            node = queue.pop(0)
            for nb in adj[node]:
                if nb not in visited:
                    visited.add(nb)
                    queue.append(nb)
        return visited