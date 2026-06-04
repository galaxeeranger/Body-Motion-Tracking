from config          import NUM_CAMERAS, MAX_CAPTURES, USE_BUNDLE_ADJUST
from core.state      import CalibState
from calib.board     import BoardConfig
from calib.capture   import CaptureSession
from calib.intrinsic import IntrinsicCalibrator
from calib.stereo    import StereoCalibrator
from calib.graph     import PoseGraph
from calib.bundle_adjust import BundleAdjuster
from calib.saver     import CalibSaver


class CalibPipeline:
    """
    Orchestrator — sequences all calibration stages.

    Stage 1 — per-camera intrinsic calibration (independent, existing logic)
    Stage 2 — pairwise capture  (any subset of cameras can see board per frame)
    Stage 3 — stereo calibrate all viable (i,j) pairs
    Stage 4 — BFS spanning tree  -> initial world poses
    Stage 5 — bundle adjustment  -> refined world poses (no error accumulation)
    Stage 6 — save
    """

    def __init__(self):
        self.cfg   = BoardConfig()
        self.state = CalibState(NUM_CAMERAS, MAX_CAPTURES)
        self._build_stages()

    def _build_stages(self):
        self.session      = CaptureSession(self.cfg, self.state)
        self.intrinsic    = IntrinsicCalibrator(self.cfg, self.state)
        self.stereo       = StereoCalibrator(self.cfg, self.state)
        self.pose_graph   = PoseGraph(NUM_CAMERAS, self.state)
        self.bundle_adj   = BundleAdjuster(self.cfg, self.state)
        self.saver        = CalibSaver(self.state)

    def reset(self):
        self.state.reset()
        self._build_stages()
        self.state.log("Recalibration started")

    def run(self):
        self.state.update(board=self.cfg.to_dict())

        # ── stage 1: per-camera intrinsics ──────────────────────
        # NOTE: for a real room setup, run this ONCE per camera independently
        # before this pipeline. Here we run it on the same capture session
        # for simplicity — or you can load pre-saved intrinsics.
        self.state.update(phase="capturing")
        self.session.run()

        grays      = self.session.last_grays
        image_size = grays[0].shape[::-1]   # (width, height)

        # ── stage 2: intrinsic calibration ──────────────────────
        self.state.update(phase="calibrating")
        K_list, dist_list, _ = self.intrinsic.calibrate_all(
            self.session.all_charuco_corners,
            self.session.all_charuco_ids,
            image_size
        )

        # ── stage 3: stereo calibrate all viable pairs ──────────
        self.state.log("\nStage 3: pairwise stereo calibration")
        pair_transforms = self.stereo.calibrate_all(
            self.session, K_list, dist_list, image_size
        )

        if not pair_transforms:
            self.state.log("ERROR: no valid camera pairs found. Aborting.")
            self.state.update(phase="error")
            return

        # ── stage 4: BFS to get initial world poses ─────────────
        self.state.log("\nStage 4: building pose graph (BFS)")
        R_world_init, T_world_init = self.pose_graph.build_initial_poses(
            pair_transforms
        )

        # ── stage 5: bundle adjustment ──────────────────────────
        # Re-enabled for multi-camera scaling (SCALING_8_CAMERAS.md §2.1).
        # BFS chaining accumulates error around a ring; global BA distributes it
        # evenly. The optimiser keeps board geometry fixed (no scale drift) and
        # falls back to the BFS poses if it has too few observations.
        if USE_BUNDLE_ADJUST:
            self.state.log("\nStage 5: bundle adjustment")
            R_world, T_world = self.bundle_adj.optimise(
                self.session,
                K_list, dist_list,
                R_world_init, T_world_init
            )
        else:
            self.state.log("\nStage 5: bundle adjustment (skipped)")
            R_world = R_world_init
            T_world = T_world_init
        # ── stage 6: save ────────────────────────────────────────
        self.state.log("\nStage 6: saving calibration")
        self.saver.save(K_list, dist_list, R_world, T_world, image_size)