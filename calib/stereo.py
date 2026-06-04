import cv2
import numpy as np
from config import MIN_PAIR_FRAMES


class StereoCalibrator:
    """
    Calibrates every viable (i, j) camera pair using their shared frames.

    Returns
    -------
    pair_transforms : dict  (i, j) -> {"R": R_ij, "T": T_ij, "rms": float}
        R_ij, T_ij = pose of camera j expressed in camera i's coordinate frame.
        i < j always.
    """

    def __init__(self, board_config, state):
        self.cfg   = board_config
        self.state = state

    def calibrate_all(self, session, K_list, dist_list, image_size):
        """
        Parameters
        ----------
        session    : CaptureSession  (provides all_charuco_corners/ids, overlap_pairs)
        K_list     : list of (3,3) camera matrices
        dist_list  : list of distortion coefficient arrays
        image_size : (width, height)

        Returns
        -------
        pair_transforms : dict (i,j) -> {"R", "T", "rms"}
        """
        pair_transforms = {}

        viable_pairs = [
            (i, j) for (i, j), frames in session.overlap_pairs.items()
            if len(frames) >= MIN_PAIR_FRAMES
        ]

        if not viable_pairs:
            raise RuntimeError(
                "No viable camera pairs — not enough shared frames. "
                "Walk the board through overlapping camera views."
            )

        self.state.log(f"\nStereo calibration: {len(viable_pairs)} pairs")

        for (i, j) in viable_pairs:
            result = self._calibrate_pair(
                i, j,
                session, K_list, dist_list, image_size
            )
            if result is not None:
                pair_transforms[(i, j)] = result

        return pair_transforms

    # ──────────────────────────────────────────────────────────────
    def _calibrate_pair(self, i, j, session, K_list, dist_list, image_size):
        """Stereo-calibrate a single (i, j) pair using their shared frame list."""
        shared_frames = session.overlap_pairs[(i, j)]
        self.state.log(f"\n  Cam{i} <-> Cam{j}  ({len(shared_frames)} shared frames)")

        objpoints  = []
        imgpoints_i = []
        imgpoints_j = []

        for frame_idx in shared_frames:
            ids_i     = session.all_charuco_ids[i][frame_idx]
            ids_j     = session.all_charuco_ids[j][frame_idx]
            corners_i = session.all_charuco_corners[i][frame_idx]
            corners_j = session.all_charuco_corners[j][frame_idx]

            # both must have valid detections in this frame
            if ids_i is None or ids_j is None:
                continue

            # find common charuco corner IDs visible in both cameras
            common_ids = np.intersect1d(ids_i.flatten(), ids_j.flatten())
            if len(common_ids) < 6:
                continue

            objp  = []
            imgp_i = []
            imgp_j = []

            # board corner 3D positions come from the ChArUco board definition
            board_corners_3d = self.cfg.board.getChessboardCorners()

            for cid in common_ids:
                cid = int(cid)
                idx_i = np.where(ids_i.flatten() == cid)[0][0]
                idx_j = np.where(ids_j.flatten() == cid)[0][0]
                objp.append(board_corners_3d[cid])
                imgp_i.append(corners_i[idx_i][0])
                imgp_j.append(corners_j[idx_j][0])

            if len(objp) < 6:
                continue

            objpoints.append(np.array(objp,   dtype=np.float32))
            imgpoints_i.append(np.array(imgp_i, dtype=np.float32))
            imgpoints_j.append(np.array(imgp_j, dtype=np.float32))

        if len(objpoints) < 5:
            self.state.log(
                f"  SKIP Cam{i}<->Cam{j}: only {len(objpoints)} valid frames "
                f"(need 5+). Move board through their shared FOV more."
            )
            return None

        self.state.log(f"  Valid frames for stereo: {len(objpoints)}")

        rms, _, _, _, _, R, T, _, _ = cv2.stereoCalibrate(
            objpoints,
            imgpoints_i,
            imgpoints_j,
            K_list[i], dist_list[i],
            K_list[j], dist_list[j],
            image_size,
            flags=cv2.CALIB_FIX_INTRINSIC
        )

        self.state.log(f"  RMS reprojection error: {rms:.4f} px")
        self.state.log(f"  R:\n{np.round(R, 4)}")
        self.state.log(f"  T: {np.round(T.flatten(), 4)} m")

        return {"R": R, "T": T, "rms": rms}