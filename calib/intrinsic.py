
import cv2

class IntrinsicCalibrator:

    def __init__(self, board_config, state):
        self.cfg   = board_config
        self.state = state

    def calibrate_all(self, all_corners, all_ids, image_size):

        aruco      = cv2.aruco
        K_list     = []
        dist_list  = []
        tvecs_list = []

        for i in range(len(all_corners)):

            self.state.log(f"\n📷 Calibrating Camera {i}")

            valid_corners = []
            valid_ids     = []

            # ── FILTER INVALID FRAMES ─────────────────────
            for corners, ids in zip(all_corners[i], all_ids[i]):

                if corners is None or ids is None:
                    continue

                # must have enough points
                if len(ids) < 4:
                    continue

                # sanity check (VERY IMPORTANT)
                if len(corners) != len(ids):
                    continue

                valid_corners.append(corners)
                valid_ids.append(ids)

            # ── SAFETY CHECK ──────────────────────────────
            if len(valid_corners) < 5:
                raise Exception(
                    f"❌ Camera {i} has too few valid frames "
                    f"({len(valid_corners)}) for calibration"
                )

            self.state.log(
                f"  Using {len(valid_corners)} valid frames (filtered)"
            )

            # ── CALIBRATION ───────────────────────────────
            ret, K, dist, rvecs, tvecs = aruco.calibrateCameraCharuco(
                charucoCorners = valid_corners,
                charucoIds     = valid_ids,
                board          = self.cfg.board,
                imageSize      = image_size,
                cameraMatrix   = None,
                distCoeffs     = None
            )

            self.state.log(f"✅ Camera {i} K:\n{K}")
            self.state.log(f"✅ Camera {i} dist:\n{dist}")

            K_list.append(K)
            dist_list.append(dist)
            tvecs_list.append(tvecs)

        return K_list, dist_list, tvecs_list